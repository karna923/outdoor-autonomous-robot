#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pins_esp2.hpp"
#include "motor_control.hpp"
#include <cstddef>
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "wifi_config.hpp"
#include <cstring>
#include <esp_timer.h>


//microros
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <uros_network_interfaces.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/range.h>
#include <cmath>

Motor* motor_rl = nullptr;
Motor* motor_rr = nullptr;


volatile int32_t enc_count_rl = 0;
volatile int32_t enc_count_rr = 0;

#define DESKTOP_IP "192.168.86.24"
#define UROS_PORT 8888

// Servo LEDC
#define SERVO_LEDC_TIMER    LEDC_TIMER_2
#define SERVO_LEDC_CHANNEL  LEDC_CHANNEL_4
#define SERVO_FREQ_HZ       50
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_RES           LEDC_TIMER_14_BIT

// Sweep angles (degrees), center = 0
#define SWEEP_MIN_DEG      -50
#define SWEEP_MAX_DEG       50
#define SWEEP_STEP_DEG      10

// micro-ROS
rcl_node_t node;
rcl_publisher_t range_pub;
sensor_msgs__msg__Range range_msg;

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
  } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGW("wifi", "Disconnected, retrying...");
    esp_wifi_connect();
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI("wifi", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

void wifi_init_sta(void) {
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {};
  memcpy(wifi_config.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
  memcpy(wifi_config.sta.password, WIFI_PASSWORD, strlen(WIFI_PASSWORD));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

void IRAM_ATTR isr_enc_rl(void* arg) {
    enc_count_rl = enc_count_rl + 1;
}
  
void IRAM_ATTR isr_enc_rr(void* arg) {
    enc_count_rr = enc_count_rr + 1;
  }


static uint32_t angle_to_duty(float deg) {
  float us = SERVO_MIN_US + (deg + 90.0f) /  180.0f * (SERVO_MAX_US - SERVO_MIN_US);
  uint32_t max_duty = (1 << 14) - 1;
  return (uint32_t) (us / (1000000.0f / SERVO_FREQ_HZ) * max_duty);
  }

static void servo_init(void) {
  ledc_timer_config_t timer = {};
  timer.speed_mode       = LEDC_LOW_SPEED_MODE;
  timer.timer_num        = SERVO_LEDC_TIMER;
  timer.duty_resolution  = SERVO_RES;
  timer.freq_hz          = SERVO_FREQ_HZ;
  timer.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&timer);

  ledc_channel_config_t ch = {};
  ch.speed_mode = LEDC_LOW_SPEED_MODE;
  ch.channel    = SERVO_LEDC_CHANNEL;
  ch.timer_sel  = SERVO_LEDC_TIMER;
  ch.gpio_num   = SERVO_PWM;
  ch.duty       = angle_to_duty(0);
  ch.hpoint     = 0;
  ledc_channel_config(&ch);
}

static float hcsr04_read_cm(void) {
  // Trigger: 10us pulse
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG, 0);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)ULTRASONIC_TRIG, 0);

    // Wait for echo high, timeout ~30ms
    int64_t start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)ULTRASONIC_ECHO) == 0) {
        if (esp_timer_get_time() - start > 30000) return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)ULTRASONIC_ECHO) == 1) {
        if (esp_timer_get_time() - echo_start > 30000) return -1.0f;
    }
    int64_t echo_end = esp_timer_get_time();

    float duration_us = (float)(echo_end - echo_start);
    return (duration_us * 0.0343f) / 2.0f;
}


void control_task(void *param) {

  int32_t last_rl = 0, last_rr = 0;
  while(1) {
    
    int32_t curr_rl = enc_count_rl, curr_rr = enc_count_rr;

    //delta tick and velocity conversion

    int32_t delta_rl = curr_rl - last_rl;
    float vel_rl = delta_rl * 0.000224f / 0.001f;

    int32_t delta_rr = curr_rr - last_rr;
    float vel_rr = delta_rr * 0.000224f / 0.001f;

    //compute call
    motor_rl->compute(0.0f, vel_rl);
    motor_rr->compute(0.0f, vel_rr);

    last_rl = curr_rl;
    last_rr = curr_rr;

    vTaskDelay(pdMS_TO_TICKS(1));
    
  }
}

void microros_task(void *param) {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    rmw_uros_options_set_udp_address(DESKTOP_IP, "8888", rmw_options);

    while (rmw_uros_ping_agent(100, 3) != RCL_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    rclc_node_init_default(&node, "rear_node", "", &support);

    rclc_publisher_init_default(
        &range_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonic_range"
    );

    range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    range_msg.field_of_view  = 0.261799f;
    range_msg.min_range      = 0.02f;
    range_msg.max_range      = 4.0f;

    static char frame_id[] = "ultrasonic_link";
    range_msg.header.frame_id.data     = frame_id;
    range_msg.header.frame_id.size     = strlen(frame_id);
    range_msg.header.frame_id.capacity = strlen(frame_id) + 1;

    servo_init();

    gpio_config_t trig_conf = {};
    trig_conf.pin_bit_mask = (1ULL << ULTRASONIC_TRIG);
    trig_conf.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {};
    echo_conf.pin_bit_mask = (1ULL << ULTRASONIC_ECHO);
    echo_conf.mode         = GPIO_MODE_INPUT;
    gpio_config(&echo_conf);

    while (1) {
        for (int deg = SWEEP_MIN_DEG; deg <= SWEEP_MAX_DEG; deg += SWEEP_STEP_DEG) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL, angle_to_duty((float)deg));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, SERVO_LEDC_CHANNEL);

            vTaskDelay(pdMS_TO_TICKS(150));

            float dist_cm = hcsr04_read_cm();
            if (dist_cm < 0.0f) continue;

            int64_t now = esp_timer_get_time();
            range_msg.header.stamp.sec     = (int32_t)(now / 1000000);
            range_msg.header.stamp.nanosec = (uint32_t)((now % 1000000) * 1000);
            range_msg.range = dist_cm / 100.0f;

            rcl_publish(&range_pub, &range_msg, NULL);
        }
    }
}
extern "C" void app_main(void) {
  Motor::init_timer();

  gpio_install_isr_service(0);

  gpio_config_t enc_config_rl = {};
  enc_config_rl.pin_bit_mask = (1ULL << ENC_RL);
  enc_config_rl.mode = GPIO_MODE_INPUT;
  enc_config_rl.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_rl.pull_down_en = GPIO_PULLDOWN_DISABLE;
  enc_config_rl.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&enc_config_rl);

  gpio_config_t enc_config_rr = {};
  enc_config_rr.pin_bit_mask = (1ULL << ENC_RR);
  enc_config_rr.mode = GPIO_MODE_INPUT;
  enc_config_rr.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_rr.pull_down_en = GPIO_PULLDOWN_DISABLE;
  enc_config_rr.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&enc_config_rr);


  gpio_isr_handler_add((gpio_num_t)ENC_RL, isr_enc_rl, nullptr);
  gpio_isr_handler_add((gpio_num_t)ENC_RR, isr_enc_rr, nullptr);

  motor_rl = new Motor(MOTOR_RL_A, MOTOR_RL_B, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
  motor_rr = new Motor(MOTOR_RR_A, MOTOR_RR_B, LEDC_CHANNEL_2, LEDC_CHANNEL_3);


  motor_rl->setPID(50.0f, 5.0f, 0.5f);
  motor_rr->setPID(50.0f, 5.0f, 0.5f);

  wifi_init_sta();

  xTaskCreate(microros_task, "microros_task",
            8192, NULL,
            4, NULL);

  xTaskCreate(control_task, "control_task",
              4096, (void*)"Hello ESP32",
              5, NULL);

  
  }

