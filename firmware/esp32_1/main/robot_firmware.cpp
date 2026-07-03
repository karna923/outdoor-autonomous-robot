#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pins_esp1.hpp"
#include "motor_control.hpp"
#include <cstddef>
#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "wifi_config.hpp"
#include <cstring>


//micro-ROS
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <uros_network_interfaces.h>
#include <rmw_microxrcedds_c/config.h>

//UDP
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <cmath>

//IMU
#include "driver/i2c.h"

//GPS
#include "driver/uart.h"
#define GPS_UART_NUM UART_NUM_2
#define GPS_BAUD_RATE 9600
#define GPS_BUF_SIZE 512


#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_OUT 0x3B
#define MPU6050_ACCEL_SCALE  16384.0f
#define MPU6050_GYRO_SCALE   131.0f
#define GRAVITY_MS2          9.81f
#define DEG_TO_RAD           (3.14159265358979f / 180.0f)

// Wheel/encoder constants — derive meters/tick from actual wheel size instead of
// hardcoding a magic number in six places. Update WHEEL_DIAMETER_M when the wheel changes.
#define WHEEL_DIAMETER_M 0.085f
#define TICKS_PER_REV 1400.0f
#define METERS_PER_TICK (M_PI * WHEEL_DIAMETER_M / TICKS_PER_REV)  // 0.0001907 m/tick

// Middle motors (ML/MR) are not physically installed yet. Set this to 1 once the
// hardware arrives and is wired up. Do NOT just delete the ML/MR code, flip this instead.
#define MIDDLE_MOTORS_INSTALLED 0


Motor* motor_fl = nullptr;
Motor* motor_fr = nullptr;
#if MIDDLE_MOTORS_INSTALLED
Motor* motor_ml = nullptr;
Motor* motor_mr = nullptr;
#endif

volatile int32_t enc_count_fl = 0;
volatile int32_t enc_count_fr = 0;
#if MIDDLE_MOTORS_INSTALLED
volatile int32_t enc_count_ml = 0;
volatile int32_t enc_count_mr = 0;
#endif

// micro-ROS globals
rcl_node_t node;
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t gps_pub;
rclc_support_t support;
rcl_allocator_t allocator;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__NavSatFix gps_msg;

// Odometry state
float odom_x = 0.0f;
float odom_y = 0.0f;
float odom_theta = 0.0f;
#define WHEEL_BASE 0.4f  // meters — measure your actual chassis. Unaffected by ML/MR presence;
                         // FL/FR are still the leftmost/rightmost wheels.
#define DESKTOP_IP "192.168.86.24"
#define UROS_PORT 8888



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


void IRAM_ATTR isr_enc_fl(void* arg) {
    enc_count_fl = enc_count_fl + 1;
}
  
void IRAM_ATTR isr_enc_fr(void* arg) {
    enc_count_fr = enc_count_fr + 1;
  }

#if MIDDLE_MOTORS_INSTALLED
void IRAM_ATTR isr_enc_ml(void* arg) {
    enc_count_ml = enc_count_ml + 1;
  }

void IRAM_ATTR isr_enc_mr(void* arg) {
    enc_count_mr = enc_count_mr + 1;
  }
#endif

void control_task(void *param) {

#if MIDDLE_MOTORS_INSTALLED
  int32_t last_fl = 0, last_fr = 0, last_ml = 0, last_mr = 0;
#else
  int32_t last_fl = 0, last_fr = 0;
#endif
  while(1) {
    
#if MIDDLE_MOTORS_INSTALLED
    int32_t curr_fl = enc_count_fl, curr_fr = enc_count_fr, curr_ml = enc_count_ml, curr_mr = enc_count_mr;
#else
    int32_t curr_fl = enc_count_fl, curr_fr = enc_count_fr;
#endif

    //delta tick and velocity conversion

    int32_t delta_fl = curr_fl - last_fl;
    float vel_fl = delta_fl * METERS_PER_TICK / 0.001f;

    int32_t delta_fr = curr_fr - last_fr;
    float vel_fr = delta_fr * METERS_PER_TICK / 0.001f;

#if MIDDLE_MOTORS_INSTALLED
    int32_t delta_ml = curr_ml - last_ml;
    float vel_ml = delta_ml * METERS_PER_TICK / 0.001f;

    int32_t delta_mr = curr_mr - last_mr;
    float vel_mr = delta_mr * METERS_PER_TICK / 0.001f;
#endif

    //compute call
    motor_fl->compute(0.0f, vel_fl);
    motor_fr->compute(0.0f, vel_fr);
#if MIDDLE_MOTORS_INSTALLED
    motor_ml->compute(0.0f, vel_ml);
    motor_mr->compute(0.0f, vel_mr);
#endif

    last_fl = curr_fl;
    last_fr = curr_fr;
#if MIDDLE_MOTORS_INSTALLED
    last_ml = curr_ml;
    last_mr = curr_mr;
#endif

    vTaskDelay(pdMS_TO_TICKS(1));
    
  }
}


void imu_init() {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = IMU_SDA;
  conf.scl_io_num = IMU_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;

  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);

  //wakeup
  uint8_t wake_cmd[] = {MPU6050_PWR_MGMT_1, 0x00};
  i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, wake_cmd, sizeof(wake_cmd), pdMS_TO_TICKS(100));
  
}

void gps_init() {
  uart_config_t uart_config = {};
  uart_config.baud_rate = GPS_BAUD_RATE;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity    = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  uart_param_config(GPS_UART_NUM, &uart_config);
  uart_set_pin(GPS_UART_NUM, GPS_TX, GPS_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0);
}

void microros_task(void *param) {
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);

  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  rmw_uros_options_set_udp_address(DESKTOP_IP, "8888", rmw_options);

  // wait for agent
  while (rmw_uros_ping_agent(100, 3) != RCL_RET_OK) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node,   "robot_node", "", &support);

  rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg, Odometry), "/odom");

  rclc_publisher_init_default(&imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu");

  rclc_publisher_init_default(&gps_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix), "/gps");

  //publish loop
  int64_t last_odom_us = esp_timer_get_time();
#if MIDDLE_MOTORS_INSTALLED
  int32_t last_fl = 0, last_fr = 0, last_ml = 0, last_mr = 0;
#else
  int32_t last_fl = 0, last_fr = 0;
#endif
  
  while (true) {
    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - last_odom_us) / 1e6f;
    last_odom_us = now_us;

    //odom
#if MIDDLE_MOTORS_INSTALLED
    int32_t curr_fl = enc_count_fl, curr_fr = enc_count_fr;
    int32_t curr_ml = enc_count_ml, curr_mr = enc_count_mr;

    // Average both wheels per side (FL+ML, FR+MR) — valid only when both wheels
    // per side are actually present and contributing real ticks.
    float v_left = ((curr_fl - last_fl) + (curr_ml - last_ml)) * METERS_PER_TICK / (2.0f * dt);
    float v_right = ((curr_fr - last_fr) + (curr_mr - last_mr)) * METERS_PER_TICK / (2.0f * dt);

    last_fl = curr_fl; last_fr = curr_fr;
    last_ml = curr_ml; last_mr = curr_mr;
#else
    int32_t curr_fl = enc_count_fl, curr_fr = enc_count_fr;

    // Only FL/FR physically present right now. Do NOT divide by 2 here, that "2" in
    // the MIDDLE_MOTORS_INSTALLED branch above accounts for averaging two wheels per
    // side. With one wheel per side, dividing by 2 would halve your real velocity.
    float v_left = (curr_fl - last_fl) * METERS_PER_TICK / dt;
    float v_right = (curr_fr - last_fr) * METERS_PER_TICK / dt;

    last_fl = curr_fl; last_fr = curr_fr;
#endif

    float v = (v_left + v_right) / 2.0f;
    float omega = (v_right - v_left) / WHEEL_BASE;

    odom_theta += omega * dt;
    odom_x += v * cosf(odom_theta) * dt;
    odom_y += v * sinf(odom_theta) * dt;

    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;

    odom_msg.pose.pose.orientation.z = sinf(odom_theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(odom_theta / 2.0f);
    odom_msg.twist.twist.linear.x = v;
    odom_msg.twist.twist.angular.z = omega;

    rcl_publish(&odom_pub, &odom_msg, NULL);

    //imu
    uint8_t reg = MPU6050_ACCEL_OUT;
    uint8_t imu_buf[14];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, &reg, 1, imu_buf, 14, pdMS_TO_TICKS(100));

    if (err == ESP_OK) {
      int16_t ax = (imu_buf[0] << 8) | imu_buf[1];
      int16_t ay = (imu_buf[2] << 8) | imu_buf[3];
      int16_t az = (imu_buf[4] << 8) | imu_buf[5];
      int16_t gx = (imu_buf[8] << 8) | imu_buf[9];
      int16_t gy = (imu_buf[10] << 8) | imu_buf[11];
      int16_t gz = (imu_buf[12] << 8) | imu_buf[13];

      imu_msg.linear_acceleration.x = ax / MPU6050_ACCEL_SCALE * GRAVITY_MS2;
      imu_msg.linear_acceleration.y = ay / MPU6050_ACCEL_SCALE * GRAVITY_MS2;
      imu_msg.linear_acceleration.z = az / MPU6050_ACCEL_SCALE * GRAVITY_MS2;

      imu_msg.angular_velocity.x = gx / MPU6050_GYRO_SCALE * DEG_TO_RAD;
      imu_msg.angular_velocity.y = gy / MPU6050_GYRO_SCALE * DEG_TO_RAD;
      imu_msg.angular_velocity.z = gz / MPU6050_GYRO_SCALE * DEG_TO_RAD;

      rcl_publish(&imu_pub, &imu_msg, NULL);
    }

    // GPS read
  uint8_t gps_buf[GPS_BUF_SIZE];
  int len = uart_read_bytes(GPS_UART_NUM, gps_buf, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(10));
  if (len > 0) {
    gps_buf[len] = '\0';
    char *line = strstr((char*)gps_buf, "$GNGGA");
    if (!line) line = strstr((char*)gps_buf, "$GPGGA");
    if (line) {
        double lat, lon, alt;
        int fix, sats;
        char lat_dir, lon_dir;
        // parse: time, lat, lat_dir, lon, lon_dir, fix, sats, hdop, alt
        int parsed = sscanf(line, "$G%*2c,%*f,%lf,%c,%lf,%c,%d,%d,%*f,%lf",
                            &lat, &lat_dir, &lon, &lon_dir, &fix, &sats, &alt);
        if (parsed == 7 && fix > 0) {
            // convert ddmm.mmmm to decimal degrees
            double lat_deg = (int)(lat / 100) + fmod(lat, 100.0) / 60.0;
            double lon_deg = (int)(lon / 100) + fmod(lon, 100.0) / 60.0;
            if (lat_dir == 'S') lat_deg = -lat_deg;
            if (lon_dir == 'W') lon_deg = -lon_deg;

            gps_msg.latitude  = lat_deg;
            gps_msg.longitude = lon_deg;
            gps_msg.altitude  = alt;
            gps_msg.status.status = 0; // STATUS_FIX
            rcl_publish(&gps_pub, &gps_msg, NULL);
        }
    }
  }

    vTaskDelay(pdMS_TO_TICKS(20));
  }


}

extern "C" void app_main(void) {
  Motor::init_timer();

  gpio_install_isr_service(0);

  gpio_config_t enc_config_fl = {};
  enc_config_fl.pin_bit_mask = (1ULL << ENC_FL);
  enc_config_fl.mode = GPIO_MODE_INPUT;
  enc_config_fl.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_fl.pull_down_en = GPIO_PULLDOWN_DISABLE;
  enc_config_fl.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&enc_config_fl);

  gpio_config_t enc_config_fr = {};
  enc_config_fr.pin_bit_mask = (1ULL << ENC_FR);
  enc_config_fr.mode = GPIO_MODE_INPUT;
  enc_config_fr.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_fr.pull_down_en = GPIO_PULLDOWN_DISABLE;
  enc_config_fr.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&enc_config_fr);

#if MIDDLE_MOTORS_INSTALLED
  gpio_config_t enc_config_ml = {};
  enc_config_ml.pin_bit_mask = (1ULL << ENC_ML);
  enc_config_ml.mode = GPIO_MODE_INPUT;
  enc_config_ml.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_ml.pull_down_en = GPIO_PULLDOWN_DISABLE;
  enc_config_ml.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&enc_config_ml);

  gpio_config_t enc_config_mr = {};
  enc_config_mr.pin_bit_mask = (1ULL << ENC_MR);
  enc_config_mr.mode = GPIO_MODE_INPUT;
  enc_config_mr.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_mr.pull_down_en = GPIO_PULLDOWN_DISABLE;
  enc_config_mr.intr_type = GPIO_INTR_ANYEDGE;
  gpio_config(&enc_config_mr);
#else

  gpio_config_t enc_config_ml_unused = {};
  enc_config_ml_unused.pin_bit_mask = (1ULL << ENC_ML);
  enc_config_ml_unused.mode = GPIO_MODE_INPUT;
  enc_config_ml_unused.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_ml_unused.pull_down_en = GPIO_PULLDOWN_ENABLE;
  enc_config_ml_unused.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&enc_config_ml_unused);

  gpio_config_t enc_config_mr_unused = {};
  enc_config_mr_unused.pin_bit_mask = (1ULL << ENC_MR);
  enc_config_mr_unused.mode = GPIO_MODE_INPUT;
  enc_config_mr_unused.pull_up_en = GPIO_PULLUP_DISABLE;
  enc_config_mr_unused.pull_down_en = GPIO_PULLDOWN_ENABLE;
  enc_config_mr_unused.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&enc_config_mr_unused);
#endif

  gpio_isr_handler_add((gpio_num_t)ENC_FL, isr_enc_fl, nullptr);
  gpio_isr_handler_add((gpio_num_t)ENC_FR, isr_enc_fr, nullptr);
#if MIDDLE_MOTORS_INSTALLED
  gpio_isr_handler_add((gpio_num_t)ENC_ML, isr_enc_ml, nullptr);
  gpio_isr_handler_add((gpio_num_t)ENC_MR, isr_enc_mr, nullptr);
#endif

  motor_fl = new Motor(MOTOR_FL_A, MOTOR_FL_B, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
  motor_fr = new Motor(MOTOR_FR_A, MOTOR_FR_B, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
#if MIDDLE_MOTORS_INSTALLED
  motor_ml = new Motor(MOTOR_ML_A, MOTOR_ML_B, LEDC_CHANNEL_4, LEDC_CHANNEL_5);
  motor_mr = new Motor(MOTOR_MR_A, MOTOR_MR_B, LEDC_CHANNEL_6, LEDC_CHANNEL_7);
#endif

  motor_fl->setPID(50.0f, 5.0f, 0.5f);
  motor_fr->setPID(50.0f, 5.0f, 0.5f);
#if MIDDLE_MOTORS_INSTALLED
  motor_ml->setPID(50.0f, 5.0f, 0.5f);
  motor_mr->setPID(50.0f, 5.0f, 0.5f);
#endif


  wifi_init_sta();
  imu_init();
  gps_init();

  xTaskCreate(microros_task, "microros_task", 8192, NULL, 4, NULL);
  xTaskCreate(control_task, "control_task",
              4096, (void*)"Hello ESP32",
              5, NULL);
  


  }