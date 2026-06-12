#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pins_esp2.hpp"
#include "motor_control.hpp"
#include <cstddef>
#include "driver/gpio.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "wifi_config.hpp"
#include <cstring>

#include "lwip/sockets.h"
#include "netinet/in.h"
#include "arpa/inet.h"
#include "driver/uart.h"

Motor* motor_rl = nullptr;
Motor* motor_rr = nullptr;


volatile int32_t enc_count_rl = 0;
volatile int32_t enc_count_rr = 0;

void udp_to_uart_task(void *param) {
  uint8_t buf[1024];

  int sock = socket(AF_INET, SOCK_DGRAM, 0);

  struct sockaddr_in dest;
  dest.sin_family = AF_INET;
  dest.sin_port = htons(LIDAR_CMD_PORT);
  dest.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (struct sockaddr*)&dest, sizeof(dest));
  while(1) {
      int len = recvfrom(sock, buf, sizeof(buf), 0, NULL, NULL);
      uart_write_bytes(UART_NUM_2, buf, len);
  }
}

void uart_to_udp_task(void *param) {
  uint8_t buf[1024];

  int sock = socket(AF_INET, SOCK_DGRAM, 0);

  struct sockaddr_in dest;
  dest.sin_family = AF_INET;
  dest.sin_port = htons(LIDAR_BRIDGE_PORT);
  dest.sin_addr.s_addr = inet_addr(DESKTOP_IP);

  while(1) {
    int len = uart_read_bytes(UART_NUM_2, buf, sizeof(buf), pdMS_TO_TICKS(10));
    if (len > 0) {
      sendto(sock, buf, len, 0, (struct sockaddr*)&dest, sizeof(dest));
    }
  }
}

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


  uart_config_t uart_config = {};
  uart_config.baud_rate =  460800;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, RPLIDAR_TX, RPLIDAR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM_2, 2048, 0, 0, NULL, 0);

  wifi_init_sta();


  xTaskCreate(control_task, "control_task",
              4096, (void*)"Hello ESP32",
              5, NULL);

  xTaskCreate(uart_to_udp_task, "uart_to_udp", 4096, NULL, 4, NULL);

  xTaskCreate(udp_to_uart_task, "udp_to_uart", 4096, NULL, 4, NULL);
  }

