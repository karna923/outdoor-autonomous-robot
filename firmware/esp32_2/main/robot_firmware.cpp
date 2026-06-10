#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pins_esp2.hpp"
#include "motor_control.hpp"
#include <cstddef>
#include "driver/gpio.h"

Motor* motor_rl = nullptr;
Motor* motor_rr = nullptr;


volatile int32_t enc_count_rl = 0;
volatile int32_t enc_count_rr = 0;



void IRAM_ATTR isr_enc_rl(void* arg) {
    enc_count_rl+=1;
}
  
void IRAM_ATTR isr_enc_rr(void* arg) {
    enc_count_rr+=1;
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


  xTaskCreate(control_task, "control_task",
              4096, (void*)"Hello ESP32",
              5, NULL);


  }

