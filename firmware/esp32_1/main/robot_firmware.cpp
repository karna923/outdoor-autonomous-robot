#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pins_esp1.hpp"
#include "motor_control.hpp"
#include <cstddef>
#include "driver/gpio.h"

Motor* motor_fl = nullptr;
Motor* motor_fr = nullptr;
Motor* motor_ml = nullptr;
Motor* motor_mr = nullptr;

volatile int32_t enc_count_fl = 0;
volatile int32_t enc_count_fr = 0;
volatile int32_t enc_count_ml = 0;
volatile int32_t enc_count_mr = 0;


void IRAM_ATTR isr_enc_fl(void* arg) {
    enc_count_fl+=1;
}
  
void IRAM_ATTR isr_enc_fr(void* arg) {
    enc_count_fr+=1;
  }

void IRAM_ATTR isr_enc_ml(void* arg) {
    enc_count_ml+=1;
  }

void IRAM_ATTR isr_enc_mr(void* arg) {
    enc_count_mr+=1;
  }
void control_task(void *param) {

  int32_t last_fl = 0, last_fr = 0, last_ml = 0, last_mr = 0;
  while(1) {
    
    int32_t curr_fl = enc_count_fl, curr_fr = enc_count_fr, curr_ml = enc_count_ml, curr_mr = enc_count_mr;

    //delta tick and velocity conversion

    int32_t delta_fl = curr_fl - last_fl;
    float vel_fl = delta_fl * 0.000224f / 0.001f;

    int32_t delta_fr = curr_fr - last_fr;
    float vel_fr = delta_fr * 0.000224f / 0.001f;

    int32_t delta_ml = curr_ml - last_ml;
    float vel_ml = delta_ml * 0.000224f / 0.001f;

    int32_t delta_mr = curr_mr - last_mr;
    float vel_mr = delta_mr * 0.000224f / 0.001f;

    //compute call
    motor_fl->compute(0.0f, vel_fl);
    motor_fr->compute(0.0f, vel_fr);
    motor_ml->compute(0.0f, vel_ml);
    motor_mr->compute(0.0f, vel_mr);

    last_fl = curr_fl;
    last_fr = curr_fr;
    last_ml = curr_ml;
    last_mr = curr_mr;

    vTaskDelay(pdMS_TO_TICKS(1));
    
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

  gpio_isr_handler_add((gpio_num_t)ENC_FL, isr_enc_fl, nullptr);
  gpio_isr_handler_add((gpio_num_t)ENC_FR, isr_enc_fr, nullptr);
  gpio_isr_handler_add((gpio_num_t)ENC_ML, isr_enc_ml, nullptr);
  gpio_isr_handler_add((gpio_num_t)ENC_MR, isr_enc_mr, nullptr);

  motor_fl = new Motor(MOTOR_FL_A, MOTOR_FL_B, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
  motor_fr = new Motor(MOTOR_FR_A, MOTOR_FR_B, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
  motor_ml = new Motor(MOTOR_ML_A, MOTOR_ML_B, LEDC_CHANNEL_4, LEDC_CHANNEL_5);
  motor_mr = new Motor(MOTOR_MR_A, MOTOR_MR_B, LEDC_CHANNEL_6, LEDC_CHANNEL_7);

  motor_fl->setPID(50.0f, 5.0f, 0.5f);
  motor_fr->setPID(50.0f, 5.0f, 0.5f);
  motor_ml->setPID(50.0f, 5.0f, 0.5f);
  motor_mr->setPID(50.0f, 5.0f, 0.5f);


  xTaskCreate(control_task, "control_task",
              4096, (void*)"Hello ESP32",
              5, NULL);


  }

