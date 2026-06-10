#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "pins_esp1.hpp"
#include "motor_control.hpp"
#include <cstddef>


Motor* motor_fl = nullptr;
Motor* motor_fr = nullptr;
Motor* motor_ml = nullptr;
Motor* motor_mr = nullptr;

void control_task(void *param) {
  while(1) {
    printf("control loop running\n");
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main(void) {
  Motor::init_timer();
  motor_fl = new Motor(MOTOR_FL_A, MOTOR_FL_B, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
  motor_fr = new Motor(MOTOR_FR_A, MOTOR_FR_B, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
  motor_ml = new Motor(MOTOR_ML_A, MOTOR_ML_B, LEDC_CHANNEL_4, LEDC_CHANNEL_5);
  motor_mr = new Motor(MOTOR_MR_A, MOTOR_MR_B, LEDC_CHANNEL_6, LEDC_CHANNEL_7);
  xTaskCreate(control_task, "control_task",
              2048, (void*)"Hello ESP32",
              5, NULL);

  
  }
