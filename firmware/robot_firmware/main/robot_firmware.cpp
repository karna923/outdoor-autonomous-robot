#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.hpp"

void control_task(void *param) {
  while(1) {
    printf("control loop running\n");
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main(void) {
  xTaskCreate(control_task, "control_task",
              2048, (void*)"Hello ESP32",
              5, NULL);
  }
