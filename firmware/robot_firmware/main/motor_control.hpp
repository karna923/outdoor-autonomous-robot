#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "driver/ledc.h"
#include "pins.hpp"
class Motor {
  private:
    int pin_a_;
    int pin_b_;
    ledc_channel_t channel_a_;
    ledc_channel_t channel_b_;
    int current_speed_;

  public:
    Motor(int pin_a, int pin_b, ledc_channel_t channel_a, ledc_channel_t channel_b);

    void setSpeed(int speed);
    void brake();
    void stop();
    int getSpeed();
};

#endif