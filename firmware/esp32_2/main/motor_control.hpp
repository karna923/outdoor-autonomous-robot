#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include "driver/ledc.h"
#include <cstdint>
class Motor {
  private:
    int pin_a_;
    int pin_b_;
    ledc_channel_t channel_a_;
    ledc_channel_t channel_b_;
    int current_speed_;
    float kp_;
    float ki_;
    float kd_;

    float integral_;
    float prev_error_;



  public:
    Motor(int pin_a, int pin_b, ledc_channel_t channel_a, ledc_channel_t channel_b);

    void setSpeed(int speed);
    void brake();
    void stop();
    int getSpeed();
    static void init_timer();
    float compute(float setpoint, float actual_velocity);

    void setPID(float kp, float ki, float kd);


};

#endif