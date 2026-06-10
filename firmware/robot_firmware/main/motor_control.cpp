#include "motor_control.hpp"
#include <algorithm>
#include <cstdint>

Motor::Motor(int pin_a, int pin_b, ledc_channel_t channel_a, ledc_channel_t channel_b) {
  pin_a_ = pin_a;
  pin_b_ = pin_b;
  channel_a_ = channel_a;
  channel_b_ = channel_b;
  current_speed_ = 0;

  ledc_channel_config_t channel_a_config = {};
  channel_a_config.gpio_num = pin_a_;
  channel_a_config.speed_mode = LEDC_LOW_SPEED_MODE;
  channel_a_config.channel = channel_a_;
  channel_a_config.timer_sel = LEDC_TIMER_0;
  channel_a_config.duty = 0;
  channel_a_config.hpoint = 0;
  ledc_channel_config(&channel_a_config);

  ledc_channel_config_t channel_b_config = {};
  channel_b_config.gpio_num = pin_b_;
  channel_b_config.speed_mode = LEDC_LOW_SPEED_MODE;
  channel_b_config.channel = channel_b_;
  channel_b_config.timer_sel = LEDC_TIMER_0;
  channel_b_config.duty = 0;
  channel_b_config.hpoint = 0;
  ledc_channel_config(&channel_b_config);
}

  

void Motor::setSpeed(int speed) {
  speed = std::clamp(speed, -100, 100);
  current_speed_ = speed;
  uint32_t duty = (uint32_t)(std::abs(speed) * 1023 / 100);

  if (speed > 0) {
    //forward — A gets PWM B gets 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_a_, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_a_);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_b_, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_b_);
  } else if (speed < 0) {
    //backward — A gets 0 B gets PWM
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_a_, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_a_);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_b_, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_b_);
  } else {
    //stopped — both get 0
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_a_, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_a_);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_b_, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_b_);
  }
}

void Motor::brake() {
  setSpeed(0);
}

void Motor::stop() {
  setSpeed(0);
}

int Motor::getSpeed() {
  return current_speed_;
}

void Motor::init_timer() {
  ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
    timer_config.duty_resolution = LEDC_TIMER_10_BIT;
    timer_config.timer_num = LEDC_TIMER_0;
    timer_config.freq_hz = 20000;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_config);

}