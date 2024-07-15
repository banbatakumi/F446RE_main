#include "dribbler.h"

#include "mbed.h"

Dribbler::Dribbler(PinName motor_a_, PinName motor_b_) : motor_a(motor_a_), motor_b(motor_b_) {
      motor_a = 0;
      motor_b = 0;
}

void Dribbler::SetPwmPeriod(uint16_t pwm_period_) {
      motor_a.period_us(pwm_period_);
}

void Dribbler::Hold(uint8_t speed_) {
      this->speed = speed_;
      if (speed > 100) speed = 100;
      rc_speed = (1 - RC) * speed + RC * rc_speed;
      motor_a = rc_speed * 0.01f;
      motor_b = 0;
}

void Dribbler::Kick() {
      motor_a = 0;
      motor_b = 1;
}

void Dribbler::Stop() {
      motor_a = 0;
      motor_b = 0;
}

void Dribbler::Brake(uint16_t brake_time_) {
      motor_a = 1;
      motor_b = 1;
      wait_us(brake_time_ * 1000);
}