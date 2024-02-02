#include "dribbler.h"

#include "mbed.h"

Dribbler::Dribbler(PinName motor_a_, PinName motor_b_) : motor_a(motor_a_), motor_b(motor_b_) {
      motor_a = 0;
      motor_b = 0;

      motorAve.SetLength(25);
}

void Dribbler::SetPwmPeriod(uint16_t pwm_period_) {
      motor_a.period_us(pwm_period_);
}

void Dribbler::Hold(uint8_t speed_) {
      int16_t speed = speed_;
      motorAve.Compute(&speed);
      if (speed > 100) speed = 100;
      motor_a = speed / 100.000;
      motor_b = 0;
}

void Dribbler::Kick() {
      motor_a = 0;
      motor_b = 1;
}

void Dribbler::Stop() {
      motor_a = 0;
      motor_b = 0;
      motorAve.Reset();
}

void Dribbler::Brake() {
      motor_a = 1;
      motor_b = 1;
      motorAve.Reset();
}