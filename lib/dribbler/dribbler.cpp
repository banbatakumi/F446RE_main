#include "dribbler.h"

#include "mbed.h"

Dribbler::Dribbler(PinName motor_a_, PinName motor_b_) : motor_a(motor_a_), motor_b(motor_b_) {
      motor_a = 0;
      motor_b = 0;
}

void Dribbler::SetPwm() {
      motor_a.period_us(MOTOR_FREQUENCY);
}

void Dribbler::Hold(uint8_t speed) {
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
}