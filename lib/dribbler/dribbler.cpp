#include "dribbler.h"

#include "mbed.h"

dribbler::dribbler(PinName f_motor_1_, PinName f_motor_2_, PinName b_motor_1_, PinName b_motor_2_) : f_motor_1(f_motor_1_), f_motor_2(f_motor_2_), b_motor_1(b_motor_1_), b_motor_2(b_motor_2_) {
      f_motor_1 = 0;
      f_motor_2 = 0;
      b_motor_1 = 0;
      b_motor_2 = 0;
}

void dribbler::set_pwm() {
      f_motor_1.period_us(MOTOR_FREQUENCY);
      b_motor_1.period_us(MOTOR_FREQUENCY);
}

void dribbler::f_test() {
      for (uint8_t i = 0; i < 10; i++) {
            f_motor_1 = i / 10.000;
            f_motor_2 = 0;
            wait_us(100000);
      }
      wait_us(1000000);
      f_motor_1 = 0;
      f_motor_2 = 0;
      wait_us(100000);
      f_motor_1 = 0;
      f_motor_2 = 1;
      wait_us(100000);
      f_motor_1 = 0;
      f_motor_2 = 0;
}

void dribbler::f_hold(uint8_t speed) {
      f_motor_1 = speed / 100.000;
      f_motor_2 = 0;
}

void dribbler::f_kick() {
      f_motor_1 = 0;
      f_motor_2 = 1;
}

void dribbler::f_stop() {
      f_motor_1 = 0;
      f_motor_2 = 0;
}

void dribbler::b_test() {
      for (uint8_t i = 0; i < 10; i++) {
            b_motor_1 = i / 10.000;
            b_motor_2 = 0;
            wait_ms(100);
      }
      wait_us(1000000);
      b_motor_1 = 0;
      b_motor_2 = 0;
      wait_us(100000);
      b_motor_1 = 0;
      b_motor_2 = 1;
      wait_us(100000);
      b_motor_1 = 0;
      b_motor_2 = 0;
}

void dribbler::b_hold(uint8_t speed) {
      b_motor_1 = speed / 100.000;
      b_motor_2 = 0;
}

void dribbler::b_kick() {
      b_motor_1 = 0;
      b_motor_2 = 1;
}

void dribbler::b_stop() {
      b_motor_1 = 0;
      b_motor_2 = 0;
}

void dribbler::stop() {
      f_motor_1 = 0;
      f_motor_2 = 0;
      b_motor_1 = 0;
      b_motor_2 = 0;
}