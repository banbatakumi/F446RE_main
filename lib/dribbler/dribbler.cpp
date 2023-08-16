#include "dribbler.h"

#include "mbed.h"

dribbler::dribbler(PinName f_motor_1_, PinName f_motor_2_, PinName b_motor_1_, PinName b_motor_2_) : f_motor_1(f_motor_1_), f_motor_2(f_motor_2_), b_motor_1(b_motor_1_), b_motor_2(b_motor_2_) {
      f_motor_1 = 0;
      f_motor_2 = 0;
      b_motor_1 = 0;
      b_motor_2 = 0;
}

void dribbler::f_test() {
      for (uint8_t count = 0; count < 10; count++) {
            f_motor_1 = count / 10.000;
            f_motor_2 = 0;
            wait_ms(100);
      }
      wait_ms(1000);
      f_motor_1 = 0;
      f_motor_2 = 0;
      wait_ms(100);
      for (uint8_t count = 10; count > 0; count--) {
            f_motor_1 = 0;
            f_motor_2 = count / 10.000;
            wait_ms(100);
      }
}

void dribbler::f_hold(uint8_t speed) {
      f_motor_1 = speed / 100.000;
      f_motor_2 = 0;
}

void dribbler::f_kick(uint8_t speed) {
      f_motor_1 = 0;
      f_motor_2 = speed / 100.000;
}

void dribbler::f_stop() {
      f_motor_1 = 0;
      f_motor_2 = 0;
}

void dribbler::b_test() {
      for (uint8_t count = 0; count < 10; count++) {
            f_motor_1 = count / 10.000;
            f_motor_2 = 0;
            wait_ms(100);
      }
      wait_ms(1000);
      f_motor_1 = 0;
      f_motor_2 = 0;
      wait_ms(100);
      for (uint8_t count = 10; count > 0; count--) {
            f_motor_1 = 0;
            f_motor_2 = count / 10.000;
            wait_ms(100);
      }
}

void dribbler::b_hold(uint8_t speed) {
      f_motor_1 = speed / 100.000;
      f_motor_2 = 0;
}

void dribbler::b_kick(uint8_t speed) {
      f_motor_1 = 0;
      f_motor_2 = speed / 100.000;
}

void dribbler::b_stop() {
      f_motor_1 = 0;
      f_motor_2 = 0;
}