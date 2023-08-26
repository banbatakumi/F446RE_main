#ifndef MBED_dribbler_H
#define MBED_dribbler_H

#include "mbed.h"

#define MOTOR_FREQUENCY 60000   // モーターのPWM周波数

class dribbler {
     public:
      dribbler(PinName f_motor_1_, PinName f_motor_2_, PinName b_motor_1_, PinName b_motor_2_);

      void set_pwm();
      
      void f_test();
      void f_hold(uint8_t speed = 50);
      void f_kick(uint8_t speed = 100);
      void f_stop();

      void b_test();
      void b_hold(uint8_t speed = 50);
      void b_kick(uint8_t speed = 100);
      void b_stop();

     private:
      PwmOut f_motor_1;
      PwmOut f_motor_2;
      PwmOut b_motor_1;
      PwmOut b_motor_2;
};

#endif