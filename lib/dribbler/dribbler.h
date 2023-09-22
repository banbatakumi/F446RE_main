#ifndef MBED_DRIBBLER_H
#define MBED_DRIBBLER_H

#include "mbed.h"

#define MOTOR_FREQUENCY 30000   // モーターのPWM周波数

class Dribbler {
     public:
      Dribbler(PinName motor_a_, PinName motor_b_);

      void SetPwmPeriod(uint16_t pwm_period = 30000);

      void Hold(uint8_t speed = 50);
      void Kick();
      void Stop();

     private:
      PwmOut motor_a;
      DigitalOut motor_b;
};

#endif