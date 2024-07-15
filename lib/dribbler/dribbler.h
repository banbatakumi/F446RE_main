#ifndef MBED_DRIBBLER_H
#define MBED_DRIBBLER_H

#include "FastPWM.h"
#include "mbed.h"

#define RC 0.99
class Dribbler {
     public:
      Dribbler(PinName motor_a_, PinName motor_b_);

      void SetPwmPeriod(uint16_t pwm_period_);

      void Hold(uint8_t speed_ = 50);
      void Kick();
      void Stop();
      void Brake(uint16_t brake_time_ = 0);

     private:
      FastPWM motor_a;
      DigitalOut motor_b;
      float speed, rc_speed;
};

#endif