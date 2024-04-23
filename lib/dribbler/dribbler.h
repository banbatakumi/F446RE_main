#ifndef MBED_DRIBBLER_H
#define MBED_DRIBBLER_H

#include "mbed.h"
#include "moving_ave.h"
#include "FastPWM.h"
class Dribbler {
     public:
      Dribbler(PinName motor_a_, PinName motor_b_);

      void SetPwmPeriod(uint16_t pwm_period_);

      void Hold(uint8_t speed_ = 50);
      void Kick();
      void Stop();
      void Brake(uint16_t brake_time_ = 0);

     private:
      MovingAve motorAve;
      FastPWM motor_a;
      DigitalOut motor_b;
};

#endif