#ifndef MBED_KICKER_H
#define MBED_KICKER_H

#include "mbed.h"

#define readms(timer_name_) chrono::duration_cast<chrono::milliseconds>((timer_name_).elapsed_time()).count()

#define KICK_TIME 200ms
#define DEAD_TIME 10ms
#define CHARGE_PWM_PERIOD 100us
class Kicker {
     public:
      Kicker(PinName capaciter_, PinName solenoid_);

      void Kick();

     private:
      DigitalOut capaciter;
      DigitalOut solenoid;

      void FlipOn();
      void FlipOff();
      void ChargeStart();
      void Charge();
      void FirstCharge();

      bool enable_kick;
      uint8_t charge_count;

      Timer chargeTimer;
      Timeout deadTimeout;
      Timeout flipTimeout;
      Ticker charge;
};

#endif