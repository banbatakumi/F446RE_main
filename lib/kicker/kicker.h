#ifndef MBED_KICKER_H
#define MBED_KICKER_H

#include "mbed.h"

#define readms(timer_name_) chrono::duration_cast<chrono::milliseconds>((timer_name_).elapsed_time()).count()

#define KICK_TIME 50ms
#define CHARGE_OFF_TIME 15ms
class Kicker {
     public:
      Kicker(PinName sig_1_, PinName sig_2_);

      void Kick();
      void ChargeOff();
      void FlipOff();

     private:
      DigitalOut sig_1;
      DigitalOut sig_2;

      bool enable;

      Timer chargeTimer;
      Timeout ChargeTimeout;
      Timeout FlipTimeout;
};

#endif