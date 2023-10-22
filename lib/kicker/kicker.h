#ifndef MBED_KICKER_H
#define MBED_KICKER_H

#include "mbed.h"

class Kicker {
     public:
      Kicker(PinName sig_1_, PinName sig_2_);

      void SetPower(uint8_t power_ = 100);

      void Kick();
      void flipOff();

     private:
      DigitalOut sig_1;
      DigitalOut sig_2;

      uint32_t power;
      bool enable;

      Timer chargeTimer;
      Timeout kickerTimeout;
};

#endif