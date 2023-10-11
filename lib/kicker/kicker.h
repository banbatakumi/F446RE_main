#ifndef MBED_KICKER_H
#define MBED_KICKER_H

#include "mbed.h"

class Kicker {
     public:
      Kicker(PinName sig_1_, PinName sig_2_);

      void SetKickTime(uint16_t kick_time_ = 100);

      void Kick();

     private:
      DigitalOut sig_1;
      DigitalOut sig_2;

      uint32_t kick_time;
};

#endif