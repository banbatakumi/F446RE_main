#ifndef MBED_HOLD_H
#define MBED_HOLD_H

#include "mbed.h"

#define RC 0.5
#define CATCH_TH 100

class Hold {
     public:
      Hold(PinName sensor_);
      void Read();
      uint8_t GetVal();
      bool IsHold();

     private:
      AnalogIn sensor;

      uint8_t val;
      uint8_t rc_val;
};

#endif