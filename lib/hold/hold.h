#ifndef MBED_HOLD_H
#define MBED_HOLD_H

#include "mbed.h"

#define RC 0.5
#define SET_TH_NUM 500
class Hold {
     public:
      Hold(PinName sensor_);
      void Read();
      uint8_t GetVal();
      bool IsHold();
      void SetTh();

     private:
      AnalogIn sensor;

      uint8_t val;
      uint8_t rc_val;

      uint16_t th;
};

#endif