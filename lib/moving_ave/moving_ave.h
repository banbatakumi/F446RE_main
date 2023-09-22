#ifndef MBED_MOVING_AVE_H
#define MBED_MOVING_AVE_H

#include "mbed.h"

class MovingAve {
     public:
      void Compute(int16_t* input);
      void SetLength(uint8_t length);

     private:
      int16_t data[50];
      uint8_t cnt;
      uint8_t length;
};

#endif