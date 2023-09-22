#ifndef MBED_MOVING_AVE_H
#define MBED_MOVING_AVE_H

#include "mbed.h"

class MovingAve {
     public:
      MovingAve();
      void Ave(int16_t *val);
      void SetLength(uint8_t length);

     private:
      int16_t data[50];
      uint8_t cnt;
      uint8_t length;
};

#endif