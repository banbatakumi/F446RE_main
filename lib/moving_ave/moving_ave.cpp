#include "moving_ave.h"

#include "mbed.h"

void MovingAve::SetLength(uint8_t length) {
      this->length = length;
}

void MovingAve::Compute(int16_t* input) {
      if (cnt == length) cnt = 0;
      data[cnt] = *input;
      *input = 0;
      for (uint8_t i = 0; i < length; i++) {
            *input += data[i];
      }
      *input /= length;
      cnt++;
}