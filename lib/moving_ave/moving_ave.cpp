#include "moving_ave.h"

#include "mbed.h"

MovingAve::MovingAve() {
}

void MovingAve::SetLength(uint8_t length) {
      this->length = length;
}

void MovingAve::Ave(int16_t *val) {
      if (cnt == length) cnt = 0;
      data[cnt] = *val;
      *val = 0;
      for (uint8_t i = 0; i < length; i++) {
            *val += data[i];
      }
      *val /= length;
      cnt++;
}