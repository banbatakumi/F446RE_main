#ifndef MBED_HOLD_H
#define MBED_HOLD_H

#include "mbed.h"

#define RC 0.8

class hold {
     public:
      hold(PinName front_, PinName back_);
      void read();
      uint8_t front_read();
      uint8_t back_read();

     private:
      AnalogIn front;
      AnalogIn back;

      uint8_t front_value, front_rc_value;
      uint8_t back_value, back_rc_value;
};

#endif