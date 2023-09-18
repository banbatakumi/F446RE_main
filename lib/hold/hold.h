#ifndef MBED_HOLD_H
#define MBED_HOLD_H

#include "mbed.h"

#define RC 0.5
#define CATCH_TH 150

class hold {
     public:
      hold(PinName front_sensor_, PinName back_sensor_);
      void read();
      uint8_t get_front_val();
      uint8_t get_back_val();
      bool is_front();
      bool is_back();

     private:
      AnalogIn front_sensor;
      AnalogIn back_sensor;

      uint8_t front_val;
      uint8_t back_val;
      uint8_t front_rc_val;
      uint8_t back_rc_val;
};

#endif