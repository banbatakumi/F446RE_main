#ifndef MBED_LINE_H
#define MBED_LINE_H

#include "mbed.h"
#include "simplify_deg.h"

#define PI 3.1415926535   // 円周率

class Line {
     public:
      Line(PinName tx_, PinName rx_);

      uint8_t EncoderVal(uint8_t which_sensor_);
      void LedOn();
      void LedOff();
      uint8_t GetDepth();

      uint8_t is_left;
      uint8_t is_right;
      uint8_t white_qty;
      uint8_t interval;
      int16_t dir;
      int16_t inside_dir;
      uint8_t encoder_val[4];

     private:
      RawSerial serial;
      void Receive();

      bool do_led_on;
};

#endif