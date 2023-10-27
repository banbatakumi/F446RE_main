#ifndef MBED_LINE_H
#define MBED_LINE_H

#include "mbed.h"
#include "simplify_deg.h"

#define PI 3.1415926535   // 円周率

class Line {
     public:
      Line(PinName tx_, PinName rx_, uint8_t* mode_);

      uint8_t EncoderVal(uint8_t which_sensor_);
      bool IsLeft();
      bool IsRight();
      uint8_t WhiteNum();
      int16_t Vector();

     private:
      RawSerial serial;
      void Receive();

      uint8_t* mode;

      uint8_t encoder_val[4];
      uint8_t is_line_left;
      uint8_t is_line_right;
      uint8_t line_white_num;
      int16_t line_vector;
};

#endif