#ifndef MBED_ULTRASONIC_H
#define MBED_ULTRASONIC_H

#include "mbed.h"

#define PI 3.1415926535  // 円周率
#define SENSOR_QTY 4

class Ultrasonic {
     public:
      Ultrasonic(PinName tx_, PinName rx_, uint32_t serial_baud_);

      void Receive();
      void OnIrLed(bool led_1 = 1, bool led_2 = 1, bool led_3 = 1, bool led_4 = 1);
      void OffIrLed();

      uint8_t val[SENSOR_QTY];
      int16_t ir_dir;
      int16_t ir_dis;

     private:
      UnbufferedSerial serial;

      uint8_t on_ir_led;
};

#endif