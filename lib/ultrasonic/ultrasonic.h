#ifndef MBED_ULTRASONIC_H
#define MBED_ULTRASONIC_H

#include "mbed.h"

#define PI 3.1415926535  // 円周率
#define SENSOR_QTY 4

class Ultrasonic {
     public:
      Ultrasonic(PinName tx_, PinName rx_, uint32_t serial_baud_);

      void Receive();

      uint8_t val[SENSOR_QTY];

     private:
      UnbufferedSerial serial;
};

#endif