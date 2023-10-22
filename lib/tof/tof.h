#ifndef MBED_TOF_H
#define MBED_TOF_H

#include "mbed.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

#define TOF_QTY 16
class Tof {
     public:
      Tof();
      int16_t FrontSafeDir(uint16_t dis_limit_ = 256);
      int16_t SafeDir(uint16_t dis_limit_ = 256);
      uint8_t FrontMinSensor();
      uint8_t MinSensor();

      uint8_t val[TOF_QTY];

     private:
      float unit_vector_x[TOF_QTY];
      float unit_vector_y[TOF_QTY];
};

#endif