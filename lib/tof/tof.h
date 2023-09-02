#ifndef MBED_TOF_H
#define MBED_TOF_H

#include "mbed.h"

#define PI 3.1415926535   // 円周率

#define TOF_QTY 16

class tof {
     public:
      tof();
      int16_t safe_angle();
      uint8_t min_sensor();

      uint8_t val[TOF_QTY];

     private:
      float unit_vector_x[TOF_QTY];
      float unit_vector_y[TOF_QTY];
};

#endif