#ifndef MBED_IMU_H
#define MBED_IMU_H

#include "mbed.h"
#include "simplify_deg.h"

class Imu {
     public:
      Imu(PinName tx_, PinName rx_, uint32_t serial_baud_);

      int16_t GetDir();
      void SetZero();

     private:
      RawSerial serial;
      void Receive();

      int16_t dir;
      int16_t dir_correction;
};

#endif