#ifndef MBED_IMU_H
#define MBED_IMU_H

#include "mbed.h"
#include "simplify_deg.h"

class Imu {
     public:
      Imu(PinName tx_, PinName rx_);

      int16_t GetDir();

     private:
      RawSerial serial;

      void Receive();

      int16_t dir;
      int16_t dir_correction;

      bool is_dir_correction = 0;
};

#endif