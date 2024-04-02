#ifndef MBED_IMU_H
#define MBED_IMU_H

#include "mbed.h"
#include "simplify_deg.h"

class Imu {
     public:
      Imu(PinName tx_, PinName rx_, uint32_t serial_baud_);

      float GetYaw();
      float GetPitch();
      float GetRoll();
      void YawSetZero();

     private:
      UnbufferedSerial serial;
      void Receive();

      float yaw;
      float pitch;
      float roll;
      float yaw_correction;
};

#endif