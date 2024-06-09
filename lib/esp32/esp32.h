#ifndef MBED_ULTRASONIC_H
#define MBED_ULTRASONIC_H

#include "mbed.h"

#define PI 3.1415926535  // 円周率
#define SENSOR_QTY 4

class Esp32 {
     public:
      Esp32(PinName tx_, PinName rx_, float* own_dir_, uint8_t* mode_, uint32_t serial_baud_);

      void Receive();
      void OnIrLed();
      void OffIrLed();

      uint8_t val[SENSOR_QTY];
      int16_t ir_dir;
      int16_t ir_dis;
      bool is_ally_moving;
      bool is_ally_defense;
      bool is_ally_catch_ball;
      bool can_ally_get_pass;
      bool is_connect_to_ally;
      bool is_moving;
      bool is_defense;
      bool is_catch_ball;
      bool can_get_pass;

     private:
      UnbufferedSerial serial;

      uint8_t on_ir_led;
      float* own_dir;
      uint8_t* mode;

      int16_t top_yaw;
};

#endif