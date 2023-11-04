#ifndef MBED_CAM_H
#define MBED_CAM_H

#include "mbed.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

class Cam {
     public:
      Cam(PinName tx_, PinName rx_, int16_t* own_dir_);

      int16_t ball_dir;
      uint8_t ball_dis;

      int16_t yellow_goal_dir;
      uint8_t yellow_goal_size;
      int16_t blue_goal_dir;
      uint8_t blue_goal_size;

      int16_t front_goal_dir;
      uint8_t front_goal_size;
      int16_t back_goal_dir;
      uint8_t back_goal_size;

      int16_t ball_x, ball_y;

     private:
      RawSerial serial;
      void Receive();

      int16_t* own_dir;
      int16_t* mode;

      bool is_front_goal_yellow;
};

#endif