#ifndef MBED_CAM_H
#define MBED_CAM_H

#include "mbed.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

class Cam {
     public:
      Cam(PinName tx_, PinName rx_, int16_t* own_dir_, uint8_t* mode_);

      int16_t GetBallDir();
      uint8_t GetBallDis();
      int16_t GetGoalDir(uint8_t which_);
      uint8_t GetGoalSize(uint8_t which_);

#define FRONT 0
#define BACK 1
#define YELLOW 2
#define BLUE 3

     private:
      RawSerial serial;
      void Receive();

      int16_t* own_dir;
      uint8_t* mode;

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

      int16_t own_x;
      int16_t own_y;
};

#endif