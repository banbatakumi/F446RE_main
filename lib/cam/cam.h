#ifndef MBED_CAM_H
#define MBED_CAM_H

#include "mbed.h"
#include "moving_ave.h"
#include "my_math.h"
#include "simplify_deg.h"

#define PI 3.1415926535  // 円周率
#define SAMPLE_CYCLE 50ms
#define MOVING_AVE_NUM 5
class Cam {
     public:
      Cam(PinName tx_, PinName rx_, float* own_dir_, uint32_t serial_baud_);

      int16_t GetBallX();
      int16_t GetBallY();
      int16_t GetOwnX();
      int16_t GetOwnY();
      int16_t GetBallVelocityX();
      int16_t GetBallVelocityY();
      int16_t GetBlueGoalX();
      int16_t GetBlueGoalY();
      int16_t GetCenterDir();
      int16_t GetCenterDis();
      void ComputeVelocity();

      int16_t ball_dir;
      uint8_t ball_dis;

      int16_t ball_velocity_x;
      int16_t ball_velocity_y;

      int16_t yellow_goal_dir;
      uint8_t yellow_goal_size;
      int16_t blue_goal_dir;
      uint8_t blue_goal_size;

      int16_t ops_goal_dir;
      uint8_t ops_goal_size;
      int16_t own_goal_dir;
      uint8_t own_goal_size;

      int8_t court_own_x, court_own_y;

      uint8_t front_proximity;
      uint8_t right_proximity;
      uint8_t back_proximity;
      uint8_t left_proximity;

      bool is_goal_front;

     private:
      UnbufferedSerial serial;

      MovingAve ballVelocityXAve;
      MovingAve ballVelocityYAve;

      void Receive();

      float* own_dir;
      int16_t* mode;

      bool is_front_goal_yellow;
      uint8_t goal_front_count;

      Ticker readTicker;
};

#endif