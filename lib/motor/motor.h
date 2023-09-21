#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

#define PWM_FREQUENCY 30000   // モーターのPWM周波数

#define MIN_BRAKE 5   // モーターの最小値ブレーキ
#define POWER_LIMIT 90   // モーターの最大パワー

#define KP 1.000   // 姿勢制御比例ゲイン
#define KD 500.000   // 姿制御微分ゲイン
#define D_PERIOD 0.01

#define MOVING_AVG_CNT_NUM 30   // 移動平均フィルタの回数

#define MOTOR_QTY 4

#define ADD_SPEED_PERIOD 0.01
#define ADD_SPEED_K 1
class Motor {
     public:
      Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_);
      void Run(int16_t moving_dir, uint8_t moving_speed = 0, int16_t robot_angle = 0, uint8_t robot_angle_mode = 0, uint8_t pd_limit = POWER_LIMIT);
      void SetPwm();
      void Brake(uint16_t brake_time = 0);
      void Free();

#define FRONT 1
#define RIGHT 2
#define BACK 3
#define LEFT 4

      int16_t yaw;
      uint8_t encoder_val;

     private:
      PwmOut motor_1_a;
      PwmOut motor_1_b;
      PwmOut motor_2_a;
      PwmOut motor_2_b;
      PwmOut motor_3_a;
      PwmOut motor_3_b;
      PwmOut motor_4_a;
      PwmOut motor_4_b;

      int16_t pre_p, pd, pd_limit;
      float p, d;

      uint8_t moving_avg_cnt;

      Timer d_timer;
      Timer add_speed_timer;
};

#endif