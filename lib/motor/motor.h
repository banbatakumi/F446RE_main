#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"

#define PI 3.1415926535   // 円周率

#define MOTOR_FREQUENCY 30000   // モーターのPWM周波数

#define MIN_BRAKE 5   // モーターの最小値ブレーキ
#define POWER_LIMIT 90   // モーターの最大パワー

#define KP 1.500   // 姿勢制御比例ゲイン
#define KD 600.000   // 姿制御微分ゲイン
#define D_PERIODO 0.01

#define ENCODER_KP 0.100   // 姿勢制御比例ゲイン
#define ENCODER_KD 10.000   // 姿制御微分ゲイン

#define MOVING_AVG_CNT_NUM 10   // 移動平均フィルタの回数

#define MOTOR_QTY 4

#define ENCODER_CYCLE 0.01

class motor {
     public:
      motor(PinName motor_1_1_, PinName motor_1_2_, PinName motor_2_1_, PinName motor_2_2_, PinName motor_3_1_, PinName motor_3_2_, PinName motor_4_1_, PinName motor_4_2_);
      void run(int16_t moving_dir, uint8_t moving_speed, bool is_encoder = 0, int16_t robot_angle = 0, uint8_t robot_angle_mode = 0, uint8_t pd_limit = POWER_LIMIT);
      void set_pwm();
      void brake(uint16_t brake_time = 0);
      void free();

#define FRONT 1
#define RIGHT 2
#define BACK 3
#define LEFT 4

#define T 1
#define F 0

      int16_t motor_1();
      int16_t motor_2();
      int16_t motor_3();
      int16_t motor_4();

      int16_t yaw;
      uint8_t rotation_num_avg;

     private:
      PwmOut motor_1_1;
      PwmOut motor_1_2;
      PwmOut motor_2_1;
      PwmOut motor_2_2;
      PwmOut motor_3_1;
      PwmOut motor_3_2;
      PwmOut motor_4_1;
      PwmOut motor_4_2;

      int16_t power[4], tmp_power[4][MOVING_AVG_CNT_NUM];
      int16_t pre_p, pd, pd_limit;
      float p, d;

      uint8_t moving_avg_cnt;

      int16_t encoder_pre_p, encoder_pd;
      float encoder_p, encoder_d;

      Timer d_timer;
      Timer encoder_cycle_timer;
};

#endif