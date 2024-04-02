#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "FastPWM.h"
#include "mbed.h"
#include "moving_ave.h"
#include "pid.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535  // 円周率

#define MOTOR_QTY 4
#define MOTOR_0_DEGREE 45
#define MOTOR_1_DEGREE 135
#define MOTOR_2_DEGREE 225
#define MOTOR_3_DEGREE 315

#define BASE_POWER 10
#define ENCODER_GAIN 0.00004
#define MAX_ADD_POWER 75
#define MIN_ADD_POWER 0

#define POWER_MAX_LIMIT 95
#define POWER_MIN_LIMIT 1

#define MOVING_AVE_NUM 25

#define readus(timer_name_) (timer_name_).elapsed_time().count()
class Motor {
     public:
      Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_, float* own_dir_);
      void Run(int16_t moving_dir_ = 0, uint16_t moving_speed_ = 0, int16_t robot_angle_ = 0, uint8_t turning_mode_ = 0, uint8_t turning_limit_ = 100);
      void SetPwmPeriod(uint16_t pwm_period_);
      void SetAttitudeControlPID(float kp_, float ki_, float kd_);
      void Brake(uint16_t brake_time_ = 0);
      void Free();
      int16_t GetPreMovingDir();
      int16_t GetPreMovingSpeed();

#define FRONT 1
#define RIGHT 2
#define BACK 3
#define LEFT 4

      uint8_t encoder_val[MOTOR_QTY];

     private:
      MovingAve motor1Ave;
      MovingAve motor2Ave;
      MovingAve motor3Ave;
      MovingAve motor4Ave;

      PID attitudeControlPID;

      FastPWM motor_1_a;
      FastPWM motor_1_b;
      FastPWM motor_2_a;
      FastPWM motor_2_b;
      FastPWM motor_3_a;
      FastPWM motor_3_b;
      FastPWM motor_4_a;
      FastPWM motor_4_b;
      /*
            PwmOut motor_1_a;
            PwmOut motor_1_b;
            PwmOut motor_2_a;
            PwmOut motor_2_b;
            PwmOut motor_3_a;
            PwmOut motor_3_b;
            PwmOut motor_4_a;
            PwmOut motor_4_b;*/

      float* own_dir;

      int16_t pre_moving_dir;
      int16_t pre_moving_speed;

      Timer addPowerTimer;
};

#endif