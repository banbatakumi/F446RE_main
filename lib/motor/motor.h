#ifndef MBED_MOTOR_H
#define MBED_MOTOR_H

#include "mbed.h"
#include "moving_ave.h"
#include "pid.h"
#include "simplify_deg.h"
#include "sin_cos_table.h"

#define PI 3.1415926535   // 円周率

#define MOTOR_QTY 4
class Motor {
     public:
      Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_);
      void Run(int16_t moving_dir_ = 0, uint8_t moving_speed_ = 0, int16_t robot_angle_ = 0, uint8_t robot_angle_mode_ = 0, uint8_t pid_limit_ = 100);
      void SetPwmPeriod(uint16_t pwm_period_);
      void SetAttitudeControlPID(float kp_, float ki_, float kd_);
      void SetMovingAveLength(uint8_t length_ = 25);
      void SetPowerMaxLimit(uint8_t limit_ = 100);
      void SetPowerMinLimit(uint8_t limit_ = 0);
      void Brake(uint16_t brake_time_ = 0);
      void Free();

#define FRONT 1
#define RIGHT 2
#define BACK 3
#define LEFT 4

      int16_t own_dir;
      uint8_t encoder_val[MOTOR_QTY];

     private:
      MovingAve motor1Ave;
      MovingAve motor2Ave;
      MovingAve motor3Ave;
      MovingAve motor4Ave;

      PID attitudeControlPID;

      PwmOut motor_1_a;
      PwmOut motor_1_b;
      PwmOut motor_2_a;
      PwmOut motor_2_b;
      PwmOut motor_3_a;
      PwmOut motor_3_b;
      PwmOut motor_4_a;
      PwmOut motor_4_b;

      uint8_t power_max_limit;
      uint8_t power_min_limit;

      Timer addPowerTimer;
};

#endif