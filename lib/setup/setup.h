#ifndef _SETUP_H_
#define _SETUP_H_

#include "cam.h"
#include "dribbler.h"
#include "hold.h"
#include "imu.h"
#include "kicker.h"
#include "line.h"
#include "mbed.h"
#include "motor.h"
#include "pid.h"
#include "simplify_deg.h"
#include "ultrasonic.h"
#include "voltage.h"

#define PI 3.1415926535  // 円周率

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define UI_UART_SPEED 9600

#define HOLD_MAX_POWER 100
#define HOLD_WAIT_POWER 90

#define readms(timer_name_) chrono::duration_cast<chrono::milliseconds>((timer_name_).elapsed_time()).count()

// グローバル変数定義
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t line_moving_speed;

int16_t own_dir;

// UART通信定義 (TX, RX)
UnbufferedSerial uiSerial(PC_10, PC_11);

void Ui();

void GetSensors();

PID wrapDirPID;
PID defensePID;

Voltage voltage(PA_4);
Motor motor(PB_14, PB_15, PB_2, PB_10, PB_5, PB_3, PC_6, PC_8, &own_dir);
Dribbler dribblerFront(PB_6, PB_7);
Dribbler dribblerBack(PB_8, PB_9);
Hold holdFront(PC_4);
Hold holdBack(PC_5);
Kicker kicker(PC_0, PC_1);
Ultrasonic ultrasonic(PC_12, PD_2, 115200);  // TX, RX
Cam cam(PA_0, PA_1, &own_dir);
Imu imu(PA_9, PA_10, 115200);
Line line(PA_2, PA_3);

DigitalOut led[2] = {PA_5, PA_6};
typedef struct {
      int16_t ball_dir;
      int16_t inverse_ball_dir;
      uint8_t ball_dis;
      int16_t y_goal_dir;
      uint8_t y_goal_size;
      int16_t b_goal_dir;
      uint8_t b_goal_size;
      int16_t ops_goal_dir;
      uint8_t ops_goal_size;
      int16_t own_goal_dir;
      uint8_t own_goal_size;
      bool is_goal_front;
      int16_t ball_x;
      int16_t ball_y;
      int16_t own_x;
      int16_t own_y;
      int16_t center_dir;
      int16_t center_dis;
} type_camera;

typedef struct {
      uint8_t dis[4];
      uint8_t encoder_val[4];
      int16_t ir_dir;
      uint8_t ir_dis;
      uint8_t is_line_left;
      uint8_t is_line_right;
      uint8_t is_on_line;
      uint8_t line_interval;
      int16_t line_dir;
      int16_t line_inside_dir;
      uint8_t line_depth;
      bool hold_front;
      bool hold_back;
} type_sensors;

type_camera camera;
type_sensors sensors;

#endif