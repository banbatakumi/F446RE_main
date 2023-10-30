#ifndef _SETUP_COM_H_
#define _SETUP_COM_H_

#include "cam.h"
#include "dribbler.h"
#include "hold.h"
#include "imu.h"
#include "kicker.h"
#include "lidar.h"
#include "line.h"
#include "mbed.h"
#include "motor.h"
#include "pid.h"
#include "simplify_deg.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

#define CUT_VOLTAGE 4.0   // 全機能強制終了する電圧
#define VOLTAGE_CNT_NUM 1200   // CUT_VOLTAGE以下にこの定義回数が続いたら強制終了

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define UI_UART_SPEED 115200

// グローバル変数定義
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t line_moving_speed;

// ロボット関連
int16_t own_dir;

// UART通信定義 (TX, RX)
RawSerial uiSerial(PC_10, PC_11);

void Ui();

void OffenceMove();
void DefenceMove();

void GetSensors();

PID wrapDirPID;
PID wrapDisPID;

Voltage voltage(PA_4);
Motor motor(PB_14, PB_15, PB_2, PB_10, PB_5, PB_3, PC_6, PC_8, &own_dir);
Dribbler dribblerFront(PB_6, PB_7);
Dribbler dribblerBack(PB_8, PB_9);
Hold holdFront(PC_4);
Hold holdBack(PC_5);
Kicker kicker(PC_0, PC_1);
Lidar lidar(PC_12, PD_2, 115200);   // TX, RX
Cam cam(PA_0, PA_1, &own_dir);
Imu imu(PA_9, PA_10, 115200);
Line line(PA_2, PA_3, &mode);

DigitalOut led[2] = {PA_5, PA_6};

Timer curveShootTimer;

typedef struct {
      int16_t ball_dir;
      uint8_t ball_dis;
      int16_t y_goal_dir;
      uint8_t y_goal_size;
      int16_t b_goal_dir;
      uint8_t b_goal_size;
      int16_t front_goal_dir;
      uint8_t front_goal_size;
      int16_t back_goal_dir;
      uint8_t back_goal_size;
      int16_t own_x;
      int16_t own_y;
} type_camera;

typedef struct {
      uint8_t encoder_val[4];
      uint8_t is_line_left;
      uint8_t is_line_right;
      uint8_t line_white_num;
      int16_t line_vector;
      bool hold_front;
      bool hold_back;
} type_sensors;

type_camera camera;
type_sensors sensors;

#endif