#include "dribbler.h"
#include "hold.h"
#include "kicker.h"
#include "mbed.h"
#include "motor.h"
#include "pid.h"
#include "simplify_deg.h"
#include "tof.h"
#include "voltage.h"

#define CUT_VOLTAGE 4.0   // 全機能強制終了する電圧
#define VOLTAGE_CNT_NUM 1000   // CUT_VOLTAGE以下にこの定義回数が続いたら強制終了

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define LINE_UART_SPEED 57600
#define IMU_UART_SPEED 57600
#define UI_UART_SPEED 19200
#define LIDAR_UART_SPEED 57600
#define CAM_UART_SPEED 38400

// UART通信定義 (TX, RX)
RawSerial line(PA_2, PA_3);
RawSerial imu(PA_9, PA_10);
RawSerial ui(PC_10, PC_11);
RawSerial lidar(PC_12, PD_2);
RawSerial cam(PA_0, PA_1);

void line_rx();
void imu_rx();
void ui_rx();
void lidar_rx();
void cam_rx();

void offence_move();
void defence_move();

PID offencePID;

Voltage voltage(PA_4);
Motor motor(PB_14, PB_15, PB_2, PB_10, PB_5, PB_3, PC_6, PC_8);
Dribbler dribblerFront(PB_6, PB_7);
Dribbler dribblerBack(PB_8, PB_9);
Hold holdFront(PC_4);
Hold holdBack(PC_5);
Kicker kicker(PC_0, PC_1);
Tof tof;

DigitalOut led_1(PA_5);
DigitalOut led_2(PA_6);

// グローバル変数定義
int16_t yaw = 0;
uint8_t is_yaw_correction = 0;
uint8_t tof_val[16];
uint8_t encoder_val_avg;
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t line_left_val;
uint8_t line_right_val;

int16_t ball_dir;
uint8_t ball_dis;
int16_t y_goal_dir;
uint8_t y_goal_size;
int16_t b_goal_dir;
uint8_t b_goal_size;

bool is_voltage_decrease = 0;
uint16_t voltage_cnt;

int main() {
      // UART初期設定
      line.baud(LINE_UART_SPEED);
      // line.attach(&line_rx);
      imu.baud(IMU_UART_SPEED);
      imu.attach(&imu_rx);
      ui.baud(UI_UART_SPEED);
      ui.attach(&ui_rx);
      lidar.baud(LIDAR_UART_SPEED);
      lidar.attach(&lidar_rx);
      cam.baud(CAM_UART_SPEED);
      // cam.attach(&cam_rx);

      motor.SetPwmPeriod(30000);
      dribblerFront.SetPwmPeriod(30000);
      dribblerBack.SetPwmPeriod(30000);

      offencePID.SetGain(0.5, 0, 15);
      offencePID.SetSamplingPeriod(0.01);
      offencePID.SetLimit(70);
      offencePID.SelectType(PI_D_TYPE);

      kicker.SetKickTime(100);

      while (1) {
            voltage.Read();

            if (voltage.Get() < CUT_VOLTAGE) {
                  voltage_cnt++;
            } else {
                  voltage_cnt = 0;
            }
            if (voltage_cnt >= VOLTAGE_CNT_NUM) is_voltage_decrease = 1;

            if (is_voltage_decrease == 1) {
                  motor.Free();
                  ui.putc('E');
                  ui.putc('R');
            } else {
                  motor.yaw = yaw;
                  motor.encoder_val = encoder_val_avg;
                  holdFront.Read();
                  holdBack.Read();

                  if (line.readable() == 1) line_rx();
                  if (cam.readable() == 1) cam_rx();

                  if (mode == 0) {
                        motor.Free();
                        dribblerFront.Stop();
                        dribblerBack.Stop();
                  } else if (mode == 1) {
                        offence_move();
                  } else if (mode == 2) {
                        defence_move();
                  }
            }
      }
}

void offence_move() {
      if (holdFront.IsHold()) {
            if (y_goal_size < 30 || abs(y_goal_dir) > 30) {
                  motor.Run(y_goal_dir * 1.5, 50);
                  dribblerFront.Hold(100);
            } else {
                  motor.Run(0, 100);
                  dribblerFront.Kick();
                  kicker.Kick();
                  wait_us(100000);
            }
      } else if (holdBack.IsHold()) {
            if (y_goal_size > 35) {
                  motor.Run(0, 70, y_goal_dir > 0 ? -90 : 90);
                  dribblerBack.Hold(100);
            } else {
                  motor.Run(0, 30);
                  dribblerBack.Hold(100);
            }
      } else {
            if ((abs(ball_dir) < 30 && ball_dis > 130) || holdFront.GetVal() < 150) {
                  dribblerFront.Hold(100);
            } else {
                  dribblerFront.Stop();
            }
            if (abs(ball_dir) > 160 && ball_dis > 150) {
                  dribblerBack.Hold(100);
            } else {
                  dribblerBack.Stop();
            }
            if (abs(ball_dir) < 160) {
                  int16_t tmp_move_speed, tmp_move_angle, robot_angle = 0;

                  int16_t addend;

                  if (abs(ball_dir) < 60) {
                        addend = ball_dir * 1.5;
                  } else {
                        addend = 90 * (abs(ball_dir) / ball_dir);
                  }
                  tmp_move_angle = ball_dir + (addend * (ball_dis / 180.000));

                  offencePID.Compute(ball_dir, 0);

                  tmp_move_speed = abs(offencePID.Get()) + ((200 - ball_dis) / 3);
                  //         +(180 - ball_dis);

                  // 方向
                  if (tmp_move_speed > 70) tmp_move_speed = 70;

                  motor.Run(tmp_move_angle, tmp_move_speed, robot_angle);   // 回り込み
            } else {
                  motor.Run(ball_dir - (180 - ball_dir), (180 - abs(ball_dir)) + 25);
            }
      }
}

void defence_move() {
      motor.Run();
}

void line_rx() {
      if (line.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 4;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = line.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
                  encoder_val_avg = recv_byte[0];
                  line_left_val = recv_byte[1];
                  line_right_val = recv_byte[2];
            }
      } else {
            return;
      }
}

void imu_rx() {
      if (imu.getc() == 0xFF) {   // ヘッダーがあることを確認
            uint8_t yaw_plus = imu.getc();
            uint8_t yaw_minus = imu.getc();

            static int16_t yaw_correction = 0;

            if (is_yaw_correction == 1) {
                  yaw_correction += yaw;
            }
            yaw = SimplifyDeg((yaw_plus == 0 ? yaw_minus * -1 : yaw_plus) - yaw_correction);
      } else {
            return;
      }
}

void ui_rx() {
      static int8_t item = 0;

      static uint8_t dribbler_sig = 0;
      static uint8_t kicker_sig = 0;

      // データ受信
      if (ui.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 7;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = ui.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
                  item = recv_byte[0] - 100;
                  mode = recv_byte[1];
                  is_yaw_correction = recv_byte[2];
                  moving_speed = recv_byte[3];
                  dribbler_sig = recv_byte[4];
                  kicker_sig = recv_byte[5];
            }
      } else {
            return;
      }

      if (mode == 0) {
            if (kicker_sig == 1) kicker.Kick();

            switch (dribbler_sig) {
                  case 0:
                        dribblerFront.Stop();
                        dribblerBack.Stop();
                        break;
                  case 1:
                        dribblerFront.Hold(100);
                        break;
                  case 2:
                        dribblerFront.Kick();
                        break;
                  case 3:
                        dribblerBack.Hold(100);
                        break;
                  case 4:
                        dribblerBack.Kick();
                        break;
            }

            // データ送信
            uint8_t send_byte_num;
            uint8_t send_byte[25];
            send_byte[0] = 0xFF;

            if (item == 0) {
                  send_byte_num = 1;
                  send_byte[0] = uint8_t(voltage.Get() * 10);
            } else if (item == 1) {
                  send_byte_num = 3;
                  send_byte[1] = yaw > 0 ? yaw : 0;
                  send_byte[2] = yaw < 0 ? yaw * -1 : 0;
            } else if (item == 2) {
                  send_byte_num = 20;
                  send_byte[1] = tof.SafeDir() > 0 ? tof.SafeDir() : 0;
                  send_byte[2] = tof.SafeDir() < 0 ? tof.SafeDir() * -1 : 0;
                  send_byte[3] = tof.MinSensor();
                  for (uint8_t i = 0; i < 16; i++) {
                        send_byte[i + 5] = tof.val[i];
                  }
            } else if (item == 3) {
                  send_byte_num = 6;
                  send_byte[1] = ball_dir > 0 ? ball_dir : 0;
                  send_byte[2] = ball_dir < 0 ? ball_dir * -1 : 0;
                  send_byte[3] = ball_dis;
                  send_byte[4] = holdFront.IsHold();
                  send_byte[5] = holdBack.IsHold();
            }

            for (uint8_t i = 0; i < send_byte_num; i++) {
                  ui.putc(send_byte[i]);
            }
      }
}

void lidar_rx() {
      if (lidar.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 17;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = lidar.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
                  for (uint8_t i = 0; i < 16; i++) {
                        tof.val[i] = recv_byte[i];
                  }
            }
      } else {
            return;
      }
}

void cam_rx() {
      if (cam.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 10;
            uint8_t recv_byte[recv_byte_num];

            uint8_t ball_dir_plus, ball_dir_minus;
            uint8_t y_goal_dir_plus, y_goal_dir_minus;
            uint8_t b_goal_dir_plus, b_goal_dir_minus;

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = cam.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
                  ball_dir_plus = recv_byte[0];
                  ball_dir_minus = recv_byte[1];
                  ball_dis = recv_byte[2];
                  y_goal_dir_plus = recv_byte[3];
                  y_goal_dir_minus = recv_byte[4];
                  y_goal_size = recv_byte[5];
                  b_goal_dir_plus = recv_byte[6];
                  b_goal_dir_minus = recv_byte[7];
                  b_goal_size = recv_byte[8];
                  ball_dir = SimplifyDeg(ball_dir_plus == 0 ? ball_dir_minus * -1 : ball_dir_plus);
                  y_goal_dir = SimplifyDeg(y_goal_dir_plus == 0 ? y_goal_dir_minus * -1 : y_goal_dir_plus);
                  b_goal_dir = SimplifyDeg(b_goal_dir_plus == 0 ? b_goal_dir_minus * -1 : b_goal_dir_plus);
            }
      } else {
            return;
      }
}