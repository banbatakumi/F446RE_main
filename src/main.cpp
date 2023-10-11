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
#define CAM_UART_SPEED 115200

// UART通信定義 (TX, RX)
RawSerial cam(PA_0, PA_1);
RawSerial line(PA_2, PA_3);
RawSerial imu(PA_9, PA_10);
RawSerial ui(PC_10, PC_11);
RawSerial lidar(PC_12, PD_2);

void cam_rx();
void line_rx();
void imu_rx();
void ui_rx();
void lidar_rx();

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
bool is_yaw_correction = 0;
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

bool is_y_goal_front;
int16_t front_goal_dir;
uint8_t front_goal_size;
int16_t back_goal_dir;
uint8_t back_goal_size;

bool is_voltage_decrease = 0;
uint16_t voltage_cnt;

Timer curveShootTimer;

int main() {
      // UART初期設定
      cam.baud(CAM_UART_SPEED);
      // cam.attach(&cam_rx);
      line.baud(LINE_UART_SPEED);
      // line.attach(&line_rx);
      imu.baud(IMU_UART_SPEED);
      imu.attach(&imu_rx, Serial::RxIrq);
      ui.baud(UI_UART_SPEED);
      ui.attach(&ui_rx, Serial::RxIrq);
      lidar.baud(LIDAR_UART_SPEED);
      lidar.attach(&lidar_rx, Serial::RxIrq);

      motor.SetPwmPeriod(30000);
      dribblerFront.SetPwmPeriod(30000);
      dribblerBack.SetPwmPeriod(30000);

      offencePID.SetGain(1, 0, 2.5);
      offencePID.SetSamplingPeriod(0.01);
      offencePID.SetLimit(75);
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
                  } else if (mode == 1) {
                        offence_move();
                  } else if (mode == 2) {
                        defence_move();
                  }
            }
      }
}

void offence_move() {
      if (ball_dis == 0) {   // ボールがない時の処理
            if (tof.val[tof.MinSensor()] < 100) {
                  motor.Run(tof.SafeDir(), 50);
            } else {
                  motor.Free();
            }
      } else {
            if (curveShootTimer.read() > 0) {   // 後ろドリブラーのマカオシュート
                  static int8_t shoot_dir;

                  dribblerBack.Hold(100);
                  if (curveShootTimer.read() < 0.5) {
                        motor.Run(0, 0);
                        shoot_dir = y_goal_dir > 0 ? -1 : 1;
                  } else if (curveShootTimer.read() < 1) {
                        motor.Run(0, 0, 45 * shoot_dir, BACK, 25);
                  } else if (curveShootTimer.read() < 1.5) {
                        motor.Run(0, 0, 135 * shoot_dir);
                  } else {
                        curveShootTimer.stop();
                        curveShootTimer.reset();
                  }
                  if (curveShootTimer.read() < 1 && holdBack.IsHold() == 0) {
                        curveShootTimer.stop();
                        curveShootTimer.reset();
                  }
            } else if (holdFront.IsHold()) {   // 前に捕捉している時
                  if (y_goal_size < 25 || abs(y_goal_dir) > 25) {
                        if (y_goal_size < 25) {
                              motor.Run(y_goal_dir * 2, 75);
                              dribblerFront.Hold(100);
                        } else {
                              motor.Run(y_goal_dir * 2, 60);
                              dribblerFront.Hold(100);
                        }
                  } else {
                        motor.Run(0, 100);
                        dribblerFront.Kick();
                        kicker.Kick();
                  }
            } else if (holdBack.IsHold()) {   // 後ろに捕捉している時
                  dribblerBack.Hold(100);
                  if (y_goal_size < 30 || abs(y_goal_dir) > 30) {
                        if (y_goal_size < 30) {
                              motor.Run(y_goal_dir * 2, 75);
                        } else {
                              motor.Run(y_goal_dir * 2, 50);
                        }
                  } else {
                        curveShootTimer.start();
                  }
            } else {
                  if (abs(ball_dir) < 30 && ball_dis > 150) {
                        dribblerFront.Hold(50);
                  } else {
                        dribblerFront.Stop();
                  }
                  if (abs(ball_dir) > 160 && ball_dis > 150) {
                        dribblerBack.Hold(50);
                  } else {
                        dribblerBack.Stop();
                  }
                  if (abs(ball_dir) < 150) {   // 前の捕捉エリアに回り込む
                        int16_t tmp_move_speed, tmp_move_angle;

                        uint8_t wrap_speed_addend;
                        int16_t wrap_deg_addend;

                        // 角度
                        if (abs(ball_dir) < 45) {
                              wrap_deg_addend = ball_dir * 2;
                        } else {
                              wrap_deg_addend = 90 * (abs(ball_dir) / ball_dir);
                        }
                        tmp_move_angle = ball_dir + (wrap_deg_addend * (ball_dis / 180.000));

                        // 速度
                        offencePID.Compute(ball_dir, 0);

                        wrap_speed_addend = (25 - abs(ball_dir));
                        if (abs(ball_dir) > 25) wrap_speed_addend = 0;

                        tmp_move_speed = abs(offencePID.Get()) + wrap_speed_addend + ((200 - ball_dis) / 5);

                        if (tmp_move_speed > 75) tmp_move_speed = 75;

                        motor.Run(tmp_move_angle, tmp_move_speed);
                  } else {   // 後ろの捕捉エリアに回り込む
                        int16_t inverse_ball_dir = SimplifyDeg(ball_dir + 180);   // ボールの角度を後ろを0度に変換
                        int16_t tmp_move_speed, tmp_move_angle;

                        uint8_t wrap_speed_addend;
                        int16_t wrap_deg_addend;

                        // 角度
                        if (abs(inverse_ball_dir) < 45) {
                              wrap_deg_addend = inverse_ball_dir * 2;
                        } else {
                              wrap_deg_addend = 90 * (abs(inverse_ball_dir) / inverse_ball_dir);
                        }
                        tmp_move_angle = inverse_ball_dir + (wrap_deg_addend * (ball_dis / 180.000));

                        // 速度
                        offencePID.Compute(inverse_ball_dir, 0);

                        wrap_speed_addend = (25 - abs(inverse_ball_dir));
                        if (abs(inverse_ball_dir) > 25) wrap_speed_addend = 0;

                        tmp_move_speed = abs(offencePID.Get()) + wrap_speed_addend + ((200 - ball_dis) / 10);

                        if (tmp_move_speed > 50) tmp_move_speed = 50;

                        tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);   // ０度の時に180度に動くように変換
                        motor.Run(tmp_move_angle, tmp_move_speed);
                  }
            }
      }
}

void defence_move() {
      motor.Run();
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
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
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

                  if (is_y_goal_front == 1) {
                        front_goal_dir = y_goal_dir;
                        front_goal_size = y_goal_size;
                        back_goal_dir = b_goal_dir;
                        back_goal_size = b_goal_size;
                  } else {
                        front_goal_dir = b_goal_dir;
                        front_goal_size = b_goal_size;
                        back_goal_dir = y_goal_dir;
                        back_goal_size = y_goal_size;
                  }
            }
      } else {
            return;
      }
}

void line_rx() {
      if (line.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 4;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = line.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
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

      // データ受信
      if (ui.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 7;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = ui.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  item = recv_byte[0] - 100;
                  mode = recv_byte[1];
                  is_yaw_correction = recv_byte[2];
                  moving_speed = recv_byte[3];
                  dribbler_sig = recv_byte[4];
                  is_y_goal_front = recv_byte[5];
            }
      } else {
            return;
      }

      if (mode == 0) {
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
                        kicker.Kick();
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
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        ui.putc(send_byte[i]);
                  }
            } else if (item == 1) {
                  send_byte_num = 3;
                  send_byte[1] = yaw > 0 ? yaw : 0;
                  send_byte[2] = yaw < 0 ? yaw * -1 : 0;
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        ui.putc(send_byte[i]);
                  }
            } else if (item == 2) {
                  send_byte_num = 20;
                  send_byte[1] = tof.SafeDir() > 0 ? tof.SafeDir() : 0;
                  send_byte[2] = tof.SafeDir() < 0 ? tof.SafeDir() * -1 : 0;
                  send_byte[3] = tof.MinSensor();
                  for (uint8_t i = 0; i < 16; i++) {
                        send_byte[i + 4] = tof.val[i];
                  }
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        ui.putc(send_byte[i]);
                  }
            } else if (item == 3) {
                  send_byte_num = 6;
                  send_byte[1] = ball_dir > 0 ? ball_dir : 0;
                  send_byte[2] = ball_dir < 0 ? ball_dir * -1 : 0;
                  send_byte[3] = ball_dis;
                  send_byte[4] = holdFront.IsHold();
                  send_byte[5] = holdBack.IsHold();
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        ui.putc(send_byte[i]);
                  }
            } else if (item == 4) {
                  send_byte_num = 7;
                  send_byte[1] = y_goal_dir > 0 ? y_goal_dir : 0;
                  send_byte[2] = y_goal_dir < 0 ? y_goal_dir * -1 : 0;
                  send_byte[3] = y_goal_size;
                  send_byte[4] = b_goal_dir > 0 ? b_goal_dir : 0;
                  send_byte[5] = b_goal_dir < 0 ? b_goal_dir * -1 : 0;
                  send_byte[6] = b_goal_size;
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        ui.putc(send_byte[i]);
                  }
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
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  for (uint8_t i = 0; i < 16; i++) {
                        tof.val[i] = recv_byte[i];
                  }
            }
      } else {
            return;
      }
}