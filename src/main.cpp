#include "dribbler.h"
#include "hold.h"
#include "kicker.h"
#include "mbed.h"
#include "motor.h"
#include "pid.h"
#include "simplify_deg.h"
#include "tof.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

#define CUT_VOLTAGE 4.0   // 全機能強制終了する電圧
#define VOLTAGE_CNT_NUM 1200   // CUT_VOLTAGE以下にこの定義回数が続いたら強制終了

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define LINE_UART_SPEED 115200
#define IMU_UART_SPEED 115200
#define UI_UART_SPEED 115200
#define LIDAR_UART_SPEED 115200
#define CAM_UART_SPEED 115200

// UART通信定義 (TX, RX)
RawSerial camSerial(PA_0, PA_1);
RawSerial lineSerial(PA_2, PA_3);
RawSerial imuSerial(PA_9, PA_10);
RawSerial uiSerial(PC_10, PC_11);
RawSerial lidarSerial(PC_12, PD_2);

void Cam();
void Line();
void Imu();
void Ui();
void Lidar();

void OffenceMove();
void DefenceMove();

PID wrapDirPID;
PID wrapDisPID;

Voltage voltage(PA_4);
Motor motor(PB_14, PB_15, PB_2, PB_10, PB_5, PB_3, PC_6, PC_8);
Dribbler dribblerFront(PB_6, PB_7);
Dribbler dribblerBack(PB_8, PB_9);
Hold holdFront(PC_4);
Hold holdBack(PC_5);
Kicker kicker(PC_0, PC_1);
Tof tof;

DigitalOut led[2] = {PA_5, PA_6};

// グローバル変数定義
uint8_t tof_val[16];
uint8_t encoder_val[4];
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t line_moving_speed;

// ラインセンサ
uint8_t is_line_left;
uint8_t is_line_right;
uint8_t line_white_num;
int16_t line_vector;

// カメラ関連
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

// ロボット関連
int16_t own_x;
int16_t own_y;
int16_t own_dir;
bool is_own_dir_correction = 0;

bool is_voltage_decrease = 0;
uint16_t voltage_cnt;

Timer curveShootTimer;

void setup() {
      // UART初期設定
      camSerial.baud(CAM_UART_SPEED);
      // camSerial.attach(&Cam);
      lineSerial.baud(LINE_UART_SPEED);
      // lineSerial.attach(&Line);
      imuSerial.baud(IMU_UART_SPEED);
      imuSerial.attach(&Imu, Serial::RxIrq);
      uiSerial.baud(UI_UART_SPEED);
      uiSerial.attach(&Ui, Serial::RxIrq);
      lidarSerial.baud(LIDAR_UART_SPEED);
      lidarSerial.attach(&Lidar, Serial::RxIrq);

      // モーター
      motor.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      motor.SetAttitudeControlPID(1.5, 0, 0.075);   // デフォルトゲイン：(1.5, 0, 0.075)
      motor.SetMovingAveLength(25);
      motor.SetPowerMaxLimit(90);
      motor.SetPowerMinLimit(5);

      // ドリブラー
      dribblerFront.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      dribblerBack.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz

      // 回り込みPID
      wrapDirPID.SetGain(1, 0, 25);
      wrapDirPID.SetSamplingPeriod(0.01);
      wrapDirPID.SetLimit(100);
      wrapDirPID.SelectType(PI_D_TYPE);

      wrapDisPID.SetGain(1, 0, 50);
      wrapDisPID.SetSamplingPeriod(0.01);
      wrapDisPID.SetLimit(50);
      wrapDisPID.SelectType(PI_D_TYPE);

      // キッカー
      kicker.SetPower(100);   // 100まで
}

int main() {
      setup();
      while (1) {
            voltage.Read();

            // バッテリー電圧降下による動作停止
            if (voltage.Get() < CUT_VOLTAGE) {
                  voltage_cnt++;
            } else {
                  voltage_cnt = 0;
            }
            if (voltage_cnt >= VOLTAGE_CNT_NUM) is_voltage_decrease = 1;

            if (is_voltage_decrease == 1) {
                  motor.Free();
            } else {
                  motor.own_dir = own_dir;
                  motor.encoder_val[0] = encoder_val[0];
                  motor.encoder_val[1] = encoder_val[1];
                  motor.encoder_val[2] = encoder_val[2];
                  motor.encoder_val[3] = encoder_val[3];
                  holdFront.Read();
                  holdBack.Read();

                  if (lineSerial.readable() == 1) Line();
                  if (camSerial.readable() == 1) Cam();

                  if (mode == 0) {
                        motor.Free();
                  } else if (mode == 1) {
                        if (line_white_num != 0) {
                              motor.Run(SimplifyDeg(line_vector - 180), line_moving_speed);
                        } else if (is_line_left == 1) {
                              motor.Run(90, line_moving_speed);
                        } else if (is_line_right == 1) {
                              motor.Run(-90, line_moving_speed);
                        } else {
                              OffenceMove();
                        }
                  } else if (mode == 2) {
                        DefenceMove();
                  } else if (mode == 3) {
                        motor.Free();
                  } else if (mode == 4) {
                        motor.Run(0, 50);
                  }
            }
      }
}

void OffenceMove() {
      if (ball_dis == 0) {   // ボールがない時の処理
            if (tof.val[tof.MinSensor()] < 150) {
                  motor.Run(tof.SafeDir(), 50);   // 中央に戻る
            } else {
                  motor.Free();
            }
      } else {
            if (curveShootTimer.read() > 0) {   // 後ろドリブラーのマカオシュート
                  static int8_t shoot_dir;

                  dribblerBack.Hold(90);
                  if (curveShootTimer.read() < 0.5) {
                        motor.Run(0, 0);
                        shoot_dir = front_goal_dir > 0 ? -1 : 1;
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
                  if (front_goal_size > 25 && abs(front_goal_dir) < 15) {   // キッカーを打つ条件
                        motor.Run(0, 80);
                        dribblerFront.Brake();
                        kicker.Kick();
                  } else {
                        dribblerFront.Hold(90);
                        if (tof.val[tof.FrontMinSensor()] < 50 && (abs(front_goal_dir) < 30 || front_goal_size == 0)) {
                              motor.Run(SimplifyDeg(tof.FrontMinSensor() * 22.5 + 90 * (tof.FrontSafeDir(50) > 0 ? 1 : -1)), 60);
                        } else {
                              motor.Run(front_goal_dir * 2, 75);
                        }
                  }
            } else if (holdBack.IsHold()) {   // 後ろに捕捉している時
                  if (front_goal_size > 30 && abs(front_goal_dir) < 30) {   // カーブシュート開始
                        curveShootTimer.start();
                  } else {
                        dribblerBack.Hold(90);

                        if (tof.val[tof.FrontMinSensor()] < 50 && (abs(front_goal_dir) < 30 || front_goal_size == 0)) {
                              motor.Run(SimplifyDeg(tof.FrontMinSensor() * 22.5 + 90 * (tof.FrontSafeDir() > 0 ? 1 : -1)), 60);
                        } else {
                              motor.Run(front_goal_dir * 2, 75);
                        }
                  }
            } else {
                  if (abs(ball_dir) < 30 && ball_dis > 150) {
                        dribblerFront.Hold(90);
                  } else {
                        dribblerFront.Stop();
                  }
                  if (abs(ball_dir) > 150 && ball_dis > 150) {
                        dribblerBack.Hold(90);
                  } else {
                        dribblerBack.Stop();
                  }
                  if (abs(ball_dir) < 150) {   // 前の捕捉エリアに回り込む
                        int16_t tmp_move_speed, tmp_move_angle;

                        uint8_t wrap_speed_addend;
                        int16_t wrap_deg_addend;

                        // 角度
                        if (abs(ball_dir) < 90) {
                              wrap_deg_addend = ball_dir;
                        } else {
                              wrap_deg_addend = 90 * (abs(ball_dir) / ball_dir);
                        }
                        tmp_move_angle = ball_dir + (wrap_deg_addend * (ball_dis / 150.000));

                        // 速度
                        wrapDirPID.Compute(ball_dir, 0);
                        wrapDisPID.Compute(ball_dis, 175);

                        wrap_speed_addend = (10 - abs(ball_dir));
                        if (abs(ball_dir) > 10) wrap_speed_addend = 0;
                        tmp_move_speed = abs(wrapDirPID.Get()) + abs(wrapDisPID.Get()) + wrap_speed_addend;

                        if (tmp_move_speed > 75) tmp_move_speed = 75;

                        motor.Run(tmp_move_angle, tmp_move_speed);
                  } else {   // 後ろの捕捉エリアに回り込む
                        int16_t inverse_ball_dir = SimplifyDeg(ball_dir + 180);   // ボールの角度を後ろを0度に変換
                        int16_t tmp_move_speed, tmp_move_angle;

                        uint8_t wrap_speed_addend;
                        int16_t wrap_deg_addend;

                        // 角度
                        if (abs(inverse_ball_dir) < 90) {
                              wrap_deg_addend = inverse_ball_dir;
                        } else {
                              wrap_deg_addend = 90 * (abs(inverse_ball_dir) / inverse_ball_dir);
                        }
                        tmp_move_angle = inverse_ball_dir + (wrap_deg_addend * (ball_dis / 150.000));

                        // 速度
                        wrapDirPID.Compute(ball_dir, 0);
                        wrapDisPID.Compute(ball_dis, 150);

                        wrap_speed_addend = (10 - abs(ball_dir));
                        if (abs(ball_dir) > 10) wrap_speed_addend = 0;
                        tmp_move_speed = abs(wrapDirPID.Get()) + abs(wrapDisPID.Get()) + wrap_speed_addend;

                        if (tmp_move_speed > 60) tmp_move_speed = 60;

                        tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);   // ０度の時に180度に動くように変換
                        motor.Run(tmp_move_angle, tmp_move_speed);
                  }
            }
      }
}

void DefenceMove() {
      /*
      if(is_line_right){
            motor.Run(-90,90);
      }else{
            motor.Run(90, 50);
      }*/
      if (line_white_num == 0) {
            motor.Free();
      } else {
            motor.Run(SimplifyDeg(line_vector - 180), line_moving_speed);
      }
      // motor.Run();
}

void Cam() {
      if (camSerial.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 10;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = camSerial.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  uint8_t ball_dir_plus, ball_dir_minus;
                  uint8_t y_goal_dir_plus, y_goal_dir_minus;
                  uint8_t b_goal_dir_plus, b_goal_dir_minus;

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
                  if (ball_dis == 0) ball_dir = 0;
                  if (y_goal_size == 0) y_goal_dir = 0;
                  if (b_goal_size == 0) b_goal_dir = 0;

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
            } else {
                  return;
            }
      } else {
            return;
      }

      // 送信
      camSerial.putc(mode);
}

void Line() {
      // 受信
      if (lineSerial.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 10;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = lineSerial.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  uint8_t line_vector_plus, line_vector_minus;

                  encoder_val[0] = recv_byte[0];
                  encoder_val[1] = recv_byte[1];
                  encoder_val[2] = recv_byte[2];
                  encoder_val[3] = recv_byte[3];
                  line_white_num = recv_byte[4];
                  is_line_left = recv_byte[5];
                  is_line_right = recv_byte[6];
                  line_vector_plus = recv_byte[7];
                  line_vector_minus = recv_byte[8];
                  line_vector = SimplifyDeg(line_vector_plus == 0 ? line_vector_minus * -1 : line_vector_plus);
            } else {
                  return;
            }
      } else {
            return;
      }

      // 送信
      lineSerial.putc(mode);
}

void Imu() {
      if (imuSerial.getc() == 0xFF) {   // ヘッダーがあることを確認
            uint8_t own_dir_plus = imuSerial.getc();
            uint8_t own_dir_minus = imuSerial.getc();

            static int16_t own_dir_correction = 0;

            if (is_own_dir_correction == 1) {
                  own_dir_correction += own_dir;
            }
            own_dir = SimplifyDeg((own_dir_plus == 0 ? own_dir_minus * -1 : own_dir_plus) - own_dir_correction);
      } else {
            return;
      }
}

void Ui() {
      static int8_t item = 0;

      static uint8_t dribbler_sig = 0;

      // 受信
      if (uiSerial.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 8;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = uiSerial.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  item = recv_byte[0] - 100;
                  mode = recv_byte[1];
                  is_own_dir_correction = recv_byte[2];
                  moving_speed = recv_byte[3];
                  line_moving_speed = recv_byte[4];
                  dribbler_sig = recv_byte[5];
                  is_y_goal_front = recv_byte[6];
            } else {
                  return;
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
                        dribblerFront.Hold(95);
                        break;
                  case 2:
                        dribblerFront.Stop();
                        kicker.Kick();
                        break;
                  case 3:
                        dribblerBack.Hold(95);
                        break;
                  case 4:
                        dribblerBack.Stop();
                        break;
            }
      }

      // 送信
      if (is_voltage_decrease == 1) {
            uiSerial.putc('E');
            uiSerial.putc('R');
      } else {
            uint8_t send_byte_num;
            uint8_t send_byte[25];
            send_byte[0] = 0xFF;

            if (item == 0) {
                  int16_t debug_val_1 = encoder_val[0];
                  int16_t debug_val_2 = encoder_val[1];
                  int16_t debug_val_3 = encoder_val[2];
                  int16_t debug_val_4 = encoder_val[3];

                  send_byte_num = 9;
                  send_byte[0] = uint8_t(voltage.Get() * 10);
                  send_byte[1] = debug_val_1 > 0 ? debug_val_1 : 0;
                  send_byte[2] = debug_val_1 < 0 ? debug_val_1 * -1 : 0;
                  send_byte[3] = debug_val_2 > 0 ? debug_val_2 : 0;
                  send_byte[4] = debug_val_2 < 0 ? debug_val_2 * -1 : 0;
                  send_byte[5] = debug_val_3 > 0 ? debug_val_3 : 0;
                  send_byte[6] = debug_val_3 < 0 ? debug_val_3 * -1 : 0;
                  send_byte[7] = debug_val_4 > 0 ? debug_val_4 : 0;
                  send_byte[8] = debug_val_4 < 0 ? debug_val_4 * -1 : 0;
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        uiSerial.putc(send_byte[i]);
                  }
            } else if (item == 1) {
                  send_byte_num = 3;
                  send_byte[1] = own_dir > 0 ? own_dir : 0;
                  send_byte[2] = own_dir < 0 ? own_dir * -1 : 0;
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        uiSerial.putc(send_byte[i]);
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
                        uiSerial.putc(send_byte[i]);
                  }
            } else if (item == 3) {
                  send_byte_num = 6;
                  send_byte[1] = ball_dir > 0 ? ball_dir : 0;
                  send_byte[2] = ball_dir < 0 ? ball_dir * -1 : 0;
                  send_byte[3] = ball_dis;
                  send_byte[4] = holdFront.IsHold();
                  send_byte[5] = holdBack.IsHold();
                  for (uint8_t i = 0; i < send_byte_num; i++) {
                        uiSerial.putc(send_byte[i]);
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
                        uiSerial.putc(send_byte[i]);
                  }
            }
      }
}

void Lidar() {
      if (lidarSerial.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 17;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = lidarSerial.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  for (uint8_t i = 0; i < 16; i++) {
                        tof.val[i] = recv_byte[i];
                  }
            } else {
                  return;
            }
      } else {
            return;
      }
}