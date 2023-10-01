#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "simplify_deg.h"
#include "tof.h"
#include "voltage.h"

#define PI 3.1415926535

#define CUT_VOLTAGE 6.0   // 全機能強制終了する電圧
#define VOLTAGE_CNT_NUM 500   // CUT_VOLTAGE以下にこの定義回数が続いたら強制終了

#define MOTOR_PWM_PERIOD 30000   // モーターのPWM周波数(default: 30000)

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define LINE_UART_SPEED 57600
#define IMU_UART_SPEED 115200
#define UI_UART_SPEED 19200
#define LIDAR_UART_SPEED 57600
#define CAM_UART_SPEED 57600

#define EMPTY_READ_BUF_TIMES 20   // 毎回データを空にするためにSerial.read()する回数

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

Voltage voltage(PA_4);
Motor motor(PB_14, PB_15, PB_2, PB_10, PB_5, PB_3, PC_6, PC_8);
Dribbler dribblerFront(PB_6, PB_7);
Dribbler dribblerBack(PB_8, PB_9);
Hold holdFront(PC_4);
Hold holdBack(PC_5);
Tof tof;

DigitalOut led_1(PA_5);
DigitalOut led_2(PA_6);

DigitalOut kicker_sig_1(PC_0);
DigitalOut kicker_sig_2(PC_1);

// グローバル変数定義
int16_t yaw = 0;
uint8_t is_yaw_correction = 0;
uint8_t tof_val[16];
uint8_t encoder_val_avg;
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t line_left_val;
uint8_t line_right_val;
uint8_t dribbler_sig = 0;

int16_t ball_dir;
uint8_t ball_dis;
int16_t y_goal_dir;
uint8_t y_goal_dis;
uint8_t y_goal_size;
int16_t b_goal_dir;
uint8_t b_goal_dis;
uint8_t b_goal_size;

bool is_voltage_decrease = 0;
uint16_t voltage_cnt;

Timer test;

int main() {
      // UART初期設定
      line.baud(LINE_UART_SPEED);
      // line.attach(line_rx, Serial::RxIrq);
      imu.baud(IMU_UART_SPEED);
      imu.attach(&imu_rx);
      ui.baud(UI_UART_SPEED);
      ui.attach(&ui_rx);
      lidar.baud(LIDAR_UART_SPEED);
      lidar.attach(&lidar_rx);
      cam.baud(CAM_UART_SPEED);
      // cam.attach(cam_rx, Serial::RxIrq);

      motor.SetPwmPeriod(MOTOR_PWM_PERIOD);
      dribblerFront.SetPwmPeriod(MOTOR_PWM_PERIOD);
      dribblerBack.SetPwmPeriod(MOTOR_PWM_PERIOD);

      while (1) {
            voltage.Read();

            if (voltage.Get() < CUT_VOLTAGE) {
                  voltage_cnt++;
            } else {
                  voltage_cnt = 0;
            }
            if (voltage_cnt >= VOLTAGE_CNT_NUM) {
                  is_voltage_decrease = 1;
            }
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
                        kicker_sig_1 = 0;
                        kicker_sig_2 = 0;
                  } else if (mode == 1) {
                        motor.Run(ball_dir, 20);
                  } else if (mode == 2) {
                  }
            }
      }
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

            uint8_t n = 0;
            while (line.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  line.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) break;
            }
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

            uint8_t n = 0;
            while (imu.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  imu.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) break;
            }
      }
}

void ui_rx() {
      static int8_t item = 0;

      // データ受信
      if (ui.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 6;
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
            }

            uint8_t n = 0;
            while (ui.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  ui.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) break;
            }
      }

      switch (dribbler_sig) {
            case 0:
                  dribblerFront.Stop();
                  dribblerBack.Stop();
                  break;
            case 1:
                  dribblerFront.Hold(90);
                  break;
            case 2:
                  dribblerFront.Kick();
                  break;
            case 3:
                  dribblerBack.Hold(90);
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
            send_byte[1] = ball_dir > 0 ? ball_dir : 0;
            send_byte[2] = ball_dir < 0 ? ball_dir * -1 : 0;
      } else if (item == 2) {
            send_byte_num = 21;
            send_byte[1] = tof.SafeDir() > 0 ? tof.SafeDir() : 0;
            send_byte[2] = tof.SafeDir() < 0 ? tof.SafeDir() * -1 : 0;
            send_byte[3] = tof.MinSensor();
            send_byte[4] = tof.FindWall();
            for (uint8_t i = 0; i < 16; i++) {
                  send_byte[i + 5] = tof.val[i];
            }
      } else if (item == 3) {
            send_byte_num = 1;
            send_byte[0] = ball_dis;
      }

      for (uint8_t i = 0; i < send_byte_num; i++) {
            ui.putc(send_byte[i]);
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

            uint8_t n = 0;
            while (lidar.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  lidar.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) break;
            }
      }
}

void cam_rx() {
      if (cam.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 4;
            uint8_t recv_byte[recv_byte_num];

            uint8_t ball_dir_plus, ball_dir_minus;

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = cam.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
                  ball_dir_plus = recv_byte[0];
                  ball_dir_minus = recv_byte[1];
                  ball_dis = recv_byte[2];
                  ball_dir = SimplifyDeg(ball_dir_plus == 0 ? ball_dir_minus * -1 : ball_dir_plus);
            }

            uint8_t n = 0;
            while (cam.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  cam.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) break;
            }
      }
}