#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "tof.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

#define CUT_VOLTAGE 6.0
#define VOLTAGE_CNT_NUM 500

#define EMPTY_READ_BUF_TIMES 20

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define LINE_UART_SPEED 57600
#define IMU_UART_SPEED 115200
#define UI_UART_SPEED 19200
#define LIDAR_UART_SPEED 57600
#define CAM_UART_SPEED 57600

// UART通信定義 (TX, RX)
RawSerial line(PA_2, PA_3);
RawSerial imu(PA_9, PA_10);
RawSerial ui(PC_10, PC_11);
RawSerial lidar(PC_12, PD_2);
RawSerial cam(PA_0, PA_1);

// 関数定義
void line_rx();
void imu_rx();
void ui_rx();
void lidar_rx();
void cam_rx();

// ピン定義
voltage Voltage(PA_4);
motor Motor(PB_14, PB_15, PB_2, PB_10, PB_5, PB_3, PC_6, PC_8);
dribbler Dribbler(PB_6, PB_7, PB_8, PB_9);
hold Hold(PC_4, PC_5);
tof Tof;

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

uint8_t ball_dir;
uint8_t ball_dis;
uint8_t y_goal_dir;
uint8_t y_goal_dis;
uint8_t y_goal_size;
uint8_t b_goal_dir;
uint8_t b_goal_dis;
uint8_t b_goal_size;

bool is_voltage_decrease = 0;
uint16_t voltage_cnt;

Timer test;

int main() {
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

      Motor.set_pwm();
      Dribbler.set_pwm();

      test.start();

      while (1) {
            Voltage.read();

            if (Voltage.get_voltage() < CUT_VOLTAGE) {
                  voltage_cnt++;
            } else {
                  voltage_cnt = 0;
            }
            if (voltage_cnt >= VOLTAGE_CNT_NUM) {
                  is_voltage_decrease = 1;
            }
            if (is_voltage_decrease == 1) {
                  Motor.free();
                  ui.putc('E');
                  ui.putc('R');
            } else {
                  Motor.yaw = yaw;
                  Motor.encoder_val = encoder_val_avg;
                  Hold.read();

                  if (line.readable() == 1) {
                        line_rx();
                  }
                  if (cam.readable() == 1) {
                        cam_rx();
                  }

                  if (mode == 0) {
                        Motor.free();
                  } else if (mode == 1) {
                        if (line_left_val > 70) {
                              Motor.run(90, 50);
                              Dribbler.f_stop();
                        } else if (line_right_val > 70) {
                              Motor.run(-90, 50);
                        } else if (ball_dis > 50) {
                              Motor.run((ball_dir - 100) / 1.1, 30);
                              Dribbler.f_hold(50);
                        } else {
                              Motor.run(180, 50);
                              Dribbler.f_stop();
                        }
                  } else if (mode == 2) {
                        Motor.run(0, 0, 0, RIGHT);
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
                  if (n > EMPTY_READ_BUF_TIMES) {
                        break;
                  }
            }
      }
}

void imu_rx() {
      if (imu.getc() == 0xFF) {   // ヘッダーがあることを確認
            uint8_t yaw_plus = imu.getc();
            uint8_t yaw_minus = imu.getc();

            static int16_t yaw_correction = 0;

            yaw = yaw_plus == 0 ? yaw_minus * -1 : yaw_plus;
            yaw -= yaw_correction;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
            if (is_yaw_correction == 1) {
                  yaw_correction = yaw + yaw_correction;
            }

            uint8_t n = 0;
            while (imu.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  imu.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) {
                        break;
                  }
            }
      }
}

void ui_rx() {
      static int8_t item = 0;
      static uint8_t dribbler_sig = 0;

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
                  if (n > EMPTY_READ_BUF_TIMES) {
                        break;
                  }
            }
      }

      switch (dribbler_sig) {
            case 0:
                  Dribbler.stop();
                  break;
            case 1:
                  Dribbler.f_hold(90);
                  break;
            case 2:
                  Dribbler.f_kick();
                  break;
            case 3:
                  Dribbler.b_hold(90);
                  break;
            case 4:
                  Dribbler.b_kick();
                  break;
      }

      uint8_t send_byte_num;
      uint8_t send_byte[20];

      send_byte[0] = 0xFF;

      if (item == 0) {
            send_byte_num = 1;
            send_byte[0] = uint8_t(Voltage.get_voltage() * 10);
      } else if (item == 1) {
            send_byte_num = 3;
            send_byte[1] = yaw > 0 ? yaw : 0;
            send_byte[2] = yaw < 0 ? yaw * -1 : 0;
      } else if (item == 2) {
            send_byte_num = 20;
            send_byte[1] = Tof.safe_dir() > 0 ? Tof.safe_dir() : 0;
            send_byte[2] = Tof.safe_dir() < 0 ? Tof.safe_dir() * -1 : 0;
            send_byte[3] = Tof.min_sensor();
            for (uint8_t i = 0; i < 16; i++) {
                  send_byte[i + 4] = Tof.val[i];
            }
      } else if (item == 3) {
            send_byte_num = 1;
            send_byte[0] = ball_dir;
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
                        Tof.val[i] = recv_byte[i];
                  }
            }

            uint8_t n = 0;
            while (lidar.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  lidar.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) {
                        break;
                  }
            }
      }
}

void cam_rx() {
      if (cam.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 9;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  recv_byte[i] = cam.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
                  ball_dir = recv_byte[0];
                  ball_dis = recv_byte[1];
                  y_goal_dir = recv_byte[2];
                  y_goal_dis = recv_byte[3];
                  y_goal_size = recv_byte[4];
                  b_goal_dir = recv_byte[5];
                  b_goal_dis = recv_byte[6];
                  b_goal_size = recv_byte[7];
            }

            uint8_t n = 0;
            while (cam.readable() == 1) {   // 受信データがなくなるまで読み続ける・受信バッファを空にする
                  cam.getc();   // データは格納されない
                  n++;
                  if (n > EMPTY_READ_BUF_TIMES) {
                        break;
                  }
            }
      }
}