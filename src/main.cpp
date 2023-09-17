#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "tof.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

#define CUT_VOLTAGE 6.0
#define VOLTAGE_CNT_NUM 500

#define EMPTY_READ_BUF_TIMES 50

// 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200
#define LINE_UART_SPEED 38400
#define IMU_UART_SPEED 57600
#define UI_UART_SPEED 19200
#define LIDAR_UART_SPEED 19200
#define CAM_UART_SPEED 9600

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
motor Motor(PC_8, PC_6, PB_3, PB_5, PB_10, PB_2, PB_15, PB_14);
dribbler Dribbler(PB_6, PB_7, PB_8, PB_9);
hold Hold(PC_5, PC_4);
tof Tof;

DigitalOut led_1(PA_5);
DigitalOut led_2(PA_6);

// グローバル変数定義
int16_t yaw = 0, yaw_correction = 0;
uint8_t tof_val[16];
uint8_t motor_rotation_num_avg;
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t line_left_val;
uint8_t line_right_val;

uint8_t front_ball_x;
uint8_t front_ball_y;
uint8_t front_y_goal_x;
uint8_t front_y_goal_y;
uint8_t front_y_goal_size;
uint8_t front_b_goal_x;
uint8_t front_b_goal_y;
uint8_t front_b_goal_size;

bool is_voltage_decrease = 0;
uint16_t voltage_cnt;

Timer test;

int main() {
      line.baud(LINE_UART_SPEED);
      // line.attach(line_rx, Serial::RxIrq);
      imu.baud(IMU_UART_SPEED);
      imu.attach(&imu_rx);
      ui.baud(UI_UART_SPEED);
      // ui.attach(&ui_rx);
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
                  if (line.readable() == 1) {
                        line_rx();
                  }
                  if (cam.readable() == 1) {
                        cam_rx();
                  }
                  if (ui.readable() == 1) {
                        ui_rx();
                  }

                  if (mode == 0) {
                        Motor.free();
                        printf("%d", mode);
                  } else if (mode == 1) {
                        /*
                        if (line_left_val > 30) {
                              Motor.run(90, 90);
                        } else if (line_right_val > 30) {
                              Motor.run(-90, 90);
                        } else {
                              if (front_ball_x > 100) {
                                    if (Tof.val[8] > 100) {
                                          Motor.run(135, abs(100 - front_ball_x) * 2 + abs(100 - Tof.val[8]) * 2);
                                    } else {
                                          Motor.run(45, abs(100 - front_ball_x) * 2 + abs(100 - Tof.val[8]) * 2);
                                    }
                              } else {
                                    if (Tof.val[8] > 100) {
                                          Motor.run(-135, abs(100 - front_ball_x) * 2 + abs(100 - Tof.val[8]) * 2);
                                    } else {
                                          Motor.run(-45, abs(100 - front_ball_x) * 2 + abs(100 - Tof.val[8]) * 2);
                                    }
                              }
                        }*/
                        if (line_right_val > 65 || line_left_val > 65) {
                              Motor.run(90, 50);
                        } else {
                              Motor.run(-90, motor_rotation_num_avg < 2 ? 90 : 30);
                        }
                        // Motor.run(0, 0);
                  } else if (mode == 2) {
                        if (Tof.val[Tof.min_sensor()] < 100) {
                              Motor.run(Tof.safe_dir(), 40);
                        } else {
                              Motor.run(0, Tof.val[Tof.min_sensor()] / 4);
                        }
                  }
            }
      }
}

void line_rx() {
      const uint8_t recv_byte_num = 5;
      uint8_t recv_byte[recv_byte_num];

      for (int i = 0; i < recv_byte_num; i++) {
            recv_byte[i] = line.getc();   // 一旦すべてのデータを格納する
      }
      if (recv_byte[0] == 0xFF && recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
            motor_rotation_num_avg = recv_byte[1];
            line_left_val = recv_byte[2];
            line_right_val = recv_byte[3];
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

void imu_rx() {
      const uint8_t recv_byte_num = 4;
      uint8_t recv_byte[recv_byte_num];

      for (int i = 0; i < recv_byte_num; i++) {
            recv_byte[i] = imu.getc();   // 一旦すべてのデータを格納する
      }
      if (recv_byte[0] == 0xFF && recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
            uint8_t yaw_plus = recv_byte[1];
            uint8_t yaw_minus = recv_byte[2];

            yaw = yaw_plus == 0 ? yaw_minus * -1 : yaw_plus;
            yaw -= yaw_correction;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
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

void ui_rx() {
      int8_t item = 0;
      uint8_t dribbler_sig = 0;

      if (ui.getc() == 0xFF) {
            item = ui.getc() - 10;
            if (item == 0) {
                  mode = ui.getc();
            } else if (item == 1) {
                  if (ui.getc() == 1) {
                        yaw_correction = yaw + yaw_correction;
                  }
            } else if (item == -1) {
                  moving_speed = ui.getc();
            } else if (item == -2) {
                  dribbler_sig = ui.getc();
                  if (dribbler_sig == 1) {
                        Dribbler.f_hold(95);
                  } else if (dribbler_sig == 2) {
                        Dribbler.f_kick();
                  } else if (dribbler_sig == 3) {
                        Dribbler.b_hold(95);
                  } else if (dribbler_sig == 4) {
                        Dribbler.b_kick();
                  } else {
                        Dribbler.stop();
                  }
            }
      }

      if (item == 0 && mode == 0) {
            ui.putc(uint8_t(Voltage.get_voltage() * 10));
      } else if (item == 1) {
            uint8_t yaw_plus = yaw > 0 ? yaw : 0;
            uint8_t yaw_minus = yaw < 0 ? yaw * -1 : 0;

            ui.putc(0xFF);
            ui.putc(yaw_plus);
            ui.putc(yaw_minus);
      } else if (item == 2) {
            uint8_t safe_dir_plus = Tof.safe_dir() > 0 ? Tof.safe_dir() : 0;
            uint8_t safe_dir_minus = Tof.safe_dir() < 0 ? Tof.safe_dir() * -1 : 0;

            ui.putc(0xFF);
            ui.putc(safe_dir_plus);
            ui.putc(safe_dir_minus);
            ui.putc(Tof.min_sensor());
            for (uint8_t i = 0; i < 16; i++) {
                  ui.putc(Tof.val[i]);
            }
      } else if (item == 3) {
            ui.putc(motor_rotation_num_avg);
      }
}

void lidar_rx() {
      const uint8_t recv_byte_num = 18;
      uint8_t recv_byte[recv_byte_num];

      for (int i = 0; i < recv_byte_num; i++) {
            recv_byte[i] = lidar.getc();   // 一旦すべてのデータを格納する
      }
      if (recv_byte[0] == 0xFF && recv_byte[recv_byte_num - 1] == 0xAA) {   // ヘッダーとフッターがあることを確認
            for (uint8_t i = 0; i < 16; i++) {
                  Tof.val[i] = recv_byte[i + 1];
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

void cam_rx() {
      if (cam.getc() == 'H') {
            front_ball_x = cam.getc();
            front_ball_y = cam.getc();
            front_y_goal_x = cam.getc();
            front_y_goal_y = cam.getc();
            front_y_goal_size = cam.getc();
            front_b_goal_x = cam.getc();
            front_b_goal_y = cam.getc();
            front_b_goal_size = cam.getc();
      }
}