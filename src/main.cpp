#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "voltage.h"

// UART通信定義 (TX, RX)
Serial line(PA_2, PA_3);
Serial imu(PA_9, PA_10);
Serial ui(PC_10, PC_11);
Serial lidar(PC_12, PD_2);
Serial cam(PA_0, PA_1);

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

DigitalOut led_1(PA_5);
DigitalOut led_2(PA_6);

// グローバル変数定義
int16_t yaw = 0;
uint8_t tof_value[16];
uint8_t mode = 0;

uint8_t yaw_plus, yaw_minus;

Timer test;

int main() {
      // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200

      // line.baud(38400);
      // line.attach(line_rx, Serial::RxIrq);
      imu.baud(38400);
      imu.attach(imu_rx, Serial::RxIrq);
      ui.baud(38400);
      ui.attach(ui_rx, Serial::RxIrq);
      lidar.baud(9600);
      lidar.attach(lidar_rx, Serial::RxIrq);
      // cam.baud(38400);
      // cam.attach(cam_rx, Serial::RxIrq);

      wait_us(10000000);
      test.start();

      Motor.set_pwm();
      Dribbler.set_pwm();

      while (1) {
            Voltage.read();
            Motor.yaw = yaw;
            if (mode == 0) {
                  Motor.free();
            } else if (mode == 1) {
                  Motor.run(0, 0);
            } else if (mode == 2) {
                  uint8_t a = 16;
                  for (int i = 0; i < 16; i++) {
                        if (tof_value[i] < 75) a = i;
                  }
                  if (a != 16) {
                        Motor.run(a * 22.5 + 180, 30);
                  } else {
                        Motor.run(0, 30);
                  }
            }
      }
}

void line_rx() {
}

void imu_rx() {
      if (imu.getc() == 'a') {
            yaw_plus = imu.getc();
            yaw_minus = imu.getc();

            yaw = yaw_plus == 0 ? yaw_minus * -1 : yaw_plus;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
      }
}

void ui_rx() {
      uint8_t item = 0;
      if (ui.getc() == 'H') {
            item = ui.getc();
            if (item == 0) {
                  mode = ui.getc();
            }
      }

      if (item == 1) {
            ui.putc('H');
            ui.putc(yaw_plus);
            ui.putc(yaw_minus);
      } else if (item == 2) {
            ui.putc('H');
            for (int i = 0; i < 16; i++) {
                  ui.putc(tof_value[i]);
            }
      }
}

void lidar_rx() {
      if (lidar.getc() == 'H') {
            for (int i = 0; i < 16; i++) {
                  tof_value[i] = lidar.getc();
            }
      }
}

void cam_rx() {
}