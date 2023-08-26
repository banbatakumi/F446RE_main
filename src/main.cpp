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
int16_t yaw;

Timer test;

int main() {
      // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200

      line.baud(38400);
      line.attach(line_rx, Serial::RxIrq);
      imu.baud(38400);
      // imu.attach(imu_rx, Serial::RxIrq);
      ui.baud(38400);
      ui.attach(ui_rx, Serial::RxIrq);
      lidar.baud(38400);
      lidar.attach(lidar_rx, Serial::RxIrq);
      cam.baud(38400);
      cam.attach(cam_rx, Serial::RxIrq);
      test.start();

      Motor.set_pwm();
      Dribbler.set_pwm();

      while (1) {
            Voltage.read();
            Motor.yaw = yaw;
            Dribbler.b_hold(100);
      }
}

void line_rx() {
}

void imu_rx() {
}

void ui_rx() {
}

void lidar_rx() {
}

void cam_rx() {
}