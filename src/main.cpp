#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "tof.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率

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
tof Tof;

DigitalOut led_1(PA_5);
DigitalOut led_2(PA_6);

// グローバル変数定義
int16_t yaw = 0, yaw_correction = 0;
uint8_t tof_value[16];
uint8_t encoder[4];
uint8_t mode = 0;
uint8_t moving_speed;
uint8_t test;

Timer test_t;

int main() {
      // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200

      line.baud(14400);
      line.attach(line_rx, Serial::RxIrq);
      imu.baud(57600);
      imu.attach(imu_rx, Serial::RxIrq);
      ui.baud(57600);
      ui.attach(ui_rx, Serial::RxIrq);
      lidar.baud(57600);
      lidar.attach(lidar_rx, Serial::RxIrq);
      // cam.baud(38400);
      // cam.attach(cam_rx, Serial::RxIrq);

      Motor.set_pwm();
      Dribbler.set_pwm();
      float speed = 20;
      test_t.start();

      while (1) {
            Voltage.read();
            Motor.yaw = yaw;

            if (mode == 0) {
                  Motor.free();
                  // Motor.run(0, 0);
            } else if (mode == 1) {
                  if (test_t.read() > 0.01) {
                        speed -= ((test - 5) * 0.5);

                        test_t.reset();
                  }

                  if (speed < 0) speed = 0;
                  if (speed > 100) speed = 100;
                  Motor.run(0, speed);
            } else if (mode == 2) {
                  if (Tof.value[Tof.min_sensor()] < 100) {
                        Motor.run(Tof.min_sensor() * 22.5 - 180, (200 - Tof.value[Tof.min_sensor()]) / 3);
                  } else {
                        Motor.run(0, 0);
                  }
            }
      }
}

void line_rx() {
      if (line.getc() == 'H') {
            encoder[0] = line.getc();
            encoder[1] = line.getc();
            encoder[2] = line.getc();
            encoder[3] = line.getc();
      }
}

void imu_rx() {
      if (imu.getc() == 'H') {
            uint8_t yaw_plus = imu.getc();
            uint8_t yaw_minus = imu.getc();

            yaw = yaw_plus == 0 ? yaw_minus * -1 : yaw_plus;
            yaw -= yaw_correction;
            yaw -= yaw > 180 ? 360 : (yaw < -180 ? -360 : 0);
      }
}

void ui_rx() {
      int8_t item = 0;
      uint8_t dribbler_sig = 0;

      if (ui.getc() == 'H') {
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
                        Dribbler.f_hold(80);
                  } else if (dribbler_sig == 2) {
                        Dribbler.f_kick();
                  } else if (dribbler_sig == 3) {
                        Dribbler.b_hold(80);
                  } else if (dribbler_sig == 4) {
                        Dribbler.b_kick();
                  } else {
                        Dribbler.stop();
                  }
            }
      }

      if (item == 0) {
            ui.putc(encoder[0]);
      } else if (item == 1) {
            uint8_t yaw_plus = yaw > 0 ? yaw : 0;
            uint8_t yaw_minus = yaw < 0 ? yaw * -1 : 0;
            ui.putc('H');
            ui.putc(yaw_plus);
            ui.putc(yaw_minus);
      } else if (item == 2) {
            uint8_t safe_angle_plus = Tof.safe_angle() > 0 ? Tof.safe_angle() : 0;
            uint8_t safe_angle_minus = Tof.safe_angle() < 0 ? Tof.safe_angle() * -1 : 0;
            ui.putc('H');
            ui.putc(safe_angle_plus);
            ui.putc(safe_angle_minus);
            ui.putc(Tof.min_sensor());
            for (int i = 0; i < 16; i++) {
                  ui.putc(Tof.value[i]);
            }
      }
}

void lidar_rx() {
      if (lidar.getc() == 'H') {
            for (int i = 0; i < 16; i++) {
                  Tof.value[i] = lidar.getc();
            }
      }
}

void cam_rx() {
}