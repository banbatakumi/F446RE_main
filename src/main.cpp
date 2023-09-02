#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "tof.h"
#include "voltage.h"

#define PI 3.1415926535   // 円周率
#define CUT_VOLTAGE 6.0
#define VOLTAGE_COUNT_NUM 1000

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
uint8_t tof_val[16];
uint8_t motor_rotaion_number[4];
uint8_t motor_rotation_num_avg;
uint8_t mode = 0;
uint8_t moving_speed;

bool voltage_decrease = 0;
uint16_t voltage_count;

Timer test;

int main() {
      // 通信速度: 9600, 14400, 19200, 28800, 38400, 57600, 115200

      line.baud(38400);
      // line.attach(line_rx, Serial::RxIrq);
      imu.baud(57600);
      imu.attach(imu_rx, Serial::RxIrq);
      ui.baud(38400);
      ui.attach(ui_rx, Serial::RxIrq);
      lidar.baud(38400);
      lidar.attach(lidar_rx, Serial::RxIrq);
      cam.baud(38400);
      // cam.attach(cam_rx, Serial::RxIrq);

      Motor.set_pwm();
      Dribbler.set_pwm();

      while (1) {
            Voltage.read();

            if (Voltage.get_voltage() < CUT_VOLTAGE) {
                  voltage_count++;
            } else {
                  voltage_count = 0;
            }
            if (voltage_count >= VOLTAGE_COUNT_NUM) {
                  voltage_decrease = 1;
            }
            if (voltage_decrease == 1) {
                  Motor.free();
                  ui.putc('E');
                  ui.putc('R');
            } else {
                  Motor.yaw = yaw;
                  Motor.rotation_num_avg = motor_rotation_num_avg;
                  line_rx();

                  if (mode == 0) {
                        Motor.free();
                        // Motor.run(0, 0, 0);
                  } else if (mode == 1) {
                        Motor.run(1, 0, 15);
                  } else if (mode == 2) {
                        if (Tof.val[Tof.min_sensor()] < 100) {
                              Motor.run(0, Tof.min_sensor() * 22.5 - 180, (200 - Tof.val[Tof.min_sensor()]) / 3);
                        } else {
                              Motor.run(0, 0, 0);
                        }
                  }
            }
      }
}

void line_rx() {
      if (line.getc() == 'H') {
            for (int i = 0; i < 4; i++) {
                  motor_rotaion_number[i] = line.getc();
            }
      }
      motor_rotation_num_avg = 0;
      for (int i = 0; i < 3; i++) {
            motor_rotation_num_avg += motor_rotaion_number[i] / 3;
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
                        Dribbler.f_hold(90);
                  } else if (dribbler_sig == 2) {
                        Dribbler.f_kick();
                  } else if (dribbler_sig == 3) {
                        Dribbler.b_hold(90);
                  } else if (dribbler_sig == 4) {
                        Dribbler.b_kick();
                  } else {
                        Dribbler.stop();
                  }
            }
      }

      if (item == 0 && mode == 0) {
            ui.putc('H');
            ui.putc(uint8_t(Voltage.get_voltage() * 10));
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
            for (uint8_t i = 0; i < 16; i++) {
                  ui.putc(Tof.val[i]);
            }
      }
}

void lidar_rx() {
      if (lidar.getc() == 'H') {
            for (uint8_t i = 0; i < 16; i++) {
                  Tof.val[i] = lidar.getc();
            }
      }
}

void cam_rx() {
}