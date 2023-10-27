#include "imu.h"

#include "mbed.h"

Imu::Imu(PinName tx_, PinName rx_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Imu::Receive), Serial::RxIrq);
}

void Imu::Receive() {
      static uint8_t data_length;   // データの長さ
      static uint8_t dir_plus;
      static uint8_t dir_minus;

      if (data_length == 0) {   // ヘッダの受信
            uint8_t head = serial.getc();
            if (head == 0xFF) {
                  data_length += 1;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 1) {
            dir_plus = serial.getc();
            data_length += 1;
      } else if (data_length == 2) {
            dir_minus = serial.getc();

            dir = SimplifyDeg((dir_plus == 0 ? dir_minus * -1 : dir_plus) - dir_correction);
            data_length = 0;
      }
}

int16_t Imu::GetDir() {
      return dir;
}

void Imu::SetZero() {
      dir_correction += dir;
}