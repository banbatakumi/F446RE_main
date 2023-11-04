#include "imu.h"

#include "mbed.h"

Imu::Imu(PinName tx_, PinName rx_, uint32_t serial_baud_) : serial(tx_, rx_) {
      serial.baud(serial_baud_);
      serial.attach(callback(this, &Imu::Receive), Serial::RxIrq);
}

void Imu::Receive() {
      static uint8_t data_length;   // データの長さ
      static uint8_t recv_data[2];
      static uint8_t dir_plus;
      static uint8_t dir_minus;

      if (data_length == 0) {   // ヘッダの受信
            if (serial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 3) {
            if (serial.getc() == 0xAA) {
                  dir_plus = recv_data[0];
                  dir_minus = recv_data[1];

                  dir = SimplifyDeg((dir_plus == 0 ? dir_minus * -1 : dir_plus) - dir_correction);
            }

            data_length = 0;
      } else {
            recv_data[data_length - 1] = serial.getc();
            data_length++;
      }
}

int16_t Imu::GetDir() {
      return dir;
}

void Imu::SetZero() {
      dir_correction += dir;
}