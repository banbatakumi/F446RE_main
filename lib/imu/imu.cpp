#include "imu.h"

#include "mbed.h"

Imu::Imu(PinName tx_, PinName rx_, uint32_t serial_baud_) : serial(tx_, rx_) {
      serial.baud(serial_baud_);
      serial.attach(callback(this, &Imu::Receive), SerialBase::RxIrq);
}

void Imu::Receive() {
      static uint8_t data_length;  // データの長さ
      const uint8_t recv_data_num = 6;
      static uint8_t recv_data[recv_data_num];
      static uint8_t read_byte;
      static uint8_t yaw_H;
      static uint8_t yaw_L;
      static uint8_t pitch_H;
      static uint8_t pitch_L;
      static uint8_t roll_H;
      static uint8_t roll_L;
      serial.read(&read_byte, 1);

      if (data_length == 0) {  // ヘッダの受信
            if (read_byte == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (read_byte == 0xAA) {
                  yaw_H = recv_data[0];
                  yaw_L = recv_data[1];
                  pitch_H = recv_data[2];
                  pitch_L = recv_data[3];
                  roll_H = recv_data[4];
                  roll_L = recv_data[5];

                  yaw = ((((uint16_t)yaw_H << 8) & 0xFF00) | ((int16_t)yaw_L & 0x00FF)) * 0.1 - 180;
                  pitch = ((((uint16_t)pitch_H << 8) & 0xFF00) | ((int16_t)pitch_L & 0x00FF)) * 0.1 - 180;
                  roll = ((((uint16_t)roll_H << 8) & 0xFF00) | ((int16_t)roll_L & 0x00FF)) * 0.1 - 180;
                  yaw = SimplifyDegFloat(yaw - yaw_correction);
            }

            data_length = 0;
      } else {
            recv_data[data_length - 1] = read_byte;
            data_length++;
      }
}

float Imu::GetYaw() {
      return yaw;
}
float Imu::GetPitch() {
      return pitch;
}
float Imu::GetRoll() {
      return roll;
}

void Imu::YawSetZero() {
      yaw_correction += yaw;
}