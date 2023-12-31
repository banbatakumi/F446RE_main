#include "lidar.h"

#include "mbed.h"

Lidar::Lidar(PinName tx_, PinName rx_, uint32_t serial_baud_) : serial(tx_, rx_) {
      serial.baud(serial_baud_);
      serial.attach(callback(this, &Lidar::Receive), Serial::RxIrq);

      for (uint8_t i = 0; i < TOF_QTY; i++) {   // それぞれのセンサにベクトルを与える
            unit_vector_x[i] = MyCos(i * 360.00000 / TOF_QTY);
            unit_vector_y[i] = MySin(i * 360.00000 / TOF_QTY);
      }
}

void Lidar::Receive() {
      //if (serial.readable()) {
            static uint8_t data_length;   // データの長さ
            const uint8_t recv_data_num = 7;
            static uint8_t recv_data[recv_data_num];

            if (data_length == 0) {   // ヘッダの受信
                  if (serial.getc() == 0xFF) {
                        data_length++;
                  } else {
                        data_length = 0;
                  }
            } else if (data_length == recv_data_num + 1) {
                  if (serial.getc() == 0xAA) {
                        val[0] = recv_data[0];
                        val[1] = recv_data[1];
                        val[2] = recv_data[2];
                        val[3] = recv_data[3];
                        val[4] = recv_data[4];
                        val[5] = recv_data[5];
                        val[6] = recv_data[6];
                  }

                  data_length = 0;
            } else {
                  recv_data[data_length - 1] = serial.getc();
                  data_length++;
            }
      //}
}

int16_t Lidar::SafeDir(uint16_t dis_limit_) {
      int16_t result_vector_x = 0;
      int16_t result_vector_y = 0;
      int16_t safe_dir;
      uint8_t tmp_val[TOF_QTY];

      for (uint8_t i = 0; i < TOF_QTY; i++) {   // 全てのセンサのベクトルを合成
            if (val[i] > dis_limit_) {
                  tmp_val[i] = dis_limit_;
            } else {
                  tmp_val[i] = val[i];
            }
      }
      for (uint8_t i = 0; i < TOF_QTY; i++) {   // 全てのセンサのベクトルを合成
            result_vector_x += tmp_val[i] * unit_vector_x[i];
            result_vector_y += tmp_val[i] * unit_vector_y[i];
      }
      safe_dir = atan2(result_vector_y, result_vector_x) * 180.00 / PI;

      return safe_dir;
}

int16_t Lidar::FrontSafeDir(uint16_t dis_limit_) {
      int16_t result_vector_x = 0;
      int16_t result_vector_y = 0;
      int16_t safe_dir;
      uint8_t tmp_val[TOF_QTY];

      for (uint8_t i = 0; i < TOF_QTY; i++) {   // 全てのセンサのベクトルを合成
            if (val[i] > dis_limit_) {
                  tmp_val[i] = dis_limit_;
            } else {
                  tmp_val[i] = val[i];
            }
      }
      for (uint8_t i = 0; i <= 2; i++) {   // 全てのセンサのベクトルを合成
            result_vector_x += tmp_val[i] * unit_vector_x[i];
            result_vector_y += tmp_val[i] * unit_vector_y[i];
      }
      for (uint8_t i = 14; i <= 15; i++) {   // 全てのセンサのベクトルを合成
            result_vector_x += tmp_val[i] * unit_vector_x[i];
            result_vector_y += tmp_val[i] * unit_vector_y[i];
      }
      safe_dir = atan2(result_vector_y, result_vector_x) * 180.00 / PI;

      return safe_dir;
}

uint8_t Lidar::FrontMinSensor() {
      uint8_t min_val_sensor_num = 0;
      uint8_t min_val = val[0];

      for (uint8_t i = 0; i <= 2; i++) {
            if (val[i] < min_val) {   // 最小値よりもval[i] の方が小さければ最小値を更新
                  min_val = val[i];
                  min_val_sensor_num = i;
            }
      }
      for (uint8_t i = 14; i <= 15; i++) {
            if (val[i] < min_val) {   // 最小値よりもval[i] の方が小さければ最小値を更新
                  min_val = val[i];
                  min_val_sensor_num = i;
            }
      }

      return min_val_sensor_num;
}

uint8_t Lidar::MinSensor() {
      uint8_t min_val_sensor_num = 0;
      uint8_t min_val = val[0];

      for (uint8_t i = 0; i < TOF_QTY; i++) {
            if (val[i] < min_val) {   // 最小値よりもval[i] の方が小さければ最小値を更新
                  min_val = val[i];
                  min_val_sensor_num = i;
            }
      }

      return min_val_sensor_num;
}