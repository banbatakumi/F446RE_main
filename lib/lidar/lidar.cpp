#include "lidar.h"

#include "mbed.h"

Lidar::Lidar(PinName tx_, PinName rx_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Lidar::Receive), Serial::RxIrq);

      for (uint8_t i = 0; i < TOF_QTY; i++) {   // それぞれのセンサにベクトルを与える
            unit_vector_x[i] = MyCos(i * 360.00000 / TOF_QTY);
            unit_vector_y[i] = MySin(i * 360.00000 / TOF_QTY);
      }
}

void Lidar::Receive() {
      static uint8_t data_length;   // データの長さ

      if (data_length == 0) {   // ヘッダの受信
            uint8_t head = serial.getc();
            if (head == 0xFF) {
                  data_length += 1;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 1) {
            val[0] = serial.getc();
            data_length += 1;
      } else if (data_length == 2) {
            val[1] = serial.getc();
            data_length += 1;
      } else if (data_length == 3) {
            val[2] = serial.getc();
            data_length += 1;
      } else if (data_length == 4) {
            val[3] = serial.getc();
            data_length += 1;
      } else if (data_length == 5) {
            val[4] = serial.getc();
            data_length += 1;
      } else if (data_length == 6) {
            val[5] = serial.getc();
            data_length += 1;
      } else if (data_length == 7) {
            val[6] = serial.getc();
            data_length += 1;
      } else if (data_length == 8) {
            val[7] = serial.getc();
            data_length += 1;
      } else if (data_length == 9) {
            val[8] = serial.getc();
            data_length += 1;
      } else if (data_length == 10) {
            val[9] = serial.getc();
            data_length += 1;
      } else if (data_length == 11) {
            val[10] = serial.getc();
            data_length += 1;
      } else if (data_length == 12) {
            val[11] = serial.getc();
            data_length += 1;
      } else if (data_length == 13) {
            val[12] = serial.getc();
            data_length += 1;
      } else if (data_length == 14) {
            val[13] = serial.getc();
            data_length += 1;
      } else if (data_length == 15) {
            val[14] = serial.getc();
            data_length += 1;
      } else if (data_length == 16) {
            val[15] = serial.getc();
            data_length = 0;
      }
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