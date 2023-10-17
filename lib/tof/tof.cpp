#include "tof.h"

#include "mbed.h"

Tof::Tof() {
      for (uint8_t i = 0; i < TOF_QTY; i++) {   // それぞれのセンサにベクトルを与える
            unit_vector_x[i] = MyCos(i * 360.00000 / TOF_QTY);
            unit_vector_y[i] = MySin(i * 360.00000 / TOF_QTY);
      }
}

int16_t Tof::SafeDir(uint16_t dis_limit_) {
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

int16_t Tof::FrontSafeDir(uint16_t dis_limit_) {
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

uint8_t Tof::FrontMinSensor() {
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

uint8_t Tof::MinSensor() {
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