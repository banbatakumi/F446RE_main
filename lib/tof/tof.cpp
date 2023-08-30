#include "tof.h"

#include "mbed.h"

tof::tof() {
      for (uint8_t i = 0; i < TOF_QTY; i++) {
            unit_vector_x[i] = cos((i * 360.00000 / TOF_QTY) * PI / 180.00000);
            unit_vector_y[i] = sin((i * 360.00000 / TOF_QTY) * PI / 180.00000);
      }
}

int16_t tof::safe_angle() {
      int16_t result_vector_x = 0;
      int16_t result_vector_y = 0;
      int16_t tmp_safe_angle;

      for (uint8_t i = 0; i < TOF_QTY; i++) {
            result_vector_x += value[i] * unit_vector_x[i];
            result_vector_y += value[i] * unit_vector_y[i];
      }
      tmp_safe_angle = atan2(result_vector_y, result_vector_x) / PI * 180.00000;
      return tmp_safe_angle;
}

uint8_t tof::min_sensor() {
      uint8_t min_value_sensor = 0;
      uint8_t min_value = value[0];

      for (uint8_t i = 0; i < TOF_QTY; i++) {
            if (value[i] < min_value) {
                  min_value = value[i];   // 最小値よりもvalue[i] の方が小さければ最小値を更新
                  min_value_sensor = i;
            }
      }

      return min_value_sensor;
}