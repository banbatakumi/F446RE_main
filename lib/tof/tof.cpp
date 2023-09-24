#include "tof.h"

#include "mbed.h"

Tof::Tof() {
      for (uint8_t i = 0; i < TOF_QTY; i++) {   // それぞれのセンサにベクトルを与える
            unit_vector_x[i] = cos((i * 360.00000 / TOF_QTY) * PI / 180.00000);
            unit_vector_y[i] = sin((i * 360.00000 / TOF_QTY) * PI / 180.00000);
      }
}

int16_t Tof::SafeDir() {
      int16_t result_vector_x = 0;
      int16_t result_vector_y = 0;
      int16_t tmp_safe_dir;

      for (uint8_t i = 0; i < TOF_QTY; i++) {   // 全てのセンサのベクトルを合成
            result_vector_x += val[i] * unit_vector_x[i];
            result_vector_y += val[i] * unit_vector_y[i];
      }
      tmp_safe_dir = MyAtan2(result_vector_y, result_vector_x);

      return tmp_safe_dir;
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

uint8_t Tof::FindWall() {
      int16_t wall;
      int16_t min_wall;
      int16_t min_wall_num;

      min_wall = abs(val[2] - MyCos(45) * val[0]) + abs(val[2] - MyCos(22.5) * val[1]) + abs(val[2] - MyCos(22.5) * val[3]) + abs(val[2] - MyCos(45) * val[4]);
      for (uint8_t i = 0; i < TOF_QTY; i++) {
            uint8_t i_1 = i + 1;
            uint8_t i_2 = i + 2;
            uint8_t i_3 = i + 3;
            uint8_t i_4 = i + 4;
            if (i_1 > 15) i_1 -= 16;
            if (i_2 > 15) i_2 -= 16;
            if (i_3 > 15) i_3 -= 16;
            if (i_4 > 15) i_4 -= 16;

            wall = abs(val[i_2] - MyCos(45) * val[i]) + abs(val[i_2] - MyCos(22.5) * val[i_1]) + abs(val[i_2] - MyCos(22.5) * val[i_3]) + abs(val[i_2] - MyCos(45) * val[i_4]);
            if (wall < min_wall) {   // 最小値よりもval[i] の方が小さければ最小値を更新
                  min_wall = wall;
                  min_wall_num = i_2;
            }
      }

      return min_wall_num;
}