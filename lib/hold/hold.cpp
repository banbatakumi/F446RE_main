#include "hold.h"

#include "mbed.h"

Hold::Hold(PinName sensor_) : sensor(sensor_) {
}

void Hold::Read() {
      val = sensor.read_u16() / 256;

      rc_val = rc_val * RC + val * (1 - RC);
}

uint8_t Hold::GetVal() {
      return rc_val;
}

bool Hold::IsHold() {
      bool is_catch = 1;
      if (rc_val > th) {
            if (off_cnt < 100) off_cnt++;
      } else {
            off_cnt = 0;
      }
      if (off_cnt >= 100) is_catch = 0;
      return is_catch;
}

void Hold::SetTh() {
      th = 0;
      for (uint16_t i = 0; i < SET_TH_NUM; i++) {
            Read();
            th += rc_val;
      }
      th /= SET_TH_NUM;
      th -= 30;
}