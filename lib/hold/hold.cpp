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
      bool is_catch = 0;
      if (rc_val < CATCH_TH) is_catch = 1;   // キャッチしたかの判定

      return is_catch;
}