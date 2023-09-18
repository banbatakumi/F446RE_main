#include "hold.h"

#include "mbed.h"

hold::hold(PinName front_sensor_, PinName back_sensor_) : front_sensor(front_sensor_), back_sensor(back_sensor_) {
}

void hold::read() {
      front_val = front_sensor.read_u16() / 256;
      back_val = back_sensor.read_u16() / 256;

      front_rc_val = front_rc_val * RC + front_val * (1 - RC);
      back_rc_val = back_rc_val * RC + back_val * (1 - RC);
}

uint8_t hold::get_front_val() {
      return front_rc_val;
}

uint8_t hold::get_back_val() {
      return back_rc_val;
}

bool hold::is_front() {
      bool is_catch = 0;
      if (front_rc_val < CATCH_TH) {
            is_catch = 1;
      }
      return is_catch;
}

bool hold::is_back() {
      bool is_catch = 0;
      if (back_rc_val < CATCH_TH) {
            is_catch = 1;
      }
      return is_catch;
}