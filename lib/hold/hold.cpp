#include "hold.h"

#include "mbed.h"

hold::hold(PinName front_, PinName back_) : front(front_), back(back_) {
}

void hold::read() {
      front_value = front.read_u16() / 256;
      back_value = back.read_u16() / 256;

      // RCフィルタリング
      front_rc_value = front_value * (1 - RC) + front_rc_value * RC;
      back_rc_value = back_value * (1 - RC) + back_rc_value * RC;
}

uint8_t hold::front_read() {
      return front_rc_value;
}

uint8_t hold::back_read() {
      return back_rc_value;
}