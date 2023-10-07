#include "kicker.h"

#include "mbed.h"

Kicker::Kicker(PinName sig_1_, PinName sig_2_) : sig_1(sig_1_), sig_2(sig_2_) {
      sig_1 = 0;
      sig_2 = 0;
}

void Kicker::SetKickTime(uint16_t kick_time_) {
      this->kick_time = kick_time_ * 1000;   // msからusへの変換
}

void Kicker::Kick() {
      sig_1 = 1;
      sig_2 = 1;
      wait_us(kick_time);
      sig_1 = 0;
      sig_2 = 0;
}