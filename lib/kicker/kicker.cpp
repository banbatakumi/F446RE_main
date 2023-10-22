#include "kicker.h"

#include "mbed.h"

Kicker::Kicker(PinName sig_1_, PinName sig_2_) : sig_1(sig_1_), sig_2(sig_2_) {
      sig_1 = 0;
      sig_2 = 0;

      chargeTimer.start();
}

void Kicker::SetPower(uint8_t power_) {
      this->power = power_ * 1000;

      if (power > 100000) power = 100000;
}

void Kicker::Kick() {
      if (chargeTimer.read() > 0.25) {
            kickerTimeout.attach_us(mbed::callback(this, &Kicker::flipOff), power);

            sig_1 = 1;
            sig_2 = 1;
      } else {
            sig_1 = 1;
            sig_2 = 0;
      }
}

void Kicker::flipOff() {
      sig_1 = 1;
      sig_2 = 0;

      chargeTimer.reset();
}