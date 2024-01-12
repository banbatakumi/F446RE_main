#include "kicker.h"

#include "mbed.h"

Kicker::Kicker(PinName sig_1_, PinName sig_2_) : sig_1(sig_1_), sig_2(sig_2_) {
      sig_1 = 0;
      sig_2 = 0;

      chargeTimer.start();
}

void Kicker::SetPower(uint8_t power_) {
      this->power = power_ * 1000;

      if (power > 250000) power = 250000;
}

void Kicker::Kick() {
      if (chargeTimer.read() > 0.5) {
            if (enable == 0) {
                  ChargeTimeout.attach_us(mbed::callback(this, &Kicker::ChargeOff), power * 0.5);
                  FlipTimeout.attach_us(mbed::callback(this, &Kicker::FlipOff), power);
                  sig_1 = 1;
                  sig_2 = 1;

                  enable = 1;
            }
      } else {
            sig_1 = 1;
            sig_2 = 0;
      }
}

void Kicker::ChargeOff() {
      sig_1 = 0;
      sig_2 = 1;
}

void Kicker::FlipOff() {
      sig_1 = 1;
      sig_2 = 0;
      enable = 0;
      chargeTimer.reset();
}