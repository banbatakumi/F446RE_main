#include "kicker.h"

#include "mbed.h"

Kicker::Kicker(PinName sig_1_, PinName sig_2_) : sig_1(sig_1_), sig_2(sig_2_) {
      sig_1 = 0;
      sig_2 = 0;

      chargeTimer.start();
}

void Kicker::Kick() {
      if (readms(chargeTimer) > 500) {
            if (enable == 0) {
                  ChargeTimeout.attach(mbed::callback(this, &Kicker::ChargeOff), CHARGE_OFF_TIME);
                  FlipTimeout.attach(mbed::callback(this, &Kicker::FlipOff), KICK_TIME);
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