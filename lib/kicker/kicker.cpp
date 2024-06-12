#include "kicker.h"

#include "mbed.h"

Kicker::Kicker(PinName capaciter_, PinName solenoid_) : capaciter(capaciter_), solenoid(solenoid_) {
      capaciter = 0;
      solenoid = 0;

      // charge.attach(mbed::callback(this, &Kicker::FirstCharge), CHARGE_PWM_PERIOD);
      // chargeTimer.start();
      // chargeTimer.reset();

      enable_kick = 1;
}

void Kicker::Kick() {
      if (enable_kick == 1) {
            deadTimeout.attach(mbed::callback(this, &Kicker::FlipOn), DEAD_TIME);
            capaciter = 0;
            solenoid = 0;

            enable_kick = 0;
      }
}

void Kicker::FlipOn() {
      capaciter = 0;
      solenoid = 1;
      flipTimeout.attach(mbed::callback(this, &Kicker::FlipOff), KICK_TIME);
}

void Kicker::FlipOff() {
      deadTimeout.attach(mbed::callback(this, &Kicker::ChargeStart), DEAD_TIME);
      capaciter = 0;
      solenoid = 0;
}

void Kicker::ChargeStart() {
      charge.attach(mbed::callback(this, &Kicker::Charge), CHARGE_PWM_PERIOD);
      chargeTimer.start();
      chargeTimer.reset();
}

void Kicker::Charge() {
      if (charge_count > 50) {
            capaciter = 1;
            charge_count = 0;
      } else {
            if (charge_count == 0) capaciter = 0;
            charge_count++;
      }
      if (readms(chargeTimer) > 1000) {
            enable_kick = 1;
            capaciter = 1;
            charge.detach();
      }
}

void Kicker::FirstCharge() {
      if (readms(chargeTimer) > 2000) {
            enable_kick = 1;
            capaciter = 1;
            charge.detach();
      }
      if (charge_count > 100) {
            capaciter = 1;
            charge_count = 0;
      } else {
            if (charge_count == 0) capaciter = 0;
            charge_count++;
      }
}