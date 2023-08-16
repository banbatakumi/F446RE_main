#include "voltage.h"

#include "mbed.h"

voltage::voltage(PinName battery_voltage_) : battery_voltage(battery_voltage_) {
      sampling_timer.start();
      battery_voltage_value = 10;
}

void voltage::read() {
      if (sampling_timer > 1) {
            battery_voltage_value = (battery_voltage.read_u16() * CHANGE_VOLTAGE) * (1 - RC) + battery_voltage_value * RC;   // 電圧のRCフィルタリング

            sampling_timer.reset();
      }
}

float voltage::get_battery_voltage() {
      return battery_voltage_value;
}