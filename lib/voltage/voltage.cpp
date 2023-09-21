#include "voltage.h"

#include "mbed.h"

Voltage::Voltage(PinName battery_voltage_) : battery_voltage(battery_voltage_) {
      sampling_timer.start();
      battery_voltage_val = 8.4;
}

void Voltage::Read() {
      if (sampling_timer > SAMPLE_CYCLE) {
            battery_voltage_val = (battery_voltage.read_u16() * CHANGE_VOLTAGE) * (1 - RC) + battery_voltage_val * RC;   // 電圧のRCフィルタリング

            sampling_timer.reset();
      }
}

float Voltage::Get() {
      return battery_voltage_val;
}