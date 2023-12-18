#ifndef MBED_VOLTAGE_H
#define MBED_VOLTAGE_H

#include "mbed.h"

#define RC 0.5
#define SAMPLE_CYCLE 0.5
#define CHANGE_VOLTAGE 0.0002125037

class Voltage {
     public:
      Voltage(PinName battery_voltage_);
      void Read();
      float Get();

     private:
      AnalogIn battery_voltage;

      float battery_voltage_val;

      Timer sampling_timer;
};

#endif