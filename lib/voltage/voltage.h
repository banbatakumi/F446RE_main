#ifndef MBED_VOLTAGE_H
#define MBED_VOLTAGE_H

#include "mbed.h"

#define RC 0.9
#define SAMPLE_CYCLE 0.5
#define CHANGE_VOLTAGE 0.0002125037

class voltage {
     public:
      voltage(PinName battery_voltage_);
      void read();
      float get_voltage();

     private:
      AnalogIn battery_voltage;

      float battery_voltage_val;

      Timer sampling_timer;
};

#endif