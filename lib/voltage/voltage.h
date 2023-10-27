#ifndef MBED_VOLTAGE_H
#define MBED_VOLTAGE_H

#include "mbed.h"

#define RC 0.9
#define SAMPLE_CYCLE 0.5
#define CHANGE_VOLTAGE 0.0002125037

class Voltage {
     public:
      Voltage(PinName battery_voltage_);
      void Read();
      float Get();
      bool IsLowVoltage();
      void SetLowVoltageTh(float low_voltage_th_ = 6.0);

     private:
      AnalogIn battery_voltage;

      float battery_voltage_val;
      float low_voltage_th;

      Timer sampling_timer;
};

#endif