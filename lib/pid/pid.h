#ifndef MBED_PID_H
#define MBED_PID_H

#include "mbed.h"
#include "simplify_deg.h"

#define readms(timer_name_) chrono::duration_cast<chrono::milliseconds>((timer_name_).elapsed_time()).count()

class PID {
     public:
      PID();

#define PID_TYPE 0
#define PI_D_TYPE 1

      void SetGain(float kp_, float ki_, float kd_);
      void SetSamplingPeriod(float sampling_period_ = 0.01);
      void SetLimit(float limit_ = 100);
      void SelectType(uint8_t type_ = PID_TYPE);

      void Compute(float input_, float target_);
      float Get();

     private:
      float p, i, d;
      float pre_p;
      float kp, ki, kd;
      float pid;
      float pre_input;

      uint16_t sampling_period;
      float limit;

      uint8_t type;

      Timer sampling_timer;
};

#endif