#ifndef MBED_PID_H
#define MBED_PID_H

#include "mbed.h"
#include "simplify_deg.h"

class PID {
     public:
      PID();
      void SetGain(float kp_, float ki_, float kd_);
      void SetSamplingPeriod(float sampling_period_);
      void SetLimit(float limit_);

      void Compute(float input_, float target_);
      float Get();

     private:
      float p, i, d;
      float pre_p;
      float kp, ki, kd;
      float pid;
      float sampling_period;
      float limit;

      Timer sampling_timer;
};

#endif