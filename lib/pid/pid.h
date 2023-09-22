#ifndef MBED_PID_H
#define MBED_PID_H

#include "mbed.h"
#include "simplify_deg.h"

class PID {
     public:
      PID();
      void SetGain(float kp, float ki, float kd);
      void SetSamplingPeriod(float sampling_period);
      void SetLimit(float limit);

      void Compute(float input, float target);
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