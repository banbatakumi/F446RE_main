#include "pid.h"

#include "mbed.h"

PID::PID() {
      sampling_period = 0.01;
      sampling_timer.start();
}

void PID::SetGain(float kp, float ki, float kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
}

void PID::SetSamplingPeriod(float sampling_period) {
      this->sampling_period = sampling_period;
}

void PID::SetLimit(float limit) {
      this->limit = limit;
}

void PID::Compute(float input, float target) {
      if (sampling_timer.read() > sampling_period) {
            pre_p = p;
            p = SimplifyDeg(target - input);   // 比例
            d = (p - pre_p) * sampling_timer.read();   // 微分
            i += (p + pre_p) * sampling_timer.read();

            pid = p * kp + i * ki + d * kd;
            if(abs(pid) > limit){
                  pid = limit * (abs(pid) / pid);
            }

            sampling_timer.reset();
      }
}

float PID::Get() {
      return pid;
}