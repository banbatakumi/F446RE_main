#include "pid.h"

#include "mbed.h"

PID::PID() {
      sampling_period = 0.01;
      sampling_timer.start();
}

void PID::SetGain(float kp_, float ki_, float kd_) {
      this->kp = kp_;
      this->ki = ki_;
      this->kd = kd_;
}

void PID::SetSamplingPeriod(float sampling_period_) {
      this->sampling_period = sampling_period_;
}

void PID::SetLimit(float limit_) {
      this->limit = limit_;
}

void PID::Compute(float input_, float target_) {
      if (sampling_timer.read() > sampling_period) {
            pre_p = p;
            p = SimplifyDeg(target_ - input_);   // 比例
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