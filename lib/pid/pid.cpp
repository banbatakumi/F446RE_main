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
      this->sampling_period = sampling_period_ * 1000;
}

void PID::SetLimit(float limit_) {
      this->limit = limit_;
}

void PID::SelectType(uint8_t type_) {
      this->type = type_;
}

void PID::Compute(float input_, float target_) {
      if (readms(sampling_timer) > sampling_period) {
            if (type == 0) {                                                    // 普通のやつ
                  p = target_ - input_;                                         // 比例
                  d = (p - pre_p) / (readms(sampling_timer) / 1000.0f);         // 微分
                  i += (p + pre_p) * (readms(sampling_timer) / 1000.0f) * 0.5;  // 台形積分
                  if ((i > 0 && input_ > 0) || (i < 0 && input_ < 0)) i = 0;
                  if (abs(i) > limit) i = limit * (i / abs(i));
                  pre_p = p;

                  pid = p * kp + i * ki + d * kd;
            } else if (type == 1) {                                               // 微分先行型
                  p = target_ - input_;                                           // 比例
                  d = (input_ - pre_input) / (readms(sampling_timer) / 1000.0f);  // 微分
                  i += (p + pre_p) * (readms(sampling_timer) / 1000.0f) * 0.5;    // 台形積分
                  if ((i > 0 && input_ > 0) || (i < 0 && input_ < 0)) i = 0;
                  if (abs(i) > limit) i = limit * (i / abs(i));
                  pre_p = p;
                  pre_input = input_;

                  pid = p * kp + i * ki + d * kd * -1;
            }

            if (abs(pid) > limit) {
                  pid = limit * (abs(pid) / pid);
            }

            sampling_timer.reset();
      }
}

float PID::Get() {
      return pid;
}