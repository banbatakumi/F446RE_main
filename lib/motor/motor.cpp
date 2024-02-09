#include "motor.h"

#include "mbed.h"

Motor::Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_, int16_t *own_dir_)
    : motor_1_a(motor_1_a_), motor_1_b(motor_1_b_), motor_2_a(motor_2_a_), motor_2_b(motor_2_b_), motor_3_a(motor_3_a_), motor_3_b(motor_3_b_), motor_4_a(motor_4_a_), motor_4_b(motor_4_b_) {
      motor_1_a = 0;
      motor_1_b = 0;
      motor_2_a = 0;
      motor_2_b = 0;
      motor_3_a = 0;
      motor_3_b = 0;
      motor_4_a = 0;
      motor_4_b = 0;

      this->own_dir = own_dir_;

      motor1Ave.SetLength(MOVING_AVE_NUM);
      motor2Ave.SetLength(MOVING_AVE_NUM);
      motor3Ave.SetLength(MOVING_AVE_NUM);
      motor4Ave.SetLength(MOVING_AVE_NUM);

      addPowerTimer.start();
}

void Motor::Run(int16_t moving_dir_, uint16_t moving_speed_, int16_t robot_angle_, uint8_t turning_mode_, uint8_t turning_limit_) {
      int16_t power[MOTOR_QTY];
      uint8_t moving_speed = moving_speed_;
      int16_t moving_dir = SimplifyDeg(moving_dir_);

      if (moving_speed > POWER_MAX_LIMIT) moving_speed = POWER_MAX_LIMIT;  // 指定速度が限界を超えていたときに補正

      // 角度とスピードを各モーターの値に変更
      power[0] = MySin(moving_dir - MOTOR_0_DEGREE) * moving_speed * -1;
      power[1] = MySin(moving_dir - MOTOR_1_DEGREE) * moving_speed * -1;
      power[2] = MySin(moving_dir - MOTOR_2_DEGREE) * moving_speed * 1;
      power[3] = MySin(moving_dir - MOTOR_3_DEGREE) * moving_speed * 1;

      // モーターの最大パフォーマンス発揮
      uint8_t max_power = 0;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (max_power < abs(power[i])) max_power = abs(power[i]);
      }
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] *= float(moving_speed) / max_power;
      }

      // エンコーダー処理
      static float add_power[MOTOR_QTY];
      static int16_t corrected_power[MOTOR_QTY];
      static int16_t pre_corrected_power[MOTOR_QTY];
      static float d, pre_d;
      float add_power_limit;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            corrected_power[i] = power[i];
            if (abs(pre_corrected_power[i] - corrected_power[i]) > 50) add_power[i] = 0;  // 急に速度が変わった場合は積分をリセット
            pre_corrected_power[i] = corrected_power[i];

            if (abs(power[i]) >= 10 && readms(addPowerTimer) < 100) {
                  d = abs(encoder_val[i] - abs(corrected_power[i]) / (100.0f / BASE_POWER));  // 指定速度と実際の速度の差
                  if (encoder_val[i] < abs(corrected_power[i]) / (100.0f / BASE_POWER)) {
                        add_power[i] += (d + pre_d) * readus(addPowerTimer) / 2 * ENCODER_GAIN;  // 台形積分
                        add_power_limit = POWER_MAX_LIMIT - abs(power[i]);
                        if (add_power_limit > MAX_ADD_POWER) add_power_limit = MAX_ADD_POWER;  // 加算する速度が上がりすぎないように補正
                        if (add_power[i] > add_power_limit) add_power[i] = add_power_limit;    // 加算後の速度が100を超えないように補正
                  } else {
                        add_power[i] -= (d + pre_d) * readus(addPowerTimer) / 2 * ENCODER_GAIN;  // 台形積分
                        if (add_power[i] < MIN_ADD_POWER) add_power[i] = MIN_ADD_POWER;          // 加算する速度が下がりすぎないように補正
                  }
                  pre_d = d;
            } else {
                  add_power[i] = 0;  // 指定速度が小さすぎるor時間が経ちすぎていた時は積分をリセット
            }
      }
      addPowerTimer.reset();

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (power[i] > 0) {
                  corrected_power[i] = power[i] + add_power[i];
            } else {
                  corrected_power[i] = power[i] - add_power[i];
            }
      }

      // PID姿勢制御
      attitudeControlPID.Compute(*own_dir, robot_angle_);
      attitudeControlPID.SetLimit(turning_limit_);
      float tmp_pid = attitudeControlPID.Get();

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            // ボールを捕捉しながら回転するために姿勢制御を与えるモーターを制限
            if (turning_mode_ == 0) {
                  corrected_power[i] += i < 2 ? tmp_pid * -1 : tmp_pid;
            } else if (turning_mode_ == 1) {  // ボールを前に捕捉した状態
                  if (i == 1 || i == 2) corrected_power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
            } else if (turning_mode_ == 2) {  // ボールを右に捕捉した状態
                  if (i == 2 || i == 3) corrected_power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
            } else if (turning_mode_ == 3) {  // ボールを後ろに捕捉した状態
                  if (i == 0 || i == 3) corrected_power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
            } else if (turning_mode_ == 4) {  // ボールを左に捕捉した状態
                  if (i == 0 || i == 1) corrected_power[i] += i < 2 ? tmp_pid * -2 : tmp_pid * 2;
            }
      }

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (corrected_power[i] > POWER_MAX_LIMIT) corrected_power[i] = POWER_MAX_LIMIT * (abs(corrected_power[i]) / corrected_power[i]);
      }

      // 移動平均フィルタ
      motor1Ave.Compute(&corrected_power[0]);
      motor2Ave.Compute(&corrected_power[1]);
      motor3Ave.Compute(&corrected_power[2]);
      motor4Ave.Compute(&corrected_power[3]);

      // モーターへ出力
      motor_1_a = abs(corrected_power[0]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[0] > 0 ? corrected_power[0] * 0.01f : 0);
      motor_1_b = abs(corrected_power[0]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[0] < 0 ? corrected_power[0] * -0.01f : 0);
      motor_2_a = abs(corrected_power[1]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[1] > 0 ? corrected_power[1] * 0.01f : 0);
      motor_2_b = abs(corrected_power[1]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[1] < 0 ? corrected_power[1] * -0.01f : 0);
      motor_3_a = abs(corrected_power[2]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[2] > 0 ? corrected_power[2] * 0.01f : 0);
      motor_3_b = abs(corrected_power[2]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[2] < 0 ? corrected_power[2] * -0.01f : 0);
      motor_4_a = abs(corrected_power[3]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[3] > 0 ? corrected_power[3] * 0.01f : 0);
      motor_4_b = abs(corrected_power[3]) <= POWER_MIN_LIMIT ? 1 : (corrected_power[3] < 0 ? corrected_power[3] * -0.01f : 0);
}

void Motor::SetPwmPeriod(uint16_t pwm_period_) {
      motor_1_a.period_us(pwm_period_);
      motor_1_b.period_us(pwm_period_);
      motor_2_a.period_us(pwm_period_);
      motor_2_b.period_us(pwm_period_);
      motor_3_a.period_us(pwm_period_);
      motor_3_b.period_us(pwm_period_);
      motor_4_a.period_us(pwm_period_);
      motor_4_b.period_us(pwm_period_);
}

void Motor::SetAttitudeControlPID(float kp_, float ki_, float kd_) {
      attitudeControlPID.SetGain(kp_, ki_, kd_);
      attitudeControlPID.SetSamplingPeriod();
      attitudeControlPID.SelectType(PI_D_TYPE);
}

void Motor::Brake(uint16_t brake_time_) {
      motor_1_a = 1;
      motor_1_b = 1;
      motor_2_a = 1;
      motor_2_b = 1;
      motor_3_a = 1;
      motor_3_b = 1;
      motor_4_a = 1;
      motor_4_b = 1;
      wait_us(brake_time_ * 1000);
}

void Motor::Free() {
      motor_1_a = 0;
      motor_1_b = 0;
      motor_2_a = 0;
      motor_2_b = 0;
      motor_3_a = 0;
      motor_3_b = 0;
      motor_4_a = 0;
      motor_4_b = 0;
}