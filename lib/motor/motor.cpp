#include "motor.h"

#include "mbed.h"

Motor::Motor(PinName motor_1_a_, PinName motor_1_b_, PinName motor_2_a_, PinName motor_2_b_, PinName motor_3_a_, PinName motor_3_b_, PinName motor_4_a_, PinName motor_4_b_) : motor_1_a(motor_1_a_), motor_1_b(motor_1_b_), motor_2_a(motor_2_a_), motor_2_b(motor_2_b_), motor_3_a(motor_3_a_), motor_3_b(motor_3_b_), motor_4_a(motor_4_a_), motor_4_b(motor_4_b_) {
      motor_1_a = 0;
      motor_1_b = 0;
      motor_2_a = 0;
      motor_2_b = 0;
      motor_3_a = 0;
      motor_3_b = 0;
      motor_4_a = 0;
      motor_4_b = 0;

      motor_1.SetLength(MOVING_AVG_LENGTH);
      motor_2.SetLength(MOVING_AVG_LENGTH);
      motor_3.SetLength(MOVING_AVG_LENGTH);
      motor_4.SetLength(MOVING_AVG_LENGTH);

      d_timer.start();
      add_speed_timer.start();
}

void Motor::Run(int16_t moving_dir, uint8_t moving_speed, int16_t robot_angle, uint8_t robot_angle_mode, uint8_t pd_limit) {
      int16_t power[MOTOR_QTY];
      static uint8_t add_speed = 0;

      if (add_speed_timer.read() > ADD_SPEED_PERIOD) {
            if (encoder_val < moving_speed * 0.1) {
                  if (add_speed < 50) {
                        add_speed++;
                  }
            } else {
                  if (add_speed > 5) {
                        add_speed -= 2;
                  }
            }
            add_speed_timer.reset();
      }

      if (encoder_val < moving_speed * 0.1 || add_speed > 5) {
            moving_speed += add_speed * ADD_SPEED_K;
      }

      if (moving_speed > POWER_LIMIT) {
            moving_speed = POWER_LIMIT;
      }

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] = MySin(moving_dir - (45 + i * 90)) * moving_speed * (i < 2 ? -1 : 1);   // 角度とスピードを各モーターの値に変更
      }

      // モーターの最大パフォーマンス発揮
      uint8_t max_power = 0;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (max_power < abs(power[i])) {
                  max_power = abs(power[i]);
            }
      }
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] *= float(moving_speed) / max_power;
      }

      // PD姿勢制御
      p = SimplifyDeg(robot_angle - yaw);   // 比例
      if (d_timer.read() > D_PERIOD) {
            d = (p - pre_p) * d_timer.read();   // 微分
            pre_p = p;
            d_timer.reset();
      }
      pd = p * KP + d * KD;

      if (abs(pd) > pd_limit) {
            pd = pd_limit * (abs(pd) / pd);
      }

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            // ボールを捕捉しながら回転するために姿勢制御を与えるモーターを制限
            if (robot_angle_mode == 0) {
                  power[i] += i < 2 ? -pd : pd;
            } else if (robot_angle_mode == 1) {   // ボールを前に捕捉した状態
                  if (i == 1 || i == 2) {
                        power[i] += i < 2 ? pd * -2 : pd * 2;
                  }
            } else if (robot_angle_mode == 2) {   // ボールを右に捕捉した状態
                  if (i == 2 || i == 3) {
                        power[i] += i < 2 ? pd * -2 : pd * 2;
                  }
            } else if (robot_angle_mode == 3) {   // ボールを後ろに捕捉した状態
                  if (i == 0 || i == 3) {
                        power[i] += i < 2 ? pd * -2 : pd * 2;
                  }
            } else if (robot_angle_mode == 4) {   // ボールを左に捕捉した状態
                  if (i == 0 || i == 1) {
                        power[i] += i < 2 ? pd * -2 : pd * 2;
                  }
            }

            // モーターの上限値を超えた場合の修正
            if (abs(power[i]) > POWER_LIMIT) {
                  power[i] = POWER_LIMIT * (abs(power[i]) / power[i]);
            }
      }

      //移動平均フィルタ
      motor_1.Ave(&power[0]);
      motor_2.Ave(&power[1]);
      motor_3.Ave(&power[2]);
      motor_4.Ave(&power[3]);

      // モーターへ出力
      motor_1_a = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] > 0 ? power[0] * 0.01000 : 0);
      motor_1_b = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] < 0 ? power[0] * -0.01000 : 0);
      motor_2_a = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] > 0 ? power[1] * 0.01000 : 0);
      motor_2_b = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] < 0 ? power[1] * -0.01000 : 0);
      motor_3_a = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] > 0 ? power[2] * 0.01000 : 0);
      motor_3_b = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] < 0 ? power[2] * -0.01000 : 0);
      motor_4_a = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] > 0 ? power[3] * 0.01000 : 0);
      motor_4_b = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] < 0 ? power[3] * -0.01000 : 0);
}

void Motor::SetPwm() {
      motor_1_a.period_us(PWM_FREQUENCY);
      motor_1_b.period_us(PWM_FREQUENCY);
      motor_2_a.period_us(PWM_FREQUENCY);
      motor_2_b.period_us(PWM_FREQUENCY);
      motor_3_a.period_us(PWM_FREQUENCY);
      motor_3_b.period_us(PWM_FREQUENCY);
      motor_4_a.period_us(PWM_FREQUENCY);
      motor_4_b.period_us(PWM_FREQUENCY);
}

void Motor::Brake(uint16_t brake_time) {
      motor_1_a = 1;
      motor_1_b = 1;
      motor_2_a = 1;
      motor_2_b = 1;
      motor_3_a = 1;
      motor_3_b = 1;
      motor_4_a = 1;
      motor_4_b = 1;
      wait_us(brake_time * 1000);
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