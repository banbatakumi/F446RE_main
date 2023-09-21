#include "motor.h"

#include "mbed.h"

motor::motor(PinName motor_1_1_, PinName motor_1_2_, PinName motor_2_1_, PinName motor_2_2_, PinName motor_3_1_, PinName motor_3_2_, PinName motor_4_1_, PinName motor_4_2_) : motor_1_1(motor_1_1_), motor_1_2(motor_1_2_), motor_2_1(motor_2_1_), motor_2_2(motor_2_2_), motor_3_1(motor_3_1_), motor_3_2(motor_3_2_), motor_4_1(motor_4_1_), motor_4_2(motor_4_2_) {
      motor_1_1 = 0;
      motor_1_2 = 0;
      motor_2_1 = 0;
      motor_2_2 = 0;
      motor_3_1 = 0;
      motor_3_2 = 0;
      motor_4_1 = 0;
      motor_4_2 = 0;

      d_timer.start();
      add_speed_timer.start();
}

void motor::run(int16_t moving_dir, uint8_t moving_speed, int16_t robot_angle, uint8_t robot_angle_mode, uint8_t pd_limit) {
      uint8_t speed = moving_speed;
      int16_t power[MOTOR_QTY];
      static int16_t tmp_power[4][MOVING_AVG_CNT_NUM];
      static uint8_t add_speed = 0;

      if (add_speed_timer.read() > ADD_SPEED_PERIOD) {
            if (encoder_val < speed * 0.1) {
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

      if (encoder_val < speed * 0.1 || add_speed > 5) {
            speed += add_speed * ADD_SPEED_K;
      }

      if (speed > POWER_LIMIT) {
            speed = POWER_LIMIT;
      }

      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] = sin32_t((moving_dir - (45 + i * 90)) * PI / 180.00000) * speed * (i < 2 ? -1 : 1);   // 角度とスピードを各モーターの値に変更
      }

      // モーターの最大パフォーマンス発揮
      uint8_t max_power = 0;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (max_power < abs(power[i])) {
                  max_power = abs(power[i]);
            }
      }
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] *= float(speed) / max_power;
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

      if (moving_avg_cnt == MOVING_AVG_CNT_NUM) moving_avg_cnt = 0;
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

            // 移動平均フィルタ
            tmp_power[i][moving_avg_cnt] = power[i];
            power[i] = 0;
            for (uint8_t j = 0; j < MOVING_AVG_CNT_NUM; j++) {
                  power[i] += tmp_power[i][j];
            }
            power[i] /= MOVING_AVG_CNT_NUM;
      }
      moving_avg_cnt++;

      // モーターへ出力
      motor_1_1 = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] > 0 ? power[0] * 0.01000 : 0);
      motor_1_2 = abs(power[0]) < MIN_BRAKE ? 1 : (power[0] < 0 ? power[0] * -0.01000 : 0);
      motor_2_1 = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] > 0 ? power[1] * 0.01000 : 0);
      motor_2_2 = abs(power[1]) < MIN_BRAKE ? 1 : (power[1] < 0 ? power[1] * -0.01000 : 0);
      motor_3_1 = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] > 0 ? power[2] * 0.01000 : 0);
      motor_3_2 = abs(power[2]) < MIN_BRAKE ? 1 : (power[2] < 0 ? power[2] * -0.01000 : 0);
      motor_4_1 = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] > 0 ? power[3] * 0.01000 : 0);
      motor_4_2 = abs(power[3]) < MIN_BRAKE ? 1 : (power[3] < 0 ? power[3] * -0.01000 : 0);
}

void motor::set_pwm() {
      motor_1_1.period_us(MOTOR_FREQUENCY);
      motor_1_2.period_us(MOTOR_FREQUENCY);
      motor_2_1.period_us(MOTOR_FREQUENCY);
      motor_2_2.period_us(MOTOR_FREQUENCY);
      motor_3_1.period_us(MOTOR_FREQUENCY);
      motor_3_2.period_us(MOTOR_FREQUENCY);
      motor_4_1.period_us(MOTOR_FREQUENCY);
      motor_4_2.period_us(MOTOR_FREQUENCY);
}

void motor::brake(uint16_t brake_time) {
      motor_1_1 = 1;
      motor_1_2 = 1;
      motor_2_1 = 1;
      motor_2_2 = 1;
      motor_3_1 = 1;
      motor_3_2 = 1;
      motor_4_1 = 1;
      motor_4_2 = 1;
      wait_us(brake_time * 1000);
}

void motor::free() {
      motor_1_1 = 0;
      motor_1_2 = 0;
      motor_2_1 = 0;
      motor_2_2 = 0;
      motor_3_1 = 0;
      motor_3_2 = 0;
      motor_4_1 = 0;
      motor_4_2 = 0;
}