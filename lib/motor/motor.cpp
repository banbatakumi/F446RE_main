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

      i_timer.start();
      d_timer.start();
}

void motor::run(int16_t move_angle, int16_t move_speed, int16_t robot_angle, bool shoot_robot_angle) {
      if (move_speed > POWER_LIMIT) {
            move_speed = POWER_LIMIT;   // 速度が上限を超えていないか
      }
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] = sin((move_angle - (45 + i * 90)) * PI / 180.00000) * move_speed * (i < 2 ? -1 : 1);   // 角度とスピードを各モーターの値に変更
      }

      // モーターの最大パフォーマンス発揮
      uint8_t max_power = 0;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (max_power < abs(power[i])) {
                  max_power = abs(power[i]);
            }
      }
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            power[i] *= float(move_speed) / max_power;
      }

      // PID姿勢制御
      p = robot_angle - yaw;   // 比例
      if (p > 180) p -= 360;
      if (p < -180) p += 360;

      if (i_timer.read() > I_PERIODO) {   // Iのリセット
            i = 0;
            i_timer.reset();
      }

      if (d_timer.read() > D_PERIODO) {
            i += p * d_timer.read();   // 積分
            d = (p - pre_p) * d_timer.read();   // 微分
            pre_p = p;
            d_timer.reset();
      }

      pid = p * KP + i * KI + d * KD;
      if (abs(pid) > PD_LIMIT) {
            pid = PD_LIMIT * (abs(pid) / pid);
      }

      // 移動平均フィルタ
      if (moving_average_count == MOVING_AVERAGE_COUNT_NUMBER) moving_average_count = 0;
      for (uint8_t i = 0; i < MOTOR_QTY; i++) {
            if (robot_angle == 0 || shoot_robot_angle != 0) {
                  power[i] += i < 2 ? -pid : pid;
            } else if (i == 1 || i == 2) {
                  power[i] += i < 2 ? -pid * 2 : pid * 2;
            }
            if (abs(power[i]) > POWER_LIMIT) {
                  power[i] = POWER_LIMIT * (abs(power[i]) / power[i]);   // モーターの上限値超えた場合の修正
            }

            tmp_power[i][moving_average_count] = power[i];
            power[i] = 0;
            for (uint8_t j = 0; j < MOVING_AVERAGE_COUNT_NUMBER; j++) {
                  power[i] += tmp_power[i][j];
            }
            power[i] /= MOVING_AVERAGE_COUNT_NUMBER;
      }
      moving_average_count++;

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

int16_t motor::motor_1() {
      return power[0];
}
int16_t motor::motor_2() {
      return power[1];
}
int16_t motor::motor_3() {
      return power[2];
}
int16_t motor::motor_4() {
      return power[3];
}