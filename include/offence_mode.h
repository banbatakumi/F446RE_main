#ifndef _OFFENCE_MODE_H_
#define _OFFENCE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir) < 30 && camera.ball_dis > 90)
#define IS_BALL_NEAR_OF_BACK (abs(camera.inverse_ball_dir) < 30 && camera.ball_dis > 90)

#define DEPTH_OF_WRAP 95.0f
#define DISTORTION_OF_WRAP 2

Timer backCurveShootTimer;
Timer lineStopTimer;
Timer goToCenterTimer;
Timer wrapTimer;

bool is_pre_line_left = 0;
bool is_pre_line_right = 0;
bool is_pre_line = 0;

int16_t tmp_move_speed, tmp_move_angle;

void OffenceMove() {
      if (sensors.is_on_line == 1 || sensors.is_line_left == 1 || sensors.is_line_right == 1) {  // ラインセンサの処理
            LineMove();
      } else if (camera.ball_dis == 0) {  // ボールがない時の処理
            goToCenterTimer.start();
            if (readms(goToCenterTimer) > 1000) {
                  motor.Run(camera.center_dir, camera.center_dis * 5);
            } else {
                  motor.Run();
            }

            dribblerFront.Hold(0);
            dribblerBack.Hold(0);
      } else {
            if (readms(lineStopTimer) > 500) {
                  lineStopTimer.stop();
                  lineStopTimer.reset();
                  is_pre_line_left = 0;
                  is_pre_line_right = 0;
                  is_pre_line = 0;
            }
            goToCenterTimer.reset();

            if (is_pre_line_left == 1 && readms(lineStopTimer) < 100) {  // 左のラインセンサから復帰後一定時間コート内に戻る
                  motor.Run(90, line_moving_speed);
            } else if (is_pre_line_right == 1 && readms(lineStopTimer) < 100) {  // 右のラインセンサから復帰後一定時間コート内に戻る
                  motor.Run(-90, line_moving_speed);
            } else if (is_pre_line == 1 && readms(lineStopTimer) < 100) {  // エンジェルラインセンサから復帰後一定時間コート内に戻る
                  motor.Run();
            } else {
                  is_pre_line_left = 0;
                  is_pre_line_right = 0;
                  is_pre_line = 0;

                  if (readms(backCurveShootTimer) > 0) {  // 後ろドリブラーのマカオシュート
                        CureveShoot();
                  } else if (holdFront.IsHold()) {
                        HoldFrontMove();           // 前に捕捉している時
                  } else if (holdBack.IsHold()) {  // 後ろに捕捉している時
                        HoldBackMove();
                  } else {
                        if (IS_BALL_NEAR_OF_FRONT) {
                              dribblerFront.Hold(HOLD_WAIT_POWER);
                        } else {
                              dribblerFront.Hold(0);
                        }
                        if (IS_BALL_NEAR_OF_BACK) {
                              dribblerBack.Hold(HOLD_WAIT_POWER);
                        } else {
                              dribblerBack.Hold(0);
                        }

                        if (abs(camera.ball_dir) < 135) {  // 前の捕捉エリアに回り込む
                              WrapToFront();
                        } else {  // 後ろの捕捉エリアに回り込む
                              WrapToBack();
                        }
                  }
            }
      }
}

void WrapToFront() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.ball_dir) < 30) {
            wrap_deg_addend = camera.ball_dir * (abs(camera.ball_dir) / 10.0f);
      } else {
            wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
      }
      tmp_move_angle = camera.ball_dir + (wrap_deg_addend * pow((camera.ball_dis / DEPTH_OF_WRAP), DISTORTION_OF_WRAP));

      if (abs(camera.ball_dir) < 10) wrapTimer.start();
      if (abs(camera.ball_dir) > 30) {
            wrapTimer.reset();
            wrapTimer.stop();
      }

      // 速度
      wrapDirPID.Compute(camera.ball_dir, 0);

      if (camera.ball_dis < 80) {
            tmp_move_speed = moving_speed;
      } else if (readms(wrapTimer) > 250) {
            tmp_move_speed = abs(camera.ball_dir) + 125 - camera.ball_dis;
      } else {
            tmp_move_speed = abs(wrapDirPID.Get());
      }

      if (tmp_move_speed > moving_speed) tmp_move_speed = moving_speed;

      motor.Run(tmp_move_angle, tmp_move_speed);
}

void WrapToBack() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.inverse_ball_dir) < 30) {
            wrap_deg_addend = camera.inverse_ball_dir * (abs(camera.inverse_ball_dir) / 10.0f);
      } else {
            wrap_deg_addend = 90 * (abs(camera.inverse_ball_dir) / camera.inverse_ball_dir);
      }
      tmp_move_angle = camera.inverse_ball_dir + (wrap_deg_addend * pow((camera.ball_dis / DEPTH_OF_WRAP), DISTORTION_OF_WRAP));

      if (abs(camera.inverse_ball_dir) < 10) wrapTimer.start();
      if (abs(camera.inverse_ball_dir) > 45) {
            wrapTimer.reset();
            wrapTimer.stop();
      }

      // 速度
      wrapDirPID.Compute(camera.inverse_ball_dir, 0);

      if (camera.ball_dis < 80) {
            tmp_move_speed = moving_speed;
      } else if (readms(wrapTimer) > 500) {
            tmp_move_speed = 125 - camera.ball_dis;
      } else {
            tmp_move_speed = abs(wrapDirPID.Get());
      }

      if (tmp_move_speed > 50) tmp_move_speed = 50;

      tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);  // ０度の時に180度に動くように変換
      motor.Run(tmp_move_angle, tmp_move_speed);
}

void CureveShoot() {
      static int8_t shoot_dir;

      dribblerBack.Hold(HOLD_MAX_POWER);
      if (readms(backCurveShootTimer) < 100) {
            motor.Brake();
            shoot_dir = camera.front_goal_dir > 0 ? -1 : 1;
      } else if (readms(backCurveShootTimer) < 500) {
            motor.Run(0, 0, 45 * shoot_dir, BACK, 25);
      } else if (readms(backCurveShootTimer) < 1000) {
            motor.Run(0, 0, 180 * shoot_dir);
      } else {
            backCurveShootTimer.stop();
            backCurveShootTimer.reset();
      }
      if (readms(backCurveShootTimer) < 250 && holdBack.IsHold() == 0) {
            backCurveShootTimer.stop();
            backCurveShootTimer.reset();
      }
}

void HoldFrontMove() {
      if (camera.front_goal_size > 15 && camera.is_goal_front == 1 && sensors.dis[0] > 10) {  // キッカーを打つ条件
            motor.Run(0, moving_speed);
            dribblerFront.Brake();
            kicker.Kick();
      } else {
            dribblerFront.Hold(HOLD_MAX_POWER);

            int16_t tmp_move_dir = (camera.front_goal_dir + own_dir) * 2;
            if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
            int16_t robot_dir = camera.front_goal_dir * 1.5 + own_dir;
            if (abs(robot_dir) > 60) robot_dir = 60 * (abs(robot_dir) / robot_dir);

            if ((camera.front_goal_dir - own_dir) > 60) {
                  tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
                  motor.Run(tmp_move_dir - own_dir, 25, robot_dir, FRONT, 10);
            } else if (camera.front_goal_size > 30) {
                  motor.Run(0, 0, camera.front_goal_dir > 0 ? 60 : -60, FRONT, 25);
            } else {
                  if (sensors.dis[1] < 20 && sensors.dis[3] > 20) {
                        tmp_move_dir = -45;
                  } else if (sensors.dis[3] < 20 && sensors.dis[1] > 20) {
                        tmp_move_dir = 45;
                  }
                  motor.Run(tmp_move_dir - own_dir, moving_speed, robot_dir, FRONT, 10);
            }
      }
}

void HoldBackMove() {
      if (camera.front_goal_size > 30) {  // キッカーを打つ条件
            backCurveShootTimer.start();
      } else {
            dribblerBack.Hold(HOLD_MAX_POWER);

            int16_t tmp_move_dir = camera.front_goal_dir * 1.5;
            if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
            motor.Run(tmp_move_dir - own_dir, 25);
      }
}

void LineMove() {
      if (sensors.is_on_line == 1) is_pre_line = 1;
      if (sensors.is_line_left == 1) is_pre_line_left = 1;
      if (sensors.is_line_right == 1) is_pre_line_right = 1;

      float vector_x, vector_y, vector_mag, vector_dir;
      float line_vector_rate, ball_vector_rate;

      if (abs(camera.ball_dir + own_dir) < 90 && readms(lineStopTimer) > 500 && abs(sensors.line_inside_dir + own_dir) > 135 && sensors.dis[0] >= 10) {
            line_vector_rate = 0.4;
            ball_vector_rate = 0.6;
      } else {
            line_vector_rate = 0.75;
            ball_vector_rate = 0.25;
      }

      vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * line_vector_rate;
      vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * line_vector_rate;
      vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * ball_vector_rate;
      vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * ball_vector_rate;
      vector_dir = atan2(vector_x, vector_y) * 180.0f / PI;
      vector_mag = abs(sqrt(pow(vector_x, 2) + pow(vector_y, 2)));

      lineStopTimer.start();
      if (readms(lineStopTimer) < 100 || readms(lineStopTimer) > 5000 || camera.ball_dis == 0) {
            if (sensors.is_on_line == 1) {
                  motor.Run(sensors.line_inside_dir, line_moving_speed);
            } else if (sensors.is_line_left == 1) {
                  motor.Run(90, line_moving_speed);
            } else if (sensors.is_line_right == 1) {
                  motor.Run(-90, line_moving_speed);
            }
      } else if (holdFront.IsHold()) {
            dribblerFront.Hold(HOLD_MAX_POWER);
            if (sensors.is_on_line == 1) {
                  motor.Run(sensors.line_inside_dir, 25, 0, FRONT, 15);
            } else if (sensors.is_line_left == 1) {
                  motor.Run(90, 25, 0, FRONT, 15);
            } else if (sensors.is_line_right == 1) {
                  motor.Run(-90, 25, 0, FRONT, 15);
            }
      } else if (holdBack.IsHold()) {
            dribblerBack.Hold(HOLD_MAX_POWER);
            if (sensors.is_on_line == 1) {
                  motor.Run(sensors.line_inside_dir, 25, 0, BACK, 15);
            } else if (sensors.is_line_left == 1) {
                  motor.Run(90, 25, 0, BACK, 15);
            } else if (sensors.is_line_right == 1) {
                  motor.Run(-90, 25, 0, BACK, 15);
            }
      } else {
            if (IS_BALL_NEAR_OF_FRONT) {
                  dribblerFront.Hold(HOLD_WAIT_POWER);
            } else {
                  dribblerFront.Hold(0);
            }
            if (IS_BALL_NEAR_OF_BACK) {
                  dribblerBack.Hold(HOLD_WAIT_POWER);
            } else {
                  dribblerBack.Hold(0);
            }
            if (sensors.is_on_line == 1) {
                  uint16_t tmp_moving_speed = vector_mag * 10;
                  int16_t robot_dir = camera.ball_dir + own_dir;
                  if (abs(camera.ball_dir + own_dir) < 90) {
                        robot_dir = camera.ball_dir + own_dir;
                  } else {
                        robot_dir = camera.inverse_ball_dir + own_dir;
                  }
                  if (abs(robot_dir) > 45) robot_dir = 45 * (abs(robot_dir) / robot_dir);
                  if (tmp_moving_speed > line_moving_speed) tmp_moving_speed = line_moving_speed;
                  motor.Run(vector_dir + own_dir, tmp_moving_speed, robot_dir);
            } else if (sensors.is_line_left == 1) {
                  if (SimplifyDeg(camera.ball_dir + own_dir) < 0) {
                        motor.Run(-90, 50);
                  } else {
                        motor.Run(90, 50);
                  }
            } else if (sensors.is_line_right == 1) {
                  if (SimplifyDeg(camera.ball_dir + own_dir) > 0) {
                        motor.Run(90, 50);
                  } else {
                        motor.Run(-90, 50);
                  }
            }
      }
}

#endif