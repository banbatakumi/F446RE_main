#ifndef _OFFENSE_MODE_H_
#define _OFFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir + own_dir) < 30 && camera.ball_dis > 0 && camera.ball_dis < 15)
#define IS_BALL_NEAR_OF_BACK (abs(camera.inverse_ball_dir + own_dir) < 30 && camera.ball_dis > 0 && camera.ball_dis < 15)
#define IS_OPS_GOAL_FOUND (camera.ops_goal_size != 0)
#define IS_OWN_GOAL_FOUND (camera.own_goal_size != 0)

#define DEPTH_OF_WRAP 195.0f
#define DISTORTION_OF_WRAP 2.0

Timer CurveShootTimer;
Timer holdTimer;
Timer lineStopTimer;
Timer goToCenterTimer;
Timer accelerationTimer;

bool is_pre_line_left = 0;
bool is_pre_line_right = 0;
bool is_pre_line = 0;
bool enable_front_curve_shoot = 0;
bool enable_back_curve_shoot = 0;
bool is_first_hold = 1;
int16_t tmp_moving_dir, tmp_moving_speed, tmp_robot_dir;

int16_t hide_robot_dir = 0;
bool face_to_goal = 0;

int8_t shoot_dir;

void WrapToFront() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.ball_dir) <= 50) {
            wrap_deg_addend = camera.ball_dir * 1.5;
            if (abs(camera.ball_dir) <= 10) wrap_deg_addend *= abs(camera.ball_dir) / 10.0f;
      } else {
            wrap_deg_addend = 75 * (abs(camera.ball_dir) / camera.ball_dir);
      }
      tmp_moving_dir = SimplifyDeg(camera.ball_dir + (wrap_deg_addend * pow(((200 - camera.ball_dis) / DEPTH_OF_WRAP), DISTORTION_OF_WRAP)));

      // 速度
      if (camera.ball_x > 0) {
            tmp_moving_speed = (cam.ball_velocity_x + 2) * 10;
      } else {
            tmp_moving_speed = (cam.ball_velocity_x * -1 + 2) * 10;
      }
      wrapDirPID.Compute(camera.ball_dir, 0);
      wrapSpeedPID.Compute(camera.ball_dis, 0);
      tmp_moving_speed += abs(wrapDirPID.Get()) + abs(wrapSpeedPID.Get());
      if (abs(camera.ball_dir) <= 25) {
            tmp_moving_speed += 25 - abs(camera.ball_dir);
      }

      if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;

      motor.Drive(tmp_moving_dir, tmp_moving_speed);
}

void WrapToBack() {
      int16_t wrap_deg_addend;
      if (abs(camera.inverse_ball_dir) <= 50) {
            wrap_deg_addend = camera.inverse_ball_dir * 1.5;
            if (abs(camera.inverse_ball_dir) <= 10) wrap_deg_addend *= abs(camera.inverse_ball_dir) / 10.0f;
      } else {
            wrap_deg_addend = 75 * (abs(camera.inverse_ball_dir) / camera.inverse_ball_dir);
      }
      tmp_moving_dir = SimplifyDeg(camera.inverse_ball_dir + (wrap_deg_addend * pow(((200 - camera.ball_dis) / DEPTH_OF_WRAP), DISTORTION_OF_WRAP)));

      // 速度
      wrapDirPID.Compute(camera.inverse_ball_dir, 0);
      wrapSpeedPID.Compute(camera.ball_dis, 0);
      tmp_moving_speed = abs(wrapDirPID.Get()) + abs(wrapSpeedPID.Get()) + 10;
      if (tmp_moving_speed > 50) tmp_moving_speed = 50;

      tmp_moving_dir = SimplifyDeg(tmp_moving_dir - 180);  // ０度の時に180度に動くように変換
      motor.Drive(tmp_moving_dir, tmp_moving_speed);
}

void FrontCurveShoot() {
      dribblerFront.Hold(HOLD_MAX_POWER);
      holdTimer.start();
      accelerationTimer.start();
      if (abs(camera.ops_goal_dir) < 60 || readms(CurveShootTimer) > 10) {
            CurveShootTimer.start();
            motor.Drive(camera.ops_goal_dir * 1.5 - own_dir, 0, camera.ops_goal_dir, FRONT, 40);
            if (camera.is_goal_front || readms(CurveShootTimer) > 1000) {
                  kicker.Kick();
                  motor.Brake(200);
                  CurveShootTimer.stop();
                  CurveShootTimer.reset();
                  holdTimer.stop();
                  holdTimer.reset();
                  accelerationTimer.stop();
                  accelerationTimer.reset();
                  enable_front_curve_shoot = 0;
            }
      } else if (readms(holdTimer) < 500) {
            if (sensors.hold_front == 1) holdTimer.reset();

            tmp_moving_speed = readms(accelerationTimer) * 0.05;
            if (tmp_moving_speed > 25) tmp_moving_speed = 25;

            motor.Drive(180, tmp_moving_speed);
      } else {
            enable_front_curve_shoot = 0;
            holdTimer.stop();
            holdTimer.reset();
            accelerationTimer.stop();
            accelerationTimer.reset();
      }
}

void BackCureveShoot() {
      dribblerBack.Hold(HOLD_MAX_POWER);
      holdTimer.start();
      accelerationTimer.start();
      if ((camera.ops_goal_size > 30 && abs(camera.ops_goal_dir) < 30) || readms(CurveShootTimer) > 10) {
            CurveShootTimer.start();
            if (readms(CurveShootTimer) < 10) {
                  motor.Brake();
                  shoot_dir = camera.ops_goal_dir > 0 ? -1 : 1;
            } else if (abs(own_dir) < 45) {
                  motor.Drive(0, 0, 60 * shoot_dir, BACK, 40);
            } else if (abs(own_dir) < 135) {
                  motor.Drive(0, 0, 160 * shoot_dir);
            } else {
                  CurveShootTimer.stop();
                  CurveShootTimer.reset();
                  holdTimer.stop();
                  holdTimer.reset();
                  accelerationTimer.stop();
                  accelerationTimer.reset();
                  enable_back_curve_shoot = 0;
            }
      } else if (readms(holdTimer) < 500) {
            if (sensors.hold_back == 1) holdTimer.reset();

            tmp_moving_speed = (readms(accelerationTimer) - 200) * 0.05;
            if (tmp_moving_speed > 25) tmp_moving_speed = 25;

            if (readms(accelerationTimer) < 250) {
                  tmp_moving_speed = (250 - readms(accelerationTimer)) * 0.1;
                  if (tmp_moving_speed > 25) tmp_moving_speed = 25;
                  motor.Drive(180, tmp_moving_speed);
            } else {
                  motor.Drive(camera.ops_goal_dir * 2, tmp_moving_speed);
            }
      } else {
            enable_back_curve_shoot = 0;
            holdTimer.stop();
            holdTimer.reset();
            accelerationTimer.stop();
            accelerationTimer.reset();
      }
}

void HoldFrontMove() {
      if (camera.ops_goal_size > 15 && camera.is_goal_front) {  // キッカーを打つ条件
            dribblerFront.Brake();
            kicker.Kick();
      } else {
            dribblerFront.Hold(HOLD_MAX_POWER);

            if (is_first_hold == 1) {
                  is_first_hold = 0;
            }

            if (camera.front_proximity[2] && camera.front_proximity[3] && camera.front_proximity[4]) {
                  if (abs(own_dir) > 30) kicker.Kick();
                  motor.Drive(0, moving_speed, 45 * abs(camera.own_x) / camera.own_x, FRONT);
            } else if (camera.front_proximity[0] && camera.front_proximity[1] && camera.front_proximity[2]) {
                  if (own_dir > 30) kicker.Kick();
                  motor.Drive(0, moving_speed, 45, FRONT);
            } else if (camera.front_proximity[4] && camera.front_proximity[5] && camera.front_proximity[6]) {
                  if (own_dir < -30) kicker.Kick();
                  motor.Drive(0, moving_speed, -45, FRONT);
            } else if (IS_OPS_GOAL_FOUND) {
                  tmp_robot_dir = camera.ops_goal_dir;
                  tmp_moving_dir = camera.ops_goal_dir * 1.5;
                  if (abs(tmp_robot_dir) > 40) tmp_robot_dir = 40 * (abs(tmp_robot_dir) / tmp_robot_dir);
                  if (abs(camera.ops_goal_dir) > 60) {
                        enable_front_curve_shoot = 1;
                  } else {
                        motor.Drive(tmp_moving_dir - own_dir, moving_speed, tmp_robot_dir, FRONT, 20);
                  }
            } else {
                  if (camera.own_x > 0) {
                        tmp_moving_dir = -45;
                  } else {
                        tmp_moving_dir = 45;
                  }
                  motor.Drive(tmp_moving_dir, moving_speed, 0, FRONT, 20);
            }
      }
}

void HoldBackMove() {
      if (is_first_hold == 1) {
            is_first_hold = 0;
      }
      enable_back_curve_shoot = 1;
}

void LineMove() {
      if (sensors.is_on_line == 1) is_pre_line = 1;
      if (sensors.is_line_left == 1) is_pre_line_left = 1;
      if (sensors.is_line_right == 1) is_pre_line_right = 1;

      float vector_x, vector_y, vector_mag, vector_dir;
      float line_vector_rate, ball_vector_rate;

      if (abs(camera.ball_dir + own_dir) < 90 && readms(lineStopTimer) > 500 && abs(sensors.line_inside_dir + own_dir) > 135 && abs(camera.own_x) < 20) {  // ラインを割る
            line_vector_rate = 0.4;
            ball_vector_rate = 0.6;
      } else {  // 通常待機
            line_vector_rate = 0.6;
            ball_vector_rate = 0.4;
      }

      vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * line_vector_rate;
      vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * line_vector_rate;
      vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * ball_vector_rate;
      vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * ball_vector_rate;
      vector_dir = atan2(vector_x, vector_y) * 180.0f / PI;
      vector_mag = abs(sqrt(pow(vector_x, 2) + pow(vector_y, 2)));

      lineStopTimer.start();
      if (readms(lineStopTimer) < 100 || readms(lineStopTimer) > 3000 || camera.ball_dis == 0) {
            if (sensors.is_on_line == 1) {
                  motor.Drive(sensors.line_inside_dir, line_moving_speed);
            } else if (sensors.is_line_left == 1) {
                  motor.Drive(90, line_moving_speed);
            } else if (sensors.is_line_right == 1) {
                  motor.Drive(-90, line_moving_speed);
            }
      } else if (holdFront.IsHold()) {
            dribblerFront.Hold(HOLD_MAX_POWER);
            if (sensors.is_on_line == 1) {
                  motor.Drive(sensors.line_inside_dir, 25, 0, FRONT, 10);
            } else if (sensors.is_line_left == 1) {
                  motor.Drive(90, 25, 0, FRONT, 10);
            } else if (sensors.is_line_right == 1) {
                  motor.Drive(-90, 25, 0, FRONT, 10);
            }
      } else if (holdBack.IsHold()) {
            dribblerBack.Hold(HOLD_MAX_POWER);
            if (sensors.is_on_line == 1) {
                  motor.Drive(sensors.line_inside_dir, 25, 0, BACK, 10);
            } else if (sensors.is_line_left == 1) {
                  motor.Drive(90, 25, 0, BACK, 10);
            } else if (sensors.is_line_right == 1) {
                  motor.Drive(-90, 25, 0, BACK, 10);
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
                  tmp_moving_speed = vector_mag * 6;
                  tmp_robot_dir = 0;
                  tmp_robot_dir = camera.ball_dir;
                  if (abs(camera.ball_dir) < 120) {
                        tmp_robot_dir = camera.ball_dir;
                  } else {
                        tmp_robot_dir = camera.inverse_ball_dir;
                  }
                  if (abs(tmp_robot_dir) > 45) tmp_robot_dir = 45 * (abs(tmp_robot_dir) / tmp_robot_dir);
                  if (tmp_moving_speed > line_moving_speed) tmp_moving_speed = line_moving_speed;
                  motor.Drive(vector_dir + own_dir, tmp_moving_speed, tmp_robot_dir);
            } else if (sensors.is_line_left == 1) {
                  if (camera.ball_dir < 0) {
                        motor.Drive(-90, 50);
                  } else {
                        motor.Drive(90, line_moving_speed);
                  }
            } else if (sensors.is_line_right == 1) {
                  if (camera.ball_dir > 0) {
                        motor.Drive(90, 50);
                  } else {
                        motor.Drive(-90, line_moving_speed);
                  }
            }
      }
}

void OffenseMove() {
      if (sensors.is_on_line == 1 || ((sensors.is_line_left == 1 || sensors.is_line_right == 1) && motor.GetPreMovingSpeed() > 50 && abs(motor.GetPreMovingDir()) > 45 && abs(motor.GetPreMovingDir()) < 135)) {  // ラインセンサの処理
            LineMove();
      } else if (camera.ball_dis == 0) {  // ボールがない時の処理
            goToCenterTimer.start();
            if (readms(goToCenterTimer) > 1000) {
                  motor.Drive(camera.center_dir, camera.center_dis * 10);  // 真ん中に行く
            } else {
                  motor.Drive();
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
                  motor.Drive(90, line_moving_speed);
            } else if (is_pre_line_right == 1 && readms(lineStopTimer) < 100) {  // 右のラインセンサから復帰後一定時間コート内に戻る
                  motor.Drive(-90, line_moving_speed);
            } else if (is_pre_line == 1 && readms(lineStopTimer) < 100) {  // エンジェルラインセンサから復帰後一定時間コート内に戻る
                  motor.Drive();
            } else {
                  is_pre_line_left = 0;
                  is_pre_line_right = 0;
                  is_pre_line = 0;

                  if (abs(camera.ball_dir) > 90 && camera.own_y < -20 && abs(camera.own_x) < 20) {  // 後ろドリブラーのマカオシュート
                        motor.Drive();
                  } else if (enable_front_curve_shoot == 1) {  // 後ろドリブラーのマカオシュート
                        FrontCurveShoot();
                  } else if (enable_back_curve_shoot == 1) {  // 後ろドリブラーのマカオシュート
                        BackCureveShoot();
                  } else if (holdFront.IsHold()) {  // 前に捕捉している時
                        HoldFrontMove();
                  } else if (holdBack.IsHold()) {  // 後ろに捕捉している時
                        HoldBackMove();
                  } else {
                        is_first_hold = 1;
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
                        if (abs(camera.ball_dir) < (camera.own_y > 0 ? 150 : 120)) {  // 前の捕捉エリアに回り込む
                              WrapToFront();
                        } else {  // 後ろの捕捉エリアに回り込む
                              WrapToBack();
                        }
                  }
            }
      }
}

#endif