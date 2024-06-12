#ifndef _OFFENSE_MODE_H_
#define _OFFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir) < 30 && camera.ball_dis < 15)
#define IS_BALL_NEAR_OF_BACK (abs(camera.inverse_ball_dir) < 30 && camera.ball_dis < 15)
#define IS_OPS_GOAL_FOUND (camera.ops_goal_size != 0)
#define IS_OWN_GOAL_FOUND (camera.own_goal_size != 0)

#define DEPTH_OF_WRAP 200.0f
#define DISTORTION_OF_WRAP 2.0

Timer CurveShootTimer;
Timer holdBackTimer;
Timer lineStopTimer;
Timer goToCenterTimer;

bool is_pre_line_left = 0;
bool is_pre_line_right = 0;
bool is_pre_line = 0;
bool enable_back_curve_shoot = 0;
bool is_first_hold = 1;
int16_t tmp_moving_dir, tmp_moving_speed, robot_dir;

int16_t hide_robot_dir = 0;
bool face_to_goal = 0;
int16_t shoot_goal_dir = 0;

void WrapToFront() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.ball_dir) <= 75) {
            wrap_deg_addend = camera.ball_dir;
      } else {
            wrap_deg_addend = 75 * (abs(camera.ball_dir) / camera.ball_dir);
      }
      tmp_moving_dir = SimplifyDeg(camera.ball_dir + (wrap_deg_addend * pow(((200 - camera.ball_dis) / DEPTH_OF_WRAP), DISTORTION_OF_WRAP)));

      // 速度
      if (camera.ball_x > 0) {
            tmp_moving_speed = abs(camera.ball_dir) + (cam.ball_velocity_x + 4) * 10;
      } else {
            tmp_moving_speed = abs(camera.ball_dir) - (cam.ball_velocity_x - 4) * 10;
      }
      tmp_moving_speed += camera.ball_dis / 2;

      if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;

      motor.Drive(tmp_moving_dir, tmp_moving_speed);
}

void WrapToBack() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.inverse_ball_dir) <= 60) {
            wrap_deg_addend = camera.inverse_ball_dir * 1.5;
            if (abs(camera.inverse_ball_dir) < 30) wrap_deg_addend *= abs(camera.inverse_ball_dir) / 30.0f;
      } else {
            wrap_deg_addend = 90 * (abs(camera.inverse_ball_dir) / camera.inverse_ball_dir);
      }
      tmp_moving_dir = SimplifyDeg(camera.inverse_ball_dir + (wrap_deg_addend * pow(((200 - camera.ball_dis) / DEPTH_OF_WRAP), DISTORTION_OF_WRAP)));

      // 速度
      if (camera.ball_x > 0) {
            tmp_moving_speed = abs(camera.inverse_ball_dir) + (cam.ball_velocity_x + 3) * 6;
      } else {
            tmp_moving_speed = abs(camera.inverse_ball_dir) - (cam.ball_velocity_x - 3) * 6;
      }
      tmp_moving_speed += camera.ball_dis / 4;

      tmp_moving_dir = SimplifyDeg(tmp_moving_dir - 180);  // ０度の時に180度に動くように変換
      motor.Drive(tmp_moving_dir, tmp_moving_speed);
}

void BackCureveShoot() {
      static int8_t shoot_dir;

      dribblerBack.Hold(HOLD_MAX_POWER);
      holdBackTimer.start();
      if ((camera.ops_goal_size > 30 && abs(camera.ops_goal_dir) < 45) || readms(CurveShootTimer) > 50) {
            CurveShootTimer.start();
            if (readms(CurveShootTimer) < 50) {
                  motor.Brake();
                  shoot_dir = camera.ops_goal_dir > 0 ? -1 : 1;
            } else if (abs(own_dir) < 45) {
                  motor.Drive(0, 0, 60 * shoot_dir, BACK, 30);
            } else if (abs(own_dir) < 135) {
                  motor.Drive(0, 0, 160 * shoot_dir);
            } else {
                  CurveShootTimer.stop();
                  CurveShootTimer.reset();
                  holdBackTimer.stop();
                  holdBackTimer.reset();
                  enable_back_curve_shoot = 0;
            }
      } else if (readms(holdBackTimer) < 500) {
            if (sensors.hold_back == 1) holdBackTimer.reset();
            if (camera.ops_goal_size >= 30) {
                  tmp_moving_dir = 90 + (camera.ops_goal_size - 35) * 3;
                  if (camera.ops_goal_dir < 0) tmp_moving_dir *= -1;
                  motor.Drive(tmp_moving_dir, 40);
            } else {
                  motor.Drive(camera.ops_goal_dir * 2, 40);
            }
      } else {
            enable_back_curve_shoot = 0;
            holdBackTimer.stop();
            holdBackTimer.reset();
      }
}

void HoldFrontMove() {
      if (camera.ops_goal_size > 15 && (abs(camera.ops_goal_dir - own_dir) < 3)) {  // キッカーを打つ条件
            dribblerFront.Brake();
            kicker.Kick();
      } else {
            dribblerFront.Hold(HOLD_MAX_POWER);

            if (is_first_hold == 1) {
                  motor.Brake();
                  is_first_hold = 0;
            }

            if (IS_OPS_GOAL_FOUND) {
                  robot_dir = camera.ops_goal_dir;
                  if (abs(robot_dir) > 45) robot_dir = 45 * (abs(robot_dir) / robot_dir);
                  if (abs(camera.ops_goal_dir) > 45) {
                        motor.Drive(0, 0, robot_dir, FRONT, 25);
                  } else {
                        motor.Drive(tmp_moving_dir - own_dir, moving_speed, robot_dir, FRONT, 25);
                  }
            } else {
                  if (camera.own_x > 0) {
                        tmp_moving_dir = -45;
                  } else {
                        tmp_moving_dir = 45;
                  }
                  motor.Drive(tmp_moving_dir, moving_speed, 0, FRONT, 10);
            }
      }
}

void HoldBackMove() {
      if (is_first_hold == 1) {
            motor.Brake(100);
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

      if (abs(camera.ball_dir + own_dir) < 90 && readms(lineStopTimer) > 500 && abs(sensors.line_inside_dir + own_dir) > 135) {  // ラインを割る
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
                  tmp_moving_speed = vector_mag * 7.5;
                  robot_dir = 0;
                  robot_dir = camera.ball_dir;
                  if (abs(camera.ball_dir + own_dir) < 120) {
                        robot_dir = camera.ball_dir;
                  } else {
                        robot_dir = camera.inverse_ball_dir;
                  }
                  if (abs(robot_dir) > 45) robot_dir = 45 * (abs(robot_dir) / robot_dir);
                  if (tmp_moving_speed > line_moving_speed) tmp_moving_speed = line_moving_speed;
                  motor.Drive(vector_dir + own_dir, tmp_moving_speed, robot_dir);
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

                  if (enable_back_curve_shoot == 1) {  // 後ろドリブラーのマカオシュート
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