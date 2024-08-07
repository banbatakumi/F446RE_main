#ifndef _LIGHT_OFFENSE_MODE_H_
#define _LIGHT_OFFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(sensors.ir_dir) < 30 && sensors.ir_dis > 90)
#define IS_BALL_NEAR_OF_BACK (abs(camera.inverse_ball_dir) < 30 && sensors.ir_dis > 90)
#define IS_OPS_GOAL_FOUND (camera.ops_goal_size != 0)
#define IS_OWN_GOAL_FOUND (camera.own_goal_size != 0)

#define DEPTH_OF_WRAP 95.0f
#define DISTORTION_OF_WRAP 1.5

Timer CurveShootTimer;
Timer lineStopTimer;
Timer goToCenterTimer;
Timer wrapTimer;

bool is_pre_line_left = 0;
bool is_pre_line_right = 0;
bool is_pre_line = 0;
bool enable_back_curve_shoot = 0;
int16_t tmp_moving_dir, tmp_moving_speed, robot_dir;

void WrapToFront() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(sensors.ir_dir) < 60) {
            wrap_deg_addend = sensors.ir_dir * 1.5;
      } else {
            wrap_deg_addend = 90 * (abs(sensors.ir_dir) / sensors.ir_dir);
      }
      tmp_moving_dir = sensors.ir_dir + (wrap_deg_addend);

      // 速度
      wrapDirPID.Compute(sensors.ir_dir, 0);

      tmp_moving_speed = moving_speed;

      if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;

      robot_dir = camera.ops_goal_dir;

      motor.Drive(tmp_moving_dir, tmp_moving_speed, robot_dir);
}

void LineMove() {
      if (sensors.is_on_line == 1) is_pre_line = 1;
      if (sensors.is_line_left == 1) is_pre_line_left = 1;
      if (sensors.is_line_right == 1) is_pre_line_right = 1;

      float vector_x, vector_y, vector_mag, vector_dir;
      float line_vector_rate, ball_vector_rate;

      if (abs(sensors.ir_dir) < 90 && readms(lineStopTimer) > 500 && abs(sensors.line_inside_dir + own_dir) > 135) {  // ラインを割る
            line_vector_rate = 0.4;
            ball_vector_rate = 0.6;
      } else {  // 通常待機
            line_vector_rate = 0.6;
            ball_vector_rate = 0.4;
      }

      vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * line_vector_rate;
      vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * line_vector_rate;
      vector_x += ((24 - sensors.line_depth) * MySin(sensors.ir_dir * 1.5)) * ball_vector_rate;
      vector_y += ((24 - sensors.line_depth) * MyCos(sensors.ir_dir * 1.5)) * ball_vector_rate;
      vector_dir = atan2(vector_x, vector_y) * 180.0f / PI;
      vector_mag = abs(sqrt(pow(vector_x, 2) + pow(vector_y, 2)));

      lineStopTimer.start();
      if (readms(lineStopTimer) < 100 || readms(lineStopTimer) > 5000 || sensors.ir_dis == 0) {
            if (sensors.is_on_line == 1) {
                  motor.Drive(sensors.line_inside_dir, line_moving_speed, robot_dir);
            } else if (sensors.is_line_left == 1) {
                  motor.Drive(90, line_moving_speed, robot_dir);
            } else if (sensors.is_line_right == 1) {
                  motor.Drive(-90, line_moving_speed, robot_dir);
            }
      } else if (holdFront.IsHold()) {
            if (sensors.is_on_line == 1) {
                  motor.Drive(sensors.line_inside_dir, 25, robot_dir);
            } else if (sensors.is_line_left == 1) {
                  motor.Drive(90, 25, robot_dir);
            } else if (sensors.is_line_right == 1) {
                  motor.Drive(-90, 25, robot_dir);
            }
      } else {
            if (sensors.is_on_line == 1) {
                  tmp_moving_speed = vector_mag * 7.5;
                  if (tmp_moving_speed > line_moving_speed) tmp_moving_speed = line_moving_speed;
                  motor.Drive(vector_dir + own_dir, tmp_moving_speed);
            } else if (sensors.is_line_left == 1) {
                  if (SimplifyDeg(sensors.ir_dir + own_dir) < 0) {
                        motor.Drive(-90, 50);
                  } else {
                        motor.Drive(90, 50);
                  }
            } else if (sensors.is_line_right == 1) {
                  if (SimplifyDeg(sensors.ir_dir + own_dir) > 0) {
                        motor.Drive(90, 50);
                  } else {
                        motor.Drive(-90, 50);
                  }
            }
      }
}

void OffenseMove() {
      robot_dir = camera.ops_goal_dir;
      if (abs(robot_dir) > 30) robot_dir = 30 * (abs(robot_dir) / robot_dir);
      if (sensors.is_on_line == 1 || ((sensors.is_line_left == 1 || sensors.is_line_right == 1) && motor.GetPreMovingSpeed() > 50 && abs(motor.GetPreMovingDir()) > 45 && abs(motor.GetPreMovingDir()) < 135)) {  // ラインセンサの処理
            LineMove();
      } else if (sensors.ir_dis == 0) {  // ボールがない時の処理
            goToCenterTimer.start();
            if (readms(goToCenterTimer) > 1000) {
                  motor.Drive(camera.center_dir, camera.center_dis * 10);
            } else {
                  motor.Drive();
            }
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

                  if (holdFront.IsHold()) kicker.Kick();
                  WrapToFront();
            }
      }
}

#endif