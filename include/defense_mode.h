#ifndef _DEFENSE_MODE_H_
#define _DEFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir) < 30 && camera.ball_dis < 100)

#define BALL_WAIT_TIME 1500

Timer defenseShootTimer;
Timer backToCenterTimer;

/*
void backToCenter() {
      int16_t vector_x, vector_y;
      int16_t almost_moving_dir;
      int16_t tmp_moving_dir;
      int16_t tmp_moving_speed;

      if (camera.own_x < 0) {
            almost_moving_dir = 90;
      } else {
            almost_moving_dir = -90;
      }

      vector_x = (12 - sensors.line_interval) * MySin(sensors.line_dir) + sensors.line_interval * MySin(almost_moving_dir);
      vector_y = (12 - sensors.line_interval) * MyCos(sensors.line_dir) + sensors.line_interval * MyCos(almost_moving_dir);

      tmp_moving_dir = atan2(vector_x, vector_y) * 180.0f / PI;
      tmp_moving_speed = abs(camera.own_x) * 5;
      if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
      motor.Drive(tmp_moving_dir, tmp_moving_speed);
}*/

void LineTrace() {
      int16_t vector_x, vector_y;
      int16_t ball_almost_angle;
      int16_t tmp_moving_dir;
      int16_t tmp_moving_speed;
      ball_almost_angle = 90;
      if (camera.ball_dir + camera.ball_velocity_x * 3 < 0) ball_almost_angle *= -1;

      vector_x = (12 - sensors.line_interval) * MySin(sensors.line_dir) + sensors.line_interval * MySin(ball_almost_angle);
      vector_y = (12 - sensors.line_interval) * MyCos(sensors.line_dir) + sensors.line_interval * MyCos(ball_almost_angle);
      int16_t robot_dir = sensors.line_dir;
      if (robot_dir > 90) robot_dir = robot_dir - 180;
      if (robot_dir < -90) robot_dir = robot_dir + 180;
      if (camera.ball_dis == 0 || abs(own_dir) > 90) {
            motor.Drive(sensors.line_dir, (12 - sensors.line_interval) * 5);
      } else if ((own_dir < -50 && camera.ball_dir < 0) || (own_dir > 50 && camera.ball_dir > 0)) {
            if (abs(own_dir) > 60) {
                  motor.Drive(-own_dir, (abs(own_dir) - 60) * 1.5, robot_dir + own_dir);
            } else {
                  tmp_moving_speed = (12 - sensors.line_interval) * 5;
                  if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
                  motor.Drive(sensors.line_dir, tmp_moving_speed, robot_dir + own_dir);
            }
      } else {
            tmp_moving_dir = atan2(vector_x, vector_y) * 180.0f / PI;
            tmp_moving_speed = abs(camera.ball_dir) + camera.ball_velocity_x * 5;
            if (abs(camera.ball_dir) < 5) tmp_moving_speed = 0;
            if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
            motor.Drive(tmp_moving_dir, tmp_moving_speed, robot_dir + own_dir);
      }
}

void DefenseMove() {
      if (sensors.hold_front == 1) {
            dribblerFront.Hold(HOLD_MAX_POWER);
      } else if (IS_BALL_NEAR_OF_FRONT) {
            dribblerFront.Hold(HOLD_WAIT_POWER);
      } else {
            dribblerFront.Hold(0);
      }
      if ((abs(camera.ball_dir) < 15 && camera.ball_dis > 80) || sensors.hold_front == 1) {
            ultrasonic.OnIrLed(1, 0, 0, 1);
      } else {
            ultrasonic.OffIrLed();
      }
      if ((abs(camera.ball_dir) < 15 && camera.ball_dis < 50) || sensors.hold_front == 1) {
            defenseShootTimer.start();
      } else {
            defenseShootTimer.reset();
            defenseShootTimer.stop();
      }

      if (readms(defenseShootTimer) > BALL_WAIT_TIME || sensors.hold_front == 1) {
            if (sensors.hold_front == 1) {
                  if (sensors.ir_dis == 0 && readms(defenseShootTimer) > BALL_WAIT_TIME + 500) {
                        motor.Drive(camera.ball_dir * 1.5, moving_speed, camera.ops_goal_dir);
                        if (sensors.hold_front == 1 && readms(defenseShootTimer) > BALL_WAIT_TIME + 2000) {
                              kicker.Kick();
                              defenseShootTimer.reset();
                              defenseShootTimer.stop();
                        }
                  } else {
                        if (abs(sensors.ir_dir) < 10 && sensors.dis[0] < 30) {
                              motor.Brake(100);
                              kicker.Kick();
                              defenseShootTimer.reset();
                              defenseShootTimer.stop();
                        }
                        motor.Drive(0, 0, sensors.ir_dir, FRONT, 20);
                  }
            } else {
                  motor.Drive(camera.ball_dir * 2, 30);
                  if (readms(defenseShootTimer) > BALL_WAIT_TIME + 2500) {
                        defenseShootTimer.reset();
                        defenseShootTimer.stop();
                  }
            }
            /*
            if (readms(defenseShootTimer) < BALL_WAIT_TIME + GO_TO_GOAL_TIME) {
                  if (readms(defenseShootTimer) > 2250) {
                        if (sensors.is_on_line == 1) {
                              motor.Drive(sensors.line_inside_dir, line_moving_speed);
                        } else if (sensors.is_line_left == 1) {
                              motor.Drive(90, line_moving_speed);
                        } else if (sensors.is_line_right == 1) {
                              motor.Drive(-90, line_moving_speed);
                        } else {
                              motor.Drive(camera.ball_dir * 1.5, moving_speed, camera.ops_goal_dir);
                        }
                  } else {
                        motor.Drive(camera.ball_dir * 1.5, moving_speed, camera.ops_goal_dir);
                  }
                  if (sensors.hold_front == 1 && readms(defenseShootTimer) > 2500) kicker.Kick();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();
            }*/
      } else {
            if (sensors.is_on_line == 1) {
                  LineTrace();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();

                  if (camera.own_goal_size > 75) {
                        motor.Drive(0, moving_speed);
                  } else if (sensors.dis[2] < 10) {
                        motor.Drive(-90 * abs(camera.own_x) / camera.own_x, 50);
                  } else if (camera.own_goal_size > 30) {
                        motor.Drive(camera.own_goal_dir, 50);
                  } else {
                        motor.Drive(camera.own_goal_dir, moving_speed);
                  }
            }
      }
}

#endif