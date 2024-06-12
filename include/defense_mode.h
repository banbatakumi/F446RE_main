#ifndef _DEFENSE_MODE_H_
#define _DEFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir) < 30 && camera.ball_dis < 15)

#define BALL_WAIT_TIME 500

Timer defenseShootTimer;
Timer backToCenterTimer;

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
}

void LineTrace() {
      int16_t vector_x, vector_y;
      int16_t ball_almost_angle;
      int16_t tmp_moving_dir;
      int16_t tmp_moving_speed;
      if ((sensors.line_dir > 45 && sensors.line_dir < 135) || (sensors.line_dir < -45 && sensors.line_dir > -135)) {
            if (abs(camera.ball_dir) < 90) {
                  ball_almost_angle = 45;
            } else {
                  ball_almost_angle = 135;
            }
      } else {
            ball_almost_angle = 90;
      }
      if (camera.ball_dir < 0) ball_almost_angle *= -1;

      vector_x = (12 - sensors.line_interval) * MySin(sensors.line_dir) + sensors.line_interval * MySin(ball_almost_angle);
      vector_y = (12 - sensors.line_interval) * MyCos(sensors.line_dir) + sensors.line_interval * MyCos(ball_almost_angle);

      if (abs(sensors.line_dir) > 60 && abs(sensors.line_dir) < 120) {
            motor.Drive(0, 50);
      } else if (camera.ball_dis == 0) {
            backToCenterTimer.start();
            if (readms(backToCenterTimer) > 1000) {
                  backToCenter();
            } else {
                  motor.Drive();
            }

            dribblerFront.Hold(0);
      } else if ((sensors.line_dir > 90 && sensors.line_dir < 135 && camera.ball_dir < 0) || (sensors.line_dir < -90 && sensors.line_dir > -135 && camera.ball_dir > 0)) {
            tmp_moving_speed = (12 - sensors.line_interval) * 5;
            if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
            motor.Drive(sensors.line_dir, tmp_moving_speed);
      } else {
            backToCenterTimer.reset();
            tmp_moving_dir = atan2(vector_x, vector_y) * 180.0f / PI;
            tmp_moving_speed = abs(camera.ball_x) * 5;
            if (abs(camera.ball_dir) < 2 || abs(camera.ball_dir) > 120) tmp_moving_speed = 0;
            if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
            motor.Drive(tmp_moving_dir, tmp_moving_speed);
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
      if ((abs(camera.ball_dir) < 15 && camera.ball_dis < 50) || sensors.hold_front == 1) {
            defenseShootTimer.start();
      } else {
            defenseShootTimer.reset();
            defenseShootTimer.stop();
      }

      if (readms(defenseShootTimer) > BALL_WAIT_TIME || sensors.hold_front == 1) {
            if (sensors.hold_front == 1) {
                  // if (readms(defenseShootTimer) > BALL_WAIT_TIME + 500) {
                  //       motor.Drive(camera.ball_dir * 1.5, moving_speed, camera.ops_goal_dir);
                  //       if (sensors.hold_front == 1 && readms(defenseShootTimer) > BALL_WAIT_TIME + 2000) {
                  //             kicker.Kick();
                  //             defenseShootTimer.reset();
                  //             defenseShootTimer.stop();
                  //       }
                  // } else {
                  //       if (abs(sensors.ir_dir) < 10) {
                  //             motor.Brake(100);
                  //             kicker.Kick();
                  //             defenseShootTimer.reset();
                  //             defenseShootTimer.stop();
                  //       }
                  //       motor.Drive(0, 0, sensors.ir_dir, FRONT, 20);
                  // }
                  if (sensors.is_on_line) {
                        motor.Drive(0, moving_speed);
                  } else if (bt.is_ally_catch_ball == 1) {
                        kicker.Kick();
                  } else {
                        esp32.is_defense = 0;
                  }
            } else {
                  motor.Drive(camera.ball_dir * 2, 50);
                  if (readms(defenseShootTimer) > BALL_WAIT_TIME + 2000) {
                        defenseShootTimer.reset();
                        defenseShootTimer.stop();
                  }
            }
      } else {
            if (sensors.is_on_line == 1) {
                  LineTrace();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();

                  if (camera.own_goal_size > 75) {
                        motor.Drive(0, moving_speed);
                  } else if (camera.own_goal_size > 30) {
                        motor.Drive(camera.own_goal_dir, 50);
                  } else {
                        motor.Drive(camera.own_goal_dir, moving_speed);
                  }
            }
      }
}

#endif