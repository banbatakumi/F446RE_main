#ifndef _DEFENSE_MODE_H_
#define _DEFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir) < 30 && camera.ball_dis > 90)

Timer defenseShootTimer;

void LineTrace() {
      int16_t vector_x, vector_y;
      int16_t ball_almost_angle;
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
            motor.Run(0, 50);
      } else if (sensors.line_dir > 90 && sensors.line_dir < 135 && camera.ball_dir < 0) {
            motor.Run(sensors.line_dir, (12 - sensors.line_interval) * 5);
      } else if (sensors.line_dir < -90 && sensors.line_dir > -135 && camera.ball_dir > 0) {
            motor.Run(sensors.line_dir, (12 - sensors.line_interval) * 5);
      } else {
            defensePID.Compute(camera.ball_x, 0);
            int16_t tmp_moving_dir = atan2(vector_x, vector_y) * 180.0f / PI;
            int16_t tmp_moving_speed = abs(defensePID.Get());
            if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
            motor.Run(tmp_moving_dir, tmp_moving_speed);
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
      if ((abs(camera.ball_dir) < 30 && camera.ball_dis > 50) || sensors.hold_front == 1) {
            defenseShootTimer.start();
      } else {
            defenseShootTimer.reset();
            defenseShootTimer.stop();
      }

      if (readms(defenseShootTimer) > 2000) {
            if (readms(defenseShootTimer) < 3000) {
                  if (readms(defenseShootTimer) > 2250) {
                        if (sensors.is_on_line == 1) {
                              motor.Run(sensors.line_inside_dir, line_moving_speed);
                        } else if (sensors.is_line_left == 1) {
                              motor.Run(90, line_moving_speed);
                        } else if (sensors.is_line_right == 1) {
                              motor.Run(-90, line_moving_speed);
                        } else {
                              motor.Run(camera.ball_dir, moving_speed);
                        }
                  } else {
                        motor.Run(camera.ball_dir, moving_speed);
                  }
                  if (sensors.hold_front == 1 && readms(defenseShootTimer) > 2500) kicker.Kick();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();
            }
      } else {
            if (sensors.is_on_line == 1) {
                  LineTrace();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();

                  if (sensors.dis[2] < 15) {
                        motor.Run(0, moving_speed);
                  } else if (sensors.dis[2] < 30) {
                        motor.Run(camera.own_goal_dir, 50);
                  } else {
                        motor.Run(camera.own_goal_dir, moving_speed);
                  }
            }
      }
}

#endif