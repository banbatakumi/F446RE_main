#ifndef _DEFENSE_MODE_H_
#define _DEFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir + own_dir) < 30 && camera.ball_dis > 0 && camera.ball_dis < 10)

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
      if (sensors.line_dir > 0 && sensors.line_dir < 90) {
            ball_almost_angle = sensors.line_dir;
      } else if (sensors.line_dir > -180 && sensors.line_dir < -90) {
            ball_almost_angle = 180 + sensors.line_dir;
      } else if (sensors.line_dir > -90 && sensors.line_dir < 0) {
            ball_almost_angle = sensors.line_dir;
      } else if (sensors.line_dir > 90 && sensors.line_dir < 180) {
            ball_almost_angle = sensors.line_dir - 180;
      }
      if (camera.ball_dir < 0) {
            ball_almost_angle -= 90;
      } else {
            ball_almost_angle += 90;
      }

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
            tmp_moving_speed = abs(camera.ball_x) * 10;
            if (abs(camera.ball_dir) < 5 || abs(camera.ball_dir) > 120) tmp_moving_speed = 0;
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

      if ((abs(camera.ball_dir) < 10 && camera.ball_dis > 0 && camera.ball_dis < 50) || sensors.hold_front == 1) {
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
                  } else {
                        if (bt.is_connect_to_ally && bt.is_ally_moving) {
                              esp32.is_defense = 0;
                        } else {
                              if (camera.is_goal_front && camera.own_y > 0) kicker.Kick();
                              motor.Drive(0, moving_speed, camera.ops_goal_dir, 0, 20);
                        }
                        defenseShootTimer.reset();
                        defenseShootTimer.stop();
                  }
            } else {
                  motor.Drive(camera.ball_dir * 1.5, moving_speed);
                  if (readms(defenseShootTimer) > BALL_WAIT_TIME + 1000) {
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

                  if (sensors.is_line_left) {
                        motor.Drive(135, moving_speed);
                  } else if (sensors.is_line_right) {
                        motor.Drive(-135, moving_speed);
                  } else if (camera.own_goal_size > 70) {
                        motor.Drive(0, moving_speed);
                  } else {
                        if (camera.back_proximity[2] && camera.back_proximity[3] && camera.back_proximity[4]) {
                              motor.Drive(90 * abs(camera.own_x) / camera.own_x, 50);
                        } else if (camera.back_proximity[0] && camera.back_proximity[1] && camera.back_proximity[2]) {
                              motor.Drive(-120, 50);
                        } else if (camera.back_proximity[4] && camera.back_proximity[5] && camera.back_proximity[6]) {
                              motor.Drive(120, 50);
                        } else if (camera.own_y > -20) {
                              if (camera.own_x > 0) {
                                    motor.Drive(180 - (30 - camera.own_x) * 2, moving_speed);
                              } else {
                                    motor.Drive(180 + (30 + camera.own_x) * 2, moving_speed);
                              }
                        } else {
                              motor.Drive(camera.own_goal_dir * 0.9, 50);
                        }
                  }
            }
      }
}

#endif