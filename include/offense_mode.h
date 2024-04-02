#ifndef _OFFENSE_MODE_H_
#define _OFFENSE_MODE_H_
#include "setup.h"

#define IS_BALL_NEAR_OF_FRONT (abs(camera.ball_dir) < 30 && camera.ball_dis > 90)
#define IS_BALL_NEAR_OF_BACK (abs(camera.inverse_ball_dir) < 30 && camera.ball_dis > 90)
#define IS_OPS_GOAL_FOUND (camera.ops_goal_size != 0)
#define IS_OWN_GOAL_FOUND (camera.own_goal_size != 0)

#define DEPTH_OF_WRAP 100.0f
#define DISTORTION_OF_WRAP 2.0

Timer CurveShootTimer;
Timer holdBackTimer;
Timer lineStopTimer;
Timer goToCenterTimer;
Timer wrapTimer;
Timer frontCurveShootTimer;
Timer holdFrontTimer;

bool is_pre_line_left = 0;
bool is_pre_line_right = 0;
bool is_pre_line = 0;
bool enable_back_curve_shoot = 0;
bool enable_front_curve_shoot = 0;
bool enable_front_hide_move = 0;
bool is_first_hold = 1;
int16_t tmp_moving_dir, tmp_moving_speed, robot_dir;

int16_t hide_robot_dir = 0;
bool face_to_goal = 0;
int16_t shoot_goal_dir = 0;

void WrapToFront() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.ball_dir) <= 30) {
            wrap_deg_addend = camera.ball_dir * (abs(camera.ball_dir) / 10.0f);
      } else {
            wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
      }
      tmp_moving_dir = SimplifyDeg(camera.ball_dir + (wrap_deg_addend * pow((camera.ball_dis / DEPTH_OF_WRAP), DISTORTION_OF_WRAP)));

      if (abs(camera.ball_dir) < 10) wrapTimer.start();
      if (abs(camera.ball_dir) > 30) {
            wrapTimer.reset();
            wrapTimer.stop();
      }

      // 速度
      wrapDirPID.Compute(camera.ball_dir, 0);

      if (camera.ball_dis < 80) {
            tmp_moving_speed = moving_speed;
      } else if (readms(wrapTimer) > 250) {
            tmp_moving_speed = 125 - camera.ball_dis + abs(camera.ball_dir);
      } else if ((camera.ops_goal_size > 40 && abs(tmp_moving_dir) < 45) || (camera.own_goal_size > 40 && abs(tmp_moving_dir) > 135) || (camera.own_x > 25 && tmp_moving_dir > 45 && tmp_moving_dir < 135) || (camera.own_x < -25 && tmp_moving_dir < -45 && tmp_moving_dir > -135)) {
            tmp_moving_speed = 50;
      } else {
            tmp_moving_speed = abs(wrapDirPID.Get());
      }

      if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;

      motor.Run(tmp_moving_dir, tmp_moving_speed);
}

void WrapToBack() {
      int16_t wrap_deg_addend;
      // 角度
      if (abs(camera.inverse_ball_dir) <= 30) {
            wrap_deg_addend = camera.inverse_ball_dir * (abs(camera.inverse_ball_dir) / 10.0f);
      } else {
            wrap_deg_addend = 90 * (abs(camera.inverse_ball_dir) / camera.inverse_ball_dir);
      }
      tmp_moving_dir = camera.inverse_ball_dir + (wrap_deg_addend * pow((camera.ball_dis / DEPTH_OF_WRAP), DISTORTION_OF_WRAP));

      if (abs(camera.inverse_ball_dir) < 10) wrapTimer.start();
      if (abs(camera.inverse_ball_dir) > 45) {
            wrapTimer.reset();
            wrapTimer.stop();
      }

      // 速度
      wrapDirPID.Compute(camera.inverse_ball_dir, 0);

      if (camera.ball_dis < 80) {
            tmp_moving_speed = moving_speed;
      } else if (readms(wrapTimer) > 500) {
            tmp_moving_speed = 125 - camera.ball_dis;
      } else {
            tmp_moving_speed = abs(wrapDirPID.Get());
      }

      tmp_moving_dir = SimplifyDeg(tmp_moving_dir - 180);  // ０度の時に180度に動くように変換
      motor.Run(tmp_moving_dir, tmp_moving_speed);
}

void FrontCurveShoot() {
      if (shoot_goal_dir == 0) shoot_goal_dir = camera.ops_goal_dir > 0 ? 85 : -85;
      dribblerFront.Hold(HOLD_MAX_POWER);
      frontCurveShootTimer.start();
      holdFrontTimer.start();
      if (abs(camera.ops_goal_dir - own_dir) < 10) {
            if (face_to_goal == 0) {
                  frontCurveShootTimer.reset();
                  face_to_goal = 1;
            }
            motor.Free();
            if (readms(frontCurveShootTimer) > 250) {
                  frontCurveShootTimer.reset();
                  frontCurveShootTimer.stop();
                  holdFrontTimer.reset();
                  holdFrontTimer.stop();
                  dribblerFront.Brake();
                  kicker.Kick();
                  enable_front_curve_shoot = 0;
                  shoot_goal_dir = 0;
                  face_to_goal = 0;
            }
      } else {
            if (sensors.hold_front == 1) holdFrontTimer.reset();
            if (readms(holdFrontTimer) > 300) {
                  frontCurveShootTimer.reset();
                  frontCurveShootTimer.stop();
                  holdFrontTimer.reset();
                  holdFrontTimer.stop();
                  dribblerFront.Brake();
                  kicker.Kick();
                  enable_front_curve_shoot = 0;
                  shoot_goal_dir = 0;
                  face_to_goal = 0;
            }
            if (readms(frontCurveShootTimer) <= 1000) {
                  motor.Run(0, 0, shoot_goal_dir * (readms(frontCurveShootTimer) / 1000.0f), 0, 35);
            } else {
                  frontCurveShootTimer.reset();
            }
      }
}

void FrontHideMove() {
      if (hide_robot_dir == 0) hide_robot_dir = camera.own_x > 0 ? 60 : -60;
      holdFrontTimer.start();
      dribblerFront.Hold(HOLD_MAX_POWER);

      if (readms(holdFrontTimer) > 300 || abs(camera.ops_goal_dir) > 65) {
            if (abs(own_dir) > 10 && readms(holdFrontTimer) <= 1500) {
                  motor.Run(0, 0, hide_robot_dir * (1 - readms(holdFrontTimer) / 1500.0f), 0, 35);
            } else {
                  enable_front_hide_move = 0;
                  hide_robot_dir = 0;
                  holdFrontTimer.reset();
                  holdFrontTimer.stop();
            }
      } else {
            if (sensors.hold_front == 1) holdFrontTimer.reset();
            if (hide_robot_dir > 0) {
                  tmp_moving_dir = (35 - camera.own_x) * 3;
            } else {
                  tmp_moving_dir = (-35 - camera.own_x) * 3;
            }
            motor.Run(tmp_moving_dir - own_dir, 30, hide_robot_dir, FRONT, 10);
      }
}

void BackCureveShoot() {
      static int8_t shoot_dir;

      dribblerBack.Hold(HOLD_MAX_POWER);
      holdBackTimer.start();
      if ((camera.ops_goal_size > 35 && abs(camera.enemy_dir) < 5 && abs(camera.ops_goal_dir) < 45) || readms(CurveShootTimer) > 50) {
            CurveShootTimer.start();
            if (readms(CurveShootTimer) < 50) {
                  motor.Brake();
                  if (camera.enemy_dir != 0) {
                        shoot_dir = camera.enemy_dir > 0 ? 1 : -1;
                  } else {
                        shoot_dir = camera.ops_goal_dir > 0 ? -1 : 1;
                  }
            } else if (abs(own_dir) < 40) {
                  motor.Run(0, 0, 55 * shoot_dir, BACK, 30);
            } else if (abs(own_dir) < 135) {
                  motor.Run(0, 0, 160 * shoot_dir);
            } else {
                  CurveShootTimer.stop();
                  CurveShootTimer.reset();
                  holdBackTimer.stop();
                  holdBackTimer.reset();
                  enable_back_curve_shoot = 0;
            }
      } else if (readms(holdBackTimer) < 250) {
            if (sensors.hold_back == 1) holdBackTimer.reset();
            if (camera.ops_goal_size >= 30) {
                  tmp_moving_dir = 90 + (camera.ops_goal_size - 35) * 3;
                  if (camera.ops_goal_dir < 0) tmp_moving_dir *= -1;
                  motor.Run(tmp_moving_dir, 30);
            } else {
                  motor.Run(camera.ops_goal_dir * 2, 30);
            }
      } else {
            enable_back_curve_shoot = 0;
            holdBackTimer.stop();
            holdBackTimer.reset();
      }
      /*
      static int8_t shoot_dir;
      CurveShootTimer.start();

      dribblerBack.Hold(HOLD_MAX_POWER);
      if (readms(CurveShootTimer) < 50) {
            motor.Brake();
            shoot_dir = camera.ops_goal_dir > 0 ? -1 : 1;
      } else if (readms(CurveShootTimer) < 750) {
            motor.Run(0, 0, 0, BACK, 30);
      } else if (readms(CurveShootTimer) < 1000) {
            motor.Run(0, 0, 160 * shoot_dir);
      } else {
            CurveShootTimer.stop();
            CurveShootTimer.reset();
            enable_back_curve_shoot = 0;
      }
      if (readms(CurveShootTimer) < 50 && holdBack.IsHold() == 0) {
            CurveShootTimer.stop();
            CurveShootTimer.reset();
            enable_back_curve_shoot = 0;
      }*/
}

void HoldFrontMove() {
      if (camera.ops_goal_size > 15 && camera.is_goal_front == 1 && sensors.dis[0] > 15) {  // キッカーを打つ条件
            dribblerFront.Brake();
            kicker.Kick();
      } else {
            dribblerFront.Hold(HOLD_MAX_POWER);

            if (is_first_hold == 1) {
                  motor.Brake(100);
                  is_first_hold = 0;
            }

            if (IS_OPS_GOAL_FOUND) {
                  robot_dir = camera.ops_goal_dir;
                  if (abs(robot_dir) > 45) robot_dir = 45 * (abs(robot_dir) / robot_dir);
                  // if (camera.own_y < 0 && abs(camera.own_x) > 35) {
                  //       enable_front_hide_move = 1;
                  // } else if (abs(camera.ops_goal_dir) > 65) {
                  //       motor.Run(180, 25);
                  // } else if (abs(camera.ops_goal_dir) > 50) {
                  //       enable_front_curve_shoot = 1;
                  if (abs(camera.ops_goal_dir) > 45) {
                        tmp_moving_dir = camera.ops_goal_dir * 2.5;
                        if (abs(tmp_moving_dir) > 180) tmp_moving_dir = 180 * (abs(tmp_moving_dir) / tmp_moving_dir);
                        motor.Run(tmp_moving_dir - own_dir, 25, robot_dir, FRONT, 10);
                  } else if (camera.ops_goal_size > 30 && sensors.dis[0] <= 20) {
                        tmp_moving_dir = (90 + (20 - sensors.dis[0]) * 3) * (abs(robot_dir) / robot_dir);
                        motor.Run(tmp_moving_dir - own_dir, 50, robot_dir, FRONT, 10);
                  } else {
                        if (sensors.dis[0] < 10) {
                              if (abs(own_dir) > 30) kicker.Kick();
                              motor.Run(0, 100, 45 * (abs(robot_dir) / robot_dir), FRONT, 25);
                        } else {
                              if (sensors.dis[1] < 20 && sensors.dis[3] > 20) {
                                    tmp_moving_dir = -30;
                              } else if (sensors.dis[3] < 20 && sensors.dis[1] > 20) {
                                    tmp_moving_dir = 30;
                              } else {
                                    tmp_moving_dir = camera.ops_goal_dir * 2;
                              }
                              motor.Run(tmp_moving_dir - own_dir, moving_speed, robot_dir, FRONT, 10);
                        }
                  }
            } else {
                  if (sensors.dis[0] < 10) {
                        motor.Run(0, moving_speed, sensors.dis[1] > sensors.dis[3] ? 60 : -60);
                  } else {
                        if (sensors.dis[1] < 30 && sensors.dis[3] > 30) {
                              tmp_moving_dir = -45;
                        } else if (sensors.dis[3] < 30 && sensors.dis[1] > 30) {
                              tmp_moving_dir = 45;
                        }
                        motor.Run(tmp_moving_dir, moving_speed, 0, FRONT, 10);
                  }
            }
      }
}

void HoldBackMove() {
      if (is_first_hold == 1) {
            motor.Brake(100);
            is_first_hold = 0;
      }
      enable_back_curve_shoot = 1;
      /*
      dribblerBack.Hold(HOLD_MAX_POWER);
      if (camera.ops_goal_size > 40) {  // キッカーを打つ条件
            enable_back_curve_shoot = 1;
      } else {
            static int16_t tmp_robot_dir;
            if (is_first_hold == 1) {
                  motor.Brake(100);
                  is_first_hold = 0;
                  tmp_robot_dir = camera.own_x > 0 ? -45 : 45;
            }
            if (tmp_robot_dir > 0) {
                  tmp_moving_dir = -25 - camera.own_x;
            } else {
                  tmp_moving_dir = 25 - camera.own_x;
            }
            motor.Run(tmp_moving_dir - own_dir, 30, tmp_robot_dir, BACK, 15);
      }*/
}

void LineMove() {
      if (sensors.is_on_line == 1) is_pre_line = 1;
      if (sensors.is_line_left == 1) is_pre_line_left = 1;
      if (sensors.is_line_right == 1) is_pre_line_right = 1;

      float vector_x, vector_y, vector_mag, vector_dir;
      float line_vector_rate, ball_vector_rate;

      if (abs(camera.ball_dir + own_dir) < 90 && readms(lineStopTimer) > 500 && abs(sensors.line_inside_dir + own_dir) > 135 && sensors.dis[0] >= 10) {  // ラインを割る
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
                  tmp_moving_speed = vector_mag * 7.5;
                  robot_dir = 0;
                  robot_dir = camera.ball_dir + own_dir;
                  if (abs(camera.ball_dir + own_dir) < 120) {
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

void goToCenter() {
      if (!IS_OPS_GOAL_FOUND || !IS_OWN_GOAL_FOUND) {
            if (sensors.dis[0] > sensors.dis[2]) {
                  if (sensors.dis[1] > sensors.dis[3]) {
                        tmp_moving_dir = 45;
                  } else {
                        tmp_moving_dir = -45;
                  }
            } else {
                  if (sensors.dis[1] > sensors.dis[3]) {
                        tmp_moving_dir = 135;
                  } else {
                        tmp_moving_dir = -135;
                  }
            }
            motor.Run(tmp_moving_dir, abs(sensors.dis[0] - sensors.dis[2]) + abs(sensors.dis[1] - sensors.dis[3]));
      } else {
            motor.Run(camera.center_dir, camera.center_dis * 10);
      }
}

void OffenseMove() {
      if (sensors.is_on_line == 1 || ((sensors.is_line_left == 1 || sensors.is_line_right == 1) && motor.GetPreMovingSpeed() > 50 && abs(motor.GetPreMovingDir()) > 45 && abs(motor.GetPreMovingDir()) < 135)) {  // ラインセンサの処理
            LineMove();
      } else if (camera.ball_dis == 0) {  // ボールがない時の処理
            goToCenterTimer.start();
            if (readms(goToCenterTimer) > 1000) {
                  goToCenter();
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

                  if (sensors.ir_dis != 0) {
                        ultrasonic.OnIrLed(0, 1, 1, 0);
                  } else {
                        ultrasonic.OffIrLed();
                  }

                  if (enable_back_curve_shoot == 1) {  // 後ろドリブラーのマカオシュート
                        BackCureveShoot();
                  } else if (enable_front_curve_shoot == 1) {  // 後ろドリブラーのマカオシュート
                        FrontCurveShoot();
                  } else if (enable_front_hide_move == 1) {  // 後ろドリブラーのマカオシュート
                        FrontHideMove();
                  } else if (holdFront.IsHold()) {  // 前に捕捉している時
                        HoldFrontMove();
                  } else if (holdBack.IsHold()) {  // 後ろに捕捉している時
                        HoldBackMove();
                        //} else if (sensors.ir_dis != 0) {
                        //      dribblerBack.Hold();
                        //      tmp_moving_dir = atan2(camera.ball_x, camera.ball_y + 30) * 180.0f / PI;
                        //    tmp_moving_speed = abs(camera.inverse_ball_dir) * 3 + abs(sqrt(pow(camera.ball_x, 2) + pow(camera.ball_y + 30, 2))) * 3;
                        //      if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;
                        //      motor.Run(tmp_moving_dir, tmp_moving_speed);
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