#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include "defense_mode.h"
#include "offense_mode.h"
#include "setup.h"

Timer test;

void ModeRun() {
      voltage.Read();

      for (uint8_t i = 0; i < 4; i++) {
            motor.encoder_val[i] = sensors.encoder_val[i];
      }

      GetSensors();
      if (uiSerial.readable()) Ui();

      if (mode == 0) {
            motor.Free();
      } else if (mode == 1) {
            OffenseMove();
      } else if (mode == 2) {
            DefenseMove();
      } else if (mode == 3) {
            if (sensors.is_on_line) {
                  motor.Run(sensors.line_inside_dir, line_moving_speed);
            } else if (sensors.is_line_left) {
                  motor.Run(90, line_moving_speed);
            } else if (sensors.is_line_right) {
                  motor.Run(-90, line_moving_speed);
            } else {
                  motor.Free();
                  /*
                  if (abs(camera.ball_dir) < 30 && camera.ball_dis > 80) {
                        dribblerFront.Hold(100);
                  } else {
                        dribblerFront.Hold(0);
                  }

                  int16_t wrap_deg_addend;
                  // 角度
                  if (abs(camera.ball_dir) < 45) {
                        wrap_deg_addend = camera.ball_dir * 2;
                  } else {
                        wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
                  }
                  int16_t tmp_moving_dir = camera.ball_dir + (wrap_deg_addend * pow((camera.ball_dis / DEPTH_OF_WRAP), DISTORTION_OF_WRAP));
                  int16_t robot_dir = camera.ops_goal_dir;
                  if (abs(robot_dir) > 60) robot_dir = 60 * (abs(robot_dir) / robot_dir);

                  if (abs(camera.ball_dir) < 10) wrapTimer.start();
                  if (abs(camera.ball_dir) > 30) {
                        wrapTimer.reset();
                        wrapTimer.stop();
                  }

                  // 速度
                  wrapDirPID.Compute(camera.ball_dir, 0);

                  if (camera.ball_dis < 80) {
                        tmp_moving_speed = moving_speed;
                  } else if (readms(wrapTimer) > 500) {
                        tmp_moving_speed = abs(camera.ball_dir) + 125 - camera.ball_dis;
                  } else {
                        tmp_moving_speed = abs(wrapDirPID.Get());
                  }
                  if (tmp_moving_speed > moving_speed) tmp_moving_speed = moving_speed;

                  motor.Run(tmp_moving_dir, tmp_moving_speed, robot_dir);
                  if (sensors.hold_front) kicker.Kick();*/
            }
      }
}

void GetSensors() {
      holdFront.Read();
      holdBack.Read();

      camera.ball_dir = cam.ball_dir;
      camera.ball_dis = cam.ball_dis;
      camera.inverse_ball_dir = SimplifyDeg(cam.ball_dir + 180);
      camera.y_goal_dir = cam.yellow_goal_dir;
      camera.y_goal_size = cam.yellow_goal_size;
      camera.b_goal_dir = cam.blue_goal_dir;
      camera.b_goal_size = cam.blue_goal_size;
      camera.ops_goal_dir = cam.ops_goal_dir;
      camera.ops_goal_size = cam.ops_goal_size;
      camera.own_goal_dir = cam.own_goal_dir;
      camera.own_goal_size = cam.own_goal_size;
      camera.is_goal_front = cam.is_goal_front;
      camera.ball_x = cam.GetBallX();
      camera.ball_y = cam.GetBallY();
      camera.own_x = cam.GetOwnX();
      camera.own_y = cam.GetOwnY();
      camera.center_dir = cam.GetCenterDir();
      camera.center_dis = cam.GetCenterDis();

      sensors.dis[0] = ultrasonic.val[0];
      sensors.dis[1] = ultrasonic.val[1];
      sensors.dis[2] = ultrasonic.val[2];
      sensors.dis[3] = ultrasonic.val[3];
      sensors.ir_dir = ultrasonic.ir_dir;
      sensors.ir_dis = ultrasonic.ir_dis;
      sensors.hold_front = holdFront.IsHold();
      sensors.hold_back = holdBack.IsHold();
      sensors.is_line_left = line.is_left;
      sensors.is_line_right = line.is_right;
      sensors.encoder_val[0] = line.encoder_val[0];
      sensors.encoder_val[1] = line.encoder_val[1];
      sensors.encoder_val[2] = line.encoder_val[2];
      sensors.encoder_val[3] = line.encoder_val[3];
      sensors.is_on_line = line.is_on_line;
      sensors.line_interval = line.interval;
      sensors.line_inside_dir = line.inside_dir;
      sensors.line_dir = line.dir;
      sensors.line_depth = line.GetDepth();

      own_dir = imu.GetDir();
}

void Ui() {
      static int8_t item = 0;
      static uint8_t sub_item = 0;

      static uint8_t dribbler_sig = 0;
      static bool is_own_dir_correction = 0;

      static uint8_t data_length;
      const uint8_t recv_data_num = 7;
      static uint8_t recv_data[recv_data_num];
      uint8_t read_byte;
      uiSerial.read(&read_byte, 1);

      if (data_length == 0) {
            if (read_byte == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (read_byte == 0xAA) {
                  item = recv_data[0] - 127;
                  sub_item = recv_data[1];
                  mode = recv_data[2];
                  is_own_dir_correction = recv_data[3];
                  moving_speed = recv_data[4];
                  line_moving_speed = recv_data[5];
                  dribbler_sig = recv_data[6];

                  if (mode != 0 || item == 2) {
                        line.LedOn();
                  } else {
                        line.LedOff();
                  }

                  if (mode == 0 || mode == 3) {
                        if (is_own_dir_correction == 1 && mode == 0) imu.SetZero();
                        if (item == 4) {
                              ultrasonic.OnIrLed();
                        } else {
                              ultrasonic.OffIrLed();
                        }

                        // 送信
                        uint8_t send_byte_num = 0;
                        uint8_t send_byte[10];
                        send_byte[0] = 0xFF;

                        if (item == -2) {
                              send_byte_num = 1;
                              send_byte[0] = sensors.hold_front << 1 | sensors.hold_back;
                        } else if (item == 0) {
                              int16_t debug_val_1 = own_dir;
                              int16_t debug_val_2 = sensors.encoder_val[1];
                              uint8_t debug_val_3 = sensors.encoder_val[2];
                              uint8_t debug_val_4 = sensors.encoder_val[3];

                              send_byte_num = 7;
                              send_byte[1] = uint8_t(voltage.Get() * 20);
                              send_byte[2] = debug_val_1 > 0 ? debug_val_1 : 0;
                              send_byte[3] = debug_val_1 < 0 ? debug_val_1 * -1 : 0;
                              send_byte[4] = debug_val_2;
                              send_byte[5] = debug_val_3;
                              send_byte[6] = debug_val_4;
                        } else if (item == 1) {
                              send_byte_num = 3;
                              send_byte[1] = own_dir > 0 ? own_dir : 0;
                              send_byte[2] = own_dir < 0 ? own_dir * -1 : 0;
                        } else if (item == 2) {
                              send_byte_num = 5;
                              send_byte[1] = sensors.line_dir / 2 + 90;
                              send_byte[2] = sensors.line_inside_dir / 2 + 90;
                              send_byte[3] = sensors.line_interval;
                              send_byte[4] = sensors.is_on_line << 2 | sensors.is_line_left << 1 | sensors.is_line_right;
                        } else if (item == 3) {
                              if (sub_item == 1) {
                                    send_byte_num = 3;
                                    send_byte[1] = camera.ball_dir / 2 + 90;
                                    send_byte[2] = camera.ball_dis;
                              } else if (sub_item == 2) {
                                    send_byte_num = 5;
                                    send_byte[1] = camera.y_goal_dir / 2 + 90;
                                    send_byte[2] = camera.y_goal_size;
                                    send_byte[3] = camera.b_goal_dir / 2 + 90;
                                    send_byte[4] = camera.b_goal_size;
                              } else {
                                    send_byte_num = 5;
                                    send_byte[1] = camera.center_dir / 2 + 90;
                                    send_byte[2] = camera.center_dis;
                                    send_byte[3] = camera.own_x + 100;
                                    send_byte[4] = camera.own_y + 100;
                              }
                        } else if (item == 4) {
                              send_byte_num = 3;
                              send_byte[1] = sensors.ir_dir / 2 + 90;
                              send_byte[2] = sensors.ir_dis;
                        } else if (item == 5) {
                              send_byte_num = 5;
                              send_byte[1] = sensors.dis[0];
                              send_byte[2] = sensors.dis[1];
                              send_byte[3] = sensors.dis[2];
                              send_byte[4] = sensors.dis[3];
                        }
                        uiSerial.write(&send_byte, send_byte_num);
                  }
            }
            data_length = 0;
      } else {
            recv_data[data_length - 1] = read_byte;
            data_length++;
      }

      if (mode == 0) {
            switch (dribbler_sig) {
                  case 0:
                        dribblerFront.Hold(0);
                        dribblerBack.Hold(0);
                        // if (sensors.hold_front == 1 && item == -2) kicker.Kick();
                        break;
                  case 1:
                        if (sensors.hold_front) {
                              dribblerFront.Hold(HOLD_MAX_POWER);
                        } else {
                              dribblerFront.Hold(HOLD_WAIT_POWER);
                        }
                        break;
                  case 2:
                        kicker.Kick();
                        break;
                  case 3:
                        if (sensors.hold_back) {
                              dribblerBack.Hold(HOLD_MAX_POWER);
                        } else {
                              dribblerBack.Hold(HOLD_WAIT_POWER);
                        }
                        break;
                  case 4:
                        dribblerBack.Brake();
                        break;
            }
      }
}

#endif