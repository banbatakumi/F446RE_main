#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include "defense_mode.h"
#include "offense_mode.h"
// #include "light_defense_mode.h"
// #include "light_offense_mode.h"
#include "setup.h"

Timer changeTimer;

void ModeRun() {
      voltage.Read();

      for (uint8_t i = 0; i < 4; i++) {
            motor.encoder_val[i] = sensors.encoder_val[i];
      }

      GetSensors();
      if (uiSerial.readable()) Ui();
      esp32.is_catch_ball = sensors.hold_front | sensors.hold_back;

      if (mode == 0) {
            motor.Free();
            esp32.is_defense = 1;
      } else if (mode == 1) {
            if (bt.is_connect_to_ally) {
                  if (bt.is_ally_defense) {
                        esp32.is_defense = 0;
                        OffenseMove();
                  } else {
                        esp32.is_defense = 1;
                        DefenseMove();
                  }
            } else {
                  OffenseMove();
            }
      } else if (mode == 2) {
            if (bt.is_connect_to_ally) {
                  if (esp32.is_defense == 0) {
                        OffenseMove();
                        changeTimer.start();
                        if (readms(changeTimer) > 1000 && bt.is_ally_defense == 0) {
                              changeTimer.stop();
                              changeTimer.reset();
                              esp32.is_defense = 1;
                        }
                  } else {
                        esp32.is_defense = 1;
                        DefenseMove();
                  }
            } else {
                  DefenseMove();
            }
      } else if (mode == 3) {
            // if (esp32.moving == 1) {
            //       motor.Drive(0, esp32.speed);
            // } else if (esp32.moving == 2) {
            //       motor.Drive(45, esp32.speed);
            // } else if (esp32.moving == 3) {
            //       motor.Drive(90, esp32.speed);
            // } else if (esp32.moving == 4) {
            //       motor.Drive(135, esp32.speed);
            // } else if (esp32.moving == 5) {
            //       motor.Drive(180, esp32.speed);
            // } else if (esp32.moving == 6) {
            //       motor.Drive(-135, esp32.speed);
            // } else if (esp32.moving == 7) {
            //       motor.Drive(-90, esp32.speed);
            // } else if (esp32.moving == 8) {
            //       motor.Drive(-45, esp32.speed);
            // } else {
            //       motor.Drive();
            // }
            // if (esp32.do_kick) kicker.Kick();
            // if (esp32.do_dribble) {
            //       dribblerFront.Hold(90);
            // } else {
            //       dribblerFront.Hold(0);
            // }
            kicker.Discharge();
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
      camera.ball_velocity_x = cam.GetBallVelocityX();
      camera.ball_velocity_y = cam.GetBallVelocityY();
      camera.own_x = cam.GetOwnX();
      camera.own_y = cam.GetOwnY();
      camera.center_dir = cam.GetCenterDir();
      camera.center_dis = cam.GetCenterDis();
      camera.front_proximity[0] = cam.front_proximity >> 6 & 1;
      camera.front_proximity[1] = cam.front_proximity >> 5 & 1;
      camera.front_proximity[2] = cam.front_proximity >> 4 & 1;
      camera.front_proximity[3] = cam.front_proximity >> 3 & 1;
      camera.front_proximity[4] = cam.front_proximity >> 2 & 1;
      camera.front_proximity[5] = cam.front_proximity >> 1 & 1;
      camera.front_proximity[6] = cam.front_proximity & 1;
      camera.back_proximity[0] = cam.back_proximity >> 6 & 1;
      camera.back_proximity[1] = cam.back_proximity >> 5 & 1;
      camera.back_proximity[2] = cam.back_proximity >> 4 & 1;
      camera.back_proximity[3] = cam.back_proximity >> 3 & 1;
      camera.back_proximity[4] = cam.back_proximity >> 2 & 1;
      camera.back_proximity[5] = cam.back_proximity >> 1 & 1;
      camera.back_proximity[6] = cam.back_proximity & 1;

      sensors.ir_dir = esp32.ir_dir;
      sensors.ir_dis = esp32.ir_dis;
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

      bt.is_connect_to_ally = esp32.is_connect_to_ally;
      bt.is_ally_catch_ball = esp32.is_ally_catch_ball;
      bt.is_ally_defense = esp32.is_ally_defense;
      bt.is_ally_moving = esp32.is_ally_moving;
      bt.can_ally_get_pass = esp32.can_ally_get_pass;

      own_dir = imu.GetYaw();
      pitch = imu.GetPitch();
      roll = imu.GetRoll();
}

void Ui() {
      static int8_t item = 0;
      static uint8_t sub_item = 0;

      static uint8_t dribbler_sig = 0;
      static bool is_own_dir_correction = 0;
      static uint8_t dir_correction_cnt;

      static uint8_t data_length;
      const uint8_t recv_data_num = 5;
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
                  sub_item = recv_data[1] >> 4;
                  mode = recv_data[1] & 0b00001111;
                  is_own_dir_correction = recv_data[2] >> 4;
                  dribbler_sig = recv_data[2] & 0b00001111;
                  moving_speed = recv_data[3];
                  line_moving_speed = recv_data[4];

                  if (mode > 3) mode = 0;
                  if (mode != 0 || item == 1) {
                        line.LedOn();
                  } else {
                        line.LedOff();
                  }

                  if (mode == 0 || mode == 3) {
                        if (is_own_dir_correction == 1) {
                              dir_correction_cnt++;
                        } else {
                              dir_correction_cnt = 0;
                        }
                        if (dir_correction_cnt > 2) {
                              esp32.TopYawSetZero();
                              imu.YawSetZero();
                              dir_correction_cnt = 0;
                        }
                        if (item == 3) {
                              esp32.OnIrLed();
                        } else {
                              esp32.OffIrLed();
                        }
                        if (item == 0 && (sub_item == 1 || sub_item == 2)) {
                              esp32.EnableRorate(1);
                        } else {
                              esp32.EnableRorate(0);
                        }

                        // 送信
                        uint8_t send_byte_num = 0;
                        uint8_t send_byte[15];
                        send_byte[0] = 0xFF;

                        if (item == -2) {
                              send_byte_num = 1;
                              send_byte[0] = sensors.hold_front << 1 | sensors.hold_back;
                        } else if (item == 0) {
                              int16_t debug_val[4];
                              debug_val[0] = holdFront.GetVal();
                              debug_val[1] = camera.own_y;
                              debug_val[2] = camera.front_proximity[5];
                              debug_val[3] = camera.front_proximity[6];

                              send_byte_num = 11;
                              send_byte[1] = uint8_t(voltage.Get() * 20);
                              send_byte[2] = esp32.is_moving << 3 | bt.is_ally_catch_ball << 2 | bt.is_ally_moving << 1 | bt.is_connect_to_ally;
                              send_byte[3] = (uint8_t)(((uint16_t)(own_dir + 32768) & 0xFF00) >> 8);
                              send_byte[4] = (uint8_t)((uint16_t)(own_dir + 32768) & 0x00FF);
                              send_byte[5] = (uint8_t)(((uint16_t)(debug_val[0] + 32768) & 0xFF00) >> 8);
                              send_byte[6] = (uint8_t)((uint16_t)(debug_val[0] + 32768) & 0x00FF);
                              send_byte[7] = (uint8_t)(((uint16_t)(debug_val[1] + 32768) & 0xFF00) >> 8);
                              send_byte[8] = (uint8_t)((uint16_t)(debug_val[1] + 32768) & 0x00FF);
                              send_byte[9] = (uint8_t)(debug_val[2]);
                              send_byte[10] = (uint8_t)(debug_val[3]);
                        } else if (item == 1) {
                              send_byte_num = 5;
                              send_byte[1] = sensors.line_dir / 2 + 90;
                              send_byte[2] = sensors.line_inside_dir / 2 + 90;
                              send_byte[3] = sensors.line_interval;
                              send_byte[4] = sensors.is_on_line << 2 | sensors.is_line_left << 1 | sensors.is_line_right;
                        } else if (item == 2) {
                              if (sub_item == 1) {
                                    send_byte_num = 5;
                                    send_byte[1] = camera.ball_dir / 2 + 90;
                                    send_byte[2] = camera.ball_dis;
                                    send_byte[3] = camera.ball_velocity_x + 127;
                                    send_byte[4] = camera.ball_velocity_y + 127;
                              } else if (sub_item == 2) {
                                    send_byte_num = 5;
                                    send_byte[1] = camera.y_goal_dir / 2 + 90;
                                    send_byte[2] = camera.y_goal_size;
                                    send_byte[3] = camera.b_goal_dir / 2 + 90;
                                    send_byte[4] = camera.b_goal_size;
                              } else {
                                    send_byte_num = 7;
                                    send_byte[1] = camera.center_dir / 2 + 90;
                                    send_byte[2] = camera.center_dis;
                                    send_byte[3] = camera.own_x + 100;
                                    send_byte[4] = camera.own_y + 100;
                                    send_byte[5] = cam.front_proximity;
                                    send_byte[6] = cam.back_proximity;
                              }
                        } else if (item == 3) {
                              send_byte_num = 3;
                              send_byte[1] = sensors.ir_dir / 2 + 90;
                              send_byte[2] = sensors.ir_dis;
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