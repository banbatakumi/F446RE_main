#include "mbed.h"
#include "setup.h"

void setup() {
      // UART初期設定
      uiSerial.baud(UI_UART_SPEED);
      // uiSerial.attach(&Ui, Serial::RxIrq);

      // モーター
      motor.SetPwmPeriod(20);                       // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      motor.SetAttitudeControlPID(1.5, 0.25, 0.1);  // デフォルトゲイン：(1, 0.5, 0.1)

      // ドリブラー
      dribblerFront.SetPwmPeriod(20);  // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      dribblerBack.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz

      // 回り込みPID
      wrapDirPID.SetGain(1.5, 0, 2.5);
      wrapDirPID.SetSamplingPeriod(0.01);
      wrapDirPID.SetLimit(100);
      wrapDirPID.SelectType(PID_TYPE);

      defensePID.SetGain(10, 1, 1);
      defensePID.SetSamplingPeriod(0.01);
      defensePID.SetLimit(100);
      defensePID.SelectType(PID_TYPE);

      // キッカー
      kicker.SetPower(100);  // 250まで

      holdFront.SetTh();
      holdBack.SetTh();
}

int main() {
      setup();

      while (1) {
            voltage.Read();

            for (uint8_t i = 0; i < 4; i++) {
                  motor.encoder_val[i] = sensors.encoder_val[i];
            }

            GetSensors();
            if (uiSerial.readable()) Ui();

            if (mode == 0) {
                  motor.Free();
            } else if (mode == 1) {
                  OffenceMove();
            } else if (mode == 2) {
                  DefenseMove();
            } else if (mode == 3) {
                  motor.Run(0, 50);
            }
      }
}

void OffenceMove() {
      static bool is_pre_line_left = 0;
      static bool is_pre_line_right = 0;
      static bool is_pre_line = 0;
      if (sensors.is_on_line == 1 || sensors.is_line_left == 1 || sensors.is_line_right == 1) {  // ラインセンサの処理
            if (sensors.is_on_line == 1) is_pre_line = 1;
            if (sensors.is_line_left == 1) is_pre_line_left = 1;
            if (sensors.is_line_right == 1) is_pre_line_right = 1;

            float vector_x, vector_y, vector_mag, vector_dir;

            if (abs(camera.ball_dir) + own_dir < 90) {
                  if (lineStopTimer.read_ms() > 500 && abs(sensors.line_inside_dir) > 135) {
                        vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * 0.4;
                        vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * 0.4;
                        vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * 0.6;
                        vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * 0.6;
                  } else {
                        vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * 0.75;
                        vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * 0.75;
                        vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * 0.25;
                        vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * 0.25;
                  }
            } else {
                  vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * 0.75;
                  vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * 0.75;
                  vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * 0.25;
                  vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * 0.25;
            }
            vector_dir = atan2(vector_x, vector_y) * 180.00 / PI;
            vector_mag = abs(sqrt(pow(vector_x, 2) + pow(vector_y, 2)));

            lineStopTimer.start();
            if (lineStopTimer.read_ms() < 100 || lineStopTimer.read_ms() > 5000 || camera.ball_dis == 0) {
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
                  if (abs(camera.ball_dir) < 30 && camera.ball_dis > 80) {
                        dribblerFront.Hold(HOLD_WAIT_POWER);
                  } else {
                        dribblerFront.Hold(0);
                  }
                  if (abs(camera.inverse_ball_dir) < 30 && camera.ball_dis > 80) {
                        dribblerBack.Hold(HOLD_WAIT_POWER);
                  } else {
                        dribblerBack.Hold(0);
                  }
                  if (sensors.is_on_line == 1) {
                        uint16_t tmp_moving_speed = vector_mag * 5;
                        int16_t tmp_robot_dir = camera.ball_dir + own_dir;
                        if (abs(camera.ball_dir) + own_dir < 120) {
                              tmp_robot_dir = camera.ball_dir + own_dir;
                        } else {
                              tmp_robot_dir = camera.inverse_ball_dir + own_dir;
                        }
                        if (abs(tmp_robot_dir) > 60) tmp_robot_dir = 60 * (abs(tmp_robot_dir) / tmp_robot_dir);
                        if (tmp_moving_speed > line_moving_speed) tmp_moving_speed = line_moving_speed;
                        if (lineStopTimer.read_ms() > 500) {
                              motor.Run(vector_dir + own_dir, tmp_moving_speed, tmp_robot_dir);
                        } else {
                              motor.Run(vector_dir + own_dir, tmp_moving_speed);
                        }
                  } else if (sensors.is_line_left == 1) {
                        if (camera.ball_dir + own_dir < 0) {
                              motor.Run(-90, 50);
                        } else {
                              motor.Run(90, 50);
                        }
                  } else if (sensors.is_line_right == 1) {
                        if (camera.ball_dir + own_dir > 0) {
                              motor.Run(90, 50);
                        } else {
                              motor.Run(-90, 50);
                        }
                  }
            }
      } else if (camera.ball_dis == 0) {  // ボールがない時の処理
            goToCenterTimer.start();
            if (goToCenterTimer.read_ms() > 1000) {
                  motor.Run(camera.center_dir, camera.center_dis * 5);
            } else {
                  motor.Run();
            }

            dribblerFront.Hold(0);
            dribblerBack.Hold(0);
      } else {
            if (lineStopTimer.read_ms() > 500) {
                  lineStopTimer.stop();
                  lineStopTimer.reset();
                  is_pre_line_left = 0;
                  is_pre_line_right = 0;
                  is_pre_line = 0;
            }
            goToCenterTimer.reset();

            if (is_pre_line_left == 1 && lineStopTimer.read_ms() < 100) {
                  motor.Run(90, line_moving_speed);
            } else if (is_pre_line_right == 1 && lineStopTimer.read_ms() < 100) {
                  motor.Run(-90, line_moving_speed);
            } else if (is_pre_line == 1 && lineStopTimer.read_ms() < 100) {
                  motor.Brake();
            } else {
                  is_pre_line_left = 0;
                  is_pre_line_right = 0;
                  is_pre_line = 0;

                  if (BackCurveShootTimer.read() > 0) {  // 後ろドリブラーのマカオシュート
                        static int8_t shoot_dir;

                        dribblerBack.Hold(HOLD_MAX_POWER);
                        if (BackCurveShootTimer.read() < 0.1) {
                              motor.Brake();
                              shoot_dir = camera.front_goal_dir > 0 ? -1 : 1;
                        } else if (BackCurveShootTimer.read() < 1.1) {
                              motor.Run(0, 0, (BackCurveShootTimer.read() - 0.1) * 45 * shoot_dir, BACK);
                        } else if (BackCurveShootTimer.read() < 1.25) {
                              motor.Run(0, 0, 180 * shoot_dir);
                        } else {
                              BackCurveShootTimer.stop();
                              BackCurveShootTimer.reset();
                        }
                        if (BackCurveShootTimer.read() < 0.25 && holdBack.IsHold() == 0) {
                              BackCurveShootTimer.stop();
                              BackCurveShootTimer.reset();
                        }
                  } else if (holdFront.IsHold()) {                                                              // 前に捕捉している時
                        if (camera.front_goal_size > 15 && camera.is_goal_front == 1 && sensors.dis[0] > 10) {  // キッカーを打つ条件
                              motor.Run(0, moving_speed);
                              dribblerFront.Brake();
                              kicker.Kick();
                        } else {
                              dribblerFront.Hold(HOLD_MAX_POWER);

                              int16_t tmp_move_dir = (camera.front_goal_dir + own_dir) * 2;
                              if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
                              int16_t robot_dir = camera.front_goal_dir * 1.5 + own_dir;
                              if (abs(robot_dir) > 60) robot_dir = 60 * (abs(robot_dir) / robot_dir);

                              if ((camera.front_goal_dir - own_dir) > 60) {
                                    tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
                                    motor.Run(tmp_move_dir - own_dir, 25, robot_dir, FRONT, 10);
                              } else if (camera.front_goal_size > 30) {
                                    motor.Run(0, 0, camera.front_goal_dir > 0 ? 60 : -60, FRONT, 25);
                              } else {
                                    if (sensors.dis[1] < 20 && sensors.dis[3] > 20) {
                                          tmp_move_dir = -45;
                                    } else if (sensors.dis[3] < 20 && sensors.dis[1] > 20) {
                                          tmp_move_dir = 45;
                                    }
                                    motor.Run(tmp_move_dir - own_dir, moving_speed, robot_dir, FRONT, 10);
                              }
                        }
                  } else if (holdBack.IsHold()) {           // 後ろに捕捉している時
                        if (camera.front_goal_size > 30) {  // キッカーを打つ条件
                              BackCurveShootTimer.start();
                        } else {
                              dribblerBack.Hold(HOLD_MAX_POWER);

                              int16_t tmp_move_dir = camera.front_goal_dir * 1.5;
                              if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
                              motor.Run(tmp_move_dir - own_dir, 25);
                        }
                  } else {
                        if (abs(camera.ball_dir) < 30 && camera.ball_dis > 90) {
                              dribblerFront.Hold(HOLD_WAIT_POWER);
                        } else {
                              dribblerFront.Hold(0);
                        }
                        if (abs(camera.inverse_ball_dir) < 30 && camera.ball_dis > 90) {
                              dribblerBack.Hold(HOLD_WAIT_POWER);
                        } else {
                              dribblerBack.Hold(0);
                        }

                        if (abs(camera.ball_dir) < 135) {  // 前の捕捉エリアに回り込む
                              int16_t tmp_move_speed, tmp_move_angle;
                              int16_t wrap_deg_addend;

                              // 角度
                              if (abs(camera.ball_dir) < 30) {
                                    wrap_deg_addend = camera.ball_dir * (abs(camera.ball_dir) / 10.000);
                              } else {
                                    wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
                              }
                              tmp_move_angle = camera.ball_dir + (wrap_deg_addend * pow((camera.ball_dis / 95.000), 2));

                              if (abs(camera.ball_dir) < 10) wrapTimer.start();
                              if (abs(camera.ball_dir) > 45) {
                                    wrapTimer.reset();
                                    wrapTimer.stop();
                              }

                              // 速度
                              wrapDirPID.Compute(camera.ball_dir, 0);

                              if (camera.ball_dis < 80) {
                                    tmp_move_speed = moving_speed;
                              } else if (wrapTimer.read_ms() > 500) {
                                    tmp_move_speed = abs(camera.ball_dir) + (100 - camera.ball_dis) + 20;
                              } else {
                                    tmp_move_speed = abs(wrapDirPID.Get());
                              }

                              if (tmp_move_speed > moving_speed) tmp_move_speed = moving_speed;

                              motor.Run(tmp_move_angle, tmp_move_speed);
                        } else {  // 後ろの捕捉エリアに回り込む
                              int16_t tmp_move_speed, tmp_move_angle;
                              int16_t wrap_deg_addend;

                              // 角度
                              if (abs(camera.inverse_ball_dir) < 30) {
                                    wrap_deg_addend = camera.inverse_ball_dir * (abs(camera.inverse_ball_dir) / 10.000);
                              } else {
                                    wrap_deg_addend = 90 * (abs(camera.inverse_ball_dir) / camera.inverse_ball_dir);
                              }
                              tmp_move_angle = camera.inverse_ball_dir + (wrap_deg_addend * pow((camera.ball_dis / 95.000), 2));

                              if (abs(camera.inverse_ball_dir) < 10) wrapTimer.start();
                              if (abs(camera.inverse_ball_dir) > 45) {
                                    wrapTimer.reset();
                                    wrapTimer.stop();
                              }

                              // 速度
                              wrapDirPID.Compute(camera.inverse_ball_dir, 0);

                              if (camera.ball_dis < 80) {
                                    tmp_move_speed = moving_speed;
                              } else if (wrapTimer.read_ms() > 500) {
                                    tmp_move_speed = (100 - camera.ball_dis) + 20;
                              } else {
                                    tmp_move_speed = abs(wrapDirPID.Get());
                              }

                              if (tmp_move_speed > 50) tmp_move_speed = 50;

                              tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);  // ０度の時に180度に動くように変換
                              motor.Run(tmp_move_angle, tmp_move_speed);
                        }
                  }
            }
      }
}

void DefenseMove() {
      if (sensors.hold_front == 1) {
            dribblerFront.Hold(HOLD_MAX_POWER);
      } else if (abs(camera.ball_dir) < 30 && camera.ball_dis > 90) {
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

      if (defenseShootTimer.read() > 2) {
            if (defenseShootTimer.read() < 3) {
                  if (defenseShootTimer.read() > 2.25) {
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
                  if (sensors.hold_front == 1 && defenseShootTimer.read() > 2.5) kicker.Kick();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();
            }
      } else {
            if (sensors.is_on_line == 1) {
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
                        motor.Run(atan2(vector_x, vector_y) * 180.00 / PI, abs(defensePID.Get()));
                  }
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();

                  if (sensors.dis[2] < 15) {
                        motor.Run(0, moving_speed);
                  } else if (sensors.dis[2] < 30) {
                        motor.Run(camera.back_goal_dir, 50);
                  } else {
                        motor.Run(camera.back_goal_dir, moving_speed);
                  }
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
      camera.front_goal_dir = cam.front_goal_dir;
      camera.front_goal_size = cam.front_goal_size;
      camera.back_goal_dir = cam.back_goal_dir;
      camera.back_goal_size = cam.back_goal_size;
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
                              ultrasonic.IrLedOn();
                        } else {
                              ultrasonic.IrLedOff();
                        }

                        // 送信
                        uint8_t send_byte_num = 0;
                        uint8_t send_byte[10];
                        send_byte[0] = 0xFF;

                        if (item == -2) {
                              send_byte_num = 1;
                              send_byte[0] = sensors.hold_front << 1 | sensors.hold_back;
                        } else if (item == 0) {
                              int16_t debug_val_1 = sensors.encoder_val[0];
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
                              } else {
                                    send_byte_num = 5;
                                    send_byte[1] = camera.y_goal_dir / 2 + 90;
                                    send_byte[2] = camera.y_goal_size;
                                    send_byte[3] = camera.b_goal_dir / 2 + 90;
                                    send_byte[4] = camera.b_goal_size;
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
                        if (sensors.hold_front == 1 && item == -2) kicker.Kick();
                        break;
                  case 1:
                        if (sensors.hold_front) {
                              dribblerFront.Hold(HOLD_MAX_POWER);
                        } else {
                              dribblerFront.Hold(HOLD_WAIT_POWER);
                        }
                        break;
                  case 2:
                        dribblerFront.Brake();
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