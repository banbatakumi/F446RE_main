#include "mbed.h"
#include "setup.h"

void setup() {
      // UART初期設定
      uiSerial.baud(UI_UART_SPEED);
      // uiSerial.attach(&Ui, Serial::RxIrq);

      // モーター
      motor.SetPwmPeriod(20);                      // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      motor.SetAttitudeControlPID(1.5, 0.5, 0.1);  // デフォルトゲイン：(1, 0.5, 0.1)
      motor.SetMovingAveLength(50);
      motor.SetPowerMaxLimit(100);
      motor.SetPowerMinLimit(5);
      motor.SetEncoderGain(10);

      // ドリブラー
      dribblerFront.SetPwmPeriod(20);  // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      dribblerBack.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz

      // 回り込みPID
      wrapDirPID.SetGain(1.5, 0, 2.5);
      wrapDirPID.SetSamplingPeriod(0.01);
      wrapDirPID.SetLimit(100);
      wrapDirPID.SelectType(PID_TYPE);

      defensePID.SetGain(5, 1, 2.5);
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
            // lidar.Receive();
            if (uiSerial.readable()) Ui();

            if (mode == 0) {
                  motor.Free();
            } else if (mode == 1) {
                  static bool is_pre_line_left = 0;
                  static bool is_pre_line_right = 0;
                  static bool is_pre_line = 0;
                  if (sensors.is_on_line == 1 || sensors.is_line_left == 1 || sensors.is_line_right == 1) {  // ラインセンサの処理
                        if (abs(camera.ball_dir) < 45 && camera.ball_dis > 75) {
                              dribblerFront.Hold(100);
                        } else {
                              dribblerFront.Hold(0);
                        }
                        if (abs(camera.inverse_ball_dir) < 45 && camera.ball_dis > 75) {
                              dribblerBack.Hold(100);
                        } else {
                              dribblerBack.Hold(0);
                        }

                        if (sensors.is_on_line == 1) is_pre_line = 1;
                        if (sensors.is_line_left == 1) is_pre_line_left = 1;
                        if (sensors.is_line_right == 1) is_pre_line_right = 1;

                        float vector_x, vector_y, vector_mag, vector_dir;

                        if (abs(camera.ball_dir) + own_dir < 120) {
                              vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * 0.7;
                              vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * 0.7;
                              vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * 0.3;
                              vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * 0.3;
                        } else {
                              vector_x = sensors.line_depth * MySin(sensors.line_inside_dir) * 0.7;
                              vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir) * 0.7;
                              vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) * 0.3;
                              vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) * 0.3;
                        }
                        vector_dir = atan2(vector_x, vector_y) * 180.00 / PI;
                        vector_mag = abs(sqrt(pow(vector_x, 2) + pow(vector_y, 2)));

                        lineStopTimer.start();
                        if (lineStopTimer.read_ms() < 200 || lineStopTimer.read_ms() > 5000 || camera.ball_dis == 0) {
                              if (sensors.is_on_line == 1) {
                                    motor.Run(sensors.line_inside_dir, line_moving_speed);
                              } else if (sensors.is_line_left == 1) {
                                    motor.Run(90, line_moving_speed);
                              } else if (sensors.is_line_right == 1) {
                                    motor.Run(-90, line_moving_speed);
                              }
                        } else if (holdFront.IsHold()) {
                              if (sensors.is_on_line == 1) {
                                    motor.Run(sensors.line_inside_dir, 25, 0, FRONT, 15);
                              } else if (sensors.is_line_left == 1) {
                                    motor.Run(90, 25, 0, FRONT, 15);
                              } else if (sensors.is_line_right == 1) {
                                    motor.Run(-90, 25, 0, FRONT, 15);
                              }
                        } else if (holdBack.IsHold()) {
                              if (sensors.is_on_line == 1) {
                                    motor.Run(sensors.line_inside_dir, 15);
                              } else if (sensors.is_line_left == 1) {
                                    motor.Run(90, 25, 0, BACK, 15);
                              } else if (sensors.is_line_right == 1) {
                                    motor.Run(-90, 25, 0, BACK, 15);
                              }
                        } else {
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
                                    if (lineStopTimer.read_ms() > 1000) {
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
                              OffenceMove();
                              is_pre_line_left = 0;
                              is_pre_line_right = 0;
                              is_pre_line = 0;
                        }
                  }
            } else if (mode == 2) {
                  DefenceMove();
            } else if (mode == 3) {
                  motor.Run();
            }
      }
}

void OffenceMove() {
      if (curveShootTimer.read() > 0) {  // 後ろドリブラーのマカオシュート
            static int8_t shoot_dir;

            dribblerBack.Hold(100);
            if (curveShootTimer.read() < 0.25) {
                  motor.Brake();
                  shoot_dir = camera.front_goal_dir > 0 ? -1 : 1;
            } else if (curveShootTimer.read() < 1) {
                  motor.Run(0, 0, 60 * shoot_dir, BACK, 25);
            } else if (curveShootTimer.read() < 1.25) {
                  motor.Run(0, 0, 180 * shoot_dir);
            } else {
                  curveShootTimer.stop();
                  curveShootTimer.reset();
            }
            if (curveShootTimer.read() < 0.25 && holdBack.IsHold() == 0) {
                  curveShootTimer.stop();
                  curveShootTimer.reset();
            }
      } else if (holdFront.IsHold()) {                                       // 前に捕捉している時
            if (camera.front_goal_size > 20 && camera.is_goal_front == 1) {  // キッカーを打つ条件
                  motor.Run(0, moving_speed);
                  dribblerFront.Brake();
                  kicker.Kick();
            } else {
                  dribblerFront.Hold(100);

                  int16_t tmp_move_dir = (camera.front_goal_dir + own_dir) * 2.5;
                  if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
                  int16_t robot_dir = camera.front_goal_dir + own_dir;
                  if (abs(robot_dir) > 45) robot_dir = 45 * (abs(robot_dir) / robot_dir);

                  if (abs(tmp_move_dir) < 90) {
                        if (sensors.dis[0] <= 30) {
                              tmp_move_dir = (40 - sensors.dis[0]) * 3;
                              if (camera.front_goal_size == 0) {
                                    if (sensors.dis[1] < sensors.dis[3]) tmp_move_dir *= -1;
                              } else {
                                    if (camera.front_goal_dir < 0) tmp_move_dir *= -1;
                              }
                        }
                        if (abs(robot_dir - own_dir) > 15) {
                              motor.Run(0, 0, robot_dir, FRONT);
                        } else {
                              motor.Run(tmp_move_dir - own_dir, moving_speed, robot_dir, FRONT, 20);
                        }
                  } else {
                        motor.Run(tmp_move_dir - own_dir, 20, robot_dir, FRONT, 20);
                  }
            }
      } else if (holdBack.IsHold()) {           // 後ろに捕捉している時
            if (camera.front_goal_size > 25) {  // キッカーを打つ条件
                  curveShootTimer.start();
            } else {
                  dribblerBack.Hold(100);
                  int16_t tmp_move_dir = camera.front_goal_dir * 1.5;
                  if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);
                  if (sensors.dis[0] <= 30) {
                        tmp_move_dir = (40 - sensors.dis[0]) * 3;
                        if (camera.front_goal_size == 0) {
                              if (sensors.dis[1] < sensors.dis[3]) tmp_move_dir *= -1;
                        } else {
                              if (camera.front_goal_dir < 0) tmp_move_dir *= -1;
                        }
                  }
                  motor.Run(tmp_move_dir, 30);
            }
      } else {
            if (abs(camera.ball_dir) < 45 && camera.ball_dis > 75) {
                  dribblerFront.Hold(50);
            } else {
                  dribblerFront.Hold(0);
            }
            if (abs(camera.inverse_ball_dir) < 45 && camera.ball_dis > 75) {
                  dribblerBack.Hold(50);
            } else {
                  dribblerBack.Hold(0);
            }

            if (abs(camera.ball_dir) < 120) {  // 前の捕捉エリアに回り込む
                  int16_t tmp_move_speed, tmp_move_angle;

                  int16_t wrap_deg_addend;

                  // 角度
                  if (abs(camera.ball_dir) < 30) {
                        wrap_deg_addend = camera.ball_dir * (abs(camera.ball_dir) / 10.000);
                  } else {
                        wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
                  }
                  tmp_move_angle = camera.ball_dir + (wrap_deg_addend * pow((camera.ball_dis / 100.000), 2.5));

                  if (abs(camera.ball_dir) < 10) wrapTimer.start();
                  if (abs(camera.ball_dir) > 45) {
                        wrapTimer.reset();
                        wrapTimer.stop();
                  }

                  // 速度
                  wrapDirPID.Compute(camera.ball_dir, 0);

                  if (camera.ball_dis < 80) {
                        tmp_move_speed = moving_speed;
                  } else if (wrapTimer.read() > 0.5) {
                        tmp_move_speed = 25;
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
                  tmp_move_angle = camera.inverse_ball_dir + (wrap_deg_addend * pow((camera.ball_dis / 100.000), 2.5));

                  if (abs(camera.inverse_ball_dir) < 10) wrapTimer.start();
                  if (abs(camera.inverse_ball_dir) > 45) {
                        wrapTimer.reset();
                        wrapTimer.stop();
                  }

                  // 速度
                  wrapDirPID.Compute(camera.inverse_ball_dir, 0);

                  if (camera.ball_dis < 80) {
                        tmp_move_speed = moving_speed;
                  } else if (wrapTimer.read() > 1) {
                        tmp_move_speed = 25;
                  } else {
                        tmp_move_speed = abs(wrapDirPID.Get());
                  }

                  if (tmp_move_speed > 50) tmp_move_speed = 50;

                  tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);  // ０度の時に180度に動くように変換
                  motor.Run(tmp_move_angle, tmp_move_speed);
            }
      }
}

void DefenceMove() {
      if ((abs(camera.ball_dir) < 30 && camera.ball_dis > 75) || sensors.hold_front == 1) {
            dribblerFront.Hold(100);
      } else {
            dribblerFront.Hold(0);
      }

      if (defenseShootTimer.read() > 2) {
            if (defenseShootTimer.read() < 3 && (abs(camera.ball_dir) < 45 || sensors.hold_front == 1)) {
                  if (defenseShootTimer.read() > 2.1) {
                        if (sensors.is_line_left == 1) {
                              motor.Run(-90, line_moving_speed);
                        } else if (sensors.is_line_right == 1) {
                              motor.Run(90, line_moving_speed);
                        } else {
                              motor.Run(camera.ball_dir, moving_speed, camera.front_goal_dir + own_dir, FRONT);
                        }
                  } else {
                        motor.Run(camera.ball_dir, moving_speed, camera.front_goal_dir + own_dir, FRONT);
                  }
                  if (sensors.hold_front == 1 && defenseShootTimer.read() > 2.5) kicker.Kick();
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();
            }
      } else {
            if (sensors.is_on_line == 1) {
                  if ((abs(camera.ball_dir) < 30 && camera.ball_dis > 25) || sensors.hold_front == 1) {
                        defenseShootTimer.start();
                  } else {
                        defenseShootTimer.reset();
                        defenseShootTimer.stop();
                  }

                  int16_t vector_x, vector_y;
                  int16_t ball_almost_angle;
                  if ((sensors.line_dir > 45 && sensors.line_dir < 135) || (sensors.line_dir < -45 && sensors.line_dir > -135)) {
                        if (abs(camera.ball_dir) < 90) {
                              ball_almost_angle = 60;
                        } else {
                              ball_almost_angle = 120;
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
                        motor.Run();
                  } else if (sensors.line_dir < -90 && sensors.line_dir > -135 && camera.ball_dir > 0) {
                        motor.Run();
                  } else {
                        defensePID.Compute(camera.ball_x, 0);

                        motor.Run(atan2(vector_x, vector_y) * 180.00 / PI, abs(defensePID.Get()));
                  }
            } else {
                  defenseShootTimer.reset();
                  defenseShootTimer.stop();

                  if (camera.back_goal_size > 70) {
                        motor.Run(0, moving_speed);
                  } else if (camera.back_goal_size > 20) {
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

      static uint8_t dribbler_sig = 0;
      static bool is_own_dir_correction = 0;

      static uint8_t data_length;
      const uint8_t recv_data_num = 6;
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
                  mode = recv_data[1];
                  is_own_dir_correction = recv_data[2];
                  moving_speed = recv_data[3];
                  line_moving_speed = recv_data[4];
                  dribbler_sig = recv_data[5];

                  if (is_own_dir_correction == 1 && mode == 0) imu.SetZero();

                  if (mode != 0 || item == 2) {
                        line.LedOn();
                  } else {
                        line.LedOff();
                  }

                  // 送信
                  uint8_t send_byte_num = 0;
                  uint8_t send_byte[25];
                  send_byte[0] = 0xFF;

                  if (item == 0) {
                        int16_t debug_val_1 = sensors.dis[0];
                        int16_t debug_val_2 = sensors.dis[1];
                        uint8_t debug_val_3 = sensors.dis[2];
                        uint8_t debug_val_4 = sensors.dis[3];

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
                        send_byte_num = 4;
                        send_byte[1] = camera.ball_dir / 2 + 90;
                        send_byte[2] = camera.ball_dis;
                        send_byte[3] = sensors.hold_front << 1 | sensors.hold_back;
                  } else if (item == 4) {
                        send_byte_num = 5;
                        send_byte[1] = camera.y_goal_dir / 2 + 90;
                        send_byte[2] = camera.y_goal_size;
                        send_byte[3] = camera.b_goal_dir / 2 + 90;
                        send_byte[4] = camera.b_goal_size;
                  }
                  uiSerial.write(&send_byte, send_byte_num);
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
                        break;
                  case 1:
                        if (sensors.hold_front) {
                              dribblerFront.Hold(100);
                        } else {
                              dribblerFront.Hold(50);
                        }
                        break;
                  case 2:
                        dribblerFront.Brake();
                        kicker.Kick();
                        break;
                  case 3:
                        if (sensors.hold_back) {
                              dribblerBack.Hold(100);
                        } else {
                              dribblerBack.Hold(50);
                        }
                        break;
                  case 4:
                        dribblerBack.Hold(0);
                        break;
            }
      }
}