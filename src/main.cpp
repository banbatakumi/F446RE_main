#include "mbed.h"
#include "setup.h"

void setup() {
      // UART初期設定
      uiSerial.baud(UI_UART_SPEED);
      // uiSerial.attach(&Ui, Serial::RxIrq);

      // モーター
      motor.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      motor.SetAttitudeControlPID(1.5, 0, 0.1);   // デフォルトゲイン：(1.5, 0, 0.075)
      motor.SetMovingAveLength(10);
      motor.SetPowerMaxLimit(95);
      motor.SetPowerMinLimit(5);

      // ドリブラー
      dribblerFront.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      dribblerBack.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz

      // 回り込みPID
      wrapDirPID.SetGain(1.5, 0, 25);
      wrapDirPID.SetSamplingPeriod(0.01);
      wrapDirPID.SetLimit(100);
      wrapDirPID.SelectType(PID_TYPE);

      wrapDisPID.SetGain(0.25, 0, 5);
      wrapDisPID.SetSamplingPeriod(0.01);
      wrapDisPID.SetLimit(100);
      wrapDisPID.SelectType(PID_TYPE);

      defencePID.SetGain(1.5, 0, 2);
      defencePID.SetSamplingPeriod(0.01);
      defencePID.SetLimit(100);
      defencePID.SelectType(PID_TYPE);

      // キッカー
      kicker.SetPower(100);   // 100まで

      holdFront.SetTh();
      holdBack.SetTh();

      voltage.SetLowVoltageTh(6.0);
}

int main() {
      setup();

      while (1) {
            voltage.Read();

            // バッテリー電圧降下による動作停止
            if (voltage.IsLowVoltage()) {
                  motor.Free();
            } else {
                  motor.encoder_val[0] = sensors.encoder_val[0];
                  motor.encoder_val[1] = sensors.encoder_val[1];
                  motor.encoder_val[2] = sensors.encoder_val[2];
                  motor.encoder_val[3] = sensors.encoder_val[3];

                  GetSensors();
                  // lidar.Receive();
                  if (uiSerial.readable()) Ui();

                  if (mode == 0) {
                        motor.Free();
                  } else if (mode == 1) {
                        static bool is_pre_line = 0;
                        static bool is_pre_left_line = 0;
                        static bool is_pre_right_line = 0;
                        if (sensors.line_white_qty != 0) {   // ラインセンサの処理
                              is_pre_line = 1;
                              float vector_x, vector_y, vector_mag;

                              vector_x = sensors.line_depth * MySin(sensors.line_inside_dir);
                              vector_y = sensors.line_depth * MyCos(sensors.line_inside_dir);
                              if (lineStopTimer.read_ms() > 2000 && abs(sensors.line_inside_dir) > 135) {
                                    vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir * 2)) * 1.5;
                                    vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir * 2)) * 1.5;
                              } else {
                                    vector_x += ((24 - sensors.line_depth) * MySin(camera.ball_dir)) / 3;
                                    vector_y += ((24 - sensors.line_depth) * MyCos(camera.ball_dir)) / 3;
                              }

                              vector_mag = abs(sqrt(pow(vector_x, 2) + pow(vector_y, 2)));

                              if (abs(camera.ball_dir) < 45 && camera.ball_dis > 150) {
                                    dribblerFront.Hold(100);
                              } else {
                                    dribblerFront.Hold(0);
                              }
                              if (abs(camera.inverse_ball_dir) < 45 && camera.ball_dis > 150) {
                                    dribblerBack.Hold(100);
                              } else {
                                    dribblerBack.Hold(0);
                              }

                              lineStopTimer.start();
                              if (lineStopTimer.read_ms() < 250 || lineStopTimer.read_ms() > 5000 || camera.ball_dis == 0) {
                                    motor.Run(sensors.line_inside_dir, line_moving_speed);
                              } else if (holdFront.IsHold()) {
                                    motor.Run(sensors.line_inside_dir, 50);
                              } else {
                                    uint16_t tmp_moving_speed = vector_mag * 5;
                                    if (tmp_moving_speed > line_moving_speed) tmp_moving_speed = line_moving_speed;
                                    if (abs(camera.ball_dir + own_dir) < 90) {
                                          motor.Run(atan2(vector_x, vector_y) * 180.00 / PI, tmp_moving_speed, camera.ball_dir + own_dir, FRONT, 10);
                                    } else {
                                          motor.Run(atan2(vector_x, vector_y) * 180.00 / PI, tmp_moving_speed, camera.inverse_ball_dir + own_dir, BACK, 10);
                                    }
                              }
                        } else if (sensors.is_line_left == 1) {
                              is_pre_left_line = 1;

                              if (abs(camera.ball_dir) < 45 && camera.ball_dis > 150) {
                                    dribblerFront.Hold(100);
                              } else {
                                    dribblerFront.Hold(0);
                              }
                              if (abs(camera.inverse_ball_dir) < 45 && camera.ball_dis > 150) {
                                    dribblerBack.Hold(100);
                              } else {
                                    dribblerBack.Hold(0);
                              }

                              lineStopTimer.start();
                              if (lineStopTimer.read_ms() < 250 || lineStopTimer.read_ms() > 5000 || camera.ball_dis == 0) {
                                    motor.Run(90, line_moving_speed);
                              } else if (holdFront.IsHold()) {
                                    motor.Run(90, 50);
                              } else {
                                    if (abs(camera.ball_dir + own_dir) < 90) {
                                          motor.Run(-90, 50, camera.ball_dir + own_dir, FRONT, 10);
                                    } else {
                                          motor.Run(-90, 50, camera.inverse_ball_dir + own_dir, BACK, 10);
                                    }
                              }
                        } else if (sensors.is_line_right == 1) {
                              is_pre_right_line = 1;

                              if (abs(camera.ball_dir) < 45 && camera.ball_dis > 150) {
                                    dribblerFront.Hold(100);
                              } else {
                                    dribblerFront.Hold(0);
                              }
                              if (abs(camera.inverse_ball_dir) < 45 && camera.ball_dis > 150) {
                                    dribblerBack.Hold(100);
                              } else {
                                    dribblerBack.Hold(0);
                              }

                              lineStopTimer.start();
                              if (lineStopTimer.read_ms() < 250 || lineStopTimer.read_ms() > 5000 || camera.ball_dis == 0) {
                                    motor.Run(-90, line_moving_speed);
                              } else if (holdFront.IsHold()) {
                                    motor.Run(-90, 50);
                              } else {
                                    if (abs(camera.ball_dir + own_dir) < 90) {
                                          motor.Run(90, 50, camera.ball_dir + own_dir, FRONT, 10);
                                    } else {
                                          motor.Run(90, 50, camera.inverse_ball_dir + own_dir, BACK, 10);
                                    }
                              }
                        } else if (camera.ball_dis == 0) {   // ボールがない時の処理
                              goToCenterTimer.start();
                              if (goToCenterTimer.read_ms() > 500) {
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
                                    is_pre_left_line = 0;
                                    is_pre_right_line = 0;
                              }
                              goToCenterTimer.reset();

                              is_pre_line = 0;
                              if (is_pre_left_line == 1 && lineStopTimer.read_ms() < 100) {
                                    motor.Run(90, line_moving_speed);
                              } else if (is_pre_right_line == 1 && lineStopTimer.read_ms() < 100) {
                                    motor.Run(-90, line_moving_speed);
                              } else {
                                    OffenceMove();
                                    is_pre_left_line = 0;
                                    is_pre_right_line = 0;
                              }
                        }
                  } else if (mode == 2) {
                        DefenceMove();
                  } else if (mode == 3) {
                        motor.Free();
                  } else if (mode == 4) {
                        motor.Run(0, 0);
                  }
            }
      }
}

void OffenceMove() {
      static bool first_hold = 0;

      if (curveShootTimer.read() > 0) {   // 後ろドリブラーのマカオシュート
            static int8_t shoot_dir;

            dribblerBack.Hold(100);
            if (curveShootTimer.read() < 0.2) {
                  motor.Brake();
                  shoot_dir = camera.front_goal_dir > 0 ? -1 : 1;
            } else if (curveShootTimer.read() < 1) {
                  motor.Run(own_dir * -1, curveShootTimer.read() * 50, 45 * shoot_dir, BACK, 5);
            } else if (curveShootTimer.read() < 1.25) {
                  motor.Run(own_dir * -1, 50, 135 * shoot_dir);
            } else {
                  curveShootTimer.stop();
                  curveShootTimer.reset();
            }
            if (curveShootTimer.read() < 0.2 && holdBack.IsHold() == 0) {
                  curveShootTimer.stop();
                  curveShootTimer.reset();
            }
      } else if (holdFront.IsHold()) {   // 前に捕捉している時
            static uint8_t kick_cnt = 0;

            if (camera.front_goal_size > 20 && camera.is_goal_front == 1) {   // キッカーを打つ条件
                  kick_cnt++;
                  if (kick_cnt > 50) {
                        dribblerFront.Brake();
                        kicker.Kick();
                        motor.Run(camera.front_goal_dir * 2, camera.front_goal_dir, 90);
                  } else {
                        dribblerFront.Hold(100);
                        motor.Run(camera.front_goal_dir * 2, camera.front_goal_dir, 90);
                  }
            } else {
                  kick_cnt = 0;
                  dribblerFront.Hold(100);

                  if (first_hold == 0) {
                        first_hold = 1;
                        goToGoalTimer.reset();
                        goToGoalTimer.stop();
                        motor.Brake(100);
                  }

                  int16_t tmp_move_dir;
                  tmp_move_dir = camera.front_goal_dir * 2.5;
                  if (abs(tmp_move_dir) > 180) tmp_move_dir = 180 * (abs(tmp_move_dir) / tmp_move_dir);

                  if (abs(tmp_move_dir) < 90) {
                        motor.Run(tmp_move_dir, moving_speed);
                  } else {
                        goToGoalTimer.start();
                        if (goToGoalTimer.read_ms() < 500) {
                              motor.Run(tmp_move_dir, goToGoalTimer.read_ms() / 20);
                        } else {
                              motor.Run(tmp_move_dir, 25);
                              goToGoalTimer.stop();
                        }
                  }
            }
      } else if (holdBack.IsHold()) {   // 後ろに捕捉している時
            static uint8_t kick_cnt = 0;
            static int8_t robot_dir;

            dribblerBack.Hold(100);
            // if (camera.front_goal_size > 20) {   // カーブシュート開始
            kick_cnt++;
            if (kick_cnt > 25) {
                  curveShootTimer.start();
            } else {
                  motor.Run(0, moving_speed);
            }
            /*
      } else {
            if (first_hold == 0) {
                  first_hold = 1;
                  motor.Brake(250);
            }
            kick_cnt = 0;

            motor.Run(0, 25, robot_dir, BACK, 15);
      }*/
      } else {
            first_hold = 0;

            if (abs(camera.ball_dir) < 45 && camera.ball_dis > 150) {
                  frontHoldTimer.start();
                  frontHoldTimer.reset();
                  dribblerFront.Hold(100);
            } else {
                  if (frontHoldTimer.read_ms() > 0 && frontHoldTimer.read_ms() < 500) {
                        dribblerFront.Hold(100);
                  } else {
                        frontHoldTimer.stop();
                        dribblerFront.Hold(0);
                  }
            }

            if (abs(camera.ball_dir) > 135 && camera.ball_dis > 150) {
                  backHoldTimer.start();
                  backHoldTimer.reset();
                  dribblerBack.Hold(100);
            } else {
                  if (backHoldTimer.read_ms() > 0 && backHoldTimer.read_ms() < 500) {
                        dribblerBack.Hold(100);
                  } else {
                        backHoldTimer.stop();
                        dribblerBack.Hold(0);
                  }
            }

            if (abs(camera.ball_dir) < 135) {   // 前の捕捉エリアに回り込む
                  int16_t tmp_move_speed, tmp_move_angle;

                  uint8_t wrap_speed_addend;
                  int16_t wrap_deg_addend;

                  // 角度
                  if (abs(camera.ball_dir) < 90) {
                        wrap_deg_addend = camera.ball_dir;
                  } else {
                        wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
                  }
                  tmp_move_angle = camera.ball_dir + (wrap_deg_addend * (camera.ball_dis / 160.000));

                  // 速度
                  wrapDirPID.Compute(camera.ball_dir, 0);

                  wrap_speed_addend = (20 - abs(camera.ball_dir));
                  if (abs(camera.ball_dir) > 20) wrap_speed_addend = 0;

                  if (camera.ball_dis > 100) {
                        tmp_move_speed = abs(wrapDirPID.Get()) + wrap_speed_addend;
                  } else {
                        tmp_move_speed = moving_speed;
                  }

                  if (tmp_move_speed > moving_speed) tmp_move_speed = moving_speed;

                  motor.Run(tmp_move_angle, tmp_move_speed);
            } else {   // 後ろの捕捉エリアに回り込む
                  int16_t tmp_move_speed, tmp_move_angle;

                  uint8_t wrap_speed_addend;
                  int16_t wrap_deg_addend;

                  // 角度
                  if (abs(camera.inverse_ball_dir) < 30) {
                        wrap_deg_addend = camera.inverse_ball_dir * (abs(camera.inverse_ball_dir) / 30);
                  } else {
                        wrap_deg_addend = 90 * (abs(camera.inverse_ball_dir) / camera.inverse_ball_dir);
                  }
                  tmp_move_angle = camera.inverse_ball_dir + (wrap_deg_addend * (camera.ball_dis / 160.000));

                  // 速度
                  wrapDirPID.Compute(camera.inverse_ball_dir, 0);

                  wrap_speed_addend = (20 - abs(camera.inverse_ball_dir));
                  if (abs(camera.inverse_ball_dir) > 20) wrap_speed_addend = 0;

                  if (camera.ball_dis > 125) {
                        tmp_move_speed = abs(wrapDirPID.Get()) + wrap_speed_addend;
                  } else {
                        tmp_move_speed = 50;
                  }

                  if (tmp_move_speed > 50) tmp_move_speed = 50;

                  tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);   // ０度の時に180度に動くように変換
                  motor.Run(tmp_move_angle, tmp_move_speed);
            }
      }
}

void DefenceMove() {
      if (abs(camera.ball_dir) < 30 && camera.ball_dis > 50) {
            defenceShooTimer.start();
      } else {
            defenceShooTimer.reset();
            defenceShooTimer.stop();
      }
      if (abs(camera.ball_dir) < 30 && camera.ball_dis > 150) {
            dribblerFront.Hold((camera.ball_dis - 150) * 5);
      } else {
            dribblerFront.Hold(0);
      }
      if (sensors.hold_front) kicker.Kick();

      if (defenceShooTimer.read() > 3) {
            if (defenceShooTimer.read() < 5 && abs(camera.ball_dir) < 45) {
                  motor.Run(camera.ball_dir * 2, moving_speed, camera.front_goal_dir, FRONT);
            } else {
                  defenceShooTimer.reset();
                  defenceShooTimer.stop();
            }
      } else {
            if (sensors.line_white_qty > 0) {
                  int16_t vector_x, vector_y;
                  int16_t ball_almost_angle;
                  if (camera.ball_dir > 0) {
                        ball_almost_angle = 80;
                  } else {
                        ball_almost_angle = -80;
                  }
                  vector_x = (12 - sensors.line_interval) * MyCos(sensors.line_dir) + sensors.line_interval / 3 * MyCos(ball_almost_angle);
                  vector_y = (12 - sensors.line_interval) * MySin(sensors.line_dir) + sensors.line_interval / 3 * MySin(ball_almost_angle);

                  if (abs(camera.ball_dir) < 90) {
                        defencePID.Compute(camera.ball_x, 0);

                        motor.Run(atan2(vector_y, vector_x) * 180.00 / PI, abs(defencePID.Get()));
                  } else {
                        motor.Run();
                  }
            } else {
                  if (camera.back_goal_size > 80) {
                        motor.Run(0, moving_speed);
                  } else if (camera.back_goal_size > 20) {
                        motor.Run(camera.back_goal_dir, 50);
                  } else {
                        motor.Run(camera.back_goal_dir, moving_speed);
                  }
            }
      }
}

void GoToCenter() {
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

      sensors.hold_front = holdFront.IsHold();
      sensors.hold_back = holdBack.IsHold();
      sensors.is_line_left = line.is_left;
      sensors.is_line_right = line.is_right;
      sensors.encoder_val[0] = line.encoder_val[0];
      sensors.encoder_val[1] = line.encoder_val[1];
      sensors.encoder_val[2] = line.encoder_val[2];
      sensors.encoder_val[3] = line.encoder_val[3];
      sensors.line_white_qty = line.white_qty;
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

      if (data_length == 0) {
            if (uiSerial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (uiSerial.getc() == 0xAA) {
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
                  printf("hello");

                  // 送信
                  if (voltage.IsLowVoltage() == 1) {
                        uiSerial.putc('E');
                        uiSerial.putc('R');
                  } else {
                        uint8_t send_byte_num;
                        uint8_t send_byte[25];
                        send_byte[0] = 0xFF;

                        if (item == 0) {
                              int16_t debug_val_1 = line.encoder_val[0];
                              int16_t debug_val_2 = line.encoder_val[1];
                              int16_t debug_val_3 = line.encoder_val[3];
                              int16_t debug_val_4 = line.encoder_val[2];

                              send_byte_num = 10;
                              send_byte[1] = uint8_t(voltage.Get() * 10);
                              send_byte[2] = debug_val_1 > 0 ? debug_val_1 : 0;
                              send_byte[3] = debug_val_1 < 0 ? debug_val_1 * -1 : 0;
                              send_byte[4] = debug_val_2 > 0 ? debug_val_2 : 0;
                              send_byte[5] = debug_val_2 < 0 ? debug_val_2 * -1 : 0;
                              send_byte[6] = debug_val_3 > 0 ? debug_val_3 : 0;
                              send_byte[7] = debug_val_3 < 0 ? debug_val_3 * -1 : 0;
                              send_byte[8] = debug_val_4 > 0 ? debug_val_4 : 0;
                              send_byte[9] = debug_val_4 < 0 ? debug_val_4 * -1 : 0;
                              for (uint8_t i = 0; i < send_byte_num; i++) {
                                    uiSerial.putc(send_byte[i]);
                              }
                        } else if (item == 1) {
                              send_byte_num = 3;
                              send_byte[1] = own_dir > 0 ? own_dir : 0;
                              send_byte[2] = own_dir < 0 ? own_dir * -1 : 0;
                              for (uint8_t i = 0; i < send_byte_num; i++) {
                                    uiSerial.putc(send_byte[i]);
                              }
                        } else if (item == 2) {
                              send_byte_num = 7;
                              send_byte[1] = sensors.line_dir > 0 ? sensors.line_dir : 0;
                              send_byte[2] = sensors.line_dir < 0 ? sensors.line_dir * -1 : 0;
                              send_byte[3] = sensors.line_inside_dir > 0 ? sensors.line_inside_dir : 0;
                              send_byte[4] = sensors.line_inside_dir < 0 ? sensors.line_inside_dir * -1 : 0;
                              send_byte[5] = sensors.line_depth;
                              send_byte[6] = sensors.line_white_qty;
                              for (uint8_t i = 0; i < send_byte_num; i++) {
                                    uiSerial.putc(send_byte[i]);
                              }
                        } else if (item == 3) {
                              send_byte_num = 6;
                              send_byte[1] = camera.ball_dir > 0 ? camera.ball_dir : 0;
                              send_byte[2] = camera.ball_dir < 0 ? camera.ball_dir * -1 : 0;
                              send_byte[3] = camera.ball_dis;
                              send_byte[4] = sensors.hold_front;
                              send_byte[5] = sensors.hold_back;
                              for (uint8_t i = 0; i < send_byte_num; i++) {
                                    uiSerial.putc(send_byte[i]);
                              }
                        } else if (item == 4) {
                              send_byte_num = 7;
                              send_byte[1] = camera.y_goal_dir > 0 ? camera.y_goal_dir : 0;
                              send_byte[2] = camera.y_goal_dir < 0 ? camera.y_goal_dir * -1 : 0;
                              send_byte[3] = camera.y_goal_size;
                              send_byte[4] = camera.b_goal_dir > 0 ? camera.b_goal_dir : 0;
                              send_byte[5] = camera.b_goal_dir < 0 ? camera.b_goal_dir * -1 : 0;
                              send_byte[6] = camera.b_goal_size;
                              for (uint8_t i = 0; i < send_byte_num; i++) {
                                    uiSerial.putc(send_byte[i]);
                              }
                        }
                  }
            }

            data_length = 0;
      } else {
            recv_data[data_length - 1] = uiSerial.getc();
            data_length++;
      }

      if (mode == 0) {
            switch (dribbler_sig) {
                  case 0:
                        dribblerFront.Hold(0);
                        dribblerBack.Hold(0);
                        break;
                  case 1:
                        dribblerFront.Hold(95);
                        break;
                  case 2:
                        dribblerFront.Hold(0);
                        kicker.Kick();
                        break;
                  case 3:
                        dribblerBack.Hold(95);
                        break;
                  case 4:
                        dribblerBack.Hold(0);
                        break;
            }
      }
}