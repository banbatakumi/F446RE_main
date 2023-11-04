#include "mbed.h"
#include "setup.h"

void setup() {
      // UART初期設定
      uiSerial.baud(UI_UART_SPEED);
      // uiSerial.attach(&Ui, Serial::RxIrq);

      // モーター
      motor.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      motor.SetAttitudeControlPID(1.5, 0, 0.075);   // デフォルトゲイン：(1.5, 0, 0.075)
      motor.SetMovingAveLength(20);
      motor.SetPowerMaxLimit(80);
      motor.SetPowerMinLimit(5);

      // ドリブラー
      dribblerFront.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      dribblerBack.SetPwmPeriod(20);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz

      // 回り込みPID
      wrapDirPID.SetGain(0.5, 0, 5);
      wrapDirPID.SetSamplingPeriod(0.01);
      wrapDirPID.SetLimit(100);
      wrapDirPID.SelectType(PI_D_TYPE);

      wrapDisPID.SetGain(0.5, 0, 1);
      wrapDisPID.SetSamplingPeriod(0.01);
      wrapDisPID.SetLimit(100);
      wrapDisPID.SelectType(PI_D_TYPE);

      defencePID.SetGain(1.5, 0, 1);
      defencePID.SetSamplingPeriod(0.01);
      defencePID.SetLimit(100);
      defencePID.SelectType(PI_D_TYPE);

      // キッカー
      kicker.SetPower(75);   // 100まで

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
                  lidar.Receive();
                  if (uiSerial.readable()) Ui();

                  if (mode == 0) {
                        motor.Free();
                  } else if (mode == 1) {
                        if (sensors.line_white_num != 0) {
                              motor.Run(SimplifyDeg(sensors.line_vector - 180), line_moving_speed);
                        } else {
                              OffenceMove();
                        }
                  } else if (mode == 2) {
                        if (sensors.line_white_num != 0) {
                              motor.Run(SimplifyDeg(sensors.line_vector - 180), line_moving_speed);
                        } else {
                              DefenceMove();
                        }
                  } else if (mode == 3) {
                        motor.Free();
                  } else if (mode == 4) {
                        motor.Run();
                  }
            }
      }
}

void OffenceMove() {
      if (camera.ball_dis == 0) {   // ボールがない時の処理
            if (lidar.val[lidar.MinSensor()] < 150) {
                  motor.Run(lidar.SafeDir(), 150 - lidar.val[lidar.MinSensor()]);   // 中央に戻る
            } else {
                  motor.Free();
            }
      } else {
            if (curveShootTimer.read() > 0) {   // 後ろドリブラーのマカオシュート
                  static int8_t shoot_dir;

                  dribblerBack.Hold(100);
                  if (curveShootTimer.read() < 0.5) {
                        motor.Run(0, 0);
                        shoot_dir = camera.front_goal_dir > 0 ? -1 : 1;
                  } else if (curveShootTimer.read() < 1) {
                        motor.Run(0, 0, 45 * shoot_dir, BACK, 25);
                  } else if (curveShootTimer.read() < 1.5) {
                        motor.Run(0, 0, 135 * shoot_dir);
                  } else {
                        curveShootTimer.stop();
                        curveShootTimer.reset();
                  }
                  if (curveShootTimer.read() < 1 && holdBack.IsHold() == 0) {
                        curveShootTimer.stop();
                        curveShootTimer.reset();
                  }
            } else if (holdFront.IsHold()) {   // 前に捕捉している時
                  static uint8_t kick_cnt = 0;
                  if (camera.front_goal_size > 25 && abs(camera.front_goal_dir) < 30) {   // キッカーを打つ条件
                        kick_cnt++;
                        dribblerFront.Brake();
                        if (kick_cnt > 20) {
                              kicker.Kick();
                        } else {
                              motor.Run(0, moving_speed);
                        }
                  } else {
                        kick_cnt = 0;
                        dribblerFront.Hold(100);
                        if (lidar.val[lidar.FrontMinSensor()] < 50 && abs(camera.front_goal_dir) < 30) {
                              motor.Run(SimplifyDeg(lidar.FrontMinSensor() * 22.5 + 90 * (lidar.FrontSafeDir(50) > 0 ? 1 : -1)), moving_speed);
                        } else {
                              if (abs(camera.front_goal_dir) < 30) {
                                    motor.Run(camera.front_goal_dir * 2, moving_speed);
                              } else {
                                    motor.Run(camera.front_goal_dir * 2, 40);
                              }
                        }
                  }
            } else if (holdBack.IsHold()) {   // 後ろに捕捉している時
                  static uint8_t kick_cnt = 0;
                  if (camera.front_goal_size > 25 && abs(camera.front_goal_dir) < 30) {   // カーブシュート開始
                        kick_cnt++;
                        if (kick_cnt > 20) {
                              curveShootTimer.start();
                        } else {
                              motor.Run(0, moving_speed);
                        }
                  } else {
                        kick_cnt = 0;
                        dribblerBack.Hold(100);

                        if (lidar.val[lidar.FrontMinSensor()] < 100 && abs(camera.front_goal_dir) < 30) {
                              motor.Run(SimplifyDeg(lidar.FrontMinSensor() * 22.5 + 90 * (lidar.FrontSafeDir() > 0 ? 1 : -1)), 50);
                        } else {
                              motor.Run(camera.front_goal_dir * 2, 40);
                        }
                  }
            } else {
                  if (abs(camera.ball_dir) < 30 && camera.ball_dis > 150) {
                        dribblerFront.Hold((camera.ball_dis - 150) * 2);
                  } else {
                        dribblerFront.Stop();
                  }
                  if (abs(camera.ball_dir) > 150 && camera.ball_dis > 150) {
                        dribblerBack.Hold((camera.ball_dis - 150) * 2);
                  } else {
                        dribblerBack.Stop();
                  }
                  if (abs(camera.ball_dir) < 150) {   // 前の捕捉エリアに回り込む
                        int16_t tmp_move_speed, tmp_move_angle;

                        uint8_t wrap_speed_addend;
                        int16_t wrap_deg_addend;

                        // 角度
                        if (abs(camera.ball_dir) < 30) {
                              wrap_deg_addend = camera.ball_dir * (abs(camera.ball_dir) / 10);
                        } else {
                              wrap_deg_addend = 90 * (abs(camera.ball_dir) / camera.ball_dir);
                        }
                        tmp_move_angle = camera.ball_dir + (wrap_deg_addend * (camera.ball_dis / 175.000));

                        // 速度
                        wrapDirPID.Compute(camera.ball_dir, 0);
                        wrapDisPID.Compute(camera.ball_dis, 175);

                        wrap_speed_addend = (15 - abs(camera.ball_dir));
                        if (abs(camera.ball_dir) > 15) wrap_speed_addend = 0;
                        tmp_move_speed = abs(wrapDirPID.Get()) + abs(wrapDisPID.Get()) + wrap_speed_addend;

                        if (tmp_move_speed > moving_speed) tmp_move_speed = moving_speed;

                        motor.Run(tmp_move_angle, tmp_move_speed);
                  } else {   // 後ろの捕捉エリアに回り込む
                        int16_t inverse_ball_dir = SimplifyDeg(camera.ball_dir + 180);   // ボールの角度を後ろを0度に変換
                        int16_t tmp_move_speed, tmp_move_angle;

                        uint8_t wrap_speed_addend;
                        int16_t wrap_deg_addend;

                        // 角度
                        if (abs(inverse_ball_dir) < 45) {
                              wrap_deg_addend = inverse_ball_dir * 2;
                        } else {
                              wrap_deg_addend = 90 * (abs(inverse_ball_dir) / inverse_ball_dir);
                        }
                        tmp_move_angle = inverse_ball_dir + (wrap_deg_addend * (camera.ball_dis / 175.000));

                        // 速度
                        wrapDirPID.Compute(inverse_ball_dir, 0);
                        wrapDisPID.Compute(camera.ball_dis, 175);

                        wrap_speed_addend = (10 - abs(inverse_ball_dir));
                        if (abs(inverse_ball_dir) > 10) wrap_speed_addend = 0;
                        tmp_move_speed = abs(wrapDirPID.Get()) + abs(wrapDisPID.Get()) + wrap_speed_addend;

                        if (tmp_move_speed > 50) tmp_move_speed = 50;

                        tmp_move_angle = SimplifyDeg(tmp_move_angle - 180);   // ０度の時に180度に動くように変換
                        motor.Run(tmp_move_angle, tmp_move_speed);
                  }
            }
      }
}

void DefenceMove() {
      if (curveShootTimer.read() > 0) {   // 後ろドリブラーのマカオシュート
            dribblerFront.Hold(100);
            if (curveShootTimer.read() < 0.5) {
                  motor.Run(0, 0);
            } else if (curveShootTimer.read() < 0.75) {
                  motor.Run(0, 75);
                  dribblerFront.Stop();
            } else {
                  curveShootTimer.stop();
                  curveShootTimer.reset();
                  kicker.Kick();
            }
      } else {
            dribblerFront.Stop();
            motor.Run(0, 0, 0);
      }
      if (holdFront.IsHold()) {
            curveShootTimer.start();
      }
}

void GetSensors() {
      holdFront.Read();
      holdBack.Read();

      camera.ball_dir = cam.ball_dir;
      camera.ball_dis = cam.ball_dis;
      camera.y_goal_dir = cam.yellow_goal_dir;
      camera.y_goal_size = cam.yellow_goal_size;
      camera.b_goal_dir = cam.blue_goal_dir;
      camera.b_goal_size = cam.blue_goal_size;
      camera.front_goal_dir = cam.front_goal_dir;
      camera.front_goal_size = cam.front_goal_size;
      camera.back_goal_dir = cam.back_goal_dir;
      camera.back_goal_size = cam.back_goal_size;
      camera.ball_x = cam.ball_x;
      camera.ball_y = cam.ball_y;

      sensors.hold_front = holdFront.IsHold();
      sensors.hold_back = holdBack.IsHold();
      sensors.is_line_left = line.IsLeft();
      sensors.is_line_right = line.IsRight();
      sensors.encoder_val[0] = line.EncoderVal(0);
      sensors.encoder_val[1] = line.EncoderVal(1);
      sensors.encoder_val[2] = line.EncoderVal(2);
      sensors.encoder_val[3] = line.EncoderVal(3);
      sensors.line_white_num = line.WhiteNum();
      sensors.line_vector = line.Vector();

      own_dir = imu.GetDir();
}

void Ui() {
      static int8_t item = 0;

      static uint8_t dribbler_sig = 0;
      static bool is_own_dir_correction = 0;

      static uint8_t data_length;
      static uint8_t recv_data[6];

      if (data_length == 0) {
            if (uiSerial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 7) {
            if (uiSerial.getc() == 0xAA) {
                  item = recv_data[0] - 100;
                  mode = recv_data[1];
                  is_own_dir_correction = recv_data[2];
                  moving_speed = recv_data[3];
                  line_moving_speed = recv_data[4];
                  dribbler_sig = recv_data[5];

                  if (is_own_dir_correction == 1 && mode == 0) imu.SetZero();

                  // 送信
                  if (voltage.IsLowVoltage() == 1) {
                        uiSerial.putc('E');
                        uiSerial.putc('R');
                  } else {
                        uint8_t send_byte_num;
                        uint8_t send_byte[25];
                        send_byte[0] = 0xFF;

                        if (item == 0) {
                              int16_t debug_val_1 = camera.ball_x;
                              int16_t debug_val_2 = camera.ball_y;
                              int16_t debug_val_3 = camera.back_goal_dir;
                              int16_t debug_val_4 = camera.back_goal_size;

                              send_byte_num = 9;
                              send_byte[0] = uint8_t(voltage.Get() * 10);
                              send_byte[1] = debug_val_1 > 0 ? debug_val_1 : 0;
                              send_byte[2] = debug_val_1 < 0 ? debug_val_1 * -1 : 0;
                              send_byte[3] = debug_val_2 > 0 ? debug_val_2 : 0;
                              send_byte[4] = debug_val_2 < 0 ? debug_val_2 * -1 : 0;
                              send_byte[5] = debug_val_3 > 0 ? debug_val_3 : 0;
                              send_byte[6] = debug_val_3 < 0 ? debug_val_3 * -1 : 0;
                              send_byte[7] = debug_val_4 > 0 ? debug_val_4 : 0;
                              send_byte[8] = debug_val_4 < 0 ? debug_val_4 * -1 : 0;
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
                              static uint8_t which_data_send = 0;
                              send_byte_num = 12;
                              send_byte[1] = lidar.SafeDir() > 0 ? lidar.SafeDir() : 0;
                              send_byte[2] = lidar.SafeDir() < 0 ? lidar.SafeDir() * -1 : 0;
                              send_byte[3] = lidar.MinSensor();
                              which_data_send++;
                              if (which_data_send >= 4) which_data_send = 0;
                              if (which_data_send == 0) {
                                    send_byte[0] = 0xEE;
                                    for (uint8_t i = 0; i < 4; i++) {
                                          send_byte[i + 4] = lidar.val[i * 4];
                                    }
                              } else if (which_data_send == 1) {
                                    send_byte[0] = 0xEF;
                                    for (uint8_t i = 0; i < 4; i++) {
                                          send_byte[i + 4] = lidar.val[i * 4 + 1];
                                    }
                              } else if (which_data_send == 2) {
                                    send_byte[0] = 0xFE;
                                    for (uint8_t i = 0; i < 4; i++) {
                                          send_byte[i + 4] = lidar.val[i * 4 + 2];
                                    }
                              } else if (which_data_send == 3) {
                                    send_byte[0] = 0xFF;
                                    for (uint8_t i = 0; i < 4; i++) {
                                          send_byte[i + 4] = lidar.val[i * 4 + 3];
                                    }
                              }
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
                        dribblerFront.Stop();
                        dribblerBack.Stop();
                        break;
                  case 1:
                        dribblerFront.Hold(95);
                        break;
                  case 2:
                        dribblerFront.Stop();
                        kicker.Kick();
                        break;
                  case 3:
                        dribblerBack.Hold(95);
                        break;
                  case 4:
                        dribblerBack.Stop();
                        break;
            }
      }
}