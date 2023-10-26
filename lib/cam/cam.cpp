#include "cam.h"

#include "mbed.h"

Cam::Cam(PinName tx_, PinName rx_, int16_t* own_dir_, uint8_t* mode_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Cam::Receive), Serial::RxIrq);

      this->own_dir = own_dir_;
      this->mode = mode_;
}

void Cam::Receive() {
      /*
      if (serial.getc() == 0xFF) {   // ヘッダーがあることを確認
            const uint8_t recv_byte_num = 10;
            uint8_t recv_byte[recv_byte_num];

            for (int i = 0; i < recv_byte_num; i++) {
                  //recv_byte[i] = serial.getc();   // 一旦すべてのデータを格納する
            }
            if (recv_byte[recv_byte_num - 1] == 0xAA) {   // フッターがあることを確認
                  uint8_t ball_dir_plus, ball_dir_minus;
                  uint8_t yellow_goal_dir_plus, yellow_goal_dir_minus;
                  uint8_t blue_goal_dir_plus, blue_goal_dir_minus;

                  ball_dir_plus = recv_byte[0];
                  ball_dir_minus = recv_byte[1];
                  ball_dis = recv_byte[2];
                  yellow_goal_dir_plus = recv_byte[3];
                  yellow_goal_dir_minus = recv_byte[4];
                  yellow_goal_size = recv_byte[5];
                  blue_goal_dir_plus = recv_byte[6];
                  blue_goal_dir_minus = recv_byte[7];
                  blue_goal_size = recv_byte[8];

                  ball_dir = SimplifyDeg(ball_dir_plus == 0 ? ball_dir_minus * -1 : ball_dir_plus);
                  yellow_goal_dir = SimplifyDeg(yellow_goal_dir_plus == 0 ? yellow_goal_dir_minus * -1 : yellow_goal_dir_plus);
                  blue_goal_dir = SimplifyDeg(blue_goal_dir_plus == 0 ? blue_goal_dir_minus * -1 : blue_goal_dir_plus);

                  // 自ゴールと敵ゴールがそれぞれ青か黄かの自動判定
                  if (abs(SimplifyDeg(yellow_goal_dir - *own_dir)) <= 90 && abs(SimplifyDeg(blue_goal_dir - *own_dir)) >= 90) {
                        front_goal_dir = yellow_goal_dir;
                        front_goal_size = yellow_goal_size;
                        back_goal_dir = blue_goal_dir;
                        back_goal_size = blue_goal_size;
                  } else if (abs(SimplifyDeg(yellow_goal_dir - *own_dir)) >= 90 && abs(SimplifyDeg(blue_goal_dir - *own_dir)) <= 90) {
                        front_goal_dir = blue_goal_dir;
                        front_goal_size = blue_goal_size;
                        back_goal_dir = yellow_goal_dir;
                        back_goal_size = yellow_goal_size;
                  } else {   // 黄と青ゴールが180度以内にあった時(エラー)
                        front_goal_dir = 0;
                        front_goal_size = 0;
                        back_goal_dir = 0;
                        back_goal_size = 0;
                  }
            } else {
                  return;
            }
      } else {
            return;
      }*/

      static uint8_t data_length;   // データの長さ
      static uint8_t ball_dir_plus, ball_dir_minus;
      static uint8_t yellow_goal_dir_plus, yellow_goal_dir_minus;
      static uint8_t blue_goal_dir_plus, blue_goal_dir_minus;

      if (data_length == 0) {   // ヘッダ（C）の受信
            uint8_t head = serial.getc();
            if (head == 0xFF) {
                  data_length += 1;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 1) {
            ball_dir_plus = serial.getc();
            data_length += 1;
      } else if (data_length == 2) {
            ball_dir_minus = serial.getc();
            data_length += 1;
      } else if (data_length == 3) {
            ball_dis = serial.getc();
            data_length += 1;
      } else if (data_length == 4) {
            yellow_goal_dir_plus = serial.getc();
            data_length += 1;
      } else if (data_length == 5) {
            yellow_goal_dir_minus = serial.getc();
            data_length += 1;
      } else if (data_length == 6) {
            yellow_goal_size = serial.getc();
            data_length += 1;
      } else if (data_length == 7) {
            blue_goal_dir_plus = serial.getc();
            data_length += 1;
      } else if (data_length == 8) {
            blue_goal_dir_minus = serial.getc();
            data_length += 1;
      } else if (data_length == 9) {
            blue_goal_size = serial.getc();

            ball_dir = SimplifyDeg(ball_dir_plus == 0 ? ball_dir_minus * -1 : ball_dir_plus);
            yellow_goal_dir = SimplifyDeg(yellow_goal_dir_plus == 0 ? yellow_goal_dir_minus * -1 : yellow_goal_dir_plus);
            blue_goal_dir = SimplifyDeg(blue_goal_dir_plus == 0 ? blue_goal_dir_minus * -1 : blue_goal_dir_plus);

            // 自ゴールと敵ゴールがそれぞれ青か黄かの自動判定
            if (abs(SimplifyDeg(yellow_goal_dir - *own_dir)) <= 90 && abs(SimplifyDeg(blue_goal_dir - *own_dir)) >= 90) {
                  front_goal_dir = yellow_goal_dir;
                  front_goal_size = yellow_goal_size;
                  back_goal_dir = blue_goal_dir;
                  back_goal_size = blue_goal_size;
            } else if (abs(SimplifyDeg(yellow_goal_dir - *own_dir)) >= 90 && abs(SimplifyDeg(blue_goal_dir - *own_dir)) <= 90) {
                  front_goal_dir = blue_goal_dir;
                  front_goal_size = blue_goal_size;
                  back_goal_dir = yellow_goal_dir;
                  back_goal_size = yellow_goal_size;
            } else {   // 黄と青ゴールが180度以内にあった時(エラー)
                  front_goal_dir = 0;
                  front_goal_size = 0;
                  back_goal_dir = 0;
                  back_goal_size = 0;
            }

            // 送信
            serial.putc(*mode);
            data_length = 0;
      }
}

int16_t Cam::GetBallDir() {
      return ball_dir;
}

uint8_t Cam::GetBallDis() {
      return ball_dis;
}

int16_t Cam::GetGoalDir(uint8_t which_) {
      if (which_ == 0) {
            return front_goal_dir;
      } else if (which_ == 1) {
            return back_goal_dir;
      } else if (which_ == 2) {
            return yellow_goal_dir;
      } else if (which_ == 3) {
            return blue_goal_dir;
      }
}

uint8_t Cam::GetGoalSize(uint8_t which_) {
      if (which_ == 0) {
            return front_goal_size;
      } else if (which_ == 1) {
            return back_goal_size;
      } else if (which_ == 2) {
            return yellow_goal_size;
      } else if (which_ == 3) {
            return blue_goal_size;
      }
}