#include "cam.h"

#include "mbed.h"

Cam::Cam(PinName tx_, PinName rx_, int16_t* own_dir_) : serial(tx_, rx_) {
      serial.baud(230400);
      serial.attach(callback(this, &Cam::Receive), Serial::RxIrq);

      this->own_dir = own_dir_;
}

void Cam::Receive() {
      static uint8_t data_length;   // データの長さ
      const uint8_t recv_data_num = 10;
      static uint8_t recv_data[recv_data_num];
      static uint8_t ball_dir_plus, ball_dir_minus;
      static uint8_t yellow_goal_dir_plus, yellow_goal_dir_minus;
      static uint8_t blue_goal_dir_plus, blue_goal_dir_minus;

      if (data_length == 0) {   // ヘッダ（C）の受信
            if (serial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (serial.getc() == 0xAA) {
                  ball_dir_plus = recv_data[0];
                  ball_dir_minus = recv_data[1];
                  ball_dis = recv_data[2];
                  yellow_goal_dir_plus = recv_data[3];
                  yellow_goal_dir_minus = recv_data[4];
                  yellow_goal_size = recv_data[5];
                  blue_goal_dir_plus = recv_data[6];
                  blue_goal_dir_minus = recv_data[7];
                  blue_goal_size = recv_data[8];
                  is_goal_front = recv_data[9];

                  // 受信データの合成
                  ball_dir = ball_dir_plus == 0 ? ball_dir_minus * -1 : ball_dir_plus;
                  yellow_goal_dir = yellow_goal_dir_plus == 0 ? yellow_goal_dir_minus * -1 : yellow_goal_dir_plus;
                  blue_goal_dir = blue_goal_dir_plus == 0 ? blue_goal_dir_minus * -1 : blue_goal_dir_plus;

                  // 自ゴールと敵ゴールがそれぞれ青か黄かの自動判定
                  if (abs(SimplifyDeg(yellow_goal_dir - *own_dir)) <= 90 && abs(SimplifyDeg(blue_goal_dir - *own_dir)) >= 90) {
                        is_front_goal_yellow = 1;
                  } else if (abs(SimplifyDeg(yellow_goal_dir - *own_dir)) >= 90 && abs(SimplifyDeg(blue_goal_dir - *own_dir)) <= 90) {
                        is_front_goal_yellow = 0;
                  }

                  if (is_front_goal_yellow == 1) {   // 自ゴール青
                        front_goal_dir = yellow_goal_dir;
                        front_goal_size = yellow_goal_size;
                        back_goal_dir = blue_goal_dir;
                        back_goal_size = blue_goal_size;
                  } else {   // 自ゴール黄
                        front_goal_dir = blue_goal_dir;
                        front_goal_size = blue_goal_size;
                        back_goal_dir = yellow_goal_dir;
                        back_goal_size = yellow_goal_size;
                  }
            }

            data_length = 0;
      } else {
            recv_data[data_length - 1] = serial.getc();
            data_length++;
      }
}

int16_t Cam::GetBallX() {
      int16_t ball_x = (200 - ball_dis) * MySin(ball_dir);
      return ball_x;
}

int16_t Cam::GetBallY() {
      int16_t ball_y = (200 - ball_dis) * MyCos(ball_dir);
      return ball_y;
}

int16_t Cam::GetOwnX() {
      int16_t own_x = (200 - front_goal_size) * MySin(front_goal_dir) + (200 - back_goal_size) * MySin(back_goal_dir);
      own_x /= -2;
      return own_x;
}

int16_t Cam::GetOwnY() {
      int16_t own_y = (200 - front_goal_size) * MyCos(front_goal_dir) + (200 - back_goal_size) * MyCos(back_goal_dir);
      own_y /= -2;
      return own_y;
}

int16_t Cam::GetCenterDir() {
      int16_t center_dir = SimplifyDeg(atan2(GetOwnX(), GetOwnY()) * 180.000 / PI - 180);
      return center_dir;
}

int16_t Cam::GetCenterDis() {
      int16_t center_dis = abs(sqrt(pow(GetOwnX(), 2) + pow(GetOwnY(), 2)));
      return center_dis;
}