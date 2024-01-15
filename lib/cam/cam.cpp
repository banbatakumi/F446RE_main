#include "cam.h"

#include "mbed.h"

Cam::Cam(PinName tx_, PinName rx_, int16_t* own_dir_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Cam::Receive), SerialBase::RxIrq);

      this->own_dir = own_dir_;
}

void Cam::Receive() {
      static uint8_t data_length;   // データの長さ
      const uint8_t recv_data_num = 7;
      static uint8_t recv_data[recv_data_num];
      uint8_t read_byte;
      serial.read(&read_byte, 1);

      if (data_length == 0) {   // ヘッダ（C）の受信
            if (read_byte == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (read_byte == 0xAA) {
                  ball_dir = recv_data[0] * 2 - 180;
                  ball_dis = recv_data[1];
                  yellow_goal_dir = recv_data[2] * 2 - 180;
                  yellow_goal_size = recv_data[3];
                  blue_goal_dir = recv_data[4] * 2 - 180;
                  blue_goal_size = recv_data[5];
                  is_goal_front = recv_data[6];

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
            recv_data[data_length - 1] = read_byte;
            data_length++;
      }
}

int16_t Cam::GetBallX() {
      int16_t ball_x = (100 - ball_dis) * MySin(ball_dir);
      return ball_x;
}

int16_t Cam::GetBallY() {
      int16_t ball_y = (100 - ball_dis) * MyCos(ball_dir);
      return ball_y;
}

int16_t Cam::GetOwnX() {
      int16_t own_x = (200 - front_goal_size) * MySin(front_goal_dir + *own_dir) + (200 - back_goal_size) * MySin(back_goal_dir + *own_dir);
      own_x /= -2;
      return own_x;
}

int16_t Cam::GetOwnY() {
      int16_t own_y = (200 - front_goal_size) * MyCos(front_goal_dir + *own_dir) + (200 - back_goal_size) * MyCos(back_goal_dir + *own_dir);
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