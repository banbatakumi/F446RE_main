#include "cam.h"

#include "mbed.h"

Cam::Cam(PinName tx_, PinName rx_, int16_t* own_dir_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Cam::Receive), Serial::RxIrq);

      this->own_dir = own_dir_;
}

void Cam::Receive() {
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