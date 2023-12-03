#include "line.h"

#include "mbed.h"

Line::Line(PinName tx_, PinName rx_) : serial(tx_, rx_) {
      serial.baud(230400);
      serial.attach(callback(this, &Line::Receive), Serial::RxIrq);
}

void Line::Receive() {
      static uint8_t data_length;
      const uint8_t recv_data_num = 12;
      static uint8_t recv_data[recv_data_num];
      static uint8_t dir_plus, dir_minus;
      static uint8_t inside_dir_plus, inside_dir_minus;

      if (data_length == 0) {
            if (serial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (serial.getc() == 0xAA) {
                  encoder_val[0] = recv_data[0];
                  encoder_val[1] = recv_data[1];
                  encoder_val[2] = recv_data[2];
                  encoder_val[3] = recv_data[3];
                  interval = recv_data[4];
                  white_qty = recv_data[5];
                  is_left = recv_data[6];
                  is_right = recv_data[7];
                  dir_plus = recv_data[8];
                  dir_minus = recv_data[9];
                  inside_dir_plus = recv_data[10];
                  inside_dir_minus = recv_data[11];

                  dir = dir_plus == 0 ? dir_minus * -1 : dir_plus;
                  inside_dir = inside_dir_plus == 0 ? inside_dir_minus * -1 : inside_dir_plus;

                  // 送信
                  serial.putc(do_led_on);
            }
            data_length = 0;
      } else {
            recv_data[data_length - 1] = serial.getc();
            data_length++;
      }
}

void Line::LedOn() {
      do_led_on = 1;
}

void Line::LedOff() {
      do_led_on = 0;
}

uint8_t Line::GetDepth() {
      uint8_t depth = interval;
      if (dir == inside_dir) depth = 24 - depth;
      return depth;
}