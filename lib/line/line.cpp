#include "line.h"

#include "mbed.h"

Line::Line(PinName tx_, PinName rx_) : serial(tx_, rx_) {
      serial.baud(230400);
      serial.attach(callback(this, &Line::Receive), Serial::RxIrq);
}

void Line::Receive() {
      static uint8_t data_length;
      const uint8_t recv_data_num = 6;
      static uint8_t recv_data[recv_data_num];

      if (data_length == 0) {
            if (serial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (serial.getc() == 0xAA) {
                  encoder_val[0] = recv_data[0] >> 4;
                  encoder_val[1] = recv_data[0] & 0b00001111;
                  encoder_val[2] = recv_data[1] >> 4;
                  encoder_val[3] = recv_data[1] & 0b00001111;
                  interval = recv_data[2];
                  is_on_line = (recv_data[3] >> 2) & 1;
                  is_left = (recv_data[3] >> 1) & 1;
                  is_right = recv_data[3] & 1;
                  dir = recv_data[4] * 2 - 180;
                  inside_dir = recv_data[5] * 2 - 180;

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