#include "line.h"

#include "mbed.h"

Line::Line(PinName tx_, PinName rx_, uint8_t* mode_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Line::Receive), Serial::RxIrq);

      this->mode = mode_;
}

void Line::Receive() {
      static uint8_t data_length;
      static uint8_t recv_data[12];
      static uint8_t vector_plus, vector_minus;
      static uint8_t inside_dir_plus, inside_dir_minus;

      if (data_length == 0) {
            if (serial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 13) {
            if (serial.getc() == 0xAA) {
                  encoder_val[0] = recv_data[0];
                  encoder_val[1] = recv_data[1];
                  encoder_val[2] = recv_data[2];
                  encoder_val[3] = recv_data[3];
                  interval = recv_data[4];
                  white_qty = recv_data[5];
                  is_left = recv_data[6];
                  is_right = recv_data[7];
                  vector_plus = recv_data[8];
                  vector_minus = recv_data[9];
                  inside_dir_plus = recv_data[10];
                  inside_dir_minus = recv_data[11];

                  vector = SimplifyDeg(vector_plus == 0 ? vector_minus * -1 : vector_plus);
                  inside_dir = SimplifyDeg(inside_dir_plus == 0 ? inside_dir_minus * -1 : inside_dir_plus);

                  // 送信
                  serial.putc(*mode);
            }
            data_length = 0;
      } else {
            recv_data[data_length - 1] = serial.getc();
            data_length++;
      }
}