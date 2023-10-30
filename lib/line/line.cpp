#include "line.h"

#include "mbed.h"

Line::Line(PinName tx_, PinName rx_, uint8_t* mode_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Line::Receive), Serial::RxIrq);

      this->mode = mode_;
}

void Line::Receive() {
      static uint8_t data_length;
      static uint8_t recv_data[9];
      static uint8_t line_vector_plus, line_vector_minus;

      if (data_length == 0) {
            if (serial.getc() == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 1) {
            recv_data[0] = serial.getc();
            data_length++;
      } else if (data_length == 2) {
            recv_data[1] = serial.getc();
            data_length++;
      } else if (data_length == 3) {
            recv_data[2] = serial.getc();
            data_length++;
      } else if (data_length == 4) {
            recv_data[3] = serial.getc();
            data_length++;
      } else if (data_length == 5) {
            recv_data[4] = serial.getc();
            data_length++;
      } else if (data_length == 6) {
            recv_data[5] = serial.getc();
            data_length++;
      } else if (data_length == 7) {
            recv_data[6] = serial.getc();
            data_length++;
      } else if (data_length == 8) {
            recv_data[7] = serial.getc();
            data_length++;
      } else if (data_length == 9) {
            recv_data[8] = serial.getc();
            data_length++;
      } else if (data_length == 10) {
            if(serial.getc() == 0xAA){
                  encoder_val[0] = recv_data[0];
                  encoder_val[1] = recv_data[1];
                  encoder_val[2] = recv_data[2];
                  encoder_val[3] = recv_data[3];
                  line_white_num = recv_data[4];
                  is_line_left = recv_data[5];
                  is_line_right = recv_data[6];
                  line_vector_plus = recv_data[7];
                  line_vector_minus = recv_data[8];

                  line_vector = SimplifyDeg(line_vector_plus == 0 ? line_vector_minus * -1 : line_vector_plus);

                  // 送信
                  serial.putc(*mode);
            }
            data_length = 0;
      }
}

uint8_t Line::EncoderVal(uint8_t which_sensor_) {
      return encoder_val[which_sensor_];
}
bool Line::IsLeft() {
      return is_line_left;
}
bool Line::IsRight() {
      return is_line_right;
}
uint8_t Line::WhiteNum() {
      return line_white_num;
}
int16_t Line::Vector() {
      return line_vector;
}