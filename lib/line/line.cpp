#include "line.h"

#include "mbed.h"

Line::Line(PinName tx_, PinName rx_, uint8_t* mode_) : serial(tx_, rx_) {
      serial.baud(115200);
      serial.attach(callback(this, &Line::Receive), Serial::RxIrq);

      this->mode = mode_;
}

void Line::Receive() {
      static uint8_t data_length;
      static uint8_t line_vector_plus, line_vector_minus;

      if (data_length == 0) {
            uint8_t head = serial.getc();
            if (head == 0xFF) {
                  data_length += 1;
            } else {
                  data_length = 0;
            }
      } else if (data_length == 1) {
            encoder_val[0] = serial.getc();
            data_length += 1;
      } else if (data_length == 2) {
            encoder_val[1] = serial.getc();
            data_length += 1;
      } else if (data_length == 3) {
            encoder_val[2] = serial.getc();
            data_length += 1;
      } else if (data_length == 4) {
            encoder_val[3] = serial.getc();
            data_length += 1;
      } else if (data_length == 5) {
            line_white_num = serial.getc();
            data_length += 1;
      } else if (data_length == 6) {
            is_line_left = serial.getc();
            data_length += 1;
      } else if (data_length == 7) {
            is_line_right = serial.getc();
            data_length += 1;
      } else if (data_length == 8) {
            line_vector_plus = serial.getc();
            data_length += 1;
      } else if (data_length == 9) {
            line_vector_minus = serial.getc();
            line_vector = SimplifyDeg(line_vector_plus == 0 ? line_vector_minus * -1 : line_vector_plus);

            // 送信
            serial.putc(*mode);
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