#include "esp32.h"

#include "mbed.h"

Esp32::Esp32(PinName tx_, PinName rx_, float* own_dir_, uint8_t* mode_, uint32_t serial_baud_) : serial(tx_, rx_) {
      serial.baud(serial_baud_);
      serial.attach(callback(this, &Esp32::Receive), SerialBase::RxIrq);
      this->own_dir = own_dir_;
      this->mode = mode_;
}

void Esp32::Receive() {
      static uint8_t data_length;  // データの長さ
      const uint8_t recv_data_num = 4;
      static uint8_t recv_data[recv_data_num];
      uint8_t read_byte;
      uint8_t send_byte;
      bool do_rorate;
      serial.read(&read_byte, 1);

      if (data_length == 0) {  // ヘッダの受信
            if (read_byte == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (read_byte == 0xAA) {
                  top_yaw = recv_data[0] * 2 - 180;
                  ir_dir = recv_data[1] * 2 - 180;
                  ir_dis = recv_data[2];
                  is_connect_to_ally = (recv_data[3]) & 1;
                  is_ally_moving = (recv_data[3] >> 1) & 1;
                  is_ally_defense = (recv_data[3] >> 2) & 1;
                  is_ally_catch_ball = (recv_data[3] >> 3) & 1;
                  can_ally_get_pass = (recv_data[3] >> 4) & 1;

                  // 送信
                  do_rorate = abs(*own_dir) < 35 ? 1 : 0;
                  is_moving = *mode != 0 ? 1 : 0;
                  send_byte = can_get_pass << 5 | is_catch_ball << 4 | is_defense << 3 | is_moving << 2 | on_ir_led << 1 | do_rorate;
                  serial.write(&send_byte, 1);
            }

            data_length = 0;
      } else {
            recv_data[data_length - 1] = read_byte;
            data_length++;
      }
}

void Esp32::OnIrLed() {
      on_ir_led = 1;
}

void Esp32::OffIrLed() {
      on_ir_led = 0;
}