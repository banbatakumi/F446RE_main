#include "ultrasonic.h"

#include "mbed.h"

Ultrasonic::Ultrasonic(PinName tx_, PinName rx_, uint32_t serial_baud_) : serial(tx_, rx_) {
      serial.baud(serial_baud_);
      serial.attach(callback(this, &Ultrasonic::Receive), SerialBase::RxIrq);
}

void Ultrasonic::Receive() {
      static uint8_t data_length;  // データの長さ
      const uint8_t recv_data_num = 6;
      static uint8_t recv_data[recv_data_num];
      uint8_t read_byte;
      serial.read(&read_byte, 1);

      if (data_length == 0) {  // ヘッダの受信
            if (read_byte == 0xFF) {
                  data_length++;
            } else {
                  data_length = 0;
            }
      } else if (data_length == recv_data_num + 1) {
            if (read_byte == 0xAA) {
                  val[0] = recv_data[0];
                  val[1] = recv_data[1];
                  val[2] = recv_data[2];
                  val[3] = recv_data[3];
                  ir_dir = recv_data[4] * 2 - 180;
                  ir_dis = recv_data[5];

                  // 送信
                  serial.write(&do_ir_led_on, 1);
            }

            data_length = 0;
      } else {
            recv_data[data_length - 1] = read_byte;
            data_length++;
      }
}

void Ultrasonic::IrLedOn() {
      do_ir_led_on = 1;
}

void Ultrasonic::IrLedOff() {
      do_ir_led_on = 0;
}