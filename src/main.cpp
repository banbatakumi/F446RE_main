#include <mbed.h>

#include "config.h"

void setup() {
      // UART初期設定
      uiSerial.baud(UI_UART_SPEED);
      //uiSerial.attach(&Ui, SerialBase::RxIrq);

      // モーター
      motor.SetPwmPeriod(10);                      // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      motor.SetAttitudeControlPID(1.5, 0, 0.1);  // デフォルトゲイン：(1, 0.5, 0.1)

      // ドリブラー
      dribblerFront.SetPwmPeriod(10);  // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      dribblerBack.SetPwmPeriod(10);   // 5us:200kHz, 10us:100kHz, 20us:50kHz, 100us:10kHz, 1000us:1kHz
      
      holdFront.SetTh();
      holdBack.SetTh();
}

int main() {
      setup();
      while (1) {
            ModeRun();
      }
}