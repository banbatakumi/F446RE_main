#include "dribbler.h"
#include "hold.h"
#include "mbed.h"
#include "motor.h"
#include "voltage.h"

// ピン定義
voltage Voltage(PA_4);
motor Motor(PC_6, PC_8, PB_14, PB_15, PB_2, PB_10, PB_3, PB_5);
dribbler Dribbler(PB_8, PB_9, PB_6, PB_7);
hold Hold(PC_5, PC_4);

DigitalOut led_1(PA_5);
DigitalOut led_2(PA_6);

// グローバル変数定義
int16_t yaw;

int main() {
      while (1) {
            Voltage.read();
            Motor.yaw = yaw;
      }
}