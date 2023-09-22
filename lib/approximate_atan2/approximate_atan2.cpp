#include "approximate_atan2.h"

int16_t MyAtan2(int16_t _y, int16_t _x, uint8_t _accuracy_level) {
      int16_t x = abs(_x);
      int16_t y = abs(_y);
      float z;
      bool c;

      c = y < x;
      if (c)
            z = (float)y / x;
      else
            z = (float)x / y;

      int16_t a;
      if (_accuracy_level = 0) {
            a = z * (-1556 * z + 6072);   // 2次曲線近似
      } else if (_accuracy_level = 1) {
            a = z * (z * (-448 * z - 954) + 5894);   // 3次曲線近似
      } else if (_accuracy_level = 2) {
            a = z * (z * (z * (829 * z - 2011) - 58) + 5741);   // 4次曲線近似
      }

      if (c) {
            if (_x > 0) {
                  if (_y < 0) a *= -1;
            }
            if (_x < 0) {
                  if (_y > 0) a = 18000 - a;
                  if (_y < 0) a = a - 18000;
            }
      }

      if (!c) {
            if (_x > 0) {
                  if (_y > 0) a = 9000 - a;
                  if (_y < 0) a = a - 9000;
            }
            if (_x < 0) {
                  if (_y > 0) a = a + 9000;
                  if (_y < 0) a = -a - 9000;
            }
      }

      a /= 100;
      return a;
}
