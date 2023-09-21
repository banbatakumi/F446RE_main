#include "simplify_deg.h"

int SimplifyDeg(int deg) {
      while (deg > 180) {
            deg -= 360;
      }
      while (deg < -180) {
            deg += 360;
      }

      return deg;
}