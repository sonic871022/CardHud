#pragma once
#include <M5Unified.h>
#include "gnss.h"

// Trip statistics page (page 3). Draws directly on the LCD:
// static labels on enter(), values refreshed in-place (bg-colored strings).
class StatPage {
 public:
  void begin() {}
  void enter();               // clear screen + draw static layout
  void update(const GnssData& g, float maxKmh, float movingSec, int recPts, bool sdOk);
};
