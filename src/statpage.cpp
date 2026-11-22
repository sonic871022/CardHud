#include "statpage.h"

static constexpr uint16_t S_WHITE  = 0xFFFF;
static constexpr uint16_t S_GRAY   = 0x8C71;
static constexpr uint16_t S_DIM    = 0x5344;
static constexpr uint16_t S_GREEN  = 0x06E0;
static constexpr uint16_t S_ORANGE = 0xFD20;
static constexpr uint16_t S_BG     = 0x0000;

static void label(int x, int y, const char* txt) {
  M5.Lcd.setFont(&fonts::Font0);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(textdatum_t::top_left);
  M5.Lcd.setTextColor(S_GRAY, S_BG);
  M5.Lcd.drawString(txt, x, y);
}

static void value(int x, int y, const char* txt, uint16_t col) {
  M5.Lcd.setFont(&fonts::Font2);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(textdatum_t::top_left);
  M5.Lcd.setTextColor(col, S_BG);
  M5.Lcd.drawString(txt, x, y);
}

void StatPage::enter() {
  M5.Lcd.fillScreen(S_BG);
  label(10, 6,  "MAX SPEED");
  label(10, 44, "AVG SPEED");
  label(10, 74, "MOVE TIME");
  label(124, 6,  "TRIP");
  label(124, 44, "SAT / ALT");
  label(124, 74, "REC PTS");
}

void StatPage::update(const GnssData& g, float maxKmh, float movingSec,
                      int recPts, bool sdOk) {
  char buf[24];
  // NOTE: no "--" placeholder pre-draw — drawing then overwriting every
  // frame made the values flicker white.

  snprintf(buf, sizeof(buf), "%5.1f km/h", maxKmh);
  value(10, 16, buf, maxKmh > 80 ? S_ORANGE : S_WHITE);

  if (movingSec > 30) {
    snprintf(buf, sizeof(buf), "%5.1f km/h",
             g.trip_km / (movingSec / 3600.0f));
  } else {
    snprintf(buf, sizeof(buf), "  --  km/h");
  }
  value(10, 54, buf, S_WHITE);

  int hh = (int)(movingSec / 3600);
  int mm = (int)(movingSec / 60) % 60;
  int ss = (int)movingSec % 60;
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hh, mm, ss);
  value(10, 84, buf, S_WHITE);

  snprintf(buf, sizeof(buf), "%5.2f km", g.trip_km);
  value(124, 16, buf, S_WHITE);

  snprintf(buf, sizeof(buf), "%2d  %5.0fm", g.sats, g.altitude_m);
  value(124, 54, buf, g.sats > 0 ? S_GREEN : S_GRAY);

  snprintf(buf, sizeof(buf), "%4d %s", recPts, sdOk ? "SD ok" : "NO SD");
  value(124, 84, buf, sdOk ? S_WHITE : S_ORANGE);
}
