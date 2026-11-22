#include "M5Cardputer.h"
#include "gnss.h"
#include "hud.h"
#include "track.h"
#include "mappage.h"
#include "statpage.h"

Gnss gnss;
Hud hud;
TrackRec track;
MapPage mapPage;
StatPage statPage;

// Diagnostics shared with the HUD
bool g_micEnabled = false;
bool g_micStarted = false;
int  g_boardId = -1;

// Trip statistics
float g_maxKmh = 0;
float g_movingSec = 0;
uint32_t g_lastLoopMs = 0;

// Loop hiccup diagnostics (shown on HUD page): any slow section records a
// tag + duration; hud displays the most recent one for 3s.
//   G = gnss.update() slow   R = mic record() slow
//   F = FFT slow             H = whole loop slow (no section blamed)
char g_slowTag = 0;
uint32_t g_slowUs = 0;
uint32_t g_slowMs = 0;
void markSlow(char tag, uint32_t us) {
  g_slowTag = tag;
  g_slowUs = us;
  g_slowMs = millis();
}

enum class Page : uint8_t { HUD = 0, MAP = 1, STATS = 2 };
Page g_page = Page::HUD;

void setup() {
  auto cfg = M5.config();
  cfg.internal_imu = true;   // BMI270
  cfg.internal_mic = true;   // ES8311 + MEMS mic
  cfg.internal_spk = false;
  M5Cardputer.begin(cfg, true);

  g_boardId = (int)M5.getBoard();
  g_micEnabled = M5.Mic.isEnabled();
  g_micStarted = g_micEnabled ? M5.Mic.begin() : false;

  M5.Log.printf("CarHUD boot. board=%d IMU:%s Mic:%s/%s\n",
                g_boardId,
                M5.Imu.isEnabled() ? "ok" : "MISSING",
                g_micEnabled ? "cfg" : "none",
                g_micStarted ? "started" : "fail");

  gnss.begin();   // EXT bus UART: RX=15, TX=13 (Cap LoRa-1262 GNSS NMEA)
  hud.begin();
  track.begin();  // microSD: SPI CLK=40 MOSI=14 MISO=39 CS=12
  mapPage.begin();
  statPage.begin();
  statPage.enter();
}

static void drawPageIndicator(Page p) {
  M5.Lcd.setFont(&fonts::Font0);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextDatum(textdatum_t::middle_left);
  char buf[6];
  snprintf(buf, sizeof(buf), "%d/3", (int)p + 1);
  M5.Lcd.setTextColor(0x8C71, 0x0000);
  M5.Lcd.drawString(buf, 4, 129);
}

// The HUD page paints the sound strip at y104..135 via a full-screen sprite.
// Pages that don't redraw that band (map page sprite is only 104px tall)
// would otherwise keep a stale frozen strip image — wipe it on page switch.
static void clearSoundBand() {
  M5.Lcd.fillRect(0, 104, 240, 31, 0x0000);
}

void loop() {
  uint32_t loopT0 = micros();
  uint32_t loopStartMs = millis();
  M5Cardputer.update();
  uint32_t now = millis();
  float dt = (now - g_lastLoopMs) / 1000.0f;
  g_lastLoopMs = now;

  uint32_t tSec = micros();
  gnss.update();
  uint32_t gnssUs = micros() - tSec;
  if (gnssUs > 15000) markSlow('G', gnssUs);
  const GnssData& g = gnss.data();

  // trip statistics
  if (g.valid) {
    if (g.speed_kmh > g_maxKmh) g_maxKmh = g.speed_kmh;
    if (g.speed_kmh > 2.0f) g_movingSec += dt;
  }
  track.update(g);
  hud.updateMic();   // mic sampling runs on every page (wave never stalls)

  // ---- keyboard events (fire once per new physical press) ----
  // Trigger on: (a) rising edge of any key pressed, OR (b) the pressed-key
  // count increasing (covers combos like Fn+Arrow, where Fn is already
  // held when the second key arrives). Held keys do not re-trigger.
  static bool kbWasDown = false;
  static size_t kbLastCount = 0;
  auto& kbd = M5Cardputer.Keyboard;
  bool kbDown = kbd.isPressed();
  size_t kbCount = kbd.keyList().size();
  bool kbEvent = (kbDown && !kbWasDown) || (kbCount > kbLastCount);
  kbWasDown = kbDown;
  kbLastCount = kbCount;

  KbEvt kb;
  if (kbEvent) {
    auto st = kbd.keysState();
    kb.left = st.left;   kb.right = st.right;
    kb.up = st.up;       kb.down = st.down;
    kb.enter = st.enter; kb.backspace = st.backspace;
    kb.nword = (uint8_t)min(st.word.size(), (size_t)16);
    for (int i = 0; i < kb.nword; i++) kb.word[i] = st.word[i];
    // Bare arrow keys on CardputerADV: dedicated keys share matrix
    // positions with ; , . / (arrows live in the lib's Fn layer), so
    // decode them directly from physical coordinates — no Fn needed.
    for (const auto& p : kbd.keyList()) {
      if (p.x == 10 && p.y == 3) kb.left  = true;  // LEFT arrow
      if (p.x == 12 && p.y == 3) kb.right = true;  // RIGHT arrow
      if (p.x == 11 && p.y == 2) kb.up    = true;  // UP arrow
      if (p.x == 11 && p.y == 3) kb.down  = true;  // DOWN arrow
    }
  }

  // page switching (left/right arrows) — disabled while naming a waypoint
  if (!mapPage.isEntering() && (kb.left || kb.right)) {
    Page old = g_page;
    if (kb.right) g_page = (Page)(((int)g_page + 1) % 3);
    if (kb.left)  g_page = (Page)(((int)g_page + 2) % 3);
    if (g_page != old) {
      clearSoundBand();
      if (g_page == Page::STATS) statPage.enter();
      if (g_page == Page::MAP)   mapPage.onEnter(track);
    }
  }

  switch (g_page) {
    case Page::HUD:
      hud.update(g);
      break;
    case Page::MAP:
      mapPage.update(g, kb, track);
      break;  // sound strip only on HUD page (user preference)
    case Page::STATS:
      statPage.update(g, g_maxKmh, g_movingSec, track.trackCount(), track.sdOk());
      break;
  }

  // spectrum strip at the bottom of pages 2/3 (data keeps flowing via
  // updateMic() in the main loop, so it dances on every page)
  if (g_page != Page::HUD) hud.drawStripLcd();

  // global keys (R zero / S gps / N spectrum style / 1-3 page jump)
  if (kbEvent && !mapPage.isEntering() && kb.nword > 0) {
    for (char c : kb.word) {
      if (c == 'r' || c == 'R') hud.zeroTilt();
      if (c == 's' || c == 'S') gnss.setEnabled(!gnss.data().enabled);
      if (c == 'n' || c == 'N') hud.cycleSpectrumStyle();
      if (c >= '1' && c <= '3') {
        Page sel = (Page)(c - '1');
        if (sel != g_page) {
          g_page = sel;
          clearSoundBand();
          if (g_page == Page::STATS) statPage.enter();
          if (g_page == Page::MAP)   mapPage.onEnter(track);
        }
      }
    }
  }

  drawPageIndicator(g_page);

  // hiccup monitor: blame the whole loop only when no section was tagged
  uint32_t loopUs = micros() - loopT0;
  if (loopUs > 60000 && g_slowMs < loopStartMs) markSlow('H', loopUs);

  // Keyboard raw-position debug line (bottom of stats page, drawn OVER the
  // spectrum strip only while keys are held — release and the strip returns).
  if (g_page == Page::STATS) {
    auto& kbd = M5Cardputer.Keyboard;
    const auto& list = kbd.keyList();
    if (list.size() > 0) {
      M5.Lcd.setFont(&fonts::Font0);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextDatum(textdatum_t::top_left);
      M5.Lcd.fillRect(0, 117, 240, 11, 0x0000);
      int px = 4;
      for (size_t i = 0; i < list.size() && i < 3; i++) {
        KeyValue_t v = kbd.getKeyValue(list[i]);
        char seg[18];
        snprintf(seg, sizeof(seg), "%d,%d:%02X/%02X", list[i].x, list[i].y,
                 (uint8_t)v.value_first, (uint8_t)v.value_third);
        M5.Lcd.setTextColor(0xFD20, 0x0000);
        M5.Lcd.drawString(seg, px, 118);
        px += strlen(seg) * 6 + 6;
      }
    }
    // released: nothing to clear — the strip redraws next frame
  }
}
