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
  // permanent help hint (gray, right of the page indicator)
  M5.Lcd.setTextColor(0x5344, 0x0000);
  M5.Lcd.drawString("H:HELP", 26, 129);
}

// Full-screen modal help overlay. Rendered into an offscreen sprite
// (atomic push = no flicker). Content is taller than the screen, so it
// scrolls with UP/DN arrows; H closes. Black background.
static const struct HelpLine { const char* k; const char* d; bool head; } kHelp[] = {
    {"CARHUD KEYS",  "",               true },
    {"-- GLOBAL --", "",               true },
    {"1/2/3 ARROW",  "switch page",    false},
    {"R",            "zero tilt",      false},
    {"S",            "GPS on/off",     false},
    {"N",            "spectrum style", false},
    {"-- MAP PAGE --","",              true },
    {"G",            "record on/off",  false},
    {"F",            "load/exit nav",  false},
    {"UP/DN",        "select route",   false},
    {"M",            "mark waypoint",  false},
    {"X",            "waypoint list",  false},
    {"DEL",          "delete route",   false},
    {"ENTER",        "confirm",        false},
};
static const int kHelpN = sizeof(kHelp) / sizeof(kHelp[0]);
static const int HELP_VIEW_H = 118;   // content viewport above the hint row

static void drawHelpOverlay(int scrollPx) {
  static M5Canvas help_{&M5.Lcd};
  static bool built = false;
  static int  renderedAt = -1;
  if (!built) {
    help_.setColorDepth(8);
    help_.createSprite(240, 135);
    built = true;
  }
  if (renderedAt == scrollPx) { help_.pushSprite(0, 0); return; }
  renderedAt = scrollPx;

  const uint16_t cBg   = 0x0000;          // pure black background
  const uint16_t cHead = 0xFD20;          // orange section headers
  const uint16_t cKey  = 0xFFE0;          // yellow key names
  const uint16_t cDesc = 0x9CF3;          // dim descriptions
  help_.fillScreen(cBg);
  help_.setFont(&fonts::Font0);
  help_.setTextSize(1);
  help_.setTextDatum(textdatum_t::middle_left);
  int y = 6 - scrollPx;
  for (int i = 0; i < kHelpN; i++) {
    if (y > -10 && y < HELP_VIEW_H) {
      const HelpLine& L = kHelp[i];
      if (L.head) {
        help_.setTextColor(cHead, cBg);
        help_.drawString(L.k, 6, y);
      } else {
        help_.setTextColor(cKey, cBg);
        help_.drawString(L.k, 6, y);
        help_.setTextColor(cDesc, cBg);
        help_.drawString(L.d, 88, y);
      }
    }
    y += (i == 0) ? 11 : 10;              // title row a bit taller
  }
  // footer hint row (fixed, always visible)
  help_.setTextDatum(textdatum_t::middle_right);
  help_.setTextColor(0x5344, cBg);
  help_.drawString("H:CLOSE  UP/DN:SCROLL", 234, 129);
  help_.pushSprite(0, 0);   // single atomic blit — no flicker
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

  // ---- H-key help overlay (modal): H opens/closes, UP/DN scrolls ----
  static bool helpShow_ = false;
  static int  helpScroll_ = 0;              // px offset into the help content
  if (helpShow_) {
    if (kbEvent) {
      // scroll first, then check for H to close
      if (kb.up)   helpScroll_ = (helpScroll_ > 20) ? helpScroll_ - 20 : 0;
      if (kb.down) helpScroll_ = (helpScroll_ < 10) ? 30 : helpScroll_ + 20;
      if (helpScroll_ > 30) helpScroll_ = 30;
      for (char c : kb.word)
        if (c == 'h' || c == 'H') { helpShow_ = false; break; }
    }
  } else if (kbEvent && !mapPage.isEntering() && kb.nword > 0) {
    for (char c : kb.word)
      if (c == 'h' || c == 'H') { helpShow_ = true; helpScroll_ = 0; break; }
  }

  // page switching (left/right arrows) — disabled while naming a waypoint
  // or while the help overlay is up
  if (!helpShow_ && !mapPage.isEntering() && (kb.left || kb.right)) {
    Page old = g_page;
    if (kb.right) g_page = (Page)(((int)g_page + 1) % 3);
    if (kb.left)  g_page = (Page)(((int)g_page + 2) % 3);
    if (g_page != old) {
      clearSoundBand();
      if (g_page == Page::STATS) statPage.enter();
      if (g_page == Page::MAP)   mapPage.onEnter(track);
    }
  }

  if (helpShow_) {
    drawHelpOverlay(helpScroll_);   // modal: page updates frozen, recording/GPS continue
  } else {
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
  }

  // hiccup monitor: blame the whole loop only when no section was tagged
  uint32_t loopUs = micros() - loopT0;
  if (loopUs > 60000 && g_slowMs < loopStartMs) markSlow('H', loopUs);
}
