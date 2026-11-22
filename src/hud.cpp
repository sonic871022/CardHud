#include "hud.h"
#include <math.h>
#include <arduinoFFT.h>

// Layout (240 x 135):
//  +--------------------------------------------------+
//  | [compass]      88 km/h (big, center)   info col  |
//  | [compass]                              info col  |
//  | ---------------- sound waveform strip ---------- |
//  +--------------------------------------------------+

static constexpr int WAVE_Y = 104;
static constexpr int WAVE_H = 30;

static constexpr uint16_t C_WHITE = 0xFFFF;
static constexpr uint16_t C_GRAY  = 0x8C71;
static constexpr uint16_t C_DIM   = 0x5344;
static constexpr uint16_t C_RED   = 0xF981;
static constexpr uint16_t C_GREEN = 0x06E0;
static constexpr uint16_t C_TEAL  = 0x06F9;
static constexpr uint16_t C_ORANGE  = 0xFD20;   // saturated orange (#FFA000-ish)
static constexpr uint16_t C_ORANGE_DIM = 0x5A10; // dim orange for the baseline
static constexpr uint16_t C_BG    = 0x0000;

void Hud::begin() {
  M5.Lcd.setRotation(0);  // board default: 240x135 landscape, correct mounting
  M5.Lcd.fillScreen(C_BG);
  canvas_.setColorDepth(16);
  canvas_.createSprite(240, 135);
  // spectrum buffers start zeroed

  strip_.setColorDepth(16);
  if (!strip_.createSprite(240, 31)) return;  // spectrum strip for pages 2/3

  if (M5.Mic.isEnabled()) {
    M5.Mic.begin();
  }
}

void Hud::updateMic() {
  // self-throttled: mic.record() blocks for ~16ms at 16kHz, so cap the rate
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 30) return;
  lastMs = now;
  sampleMic();
}

void Hud::sampleMic() {
  static int16_t buf[256];
  uint32_t t0 = micros();
  recOk_ = false;
  if (!M5.Mic.isEnabled()) return;
  if (!M5.Mic.isRecording()) {
    if (!M5.Mic.begin()) return;
  }
  if (!M5.Mic.record(buf, 256, 16000)) return;
  recOk_ = true;
  uint32_t recUs = micros() - t0;
  if (recUs > 40000) markSlow('R', recUs);   // record() normally ~16ms

  // raw peak (diagnostics readout)
  int32_t peak = 0;
  for (int i = 0; i < 256; i++) {
    int v = abs(buf[i]);
    if (v > peak) peak = v;
  }
  peak_ = (uint16_t)(peak > 65535 ? 65535 : peak);

  // ---- FFT -> 24 log-spaced frequency bands ----
  for (int i = 0; i < 256; i++) {
    vReal_[i] = (double)buf[i] / 32768.0;
    vImag_[i] = 0.0;
  }
  arduinoFFT fft(vReal_, vImag_, 256, 16000);
  fft.DCRemoval();
  fft.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  fft.Compute(FFT_FORWARD);
  fft.ComplexToMagnitude();   // magnitudes in vReal_[0..127]
  uint32_t fftUs = micros() - t0 - recUs;
  if (fftUs > 15000) markSlow('F', fftUs);

  // log-spaced band edges over bins 1..127 (62.5 Hz per bin)
  static int edges[NBANDS + 1];
  static bool edgesInit = false;
  if (!edgesInit) {
    for (int i = 0; i <= NBANDS; i++)
      edges[i] = (int)(1.0f * powf(128.0f, (float)i / NBANDS));
    edgesInit = true;
  }

  constexpr int MAXH = 28;    // full bar height (strip is 30px, 1px margins)
  constexpr float BAND_GAIN = 3.5f;  // sensitivity boost for quiet music
  constexpr float NOISE_E   = 2.5f;  // per-band noise gate (post-gain)
  constexpr float HF_TILT   = 3.0f;  // extra gain at the TOP band (linear tilt:
                                     // band 0 = 1.0x unchanged, voice band ~2x,
                                     // highs up to 3x extra -> show easily)
  float bandE[NBANDS];
  float maxE = 0;
  for (int b = 0; b < NBANDS; b++) {
    float e = 0;
    for (int j = edges[b]; j < edges[b + 1] && j < 128; j++)
      if (vReal_[j] > e) e = (float)vReal_[j];
    // spectral tilt: FFT energy falls off with frequency, so highs need more
    // gain to reach the same bar height. Lowest bands (engine/road noise)
    // stay exactly as they were.
    float t = (float)b / (NBANDS - 1);
    e *= BAND_GAIN * (1.0f + (HF_TILT - 1.0f) * t);
    if (e < NOISE_E) e = 0;   // gate: background hiss never lights a bar
    bandE[b] = e;
    if (e > maxE) maxE = e;
  }

  // overall envelope: fast attack, slow release — keeps relative band
  // dynamics visible (music "dances") instead of pinning everything full.
  if (maxE > env_) env_ += (maxE - env_) * 0.30f;
  else             env_ *= 0.98f;
  if (env_ < 8.0f) env_ = 8.0f;

  for (int b = 0; b < NBANDS; b++) {
    int h = (int)(bandE[b] / env_ * MAXH);
    if (h > MAXH) h = MAXH;
    if (h < 0) h = 0;
    bars_[b] = (uint8_t)h;
    if (h >= peaks_[b]) peaks_[b] = (uint8_t)h;
    else if (peaks_[b] > 0) peaks_[b]--;   // peak-hold dot falls slowly
  }
}

void Hud::drawCompass(float course, bool hasFix) {
  constexpr int cx = 30, cy = 27, r = 24;   // top-left corner
  canvas_.drawCircle(cx, cy, r, C_GREEN);   // saturated green ring
  canvas_.drawCircle(cx, cy, r - 1, C_GREEN);

  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::middle_center);
  canvas_.setTextColor(C_RED, C_BG);        // North marker in red
  canvas_.drawString("N", cx, cy - (r - 7));
  canvas_.setTextColor(C_GRAY, C_BG);
  canvas_.drawString("S", cx, cy + (r - 7));
  canvas_.drawString("E", cx + (r - 7), cy);
  canvas_.drawString("W", cx - (r - 7), cy);

  if (!hasFix) {
    canvas_.setTextColor(C_DIM, C_BG);
    canvas_.drawString("--", cx, cy);
    return;
  }

  // Needle: 0 deg = North (up), scaled for r=24; tip kept short of the letters
  float a = course * DEG_TO_RAD;
  float dx = sinf(a), dy = -cosf(a);
  float px = -dy, py = dx;  // perpendicular
  int tipX = (int)(cx + dx * 15), tipY = (int)(cy + dy * 15);
  int b1x = (int)(cx - dx * 4 + px * 3.5f), b1y = (int)(cy - dy * 4 + py * 3.5f);
  int b2x = (int)(cx - dx * 4 - px * 3.5f), b2y = (int)(cy - dy * 4 - py * 3.5f);
  canvas_.fillTriangle(tipX, tipY, b1x, b1y, b2x, b2y, C_RED);
  // tail
  int t1x = (int)(cx - dx * 7 + px * 2), t1y = (int)(cy - dy * 7 + py * 2);
  int t2x = (int)(cx - dx * 7 - px * 2), t2y = (int)(cy - dy * 7 - py * 2);
  canvas_.fillTriangle(cx, cy, t1x, t1y, t2x, t2y, C_DIM);
}

void Hud::zeroTilt() {
  pitchOff_ += pitchS_;   // subtract current displayed angle going forward
  rollOff_  += rollS_;
  pitchS_ = 0;
  rollS_  = 0;
  zeroFlashMs_ = millis();
}

void Hud::drawTilt() {
  // Car-type inclinometer below the compass (driven by BMI270 pitch/roll)
  constexpr int cx = 30, cy = 76, r = 17;
  constexpr float SCALE = 25.0f;  // degrees mapped to full radius
  canvas_.drawCircle(cx, cy, r, C_WHITE);   // white frame
  canvas_.drawLine(cx - r + 2, cy, cx + r - 2, cy, C_DIM);
  canvas_.drawLine(cx, cy - r + 2, cx, cy + r - 2, C_DIM);
  // center target ring
  canvas_.drawCircle(cx, cy, 4, C_DIM);

  float lim = r - 2.0f;  // clamp limit for the car icon

  uint16_t col = C_GREEN;
  if (fabsf(rollS_) > 18.0f || fabsf(pitchS_) > 18.0f) col = C_RED;
  else if (fabsf(rollS_) > 8.0f || fabsf(pitchS_) > 8.0f) col = C_ORANGE;

  // Simple side-view car replaces the bubble:
  // - left/right tilt rotates the car, front/back tilt moves it up/down
  // (axis roles swapped to match the device's landscape mounting)
  float a = pitchS_ * DEG_TO_RAD;
  float ca = cosf(a), sa = sinf(a);
  float oy = rollS_ / SCALE * lim;
  if (oy > lim) oy = lim;
  if (oy < -lim) oy = -lim;
  const int8_t body[8][2] = {  // car silhouette (side view, near ring edge)
    {-13, 4}, {-13, -1}, {-8, -1}, {-5, -7}, {6, -7}, {9, -1}, {13, -1}, {13, 4}
  };
  int px[8], py[8];
  for (int i = 0; i < 8; i++) {
    float x = body[i][0], y = body[i][1];
    px[i] = (int)roundf(cx + x * ca - y * sa);
    py[i] = (int)roundf(cy + oy + x * sa + y * ca);
  }
  for (int i = 0; i < 8; i++) {
    canvas_.drawLine(px[i], py[i], px[(i + 1) % 8], py[(i + 1) % 8], col);
  }
  // wheels
  const float wx[2] = {-7, 7}, wy[2] = {6, 6};
  for (int i = 0; i < 2; i++) {
    int dx = (int)roundf(wx[i] * ca - wy[i] * sa);
    int dy = (int)roundf(wx[i] * sa + wy[i] * ca);
    canvas_.fillCircle(cx + dx, cy + (int)oy + dy, 2, col);
  }

  // tilt magnitude (deg) below the ring
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::middle_center);
  char buf[8];
  snprintf(buf, sizeof(buf), "%d", (int)roundf(sqrtf(rollS_ * rollS_ + pitchS_ * pitchS_)));
  canvas_.setTextColor(col, C_BG);
  canvas_.drawString(buf, cx, cy + r + 6);

  // brief "ZERO" confirmation after pressing R
  if (zeroFlashMs_ && millis() - zeroFlashMs_ < 900) {
    canvas_.setFont(&fonts::Font0);
    canvas_.setTextSize(1);
    canvas_.setTextDatum(textdatum_t::middle_center);
    canvas_.setTextColor(C_GREEN, C_BG);
    canvas_.drawString("ZERO", cx, cy - r - 3);
  }
}

void Hud::drawSpeed(const GnssData& g) {
  canvas_.setTextDatum(textdatum_t::middle_center);
  const int cxs = 120;  // screen horizontal center

  if (!g.enabled) {
    // GPS turned off with the S key (compact: must not overlap compass/tilt)
    canvas_.setFont(&fonts::Font2);
    canvas_.setTextSize(1);
    canvas_.setTextColor(C_DIM, C_BG);
    canvas_.drawString("GPS OFF", cxs, 38);
    canvas_.setFont(&fonts::Font0);
    canvas_.setTextSize(1);
    canvas_.setTextColor(C_GRAY, C_BG);
    canvas_.drawString("press S to enable", cxs, 60);
    return;
  }

  if (!g.rmcSeen) {
    // waiting for a valid position fix
    canvas_.setFont(&fonts::Orbitron_Light_32);
    canvas_.setTextSize(1);
    canvas_.setTextColor(C_DIM, C_BG);
    canvas_.drawString("GPS", cxs, 34);

    // heartbeat: proves the loop is alive
    if ((millis() / 500) & 1) {
      canvas_.fillCircle(cxs + 42, 34, 5, C_TEAL);
    }

    canvas_.setFont(&fonts::Font0);
    canvas_.setTextSize(1);
    canvas_.setTextDatum(textdatum_t::middle_center);
    char buf[48];
    snprintf(buf, sizeof(buf), "UART %d/%d %ld",
             g.curRx, g.curTx, (long)g.curBaud);
    canvas_.setTextColor(C_GRAY, C_BG);
    canvas_.drawString(buf, cxs, 58);
    snprintf(buf, sizeof(buf), "RX:%lu %s",
             (unsigned long)g.rxBytes, g.locked ? "LOCKED" : "scan");
    canvas_.drawString(buf, cxs, 74);
    return;
  }

  if (!g.valid) {
    canvas_.setFont(&fonts::Orbitron_Light_32);
    canvas_.setTextSize(1);
    canvas_.setTextColor(C_DIM, C_BG);
    canvas_.drawString("--", cxs, 40);
    canvas_.setFont(&fonts::Font0);
    canvas_.setTextSize(1);
    canvas_.setTextColor(C_GRAY, C_BG);
    char buf[32];
    snprintf(buf, sizeof(buf), "locating  sats:%d", g.sats);
    canvas_.drawString(buf, cxs, 68);
    return;
  }

  int kmh = (int)roundf(g.speed_kmh);
  uint16_t col = (kmh > 80) ? C_RED : C_WHITE;
  char buf[16];
  snprintf(buf, sizeof(buf), "%d", kmh);
  canvas_.setFont(&fonts::Orbitron_Light_32);
  canvas_.setTextSize(2);
  canvas_.setTextColor(col, C_BG);
  canvas_.drawString(buf, cxs, 36);
  canvas_.setTextSize(1);
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextColor(C_GRAY, C_BG);
  canvas_.drawString("km/h", cxs, 84);
}

void Hud::drawRight(const GnssData& g) {
  // Three big rows: direction / altitude / trip distance
  constexpr int x = 172;
  const char* dirs[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  char buf[16];

  canvas_.setTextDatum(textdatum_t::top_left);

  // battery at the very top-right corner (cached, refreshed every 60s
  // to avoid per-frame I2C reads that flicker when a read fails)
  static uint32_t lastBatMs = 0;
  static int cachedBat = -1;
  if (lastBatMs == 0 || millis() - lastBatMs > 60000) {
    cachedBat = M5.Power.getBatteryLevel();
    lastBatMs = millis();
  }
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::middle_right);
  char bbuf[8];
  if (cachedBat < 0) {
    snprintf(bbuf, sizeof(bbuf), "BAT ?");
    canvas_.setTextColor(C_GRAY, C_BG);
  } else {
    snprintf(bbuf, sizeof(bbuf), "%3d%%", cachedBat);
    canvas_.setTextColor(cachedBat > 50 ? C_GREEN : (cachedBat > 20 ? C_ORANGE : C_RED), C_BG);
  }
  canvas_.drawString(bbuf, 238, 6);
  canvas_.setTextDatum(textdatum_t::top_left);

  // --- DIR ---
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextColor(C_GRAY, C_BG);
  canvas_.drawString("DIR", x, 2);
  canvas_.setFont(&fonts::Font2);
  canvas_.setTextColor(C_WHITE, C_BG);
  if (g.valid) {
    int deg = (int)(g.course_deg + 0.5f) % 360;
    snprintf(buf, sizeof(buf), "%3d %s", deg, dirs[(deg + 22) / 45 % 8]);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }
  canvas_.drawString(buf, x, 11);

  // --- ALT ---
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextColor(C_GRAY, C_BG);
  canvas_.drawString("ALT", x, 34);
  canvas_.setFont(&fonts::Font2);
  canvas_.setTextColor(C_WHITE, C_BG);
  if (g.valid) {
    snprintf(buf, sizeof(buf), "%5.0fm", g.altitude_m);
  } else {
    snprintf(buf, sizeof(buf), "---");
  }
  canvas_.drawString(buf, x, 43);

  // --- TRIP ---
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextColor(C_GRAY, C_BG);
  canvas_.drawString("TRIP", x, 66);
  canvas_.setFont(&fonts::Font2);
  canvas_.setTextColor(C_WHITE, C_BG);
  if (g.trip_km < 99.95f) {
    snprintf(buf, sizeof(buf), "%4.1fkm", g.trip_km);
  } else {
    snprintf(buf, sizeof(buf), "%4.0fkm", g.trip_km);
  }
  canvas_.drawString(buf, x, 75);
}

void Hud::cycleSpectrumStyle() {
  specStyle_ = (specStyle_ + 1) % 3;
  styleFlashMs_ = millis();
}

void Hud::drawStripInto(M5Canvas& c, int y, int h) {
  c.fillRect(0, y, 240, h, C_BG);
  const int BOT = y + h - 2;                 // bar baseline
  constexpr uint16_t C_ORANGE_DIM = 0x6320;  // ~40% brightness orange
  constexpr uint16_t C_CAP        = 0xFDE0;  // bright peak cap

  if (specStyle_ == 1) {
    // ---- RIDGE: smooth cosine-interpolated curve, filled below ----
    int ys[NBANDS];
    for (int b = 0; b < NBANDS; b++) ys[b] = BOT - bars_[b];
    for (int x = 0; x < 240; x++) {
      float t = (x - 4.0f) / 10.0f;              // band centers at x=4+10b
      if (t < 0) t = 0;
      if (t > NBANDS - 1) t = NBANDS - 1;
      int b = (int)t;
      float f = t - b;
      float f2 = (1.0f - cosf(f * (float)M_PI)) * 0.5f;  // cosine smoothing
      float yy = (b < NBANDS - 1) ? ys[b] * (1.0f - f2) + ys[b + 1] * f2
                                  : (float)ys[NBANDS - 1];
      int yi = (int)lroundf(yy);
      if (yi < y + 1) yi = y + 1;
      if (yi > BOT) yi = BOT;
      if (yi < BOT) c.drawFastVLine(x, yi, BOT - yi + 1, C_ORANGE_DIM);
      c.drawPixel(x, yi, C_ORANGE);              // bright crest line
    }
    for (int b = 0; b < NBANDS; b++) {           // falling peak dots on crest
      int p = peaks_[b];
      if (p > 1) c.fillRect(b * 10 + 3, BOT - p, 3, 1, C_CAP);
    }
  } else if (specStyle_ == 2) {
    // ---- BUTTERFLY: symmetric bars expanding from the center line ----
    int cy = y + h / 2;
    for (int b = 0; b < NBANDS; b++) {
      int x = b * 10;
      int hh = bars_[b] / 2;                     // half-height (max 14)
      if (bars_[b] > 0 && hh == 0) hh = 1;
      if (hh > 0) {
        c.fillRect(x, cy - hh, 8, hh * 2, C_ORANGE);
        c.fillRect(x, cy - hh, 8, 1, C_CAP);
        c.fillRect(x, cy + hh - 1, 8, 1, C_CAP);
      }
    }
  } else {
    // ---- BARS: 24-band dancing bars + falling peak-hold dots ----
    for (int b = 0; b < NBANDS; b++) {
      int x = b * 10;
      int hb = bars_[b];
      if (hb > 0) c.fillRect(x, BOT - hb, 8, hb, C_ORANGE);
      int p = peaks_[b];
      if (p > 1) c.fillRect(x, BOT - p, 8, 1, C_CAP);  // bright cap
    }
  }

  // style name flash after switching
  if (millis() - styleFlashMs_ < 1500) {
    c.setFont(&fonts::Font0);
    c.setTextSize(1);
    c.setTextDatum(textdatum_t::middle_left);
    c.setTextColor(C_CAP, C_BG);
    static const char* kNames[3] = {"BARS", "RIDGE", "BUTTERFLY"};
    c.drawString(kNames[specStyle_], 4, y + 6);
  }
}

void Hud::drawStripLcd() {
  drawStripInto(strip_, 0, 31);
  strip_.pushSprite(0, 104);
}

void Hud::drawSpectrum() {
  drawStripInto(canvas_, WAVE_Y, 135 - WAVE_Y);

  // slow-section diagnostics: tag G=gnss R=record F=fft H=whole loop,
  // flashes for 3s (bottom-left, right of the page indicator)
  if (g_slowTag && millis() - g_slowMs < 3000) {
    char hbuf[12];
    snprintf(hbuf, sizeof(hbuf), "%c%ums", g_slowTag, (unsigned)(g_slowUs / 1000));
    canvas_.setTextDatum(textdatum_t::middle_left);
    canvas_.setTextColor(C_ORANGE, C_BG);
    canvas_.drawString(hbuf, 34, 129);
  }
}

void Hud::update(const GnssData& g) {
  uint32_t now = millis();
  if (now - lastFrame_ < 25) return;
  lastFrame_ = now;

  // FPS counter
  frames_++;
  static uint32_t fpsT0 = 0;
  if (now - fpsT0 >= 1000) {
    fps_ = (frames_ > 99) ? 99 : frames_;
    frames_ = 0;
    fpsT0 = now;
  }

  // IMU -> pitch / roll (accel based + EMA smoothing, relative to zeroTilt())
  float ax = 0, ay = 0, az = -1;
  if (M5.Imu.update()) {
    M5.Imu.getAccel(&ax, &ay, &az);
  }
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - pitchOff_;
  float roll  = atan2f(ay, az) * RAD_TO_DEG - rollOff_;
  pitchS_ += (pitch - pitchS_) * 0.15f;
  rollS_  += (roll  - rollS_)  * 0.15f;

  // NOTE: mic sampling moved to Hud::updateMic(), called from the main loop
  // on every page so the waveform keeps scrolling outside the HUD page.

  canvas_.fillSprite(C_BG);
  drawCompass(g.course_deg, g.valid);
  drawTilt();
  drawSpeed(g);
  drawRight(g);
  drawSpectrum();
  canvas_.pushSprite(0, 0);
}
