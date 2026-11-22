#pragma once
#include <M5Unified.h>
#include "gnss.h"

// Shared diagnostics (defined in main.cpp)
extern bool g_micEnabled;
extern bool g_micStarted;
extern int  g_boardId;
extern char     g_slowTag;   // slow-section tag: G/R/F/H
extern uint32_t g_slowUs;    // its duration in microseconds
extern uint32_t g_slowMs;    // millis() timestamp
void markSlow(char tag, uint32_t us);

class Hud {
 public:
  static constexpr int NBANDS = 24;   // spectrum bars
  void begin();
  void update(const GnssData& g);
  void zeroTilt();   // R key: re-zero the inclinometer at current position
  void updateMic();  // mic sampling (self-throttled, call every loop)
  void cycleSpectrumStyle();  // N key: bars -> ridge -> butterfly -> ...
  uint8_t spectrumStyle() const { return specStyle_; }
  // spectrum strip on pages 2/3: drawn into a 240x31 sprite pushed at y104
  void drawStripLcd();

 private:
  void sampleMic();
  void drawCompass(float course, bool hasFix);
  void drawTilt();
  void drawSpeed(const GnssData& g);
  void drawRight(const GnssData& g);
  void drawSpectrum();
  void drawStripInto(M5Canvas& c, int y, int h);  // 3-style visuals, any canvas

  M5Canvas canvas_{&M5.Lcd};
  M5Canvas strip_{&M5.Lcd};       // 240x31 spectrum strip for pages 2/3
  // FFT spectrum: 256 pts @16kHz -> 24 log-spaced bands (dancing bars)
  double vReal_[256];
  double vImag_[256];
  uint8_t bars_[NBANDS] = {0};    // current bar heights (px, 0..28)
  uint8_t peaks_[NBANDS] = {0};   // peak-hold markers (falling dots)
  float env_ = 40.0f;             // overall energy envelope (slow normalization)
  float pitchS_ = 0, rollS_ = 0;
  float pitchOff_ = 0, rollOff_ = 0;   // zero-reference captured by zeroTilt()
  uint32_t zeroFlashMs_ = 0;           // "ZERO" flash timestamp
  uint32_t lastFrame_ = 0;
  uint16_t frames_ = 0;
  uint8_t fps_ = 0;
  uint16_t peak_ = 0;
  bool recOk_ = false;
  float agc_ = 1500;          // adaptive gain envelope (auto-scales waveform)
  uint8_t specStyle_ = 0;     // 0=bars 1=ridge 2=butterfly
  uint32_t styleFlashMs_ = 0; // style-name flash timestamp
};
