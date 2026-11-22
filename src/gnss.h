#pragma once
#include <Arduino.h>

struct GnssData {
  bool valid = false;        // RMC status A (position fix)
  bool rmcSeen = false;      // received at least one valid RMC sentence
  uint8_t sats = 0;          // satellites in use
  float speed_kmh = 0.0f;    // filtered speed
  float course_deg = 0.0f;   // true course over ground
  float lat = 0.0f, lon = 0.0f;
  float altitude_m = 0.0f;   // GGA altitude (m)
  float trip_km = 0.0f;      // odometer, integrated from speed
  uint32_t lastUpdateMs = 0; // millis of last valid RMC
  bool locked = false;       // NMEA '$' seen -> current probe is correct
  uint32_t rxBytes = 0;      // total bytes received (diagnostics)
  char utcTime[9] = "";      // hhmmss (UTC, from RMC)
  char utcDate[9] = "";      // ddmmyy (UTC, from RMC)
  uint8_t curRx = 15, curTx = 13;  // currently active UART pins
  int32_t curBaud = 9600;
  bool enabled = true;       // S key toggle state
};

class Gnss {
 public:
  void begin();
  void update();
  void setEnabled(bool en);       // S key toggle: release / reopen the UART
  const GnssData& data() const { return d_; }

 private:
  struct Probe { uint8_t rx, tx; int32_t baud; };
  static constexpr Probe kProbes[4] = {
    {15, 13, 9600},    // EXT bus (Cap LoRa-1262), standard NMEA
    {15, 13, 115200},  // EXT bus, fast NMEA
    {1, 2, 9600},      // Grove port fallback (white=G1 RX, yellow=G2 TX)
    {1, 2, 115200},    // Grove port, fast
  };
  void openProbe(uint8_t idx);
  void processLine(char* line);
  GnssData d_;
  HardwareSerial ser_{2};
  char line_[160];
  size_t idx_ = 0;
  uint8_t probe_ = 0;
  uint32_t probeStart_ = 0;
};
