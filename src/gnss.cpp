#include "gnss.h"
#include <math.h>

// Cap LoRa-1262 GNSS NMEA comes out on the EXT bus UART:
//   ESP32-S3 RX = G15 (UART_RX), TX = G13 (UART_TX)
// Grove port (PORT.CUSTOM: white=G1, yellow=G2) is probed as fallback.
// Baud autodetect: 9600 (standard NMEA) and 115200.

constexpr Gnss::Probe Gnss::kProbes[4];

void Gnss::begin() {
  openProbe(0);
}

void Gnss::setEnabled(bool en) {
  if (en == d_.enabled) return;
  d_.enabled = en;
  if (!en) {
    ser_.end();           // release UART pins, stop feeding data
  } else {
    openProbe(probe_);    // reopen the last active probe
  }
}

void Gnss::openProbe(uint8_t i) {
  probe_ = i;
  const Probe& p = kProbes[i];
  if (ser_) {
    // Driver already running: cheap reconfig instead of end()/begin().
    // Full teardown/reinstall every 4s probe cycle caused heap churn and
    // periodic audio stutter. updateBaudRate+setPins keep the same driver.
    ser_.updateBaudRate(p.baud);
    ser_.setPins(p.rx, p.tx);
    // drop stale bytes in bulk (a few passes max; ring is 2048)
    static uint8_t dump[512];
    for (int k = 0; k < 8 && ser_.available() > 0; k++)
      ser_.read(dump, sizeof(dump));
  } else {
    ser_.begin(p.baud, SERIAL_8N1, p.rx, p.tx, false, 2048);
  }
  d_.curRx = p.rx;
  d_.curTx = p.tx;
  d_.curBaud = p.baud;
  d_.locked = false;
  probeStart_ = millis();
  idx_ = 0;
}

void Gnss::update() {
  if (!d_.enabled) return;

  // Cycle probes until we see a real NMEA sentence ('$' at line start).
  if (!d_.locked && millis() - probeStart_ > 4000) {
    openProbe((probe_ + 1) % 4);
    return;
  }

  // Bulk read: ONE mutex acquisition per chunk instead of one per char.
  // Capped at 512 bytes per loop pass — leftovers stay in the 2048-byte RX
  // ring and are picked up next pass, so gnss.update() always stays sub-ms
  // and the spectrum never stalls waiting on the UART.
  static uint8_t rbuf[512];
  int n = ser_.available();
  if (n <= 0) return;
  if (n > 512) n = 512;
  n = (int)ser_.read(rbuf, (size_t)n);
  for (int i = 0; i < n; i++) {
    char c = (char)rbuf[i];
    d_.rxBytes++;
    if (c == '\r') continue;
    if (c == '\n') {
      line_[idx_] = '\0';
      if (idx_ > 0) processLine(line_);
      idx_ = 0;
      continue;
    }
    if (idx_ < sizeof(line_) - 1) {
      line_[idx_++] = c;
    } else {
      idx_ = sizeof(line_) - 1;  // drop overflow, wait for EOL
    }
  }
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

void Gnss::processLine(char* line) {
  if (line[0] != '$') return;

  // First '$' proves this probe (pins+baud) is correct -> lock it.
  if (!d_.locked) {
    d_.locked = true;
  }

  // NMEA checksum check (XOR of chars between $ and *)
  char* star = strchr(line, '*');
  if (star) {
    uint8_t sum = 0;
    for (char* p = line + 1; p < star; p++) sum ^= (uint8_t)*p;
    int hi = hexVal(star[1]), lo = star[2] ? hexVal(star[2]) : -1;
    if (hi >= 0 && lo >= 0 && (uint8_t)((hi << 4) | lo) != sum) return;
  } else {
    star = line + strlen(line);
  }
  *star = '\0';  // cut checksum for easy field parsing

  // Split fields in place
  char* fields[20];
  int nf = 0;
  fields[nf++] = line + 1;  // skip '$', first field = sentence type
  for (char* p = line + 1; *p && nf < 20; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[nf++] = p + 1;
    }
  }

  const char* type = fields[0];
  size_t tlen = strlen(type);
  if (tlen < 5) return;
  const char* body = type + 2;  // skip talker (GN/GP/BD/GL)

  if (strcmp(body, "RMC") == 0 && nf > 8) {
    // $xxRMC,time,status,lat,N,lon,E,speed(knots),course,...
    d_.rmcSeen = true;
    // UTC time (fields[1] = hhmmss.sss) and date (fields[9] = ddmmyy)
    if (nf > 9 && strlen(fields[1]) >= 6) {
      memcpy(d_.utcTime, fields[1], 6);
      d_.utcTime[6] = '\0';
    }
    if (nf > 9 && strlen(fields[9]) >= 6) {
      memcpy(d_.utcDate, fields[9], 6);
      d_.utcDate[6] = '\0';
    }
    if (fields[2][0] == 'A') {
      d_.valid = true;
      float knots = atof(fields[7]);
      float kmh = knots * 1.852f;
      if (kmh < 2.0f) kmh = 0.0f;  // entertainment-grade zero filter
      d_.speed_kmh = kmh;
      // odometer: integrate speed over time between valid RMCs (~1 Hz)
      uint32_t now = millis();
      if (d_.lastUpdateMs) {
        float dt = (now - d_.lastUpdateMs) / 1000.0f;
        if (dt > 0.0f && dt < 5.0f) d_.trip_km += kmh * dt / 3600.0f;
      }
      d_.lastUpdateMs = now;
      float crs = atof(fields[8]);
      if (kmh > 0.0f) d_.course_deg = crs;
      // Lat/lon in ddmm.mmmm format
      float latRaw = atof(fields[3]);
      float lonRaw = atof(fields[5]);
      if (latRaw > 0 && lonRaw > 0) {
        d_.lat = (int)(latRaw / 100) + fmodf(latRaw, 100.0f) / 60.0f;
        d_.lon = (int)(lonRaw / 100) + fmodf(lonRaw, 100.0f) / 60.0f;
        if (fields[4][0] == 'S') d_.lat = -d_.lat;
        if (fields[6][0] == 'W') d_.lon = -d_.lon;
      }
    } else {
      d_.valid = false;  // V = void, no fix yet
    }
  } else if (strcmp(body, "GGA") == 0 && nf > 10) {
    // $xxGGA,time,lat,N,lon,E,fix,sats,hdop,alt,M,...
    d_.sats = (uint8_t)atoi(fields[7]);
    if (atoi(fields[6]) > 0) d_.altitude_m = atof(fields[9]);
  }
}
