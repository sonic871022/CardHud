#include "track.h"
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <strings.h>   // strcasecmp

// Cardputer ADV microSD slot (from M5Unified pin table):
//   SPI: CLK=40, MOSI(COPI)=14, MISO(CIPO)=39, CS=12
static constexpr uint8_t SD_SCK = 40, SD_MOSI = 14, SD_MISO = 39, SD_CS = 12;

float TrackRec::distM(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * 110540.0f;
  float dLon = (lon2 - lon1) * 111320.0f * cosf((lat1 + lat2) * 0.5f * DEG_TO_RAD);
  return sqrtf(dLat * dLat + dLon * dLon);
}

void TrackRec::begin() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdOk_ = SD.begin(SD_CS, SPI, 25000000);
  if (sdOk_) {
    SD.mkdir("/tracks");
    loadWaypoints();
  }
}

// UTC(+8) -> Beijing time, with day rollover. utcDate = "ddmmyy".
bool TrackRec::chinaTime(const char* utcDate, const char* utcTime,
                         int& y, int& mo, int& d, int& h, int& mi, int& s) {
  if (!utcDate[0] || !utcTime[0]) return false;
  d  = (utcDate[0] - '0') * 10 + (utcDate[1] - '0');
  mo = (utcDate[2] - '0') * 10 + (utcDate[3] - '0');
  y  = 2000 + (utcDate[4] - '0') * 10 + (utcDate[5] - '0');
  h  = (utcTime[0] - '0') * 10 + (utcTime[1] - '0') + 8;
  mi = (utcTime[2] - '0') * 10 + (utcTime[3] - '0');
  s  = (utcTime[4] - '0') * 10 + (utcTime[5] - '0');
  if (h >= 24) {
    h -= 24;
    d += 1;
    int dim = 31;
    if (mo == 2) dim = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 29 : 28;
    else if (mo == 4 || mo == 6 || mo == 9 || mo == 11) dim = 30;
    if (d > dim) { d = 1; mo += 1; if (mo > 12) { mo = 1; y += 1; } }
  }
  return true;
}

bool TrackRec::startRecord(const GnssData& g) {
  if (!sdOk_ || !g.valid || recording_) return false;
  int y, mo, d, h, mi, s;
  if (!chinaTime(g.utcDate, g.utcTime, y, mo, d, h, mi, s)) return false;
  char path[52];
  snprintf(path, sizeof(path), "/tracks/TRK_%04d%02d%02d_%02d%02d%02d.trk",
           y, mo, d, h, mi, s);
  file_ = SD.open(path, FILE_WRITE);
  if (!file_) return false;
  recording_ = true;
  n_ = 0;
  hasLast_ = false;
  sinceFlush_ = 0;
  return true;
}

void TrackRec::stopRecord() {
  if (file_) file_.close();
  recording_ = false;
}

void TrackRec::update(const GnssData& g) {
  if (!recording_ || !g.valid) return;
  int32_t la = (int32_t)lroundf(g.lat * 1e7f);
  int32_t lo = (int32_t)lroundf(g.lon * 1e7f);
  if (hasLast_ &&
      distM(lastLat_ / 1e7f, lastLon_ / 1e7f, g.lat, g.lon) < 8.0f) return;
  hasLast_ = true;
  lastLat_ = la;
  lastLon_ = lo;
  if (n_ < MAX_PTS) { pts_[n_].lat = la; pts_[n_].lon = lo; n_++; }
  if (file_) {
    file_.printf("%s,%ld,%ld\n", g.utcTime, (long)la, (long)lo);
    if (++sinceFlush_ >= 8) { file_.flush(); sinceFlush_ = 0; }
  }
}

bool TrackRec::loadRoute(const char* path) {
  rn_ = 0;
  routeLoaded_ = false;
  if (!sdOk_) return false;
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  while (f.available() && rn_ < MAX_PTS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    long la, lo;
    // CSV: hhmmss,lat1e7,lon1e7
    if (sscanf(line.c_str(), "%*[^,],%ld,%ld", &la, &lo) == 2) {
      route_[rn_].lat = (int32_t)la;
      route_[rn_].lon = (int32_t)lo;
      rn_++;
    }
  }
  f.close();
  if (rn_ < 2) { rn_ = 0; return false; }
  routeLoaded_ = true;
  // basename without extension as route name
  const char* base = strrchr(path, '/');
  base = base ? base + 1 : path;
  strncpy(routeName_, base, sizeof(routeName_) - 1);
  char* dot = strrchr(routeName_, '.');
  if (dot) *dot = '\0';
  routeName_[sizeof(routeName_) - 1] = '\0';
  return true;
}

void TrackRec::clearRoute() {
  rn_ = 0;
  routeLoaded_ = false;
  routeName_[0] = '\0';
}

void TrackRec::loadWaypoints() {
  wn_ = 0;
  File f = SD.open("/waypoints.csv", FILE_READ);
  if (!f) return;
  while (f.available() && wn_ < MAX_WPTS) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    char name[16];
    long la, lo;
    if (sscanf(line.c_str(), "%12[^,],%ld,%ld", name, &la, &lo) == 3) {
      strncpy(wpts_[wn_].name, name, 12);
      wpts_[wn_].name[12] = '\0';
      wpts_[wn_].lat = (int32_t)la;
      wpts_[wn_].lon = (int32_t)lo;
      wn_++;
    }
  }
  f.close();
}

bool TrackRec::addWaypoint(const char* name, float lat, float lon) {
  int32_t la = (int32_t)lroundf(lat * 1e7f);
  int32_t lo = (int32_t)lroundf(lon * 1e7f);
  // Same-name replacement: update the existing entry's coordinates instead
  // of appending a duplicate.
  for (int i = 0; i < wn_; i++) {
    if (strcasecmp(wpts_[i].name, name) == 0) {
      wpts_[i].lat = la;
      wpts_[i].lon = lo;
      saveWaypoints();
      return true;
    }
  }
  if (wn_ >= MAX_WPTS) return false;
  Wpt& w = wpts_[wn_];
  strncpy(w.name, name, 12);
  w.name[12] = '\0';
  w.lat = la;
  w.lon = lo;
  wn_++;
  saveWaypoints();
  return true;
}

bool TrackRec::deleteWpt(int i) {
  if (i < 0 || i >= wn_) return false;
  for (int k = i; k < wn_ - 1; k++) wpts_[k] = wpts_[k + 1];
  wn_--;
  saveWaypoints();
  return true;
}

void TrackRec::saveWaypoints() {
  if (!sdOk_) return;
  File f = SD.open("/waypoints.csv", FILE_WRITE);
  if (!f) return;
  for (int i = 0; i < wn_; i++) {
    f.printf("%s,%ld,%ld\n", wpts_[i].name, (long)wpts_[i].lat, (long)wpts_[i].lon);
  }
  f.close();
}

int TrackRec::nearestWpt(float lat, float lon, float* outM) const {
  int best = -1;
  float bd = 1e12f;
  for (int i = 0; i < wn_; i++) {
    float d = distM(lat, lon, wpts_[i].lat / 1e7f, wpts_[i].lon / 1e7f);
    if (d < bd) { bd = d; best = i; }
  }
  if (outM) *outM = bd;
  return best;
}

int TrackRec::nearestRouteIdx(float lat, float lon, float* devM) const {
  if (!rn_) { if (devM) *devM = 0; return -1; }
  int best = 0;
  float bd = 1e12f;
  for (int i = 0; i < rn_; i++) {
    float d = distM(lat, lon, route_[i].lat / 1e7f, route_[i].lon / 1e7f);
    if (d < bd) { bd = d; best = i; }
  }
  if (devM) *devM = bd;
  return best;
}

float TrackRec::routeRemainingKm(int idx) const {
  if (idx < 0 || idx >= rn_ - 1) return 0.0f;
  float sum = 0;
  for (int i = idx; i < rn_ - 1; i++) {
    sum += distM(route_[i].lat / 1e7f, route_[i].lon / 1e7f,
                 route_[i + 1].lat / 1e7f, route_[i + 1].lon / 1e7f);
  }
  return sum / 1000.0f;
}

float TrackRec::routeRemainingKmRev(int idx) const {
  if (idx < 0 || idx >= rn_) return 0.0f;
  float sum = 0;
  for (int i = 0; i < idx; i++) {
    sum += distM(route_[i].lat / 1e7f, route_[i].lon / 1e7f,
                 route_[i + 1].lat / 1e7f, route_[i + 1].lon / 1e7f);
  }
  return sum / 1000.0f;
}
