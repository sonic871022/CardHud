#pragma once
#include <Arduino.h>
#include <FS.h>
#include "gnss.h"

// GPS track recorder + reference-route loader + named waypoints (SD card).
// Points are stored as int32 lat/lon (deg * 1e7) — 8 bytes each.
struct TrackPoint {
  int32_t lat, lon;
};

class TrackRec {
 public:
  static constexpr int MAX_PTS  = 2048;
  static constexpr int MAX_WPTS = 16;
  struct Wpt { char name[13]; int32_t lat, lon; };

  void begin();                       // init SD, load waypoint list
  void update(const GnssData& g);     // record points while recording

  bool sdOk() const { return sdOk_; }

  // ---- recording (session track) ----
  bool recording() const { return recording_; }
  bool startRecord(const GnssData& g);  // false if no SD card / no fix
  void stopRecord();
  int  trackCount() const { return n_; }
  TrackPoint trackPoint(int i) const { return pts_[i]; }
  const TrackPoint* trackData() const { return pts_; }   // raw array (fast draw)

  // ---- reference route (follow / navigation mode) ----
  bool loadRoute(const char* path);     // load a .trk file from SD
  void clearRoute();
  bool routeLoaded() const { return rn_ > 0; }
  int  routeCount() const { return rn_; }
  TrackPoint routePoint(int i) const { return route_[i]; }
  const TrackPoint* routeData() const { return route_; } // raw array (fast draw)
  const char* routeName() const { return routeName_; }

  // ---- waypoints ----
  int  wptCount() const { return wn_; }
  const Wpt& wpt(int i) const { return wpts_[i]; }
  bool addWaypoint(const char* name, float lat, float lon);
  bool deleteWpt(int i);                                     // remove + rewrite CSV
  int  nearestWpt(float lat, float lon, float* outM) const;  // -1 if none

  // ---- navigation math ----
  static float distM(float lat1, float lon1, float lat2, float lon2);
  int   nearestRouteIdx(float lat, float lon, float* devM) const;
  float routeRemainingKm(int idx) const;    // length of route[idx .. end]
  float routeRemainingKmRev(int idx) const; // length of route[0 .. idx] (reverse)

 private:
  void loadWaypoints();
  void saveWaypoints();               // rewrite /waypoints.csv from RAM
  static bool chinaTime(const char* utcDate, const char* utcTime,
                        int& y, int& mo, int& d, int& h, int& mi, int& s);

  TrackPoint pts_[MAX_PTS];     int n_ = 0;      // session track (recording)
  TrackPoint route_[MAX_PTS];   int rn_ = 0;     // loaded reference route
  bool routeLoaded_ = false;
  char routeName_[40] = "";

  Wpt wpts_[MAX_WPTS];          int wn_ = 0;

  File file_;
  bool sdOk_ = false;
  bool recording_ = false;
  int32_t lastLat_ = 0, lastLon_ = 0;
  bool hasLast_ = false;
  uint8_t sinceFlush_ = 0;
};
