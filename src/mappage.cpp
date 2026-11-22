#include "mappage.h"
#include <SD.h>
#include <math.h>

// Layout: square map area at top-left (104x104, above the wave strip),
// info panel on the right (x 106..239).
static constexpr int MAP_W = 104, MAP_H = 104;

// Initial bearing from trackpoint a to b, in degrees 0..360 (0 = north).
static float bearingDeg(const TrackPoint& a, const TrackPoint& b) {
  float la1 = a.lat / 1e7f, lo1 = a.lon / 1e7f;
  float la2 = b.lat / 1e7f, lo2 = b.lon / 1e7f;
  float dlo = (lo2 - lo1) * DEG_TO_RAD;
  float y = sinf(dlo) * cosf(la2 * DEG_TO_RAD);
  float x = cosf(la1 * DEG_TO_RAD) * sinf(la2 * DEG_TO_RAD) -
            sinf(la1 * DEG_TO_RAD) * cosf(la2 * DEG_TO_RAD) * cosf(dlo);
  float brg = atan2f(y, x) * RAD_TO_DEG;
  if (brg < 0) brg += 360.0f;
  return brg;
}

// Signed course-vs-bearing difference, wrapped to [-180, +180].
static float courseDiffDeg(float course, float bearing) {
  return fmodf(course - bearing + 540.0f, 360.0f) - 180.0f;
}

// palette
static constexpr uint16_t K_BG      = 0x0000;
static constexpr uint16_t K_GRID    = 0x4208;  // dim white grid (25% gray)
static constexpr uint16_t K_BORDER  = 0x630C;  // map frame
static constexpr uint16_t K_ROUTE   = 0x07E0;  // bright green route
static constexpr uint16_t K_ROUTE2  = 0x03E0;  // dim green (secondary track)
static constexpr uint16_t K_BLUE    = 0x1C7F;  // waypoint markers
static constexpr uint16_t K_CAR     = 0xFD20;  // current position (orange)
static constexpr uint16_t K_WHITE   = 0xFFFF;
static constexpr uint16_t K_GRAY    = 0x8C71;
static constexpr uint16_t K_DIM     = 0x5344;
static constexpr uint16_t K_RED     = 0xF981;
static constexpr uint16_t K_ORANGE  = 0xFD20;

void MapPage::begin() {
  canvas_.setColorDepth(16);
  if (!canvas_.createSprite(MAP_W + 136, 104)) return;  // full area incl panel
}

void MapPage::onEnter(TrackRec& tr) {
  refreshFiles(tr);
  loadPreviews();
  lastMatch_ = 0;  // force nav re-match
}

void MapPage::refreshFiles(TrackRec& tr) {
  nFiles_ = 0;
  sel_ = -1;
  if (!tr.sdOk()) return;
  File dir = SD.open("/tracks");
  if (!dir) return;
  File f;
  while ((f = dir.openNextFile()) && nFiles_ < 10) {
    if (!f.isDirectory()) {
      const char* nm = f.name();
      const char* base = strrchr(nm, '/');
      base = base ? base + 1 : nm;
      strncpy(files_[nFiles_], base, 39);
      files_[nFiles_][39] = '\0';
      nFiles_++;
    }
    f.close();
  }
  dir.close();
  if (nFiles_) sel_ = 0;
}

// Read every .trk file, decimate to <=512 points and cache with its bbox.
void MapPage::loadPreviews() {
  freePreviews();
  static TrackPoint raw[2048];  // shared scratch buffer (16 KB)
  for (int f = 0; f < nFiles_; f++) {
    char path[56];
    snprintf(path, sizeof(path), "/tracks/%s", files_[f]);
    File fh = SD.open(path, FILE_READ);
    if (!fh) continue;
    int total = 0;
    while (fh.available() && total < 2048) {
      String line = fh.readStringUntil('\n');
      long la, lo;
      if (sscanf(line.c_str(), "%*[^,],%ld,%ld", &la, &lo) == 2) {
        raw[total].lat = (int32_t)la;
        raw[total].lon = (int32_t)lo;
        total++;
      }
    }
    fh.close();
    if (total < 2) continue;
    int stride = (total + 511) / 512;      // decimation factor
    int n = (total + stride - 1) / stride;
    TrackPoint* pts = new TrackPoint[n];
    if (!pts) continue;
    float mnl = 1e9f, mxl = -1e9f, mno = 1e9f, mxo = -1e9f;
    float lenM = 0;
    for (int i = 0, j = 0; i < total && j < n; i += stride, j++) {
      pts[j] = raw[i];
      float la = pts[j].lat / 1e7f, lo = pts[j].lon / 1e7f;
      if (la < mnl) mnl = la;
      if (la > mxl) mxl = la;
      if (lo < mno) mno = lo;
      if (lo > mxo) mxo = lo;
      if (j > 0)
        lenM += TrackRec::distM(pts[j - 1].lat / 1e7f, pts[j - 1].lon / 1e7f, la, lo);
    }
    prev_[f].pts = pts;
    prev_[f].n = n;
    prev_[f].minLat = mnl; prev_[f].maxLat = mxl;
    prev_[f].minLon = mno; prev_[f].maxLon = mxo;
    prev_[f].lenKm = lenM / 1000.0f;
  }
}

void MapPage::freePreviews() {
  for (int f = 0; f < MAX_FILES; f++) {
    delete[] prev_[f].pts;
    prev_[f].pts = nullptr;
    prev_[f].n = 0;
    prev_[f].lenKm = 0;
  }
}

void MapPage::update(const GnssData& g, const KbEvt& kb, TrackRec& tr) {
  // ---------- keyboard ----------
  if (entering_) {
    for (int i = 0; i < kb.nword && entryLen_ < 12; i++) {
      char c = kb.word[i];
      if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
          (c >= 'a' && c <= 'z')) {
        entryBuf_[entryLen_++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
      }
    }
    entryBuf_[entryLen_] = '\0';
    if (kb.backspace && entryLen_ > 0) entryLen_--;
    entryBuf_[entryLen_] = '\0';
    if (kb.left) entering_ = false;             // cancel
    if (kb.enter) {                              // save waypoint
      char name[13];
      if (entryLen_ > 0) strncpy(name, entryBuf_, 12);
      else snprintf(name, sizeof(name), "WPT%d", tr.wptCount() + 1);
      name[12] = '\0';
      tr.addWaypoint(name, g.lat, g.lon);
      entering_ = false;
    }
  } else if (wpList_) {
    // ---------- waypoint list manager ----------
    int cnt = tr.wptCount();
    if (cnt == 0) {
      wpList_ = false;
    } else {
      if (kb.up)   selW_ = (selW_ <= 0) ? cnt - 1 : selW_ - 1;
      if (kb.down) selW_ = (selW_ + 1) % cnt;
      if (kb.backspace) {                    // DEL deletes the selected one
        tr.deleteWpt(selW_);
        if (tr.wptCount() == 0) wpList_ = false;
        else if (selW_ >= tr.wptCount()) selW_ = tr.wptCount() - 1;
      }
      bool closeReq = kb.enter;
      for (int i = 0; i < kb.nword; i++)
        if (kb.word[i] == 'x' || kb.word[i] == 'X') closeReq = true;
      if (closeReq) wpList_ = false;
    }
  } else {
    if (kb.up && nFiles_) {
      sel_ = (sel_ <= 0) ? nFiles_ - 1 : sel_ - 1;
      delArm_ = false;                         // selection changed: disarm DEL
    }
    if (kb.down && nFiles_) {
      sel_ = (sel_ + 1) % nFiles_;
      delArm_ = false;
    }
    for (int i = 0; i < kb.nword; i++) {
      char c = kb.word[i];
      if (c == 'g' || c == 'G') {                // record toggle
        if (tr.recording()) {
          tr.stopRecord();
          refreshFiles(tr);                      // new .trk file appeared
          loadPreviews();
        } else if (!tr.startRecord(g)) {
          strncpy(flashMsg_, "NO FIX", sizeof(flashMsg_) - 1);
          flashMsg_[sizeof(flashMsg_) - 1] = '\0';
          noFixFlash_ = true; noFixMs_ = millis();
        }
      } else if (c == 'f' || c == 'F') {         // F: load / exit navigation
        if (tr.routeLoaded()) {
          tr.clearRoute();                       // exit nav mode
          lastMatch_ = 0;
          revNav_ = false;
        } else if (sel_ >= 0) {
          char path[56];
          snprintf(path, sizeof(path), "/tracks/%s", files_[sel_]);
          tr.loadRoute(path);
          lastMatch_ = 0;
          revNav_ = false;  // re-detect direction on the fresh route
        } else {
          // nothing to load: tell the user why
          strncpy(flashMsg_, tr.sdOk() ? "NO ROUTE" : "NO SD",
                  sizeof(flashMsg_) - 1);
          flashMsg_[sizeof(flashMsg_) - 1] = '\0';
          noFixFlash_ = true; noFixMs_ = millis();
        }
      } else if (c == 'm' || c == 'M') {         // mark waypoint here
        if (g.valid) {
          entering_ = true;
          entryLen_ = 0;
          entryBuf_[0] = '\0';
        } else {
          strncpy(flashMsg_, "NO FIX", sizeof(flashMsg_) - 1);
          flashMsg_[sizeof(flashMsg_) - 1] = '\0';
          noFixFlash_ = true; noFixMs_ = millis();
        }
      } else if (c == 'x' || c == 'X') {         // waypoint list manager
        if (tr.wptCount() > 0) {
          wpList_ = true;
          selW_ = 0;
        }
      }
    }
    // DEL: delete the selected route file (2-step confirm within 3s)
    if (kb.backspace && sel_ >= 0) {
      auto flash = [&](const char* m) {
        strncpy(flashMsg_, m, sizeof(flashMsg_) - 1);
        flashMsg_[sizeof(flashMsg_) - 1] = '\0';
        noFixFlash_ = true; noFixMs_ = millis();
      };
      if (tr.recording()) {
        flash("STOP REC");                       // never delete a live recording
      } else if (delArm_ && millis() - delMs_ < 3000) {
        char path[56];
        snprintf(path, sizeof(path), "/tracks/%s", files_[sel_]);
        SD.remove(path);
        refreshFiles(tr);
        loadPreviews();
        delArm_ = false;
        flash("DELETED");
      } else {
        delArm_ = true;                          // first press: arm only
        delMs_ = millis();
        flash("DEL? AGAIN");
      }
    }
  }

  // ---------- navigation matching (1 Hz) ----------
  if (tr.routeLoaded() && g.valid && millis() - lastMatch_ > 1000) {
    nearestIdx_ = tr.nearestRouteIdx(g.lat, g.lon, &devM_);
    remainKm_ = revNav_ ? tr.routeRemainingKmRev(nearestIdx_)
                        : tr.routeRemainingKm(nearestIdx_);
    // auto direction detection: compare GPS course with the route bearing
    // at the nearest point (hysteresis band, only judged while moving).
    if (g.speed_kmh > 10.0f && nearestIdx_ >= 0) {
      int n = tr.routeCount();
      float brg = -999.0f;
      if (nearestIdx_ + 1 < n) {
        brg = bearingDeg(tr.routePoint(nearestIdx_), tr.routePoint(nearestIdx_ + 1));
      } else if (nearestIdx_ > 0) {
        brg = bearingDeg(tr.routePoint(nearestIdx_ - 1), tr.routePoint(nearestIdx_));
      }
      if (brg > -900.0f) {
        float da = fabsf(courseDiffDeg(g.course_deg, brg));
        if (da > 100.0f && !revNav_)      revNav_ = true;   // opposite way
        else if (da < 80.0f && revNav_)   revNav_ = false;  // back on course
      }
    }
    lastMatch_ = millis();
  }

  draw(g, tr);
}

void MapPage::draw(const GnssData& g, TrackRec& tr) {
  canvas_.fillSprite(K_BG);

  // ---------- view mode ----------
  // NAV    : route loaded -> fixed scale (1px = 4m), centered on the car
  // BROWSE : no route, files exist -> fit view to the SELECTED route bbox,
  //          draw all routes dim green, selected one orange
  // FALLBACK: no route, no preview -> fit to session track + car (old behavior)
  bool browse = !tr.routeLoaded() && nFiles_ && sel_ >= 0 &&
                prev_[sel_].n >= 2;
  float scale, cLat, cLon;
  float minLat, maxLat, minLon, maxLon;

  if (tr.routeLoaded() && g.valid) {
    // ---- NAV: fixed scale around the car (turn detail visible) ----
    scale = 0.4f;  // px per meter -> 1px = 2.5m, view span ~260m (zoomed in
                   // for turn detail; was 0.25 = 4m/px, too far out)
    cLat = g.lat;
    cLon = g.lon;
    float halfLat = (MAP_H * 0.5f) / scale / 110540.0f;
    float halfLon = (MAP_W * 0.5f) / scale /
                    (111320.0f * cosf(cLat * DEG_TO_RAD));
    minLat = g.lat - halfLat; maxLat = g.lat + halfLat;
    minLon = g.lon - halfLon; maxLon = g.lon + halfLon;
  } else {
    // ---- BROWSE / FALLBACK: compute a bounding box, then fit ----
    if (browse) {
      minLat = prev_[sel_].minLat; maxLat = prev_[sel_].maxLat;
      minLon = prev_[sel_].minLon; maxLon = prev_[sel_].maxLon;
    } else {
      minLat = 1e9f; maxLat = -1e9f; minLon = 1e9f; maxLon = -1e9f;
      bool have = false;
      for (int i = 0; i < tr.routeCount(); i++) {
        TrackPoint p = tr.routePoint(i);
        float la = p.lat / 1e7f, lo = p.lon / 1e7f;
        if (la < minLat) minLat = la;
        if (la > maxLat) maxLat = la;
        if (lo < minLon) minLon = lo;
        if (lo > maxLon) maxLon = lo;
        have = true;
      }
      for (int i = 0; i < tr.trackCount(); i++) {
        TrackPoint p = tr.trackPoint(i);
        float la = p.lat / 1e7f, lo = p.lon / 1e7f;
        if (la < minLat) minLat = la;
        if (la > maxLat) maxLat = la;
        if (lo < minLon) minLon = lo;
        if (lo > maxLon) maxLon = lo;
        have = true;
      }
      if (g.valid) {
        if (g.lat < minLat) minLat = g.lat;
        if (g.lat > maxLat) maxLat = g.lat;
        if (g.lon < minLon) minLon = g.lon;
        if (g.lon > maxLon) maxLon = g.lon;
        have = true;
      }
      if (!have) { minLat = maxLat = 0; minLon = maxLon = 0; }
    }
    cLat = (minLat + maxLat) * 0.5f;
    cLon = (minLon + maxLon) * 0.5f;
    float wM = TrackRec::distM(minLat, minLon, minLat, maxLon);
    float hM = TrackRec::distM(minLat, minLon, maxLat, minLon);
    if (wM < 40) wM = 40;
    if (hM < 40) hM = 40;
    scale = fminf((MAP_W - 14) / wM, (MAP_H - 14) / hM);  // px per meter
  }
  auto toX = [&](float lon) {
    return 52.0f + (lon - cLon) * 111320.0f * cosf(cLat * DEG_TO_RAD) * scale;
  };
  auto toY = [&](float lat) {
    return 52.0f - (lat - cLat) * 110540.0f * scale;
  };

  // ---------- lat/lon grid (dim white) ----------
  static const float kSteps[] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
  float stepM = 10000;
  for (float s : kSteps) {
    if (s * scale >= 14.0f) { stepM = s; break; }
  }
  float stepLatDeg = stepM / 110540.0f;
  float stepLonDeg = stepM / (111320.0f * cosf(cLat * DEG_TO_RAD));
  canvas_.drawRect(0, 0, MAP_W, MAP_H, K_BORDER);
  for (float la = floorf(minLat / stepLatDeg) * stepLatDeg;
       la <= maxLat + stepLatDeg; la += stepLatDeg) {
    int y = (int)roundf(toY(la));
    if (y > 0 && y < MAP_H) canvas_.drawFastHLine(1, y, MAP_W - 2, K_GRID);
  }
  for (float lo = floorf(minLon / stepLonDeg) * stepLonDeg;
       lo <= maxLon + stepLonDeg; lo += stepLonDeg) {
    int x = (int)roundf(toX(lo));
    if (x > 0 && x < MAP_W) canvas_.drawFastVLine(x, 1, MAP_H - 2, K_GRID);
  }

  // ---------- polyline drawing ----------
  auto drawLine2 = [&](int x1, int y1, int x2, int y2, uint16_t c) {
    canvas_.drawLine(x1, y1, x2, y2, c);
    canvas_.drawLine(x1, y1 + 1, x2, y2 + 1, c);
  };
  auto clip = [](int* x, int* y) {
    return *x >= 0 && *x < MAP_W && *y >= 0 && *y < MAP_H;
  };
  auto drawPts = [&](const TrackPoint* p, int n, uint16_t c) {
    int px = -9999, py = -9999;
    bool pen = false;
    for (int i = 0; i < n; i++) {
      int x = (int)roundf(toX(p[i].lon / 1e7f));
      int y = (int)roundf(toY(p[i].lat / 1e7f));
      if (!clip(&x, &y) || !clip(&px, &py)) { pen = false; px = x; py = y; continue; }
      if (pen && (abs(x - px) + abs(y - py)) >= 1) {
        drawLine2(px, py, x, y, c);
      }
      pen = true;
      px = x; py = y;
    }
  };
  if (browse) {
    // session track in gray (clearly different from stored routes)
    if (tr.trackCount() >= 2)
      drawPts(tr.trackData(), tr.trackCount(), K_GRAY);
    // all stored routes dim green, the selected one orange & on top
    for (int f = 0; f < nFiles_; f++) {
      if (!prev_[f].pts || prev_[f].n < 2 || f == sel_) continue;
      drawPts(prev_[f].pts, prev_[f].n, K_ROUTE2);
    }
    drawPts(prev_[sel_].pts, prev_[sel_].n, K_ORANGE);
  } else {
    if (tr.routeLoaded())
      drawPts(tr.routeData(), tr.routeCount(), K_ROUTE);  // nav route: bright green
    if (tr.trackCount() >= 2)
      drawPts(tr.trackData(), tr.trackCount(),
              tr.routeLoaded() ? K_ROUTE2 : K_ROUTE);     // session track
  }

  // nearest waypoint (relative to current position) — shown prominently
  float wdist;
  int wi = g.valid ? tr.nearestWpt(g.lat, g.lon, &wdist) : -1;

  // ---------- waypoints: nearest = blue triangle, others = green dot ----------
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::top_center);
  for (int i = 0; i < tr.wptCount(); i++) {
    const auto& w = tr.wpt(i);
    int x = (int)roundf(toX(w.lon / 1e7f));
    int y = (int)roundf(toY(w.lat / 1e7f));
    if (x < 5 || x > MAP_W - 5 || y < 5 || y > MAP_H - 12) continue;
    if (i == wi) {
      canvas_.fillTriangle(x - 4, y - 5, x + 4, y - 5, x, y + 3, K_BLUE);
      canvas_.setTextColor(K_BLUE, K_BG);
      canvas_.drawString(w.name, x, y + 4);
    } else {
      canvas_.fillCircle(x, y - 1, 2, K_ROUTE);
      canvas_.setTextColor(K_ROUTE2, K_BG);
      canvas_.drawString(w.name, x, y + 2);
    }
  }

  // ---------- current position (orange caret, rotates with course) ----------
  if (g.valid) {
    int cx = (int)roundf(toX(g.lon)), cy = (int)roundf(toY(g.lat));
    if (cx >= 0 && cx < MAP_W && cy >= 0 && cy < MAP_H) {
      float a = g.course_deg * DEG_TO_RAD;
      float dx = sinf(a), dy = -cosf(a);
      float px = -dy, py = dx;
      canvas_.fillTriangle(
          (int)(cx + dx * 6), (int)(cy + dy * 6),
          (int)(cx - dx * 3 + px * 3.5f), (int)(cy - dy * 3 + py * 3.5f),
          (int)(cx - dx * 3 - px * 3.5f), (int)(cy - dy * 3 - py * 3.5f),
          K_CAR);
    }
  }

  // scale label
  canvas_.setTextDatum(textdatum_t::bottom_left);
  canvas_.setTextColor(K_GRAY, K_BG);
  char sbuf[12];
  if (stepM >= 1000) snprintf(sbuf, sizeof(sbuf), "%gkm", stepM / 1000);
  else snprintf(sbuf, sizeof(sbuf), "%gm", stepM);
  canvas_.drawString(sbuf, 3, MAP_H - 2);

  // ---------- info panel (x 106..239) ----------
  constexpr int px = 108;
  canvas_.setTextDatum(textdatum_t::top_left);
  canvas_.setTextSize(1);
  canvas_.setFont(&fonts::Font0);

  // status line
  if (tr.recording()) {
    canvas_.setTextColor(K_RED, K_BG);
    snprintf(sbuf, sizeof(sbuf), "*REC %dp", tr.trackCount());
  } else if (tr.routeLoaded()) {
    canvas_.setTextColor(K_ROUTE, K_BG);
    snprintf(sbuf, sizeof(sbuf), "NAV %s", tr.routeName());
  } else {
    canvas_.setTextColor(K_GRAY, K_BG);
    snprintf(sbuf, sizeof(sbuf), "IDLE");
  }
  canvas_.drawString(sbuf, px, 2);
  if (noFixFlash_ && millis() - noFixMs_ < 2500) {
    canvas_.setTextColor(K_ORANGE, K_BG);
    canvas_.drawString(flashMsg_, px + 90, 2);
  }
  if (tr.routeLoaded() && revNav_) {
    canvas_.setTextDatum(textdatum_t::middle_right);
    canvas_.setTextColor(K_ORANGE, K_BG);
    canvas_.drawString("REV", 238, 7);
    canvas_.setTextDatum(textdatum_t::top_left);
  }

  // nearest waypoint + straight distance (wi/wdist computed above)
  canvas_.setTextColor(K_BLUE, K_BG);
  if (wi >= 0) {
    snprintf(sbuf, sizeof(sbuf), "v %s", tr.wpt(wi).name);
    canvas_.drawString(sbuf, px, 12);
    canvas_.setTextColor(K_WHITE, K_BG);
    snprintf(sbuf, sizeof(sbuf), "%s%4.1fkm", wdist >= 10000 ? ">" : " ",
             wdist >= 10000 ? 99.9f : wdist / 1000.0f);
    canvas_.drawString(sbuf, px + 66, 12);
  } else {
    canvas_.drawString("v no wpt (M key)", px, 12);
  }

  // speed big
  canvas_.setFont(&fonts::Orbitron_Light_32);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::middle_center);
  canvas_.setTextColor(K_WHITE, K_BG);
  snprintf(sbuf, sizeof(sbuf), "%d", (int)roundf(g.speed_kmh));
  canvas_.drawString(sbuf, 172, 42);
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextDatum(textdatum_t::top_left);
  canvas_.setTextColor(K_GRAY, K_BG);
  canvas_.drawString("km/h", 172, 60);
  if (tr.routeLoaded()) canvas_.drawString("F:EXIT", px, 62);  // nav hint

  // progress / deviation (nav mode, direction-aware)
  if (tr.routeLoaded()) {
    int n = tr.routeCount();
    float f = (n > 1 && nearestIdx_ >= 0) ? (float)nearestIdx_ / (n - 1) : 0.0f;
    float prog = revNav_ ? (1.0f - f) : f;   // fraction toward destination
    canvas_.drawRect(px, 72, 126, 6, K_DIM);
    canvas_.fillRect(px, 72, (int)(126 * prog), 6, K_ROUTE);
    canvas_.setTextDatum(textdatum_t::top_left);
    canvas_.setTextColor(K_WHITE, K_BG);
    snprintf(sbuf, sizeof(sbuf), "%3d%%", (int)(prog * 100));
    canvas_.drawString(sbuf, px, 82);
    if (nearestIdx_ >= 0) {
      canvas_.setTextColor(devM_ > 100 ? K_ORANGE : K_GRAY, K_BG);
      snprintf(sbuf, sizeof(sbuf), "DEV %4.0fm", devM_);
      canvas_.drawString(sbuf, px + 40, 82);
      canvas_.setTextColor(K_GRAY, K_BG);
      snprintf(sbuf, sizeof(sbuf), "LEFT %4.1fkm", remainKm_);
      canvas_.drawString(sbuf, px, 92);
      // arrival: close to the route end in the current driving direction
      TrackPoint endp = tr.routePoint(revNav_ ? 0 : tr.routeCount() - 1);
      if (g.valid && TrackRec::distM(g.lat, g.lon, endp.lat / 1e7f, endp.lon / 1e7f) < 50 &&
          (millis() / 500) & 1) {
        canvas_.setTextColor(K_ROUTE, K_BG);
        canvas_.drawString("*ARRIVED*", px + 84, 92);
      }
    }
  } else {
    canvas_.setTextColor(K_GRAY, K_BG);
    canvas_.drawString(browse ? "UP/DN:sel F:LD DEL:del"
                              : "UP/DN:route F:NAV", px, 72);
    canvas_.drawString("G:REC M:WPT X:LIST", px, 82);
    if (nFiles_ && sel_ >= 0) {
      char nm[20];
      strncpy(nm, files_[sel_], 19);
      nm[19] = '\0';
      canvas_.setTextColor(K_WHITE, K_BG);
      canvas_.drawString(nm, px, 92);
      if (browse) {
        canvas_.setTextColor(K_ORANGE, K_BG);
        snprintf(sbuf, sizeof(sbuf), "%4.1fkm %dp",
                 prev_[sel_].lenKm, prev_[sel_].n);
        canvas_.drawString(sbuf, px + 66, 92);
      }
    }
  }

  if (entering_) drawEntryOverlay();
  if (wpList_)   drawWptListOverlay(tr);
  canvas_.pushSprite(0, 0);
}

void MapPage::drawWptListOverlay(TrackRec& tr) {
  int cnt = tr.wptCount();
  canvas_.fillRect(30, 14, 180, 78, 0x0000);
  canvas_.drawRect(30, 14, 180, 78, K_WHITE);
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::top_left);
  canvas_.setTextColor(K_GRAY, K_BG);
  canvas_.drawString("WAYPOINTS  DEL=delete  X=close", 36, 18);
  // scrolling window of 6 rows
  int start = (selW_ > 4) ? selW_ - 4 : 0;
  for (int row = 0; row < 6 && start + row < cnt; row++) {
    int i = start + row;
    bool sel = (i == selW_);
    canvas_.setTextColor(sel ? K_ORANGE : K_WHITE, 0x0000);
    char line[28];
    snprintf(line, sizeof(line), "%c%s", sel ? '>' : ' ', tr.wpt(i).name);
    canvas_.drawString(line, 36, 30 + row * 10);
  }
}

void MapPage::drawEntryOverlay() {
  canvas_.fillRect(20, 24, 200, 44, 0x0000);
  canvas_.drawRect(20, 24, 200, 44, K_WHITE);
  canvas_.setFont(&fonts::Font0);
  canvas_.setTextSize(1);
  canvas_.setTextDatum(textdatum_t::top_left);
  canvas_.setTextColor(K_GRAY, K_BG);
  canvas_.drawString("WAYPOINT NAME (ENTER=save,<=cancel)", 26, 28);
  canvas_.setFont(&fonts::Font2);
  canvas_.setTextColor(K_WHITE, K_BG);
  char line[16];
  snprintf(line, sizeof(line), "%s_", entryBuf_);
  canvas_.drawString(line, 26, 42);
}
