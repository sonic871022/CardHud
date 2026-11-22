#pragma once
#include <M5Unified.h>
#include "gnss.h"
#include "track.h"

// Keyboard events gathered by main.cpp each frame.
struct KbEvt {
  bool left = false, right = false, up = false, down = false;
  bool enter = false, backspace = false;
  char word[16];
  uint8_t nword = 0;
};

// Downsampled copy of one stored route, used for map preview.
struct RoutePreview {
  TrackPoint* pts = nullptr;  // heap, decimated (cap 512 pts)
  int  n = 0;
  float minLat = 0, maxLat = 0, minLon = 0, maxLon = 0;  // bbox
  float lenKm = 0;                                            // approx length
};

class MapPage {
 public:
  void begin();                       // create sprite
  void onEnter(TrackRec& tr);         // called when page becomes active
  void update(const GnssData& g, const KbEvt& kb, TrackRec& tr);
  bool isEntering() const { return entering_; }

 private:
  void refreshFiles(TrackRec& tr);
  void loadPreviews();                // read all .trk files, decimate, cache
  void freePreviews();
  void draw(const GnssData& g, TrackRec& tr);
  void drawEntryOverlay();
  void drawWptListOverlay(TrackRec& tr);

  M5Canvas canvas_{&M5.Lcd};

  // route files on SD
  static constexpr int MAX_FILES = 10;
  char files_[MAX_FILES][40];
  int  nFiles_ = 0;
  int  sel_ = -1;

  // preview cache of all stored routes (browse mode)
  RoutePreview prev_[MAX_FILES];

  // waypoint name entry
  bool entering_ = false;
  char entryBuf_[13];
  int  entryLen_ = 0;

  // waypoint list manager (X key)
  bool wpList_ = false;
  int  selW_ = 0;
  bool noFixFlash_ = false;
  uint32_t noFixMs_ = 0;
  char flashMsg_[12] = "NO FIX";   // transient warning text

  // route delete (DEL key, 2-step confirm)
  bool delArm_ = false;
  uint32_t delMs_ = 0;

  // nav match cache
  int   nearestIdx_ = -1;
  float devM_ = 0, remainKm_ = 0;
  uint32_t lastMatch_ = 0;
  bool revNav_ = false;   // driving opposite to the recorded route direction
};
