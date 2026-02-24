#pragma once
#include <Arduino.h>
#include <Preferences.h>
// FIX: Include common_types to get struct ShotProfile; do NOT redefine it here.
#include "common_types.h"

class ProfileStore {
public:
  bool begin();
  void end();
  uint8_t count() const { return _count; }
  bool load(uint8_t idx, ShotProfile& out);
  bool save(uint8_t idx, const ShotProfile& in);
  bool setActive(uint8_t idx);
  uint8_t getActive();

  // convenience: ensure defaults exist
  void ensureDefaults();

private:
  Preferences _prefs;
  uint8_t _count = MAX_PROFILES;
};