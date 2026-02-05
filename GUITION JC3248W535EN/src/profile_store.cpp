
#include "profile_store.h"

static const char* NS = "gaggia";
static const char* KEY_ACTIVE = "p_active";
static const char* KEY_COUNT  = "p_count";

static String keyFor(uint8_t idx) {
  char buf[12];
  snprintf(buf, sizeof(buf), "p%u", (unsigned)idx);
  return String(buf);
}

bool ProfileStore::begin() {
  return _prefs.begin(NS, false);
}

void ProfileStore::end() {
  _prefs.end();
}

bool ProfileStore::load(uint8_t idx, ShotProfile& out) {
  if(idx >= _count) return false;
  size_t len = _prefs.getBytesLength(keyFor(idx).c_str());
  if(len != sizeof(ShotProfile)) return false;
  _prefs.getBytes(keyFor(idx).c_str(), &out, sizeof(ShotProfile));
  return true;
}

bool ProfileStore::save(uint8_t idx, const ShotProfile& in) {
  if(idx >= _count) return false;
  size_t w = _prefs.putBytes(keyFor(idx).c_str(), &in, sizeof(ShotProfile));
  return w == sizeof(ShotProfile);
}

bool ProfileStore::setActive(uint8_t idx) {
  if(idx >= _count) return false;
  _prefs.putUChar(KEY_ACTIVE, idx);
  return true;
}

uint8_t ProfileStore::getActive() {
  return _prefs.getUChar(KEY_ACTIVE, 0);
}

void ProfileStore::ensureDefaults() {
  // If profile 0 doesn't exist, seed all.
  ShotProfile p{};
  if(load(0, p)) return;

  // CLASSIC
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "CLASSIC", sizeof(p.name)-1);
  p.brew_temp_c = 93.0f;
  p.preinfuse_ms = 3000;
  p.preinfuse_pump = 0.45f;
  p.bloom_ms = 2000;
  p.bloom_pump = 0.0f;
  p.brew_ms = 25000;
  p.brew_pump = 0.75f;
  save(0, p);

  // SLAYER-ish
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "SLAYER", sizeof(p.name)-1);
  p.brew_temp_c = 93.0f;
  p.preinfuse_ms = 8000;
  p.preinfuse_pump = 0.30f;
  p.bloom_ms = 4000;
  p.bloom_pump = 0.0f;
  p.brew_ms = 22000;
  p.brew_pump = 0.85f;
  save(1, p);

  // BLOOM
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "BLOOM", sizeof(p.name)-1);
  p.brew_temp_c = 94.0f;
  p.preinfuse_ms = 5000;
  p.preinfuse_pump = 0.40f;
  p.bloom_ms = 6000;
  p.bloom_pump = 0.0f;
  p.brew_ms = 20000;
  p.brew_pump = 0.90f;
  save(2, p);

  // FAST
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "FAST", sizeof(p.name)-1);
  p.brew_temp_c = 92.0f;
  p.preinfuse_ms = 1500;
  p.preinfuse_pump = 0.60f;
  p.bloom_ms = 0;
  p.bloom_pump = 0.0f;
  p.brew_ms = 18000;
  p.brew_pump = 1.0f;
  save(3, p);

  // GENTLE
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "GENTLE", sizeof(p.name)-1);
  p.brew_temp_c = 93.0f;
  p.preinfuse_ms = 6000;
  p.preinfuse_pump = 0.25f;
  p.bloom_ms = 3000;
  p.bloom_pump = 0.0f;
  p.brew_ms = 26000;
  p.brew_pump = 0.65f;
  save(4, p);

  // USER
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "USER", sizeof(p.name)-1);
  p.brew_temp_c = 93.0f;
  p.preinfuse_ms = 4000;
  p.preinfuse_pump = 0.40f;
  p.bloom_ms = 2000;
  p.bloom_pump = 0.0f;
  p.brew_ms = 24000;
  p.brew_pump = 0.80f;
  save(5, p);

  _prefs.putUChar(KEY_ACTIVE, 0);
  _prefs.putUChar(KEY_COUNT, _count);
}
