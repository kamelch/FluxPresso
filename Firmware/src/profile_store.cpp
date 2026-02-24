
#include "profile_store.h"

static const char* NS = "gaggia";
static const char* KEY_ACTIVE = "p_active";
static const char* KEY_COUNT  = "p_count";



// Legacy profile layouts (for migration from older firmware)
struct ShotProfileV1 {
  char name[16];
  float brew_temp_c;

  uint32_t preinfuse_ms;
  float preinfuse_pump;

  uint32_t bloom_ms;
  float bloom_pump;

  uint32_t brew_ms;
  float brew_pump;

  bool enable_pressure_profile;
  float pressure_target_bar;

  bool enable_flow_profile;
  float flow_target_ml_s;
};

static void migrateV1ToCurrent(const ShotProfileV1& v1, ShotProfile& out) {
  memset(&out, 0, sizeof(out));
  strncpy(out.name, v1.name, sizeof(out.name)-1);
  out.brew_temp_c = v1.brew_temp_c;

  out.preinfuse_ms = v1.preinfuse_ms;
  out.preinfuse_pump = v1.preinfuse_pump;

  out.bloom_ms = v1.bloom_ms;
  out.bloom_pump = v1.bloom_pump;

  out.brew_ms = v1.brew_ms;
  out.brew_pump = v1.brew_pump;

  // Newer fields: preserve legacy behavior (time-based stop) unless user enables target-yield.
  out.target_yield_g = 0.0f;
  out.stop_early_g = 1.5f;
  out.stop_lag_ms = 400;

  // Flow shaping + temp offset defaults
  out.enable_flow_shaping = false;  // keep simple on migration
  out.flow_ramp_up_ms = 2500;
  out.flow_ramp_down_ms = 2500;
  out.flow_end_pump = 0.55f;
  out.flow_temp_offset_c = 2.0f;

  // Advanced fields
  out.enable_pressure_profile = v1.enable_pressure_profile;
  out.pressure_target_bar = v1.pressure_target_bar;
  out.enable_flow_profile = v1.enable_flow_profile;
  out.flow_target_ml_s = v1.flow_target_ml_s;
}
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

  const char* key = keyFor(idx).c_str();
  size_t len = _prefs.getBytesLength(key);
  if(len == 0) return false;

  if(len == sizeof(ShotProfile)) {
    _prefs.getBytes(key, &out, sizeof(ShotProfile));
    return true;
  }

  // Migration: older firmware stored smaller structs (no target-yield, shaping, etc.)
  if(len == sizeof(ShotProfileV1)) {
    ShotProfileV1 v1{};
    _prefs.getBytes(key, &v1, sizeof(v1));
    migrateV1ToCurrent(v1, out);

    // Upgrade-in-place so next boot is fast and user settings are preserved.
    _prefs.putBytes(key, &out, sizeof(out));
    return true;
  }

  // Unknown size: refuse to load (caller will seed defaults)
  return false;
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
  // Profile format version — bump this to force re-seeding when defaults change.
  // v3 = single "SLAYER" profile only (Slayer-style pre-brew + full-brew)
  static constexpr uint8_t PROFILE_VERSION = 3;
  const uint8_t saved_ver = _prefs.getUChar("p_ver", 0);

  // If version matches AND profile 0 exists, keep user's settings
  if (saved_ver == PROFILE_VERSION) {
    ShotProfile p{};
    if (load(0, p)) {
      // Small in-place tweak: earlier defaults used 38g; for 18g dose @ 1:2 we want 36g.
      // Only auto-adjust if user never changed it (still exactly the old default).
      if (fabsf(p.target_yield_g - 38.0f) < 0.01f) {
        p.target_yield_g = 36.0f;
        save(0, p);
        Serial.println("[PROFILES] Updated default target yield 38g -> 36g (18g dose @ 1:2)");
      }
      
      // New default behavior: weight-stop to 36g + flow targeting at 1.2 g/s (≈1.2 ml/s).
      // Only auto-adjust if it still looks like an untouched old default.
      bool changed = false;
      if (p.target_yield_g <= 5.0f) {
        p.target_yield_g = 36.0f;
        changed = true;
      }
      if (fabsf(p.flow_target_ml_s - 2.0f) < 0.01f) {
        p.flow_target_ml_s = 1.2f;
        changed = true;
      }
      if (!p.enable_flow_profile) {
        p.enable_flow_profile = true;
        changed = true;
      }
      if (changed) {
        save(0, p);
        Serial.println("[PROFILES] Updated profile 0 for flow-target (1.2 ml/s) + weight-stop (36g)");
      }

      return;
    }
  }
Serial.printf("[PROFILES] Re-seeding defaults (saved_ver=%u, current=%u)\n", saved_ver, PROFILE_VERSION);

  ShotProfile p{};
  memset(&p, 0, sizeof(p));
  strncpy(p.name, "FLOW36", sizeof(p.name)-1);

  // Baseline from Slayer's own dial-in guidance:
  // - Brew temp around 93°C
  // - Pre-brew is restricted FLOW (not pressure), typically calibrated ~40–60g of clear water in 30s.
  // Here we approximate restricted flow by reducing pump power (triac dimmer).
  p.brew_temp_c = 93.0f;

  // Shot time target: cap the whole extraction at ~30s (preinfuse + brew).
  // Note: actual shot time to reach 36g still depends mostly on grind & puck prep.
  p.preinfuse_ms = 0;
  p.preinfuse_pump = 0.50f;   // 60% pre-brew ("60/100/60" style); still calibrate to your pump + OPV

  // No separate bloom/soak stage in this Slayer emulation.
  p.bloom_ms = 0;
  p.bloom_pump = 0.0f;

  // Full-brew time cap (weight-based stop preferred)
  p.brew_ms = 30000;
  p.brew_pump = 1.0f;

  // Default target yield (adjust to taste)
  p.target_yield_g = 36.0f; // 18g in @ 1:2
  p.stop_early_g = 1.5f;
  p.stop_lag_ms = 400;

  // Keep shaping off for stability; "decline" is handled in core_task using the pre-brew pump value.
  p.enable_flow_shaping = false;
  p.flow_ramp_up_ms = 2500;
  p.flow_ramp_down_ms = 2500;
  p.flow_end_pump = 0.75f;

  // Temperature headroom during pump-on
  p.flow_temp_offset_c = 2.0f;

  // Advanced profiling disabled by default
  p.enable_pressure_profile = false;
  p.pressure_target_bar = 9.0f;
  p.enable_flow_profile = true;
  p.flow_target_ml_s = 1.2f;

  save(0, p);

  _prefs.putUChar(KEY_ACTIVE, 0);
  _prefs.putUChar(KEY_COUNT, _count);
  _prefs.putUChar("p_ver", PROFILE_VERSION);
}

