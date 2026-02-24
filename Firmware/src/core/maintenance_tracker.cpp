
#include "maintenance_tracker.h"

#ifdef ARDUINO
  #include <Preferences.h>
  static Preferences g_prefs;
#endif

void MaintenanceTracker::begin() { load_(); }

void MaintenanceTracker::load_() {
#ifdef ARDUINO
    g_prefs.begin("maint", true);
    total_shots_ = g_prefs.getUInt("total_shots", 0);
    last_backflush_shots_ = g_prefs.getUInt("bf_last", 0);
    last_descale_shots_ = g_prefs.getUInt("ds_last", 0);
    g_prefs.end();
#endif
}

void MaintenanceTracker::save_() {
#ifdef ARDUINO
    g_prefs.begin("maint", false);
    g_prefs.putUInt("total_shots", total_shots_);
    g_prefs.putUInt("bf_last", last_backflush_shots_);
    g_prefs.putUInt("ds_last", last_descale_shots_);
    g_prefs.end();
#endif
    dirty_ = false;
    dirty_since_ms_ = 0;
}

MaintenanceDue MaintenanceTracker::due() const {
    MaintenanceDue d;
    d.backflush = (cfg_.backflush_interval_shots > 0) && (shots_since_backflush() >= cfg_.backflush_interval_shots);
    d.descale = (cfg_.descale_interval_shots > 0) && (shots_since_descale() >= cfg_.descale_interval_shots);
    return d;
}

void MaintenanceTracker::on_shot_complete(float /*yield_g*/) {
    total_shots_++;
    dirty_ = true;
#ifdef ARDUINO
    dirty_since_ms_ = millis();
#endif
}

void MaintenanceTracker::mark_backflush_done() {
    last_backflush_shots_ = total_shots_;
    dirty_ = true;
#ifdef ARDUINO
    dirty_since_ms_ = millis();
#endif
}

void MaintenanceTracker::mark_descale_done() {
    last_descale_shots_ = total_shots_;
    dirty_ = true;
#ifdef ARDUINO
    dirty_since_ms_ = millis();
#endif
}

void MaintenanceTracker::periodic_commit(uint32_t now_ms) {
    if (!dirty_) return;
    if (dirty_since_ms_ == 0) dirty_since_ms_ = now_ms;
    // Commit after 10s to reduce flash wear
    if ((now_ms - dirty_since_ms_) >= 10000) {
        save_();
    }
}
