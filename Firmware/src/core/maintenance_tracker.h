
#pragma once
#include "../utils/compat.h"

struct MaintenanceDue {
    bool backflush = false;
    bool descale = false;
};

class MaintenanceTracker {
public:
    struct Config {
        uint32_t backflush_interval_shots = 50;
        uint32_t descale_interval_shots = 300;
    };

    MaintenanceTracker() : cfg_() {}
    explicit MaintenanceTracker(const Config& cfg) : cfg_(cfg) {}

    void begin(); // loads from NVS if available
    void on_shot_complete(float yield_g);
    void mark_backflush_done();
    void mark_descale_done();

    uint32_t total_shots() const { return total_shots_; }
    uint32_t shots_since_backflush() const { return total_shots_ - last_backflush_shots_; }
    uint32_t shots_since_descale() const { return total_shots_ - last_descale_shots_; }
    MaintenanceDue due() const;

    void periodic_commit(uint32_t now_ms);

private:
    Config cfg_;
    uint32_t total_shots_ = 0;
    uint32_t last_backflush_shots_ = 0;
    uint32_t last_descale_shots_ = 0;

    uint32_t dirty_since_ms_ = 0;
    bool dirty_ = false;

    void load_();
    void save_();
};
