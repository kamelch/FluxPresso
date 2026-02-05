#pragma once
#include <Arduino.h>
#include "common_types.h"

// Standardized diagnostic logging.
// - Always logs to Serial
// - Keeps a small in-memory ring buffer (for UI/remote debugging later)

enum LogLevel : uint8_t { LOG_INFO=0, LOG_WARN=1, LOG_ERROR=2, LOG_FAULT=3 };

struct DiagEvent {
    uint32_t ms;
    LogLevel level;
    FaultCode fault;
    char msg[80];
};

namespace Diag {
    void init();
    void log(LogLevel lvl, const char* msg);
    void logf(LogLevel lvl, const char* fmt, ...);
    void fault(FaultCode code, const char* fmt, ...);

    // Access ring buffer (non-owning pointers)
    uint8_t count();
    const DiagEvent& at(uint8_t i);
}
