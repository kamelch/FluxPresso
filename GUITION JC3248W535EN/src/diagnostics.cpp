#include "diagnostics.h"
#include <stdarg.h>
#include <stdio.h>

namespace {
    constexpr uint8_t kBufN = 12;
    DiagEvent g_buf[kBufN]{};
    uint8_t g_head = 0;
    uint8_t g_len = 0;

    void push(LogLevel lvl, FaultCode fc, const char* msg) {
        DiagEvent& e = g_buf[g_head];
        e.ms = millis();
        e.level = lvl;
        e.fault = fc;
        snprintf(e.msg, sizeof(e.msg), "%s", msg ? msg : "");

        g_head = (uint8_t)((g_head + 1) % kBufN);
        if (g_len < kBufN) g_len++;
    }

    const char* lvl_str(LogLevel l) {
        switch (l) {
            case LOG_INFO: return "INFO";
            case LOG_WARN: return "WARN";
            case LOG_ERROR: return "ERROR";
            case LOG_FAULT: return "FAULT";
            default: return "LOG";
        }
    }
}

namespace Diag {
    void init() {
        // no-op for now (kept for future expansion)
    }

    void log(LogLevel lvl, const char* msg) {
        push(lvl, FAULT_NONE, msg);
        Serial.printf("[%lu][%s] %s\n", (unsigned long)millis(), lvl_str(lvl), msg ? msg : "");
    }

    void logf(LogLevel lvl, const char* fmt, ...) {
        char buf[80];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        log(lvl, buf);
    }

    void fault(FaultCode code, const char* fmt, ...) {
        char buf[80];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof(buf), fmt, ap);
        va_end(ap);
        push(LOG_FAULT, code, buf);
        Serial.printf("[%lu][FAULT:%d] %s\n", (unsigned long)millis(), (int)code, buf);
    }

    uint8_t count() { return g_len; }

    const DiagEvent& at(uint8_t i) {
        // Oldest-first indexing
        if (i >= g_len) i = (uint8_t)(g_len ? g_len - 1 : 0);
        uint8_t start = (uint8_t)((g_head + kBufN - g_len) % kBufN);
        uint8_t idx = (uint8_t)((start + i) % kBufN);
        return g_buf[idx];
    }
}
