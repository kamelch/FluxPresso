#pragma once

// Native/host stubs for a minimal subset of Arduino + ESP32 Preferences
// so we can unit-test logic on PlatformIO's `native` environment.

#ifndef ARDUINO

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>
#include <cstdio>
#include <algorithm>
#include <cmath>

using byte = std::uint8_t;
using String = std::string;

// Provide Arduino-style min/max templates used in some headers.
template <typename T>
static inline T min(T a, T b) { return std::min(a, b); }

template <typename T>
static inline T max(T a, T b) { return std::max(a, b); }

// Very small Serial stub (prints to stdout).
struct SerialClass {
    void println(const char* s) { if (s) std::printf("%s\n", s); }
    void print(const char* s) { if (s) std::printf("%s", s); }

    template <typename... Args>
    void printf(const char* fmt, Args... args) {
        if (!fmt) return;
        std::printf(fmt, args...);
    }
};

static inline SerialClass Serial;

// Minimal in-memory Preferences stub.
// Stores values in a per-namespace key-value map, backing blobs as raw bytes.
class Preferences {
public:
    bool begin(const char* ns, bool /*readOnly*/ = false) {
        namespace_ = ns ? ns : "";
        opened_ = true;
        return true;
    }

    void end() { opened_ = false; }

    void clear() {
        store()[namespace_].clear();
    }

    // Scalars
    uint8_t getUChar(const char* key, uint8_t defaultValue = 0) {
        uint8_t v = defaultValue;
        getBytes(key, &v, sizeof(v));
        return v;
    }

    bool putUChar(const char* key, uint8_t value) {
        return putBytes(key, &value, sizeof(value)) == sizeof(value);
    }

    float getFloat(const char* key, float defaultValue = 0.0f) {
        float v = defaultValue;
        getBytes(key, &v, sizeof(v));
        return v;
    }

    bool putFloat(const char* key, float value) {
        return putBytes(key, &value, sizeof(value)) == sizeof(value);
    }

    int32_t getInt(const char* key, int32_t defaultValue = 0) {
        int32_t v = defaultValue;
        getBytes(key, &v, sizeof(v));
        return v;
    }

    bool putInt(const char* key, int32_t value) {
        return putBytes(key, &value, sizeof(value)) == sizeof(value);
    }

    // Blobs
    size_t getBytesLength(const char* key) {
        if (!opened_) return 0;
        const std::string k = makeKey(key);
        auto& nsmap = store()[namespace_];
        auto it = nsmap.find(k);
        if (it == nsmap.end()) return 0;
        return it->second.size();
    }

    size_t getBytes(const char* key, void* out, size_t maxLen) {
        if (!opened_ || !out || maxLen == 0) return 0;
        const std::string k = makeKey(key);
        auto& nsmap = store()[namespace_];
        auto it = nsmap.find(k);
        if (it == nsmap.end()) return 0;
        const size_t n = std::min(maxLen, it->second.size());
        std::memcpy(out, it->second.data(), n);
        return n;
    }

    size_t putBytes(const char* key, const void* data, size_t len) {
        if (!opened_ || !key) return 0;
        const std::string k = makeKey(key);
        auto& nsmap = store()[namespace_];
        std::vector<uint8_t> buf(len);
        if (len && data) std::memcpy(buf.data(), data, len);
        nsmap[k] = std::move(buf);
        return len;
    }

private:
    bool opened_ = false;
    std::string namespace_;

    using StoreMap = std::unordered_map<std::string,
                       std::unordered_map<std::string, std::vector<uint8_t>>>;

    static StoreMap& store() {
        static StoreMap s;
        return s;
    }

    std::string makeKey(const char* key) const {
        return key ? std::string(key) : std::string();
    }
};

#endif // !ARDUINO
