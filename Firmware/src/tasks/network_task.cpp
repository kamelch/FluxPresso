// NOTE: project_config.h defines ENABLE_ARDUINO_OTA. It must be included
// BEFORE any #if ENABLE_ARDUINO_OTA blocks, otherwise the OTA includes will
// be skipped while later OTA code compiles, resulting in missing symbols.
#include <Arduino.h>
#include "project_config.h"
#include "../hal.h"

#include <WiFi.h>
#include <HTTPClient.h>

#if ENABLE_ARDUINO_OTA
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>
#endif


#if ENABLE_OTA_UPDATES
  #include <WiFiClientSecure.h>
  #include <HTTPUpdate.h>
#endif

#if ENABLE_BLE_PROVISIONING
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
#endif

#include "../diagnostics.h"
#include "common_types.h"

extern QueueHandle_t queue_to_net;

namespace {
    bool wifi_connected = false;
#if ENABLE_ARDUINO_OTA
    bool ota_started = false;
#endif
    bool shelly_last_state = false;
    bool shelly_desired_state = false;  // what the core task wants
    bool shelly_desired_valid = false;  // true once we've received at least one command
    uint32_t shelly_retry_ms = 0;      // next retry time

    uint32_t next_wifi_attempt_ms = 0;
    uint32_t wifi_backoff_ms = 2000; // grows up to max

    uint32_t last_shelly_cmd_ms = 0;
    const uint32_t shelly_rate_limit_ms = 800;

    uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }


#if ENABLE_BLE_PROVISIONING
    // Minimal "Just Works" BLE WiFi provisioning:
    // - Write SSID and PASS characteristics
    // - Write APPLY=1 to store and connect
    // This is intentionally unauthenticated ("Just works") as requested.
    static std::string g_ble_ssid;
    static std::string g_ble_pass;

    static const char* PROV_SERVICE_UUID = "7a9d0001-7c3e-4b86-9d18-1e5f7e3f8d01";
    static const char* SSID_UUID        = "7a9d0002-7c3e-4b86-9d18-1e5f7e3f8d01";
    static const char* PASS_UUID        = "7a9d0003-7c3e-4b86-9d18-1e5f7e3f8d01";
    static const char* APPLY_UUID       = "7a9d0004-7c3e-4b86-9d18-1e5f7e3f8d01";
    static const char* STATUS_UUID      = "7a9d0005-7c3e-4b86-9d18-1e5f7e3f8d01";

    static BLECharacteristic* g_status_char = nullptr;

    class SSIDCallbacks : public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic* c) override { g_ble_ssid = c->getValue(); }
    };
    class PASSCallbacks : public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic* c) override { g_ble_pass = c->getValue(); }
    };
    class APPLYCallbacks : public BLECharacteristicCallbacks {
        void onWrite(BLECharacteristic* c) override {
            const std::string v = c->getValue();
            if (!v.empty() && v[0] == '1') {
                // Persist to NVS (Preferences) through Arduino WiFi libs: WiFi will store in flash internally,
                // but we also store explicitly using Preferences in main firmware if desired.
                Diag::log(LOG_INFO, "BLE Prov: apply received, reconnecting WiFi");
                WiFi.disconnect(true);
                delay(100);
                WiFi.begin(g_ble_ssid.c_str(), g_ble_pass.c_str());
                if (g_status_char) {
                    g_status_char->setValue("APPLY");
                    g_status_char->notify();
                }
            }
        }
    };

    static void start_ble_provisioning() {
        static bool started = false;
        if (started) return;
        started = true;

        char name[32];
        uint64_t mac = ESP.getEfuseMac();
        snprintf(name, sizeof(name), "Gaggia-%04X", (uint16_t)(mac & 0xFFFF));

        BLEDevice::init(name);
        BLEServer* server = BLEDevice::createServer();
        BLEService* service = server->createService(PROV_SERVICE_UUID);

        BLECharacteristic* ssid = service->createCharacteristic(SSID_UUID, BLECharacteristic::PROPERTY_WRITE);
        ssid->setCallbacks(new SSIDCallbacks());

        BLECharacteristic* pass = service->createCharacteristic(PASS_UUID, BLECharacteristic::PROPERTY_WRITE);
        pass->setCallbacks(new PASSCallbacks());

        BLECharacteristic* apply = service->createCharacteristic(APPLY_UUID, BLECharacteristic::PROPERTY_WRITE);
        apply->setCallbacks(new APPLYCallbacks());

        g_status_char = service->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
        g_status_char->addDescriptor(new BLE2902());
        g_status_char->setValue("READY");

        service->start();
        BLEAdvertising* adv = BLEDevice::getAdvertising();
        adv->addServiceUUID(PROV_SERVICE_UUID);
        adv->start();

        Diag::log(LOG_INFO, "BLE Prov: started");
    }
#endif

#if ENABLE_OTA_UPDATES
    // OTA via HTTPS with fingerprint pinning.
    // OTA is a no-op unless OTA_MANIFEST_URL and OTA_HTTPS_FINGERPRINT are configured.
    static bool ota_check_and_update() {
        const char* url = OTA_MANIFEST_URL;
        const char* fp  = OTA_HTTPS_FINGERPRINT;
        if (!url || !url[0] || !fp || !fp[0]) {
            Diag::log(LOG_WARN, "OTA: disabled (missing URL/fingerprint)");
            return false;
        }

        WiFiClientSecure client;
        client.setTimeout(12000);
        client.setFingerprint(fp);

        HTTPClient http;
        if (!http.begin(client, url)) {
            Diag::log(LOG_WARN, "OTA: begin failed");
            return false;
        }
        int code = http.GET();
        if (code != 200) {
            Diag::logf(LOG_WARN, "OTA: manifest GET failed (%d)", code);
            http.end();
            return false;
        }
        String manifest = http.getString();
        http.end();

        // Manifest format (simple): first line = firmware URL
        manifest.trim();
        if (manifest.length() < 10) {
            Diag::log(LOG_WARN, "OTA: manifest empty");
            return false;
        }

        Diag::log(LOG_INFO, "OTA: starting update");
        t_httpUpdate_return ret = httpUpdate.update(client, manifest);
        if (ret == HTTP_UPDATE_OK) {
            Diag::log(LOG_INFO, "OTA: OK (rebooting)");
            ESP.restart();
            return true;
        } else {
            Diag::logf(LOG_WARN, "OTA: failed (%d)", (int)ret);
            return false;
        }
    }
#endif
    bool connect_wifi_blocking(uint32_t timeout_ms) {
        // Never print password.
        Diag::logf(LOG_INFO, "WiFi: connecting to %s", WIFI_SSID);

        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        const uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
            delay(250);
        }

        if (WiFi.status() == WL_CONNECTED) {
            wifi_connected = true;
            wifi_backoff_ms = 2000;
            Diag::logf(LOG_INFO, "WiFi: connected (%s), RSSI=%ddBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
            return true;
        }

        wifi_connected = false;
        Diag::log(LOG_WARN, "WiFi: connect failed");
        return false;
    }

    String shelly_url_for(bool turn_on) {
        const char* on_str  = turn_on ? "true" : "false";
        const char* turn_str = turn_on ? "on" : "off";

        #if SHELLY_API_MODE == 1
            // Gen1 legacy REST
            return String("http://") + SHELLY_IP + "/relay/" + String(SHELLY_SWITCH_ID) + "?turn=" + turn_str;
        #else
            // Gen2/Gen3 RPC
            return String("http://") + SHELLY_IP + "/rpc/Switch.Set?id=" + String(SHELLY_SWITCH_ID) + "&on=" + on_str;
        #endif
    }

    bool control_shelly(bool turn_on) {
        if (!wifi_connected) {
            Diag::log(LOG_WARN, "Shelly: WiFi not connected");
            return false;
        }

        const uint32_t now = millis();
        if ((now - last_shelly_cmd_ms) < shelly_rate_limit_ms) {
            // Rate-limit (avoid spamming Shelly)
            return true;
        }
        last_shelly_cmd_ms = now;

        HTTPClient http;
        const String url = shelly_url_for(turn_on);

        http.begin(url);
        http.setTimeout(2000);

        const int code = http.GET();
        http.end();

        if (code == HTTP_CODE_OK) {
            Diag::logf(LOG_INFO, "Shelly: %s OK", turn_on ? "ON" : "OFF");
            return true;
        }

        Diag::logf(LOG_WARN, "Shelly: HTTP %d", code);
        return false;
    }
}

void network_task(void *pvParameters) {
    (void)pvParameters;
    Diag::log(LOG_INFO, "Network task started");

    // Initial attempt
    next_wifi_attempt_ms = millis();

    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(NET_TASK_RATE_MS));

        const uint32_t now = millis();

        // === Connectivity maintenance (with backoff) ===
        if (WiFi.status() == WL_CONNECTED) {
            if (!wifi_connected) {
                wifi_connected = true;
                wifi_backoff_ms = 2000;
                Diag::log(LOG_INFO, "WiFi: reconnected");

#if ENABLE_ARDUINO_OTA
                if (!ota_started) {
                    ArduinoOTA.setHostname(OTA_HOSTNAME);
                    if (strlen(OTA_PASSWORD) > 0) {
                        ArduinoOTA.setPassword(OTA_PASSWORD);
                    }

                    ArduinoOTA.onStart([]() {
                        Diag::log(LOG_WARN, "OTA: start (forcing outputs OFF)");
                        HAL_ForceOutputsSafeEarly();
                    });
                    ArduinoOTA.onEnd([]() {
                        Diag::log(LOG_WARN, "OTA: end");
                    });
                    ArduinoOTA.onError([](ota_error_t error) {
                        Diag::logf(LOG_ERROR, "OTA: error=%d", (int)error);
                    });

                    ArduinoOTA.begin();
                    ota_started = true;
                    Diag::logf(LOG_INFO, "OTA: ready at %s:3232 (hostname=%s)",
                        WiFi.localIP().toString().c_str(), OTA_HOSTNAME);
                }
#endif
            }
        } else {
            if (wifi_connected) {
                wifi_connected = false;
                Diag::log(LOG_WARN, "WiFi: disconnected");
            }

            if (now >= next_wifi_attempt_ms) {
                const bool ok = connect_wifi_blocking(5000);
                if (!ok) {
                    wifi_backoff_ms = clamp_u32(wifi_backoff_ms * 2, 2000, 300000); // max 5 min
                    next_wifi_attempt_ms = now + wifi_backoff_ms;
                } else {
                    next_wifi_attempt_ms = now + 15000; // next proactive check
                }
            }
        }

        // === Commands ===
        NetCommand net_cmd{};
        while (xQueueReceive(queue_to_net, &net_cmd, 0) == pdTRUE) {
            switch (net_cmd.type) {
                case NetCommand::NET_SET_SHELLY:
                    #if SHELLY_ENABLED
                    if (control_shelly(net_cmd.shelly_on)) {
                        shelly_last_state = net_cmd.shelly_on;
                    }
                    // Always update desired state even if HTTP failed (for retry below)
                    shelly_desired_state = net_cmd.shelly_on;
                    shelly_desired_valid = true;
                    #else
                    shelly_last_state = net_cmd.shelly_on;
                    #endif
                    break;

                case NetCommand::NET_WIFI_RECONNECT:
                    Diag::log(LOG_INFO, "WiFi: forced reconnect");
                    WiFi.disconnect(true);
                    wifi_connected = false;
                    wifi_backoff_ms = 2000;
                    next_wifi_attempt_ms = now;
                    break;

                case NetCommand::NET_BLE_PROVISION_START:
#if ENABLE_BLE_PROVISIONING
                    start_ble_provisioning();
#else
                    Diag::log(LOG_WARN, "BLE Prov: disabled");
#endif
                    break;

                case NetCommand::NET_OTA_CHECK_AND_UPDATE:
#if ENABLE_OTA_UPDATES
                    ota_check_and_update();
#else
                    Diag::log(LOG_WARN, "OTA: disabled");
#endif
                    break;

                default:
                    break;
            }
        }

#if ENABLE_ARDUINO_OTA
        if (ota_started && wifi_connected) {
            ArduinoOTA.handle();
        }
#endif

        // === Shelly periodic sync / retry ===
        // If the desired state doesn't match the actual state (e.g. WiFi was down
        // when the command was sent), keep retrying every 3 seconds.
        #if SHELLY_ENABLED
        if (shelly_desired_valid && wifi_connected && 
            shelly_desired_state != shelly_last_state &&
            now >= shelly_retry_ms) {
            Serial.printf("[SHELLY] Retry: desired=%s actual=%s\n",
                shelly_desired_state ? "ON" : "OFF",
                shelly_last_state ? "ON" : "OFF");
            if (control_shelly(shelly_desired_state)) {
                shelly_last_state = shelly_desired_state;
            }
            shelly_retry_ms = now + 3000; // retry every 3s
        }
        #endif
    }
}
