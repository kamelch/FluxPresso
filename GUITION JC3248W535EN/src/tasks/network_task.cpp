#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

#include "../diagnostics.h"
#include "common_types.h"
#include "project_config.h"

extern QueueHandle_t queue_to_net;

static bool wifi_connected = false;
static bool shelly_last_state = false;
static uint32_t last_wifi_check = 0;
static uint32_t last_shelly_cmd = 0;
static int wifi_reconnect_attempts = 0;

/**
 * Connect to WiFi with timeout
 */
static bool connect_wifi(uint32_t timeout_ms) {
    Serial.println("Connecting to WiFi...");
    Serial.print("SSID: ");
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeout_ms) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("RSSI: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        wifi_reconnect_attempts = 0;
        return true;
    } else {
        Serial.println("WiFi connection failed");
        wifi_reconnect_attempts++;
        return false;
    }
}

/**
 * Control Shelly smart plug via HTTP
 */
static bool control_shelly(bool turn_on) {
    if (!wifi_connected) {
        Serial.println("Shelly: WiFi not connected");
        return false;
    }

    // Rate limiting - don't spam the device
    if (millis() - last_shelly_cmd < 1000) {
        return true;  // Skip, too soon
    }
    last_shelly_cmd = millis();

    HTTPClient http;
    String url = String("http://") + SHELLY_IP + "/relay/0?turn=" + (turn_on ? "on" : "off");
    
    Serial.print("Shelly: ");
    Serial.println(turn_on ? "ON" : "OFF");
    
    http.begin(url);
    http.setTimeout(2000);  // 2 second timeout
    
    int httpCode = http.GET();
    bool success = false;
    
    if (httpCode == HTTP_CODE_OK) {
        Serial.println("Shelly command successful");
        success = true;
    } else {
        Serial.print("Shelly command failed, code: ");
        Serial.println(httpCode);
    }
    
    http.end();
    return success;
}

/**
 * Network task - handles WiFi and Shelly control
 */
void network_task(void *pvParameters) {
    Diag::log(LOG_INFO, "Network task started");
    
    // Initial WiFi connection attempt
    wifi_connected = connect_wifi(10000);  // 10 second timeout
    
    TickType_t lastWake = xTaskGetTickCount();
    
    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(NET_TASK_RATE_MS));
        
        uint32_t now = millis();
        
        // Check network command queue
        NetCommand net_cmd{};
        while (xQueueReceive(queue_to_net, &net_cmd, 0) == pdTRUE) {
            switch (net_cmd.type) {
                case NetCommand::NET_SET_SHELLY:
                    if (net_cmd.shelly_on != shelly_last_state) {
                        #if SHELLY_ENABLED
                        control_shelly(net_cmd.shelly_on);
                        #endif
                        shelly_last_state = net_cmd.shelly_on;
                    }
                    break;

                case NetCommand::NET_WIFI_RECONNECT:
                    // Force a reconnect attempt
                    WiFi.disconnect(true);
                    wifi_connected = connect_wifi(8000);
                    break;

                default:
                    break;
            }
        }
        
        // Periodic WiFi health check
        if (now - last_wifi_check > 10000) {  // Check every 10 seconds
            last_wifi_check = now;
            
            if (WiFi.status() == WL_CONNECTED) {
                if (!wifi_connected) {
                    wifi_connected = true;
                    Serial.println("WiFi reconnected");
                }
            } else {
                if (wifi_connected) {
                    wifi_connected = false;
                    Serial.println("WiFi connection lost");
                }
                
                // Attempt reconnection with exponential backoff
                if (wifi_reconnect_attempts < 5) {
                    connect_wifi(5000);
                } else if (wifi_reconnect_attempts < 10) {
                    // Less frequent attempts after multiple failures
                    if (now % 60000 < NET_TASK_RATE_MS) {  // Once per minute
                        connect_wifi(5000);
                    }
                }
            }
        }
    }
}
