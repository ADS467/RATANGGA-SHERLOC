/*
  ================================================================
  RATANGGA SHER-LOC: PHYSICAL GROUND STATION
  Bridge: LoRa (E220) -> WiFi (MQTT HiveMQ)
  ================================================================
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ==================== CONFIG ====================
const char* wifi_ssid     = "YOUR_WIFI_SSID";
const char* wifi_password = "YOUR_WIFI_PASSWORD";

const char* mqtt_url      = "7b4f38c681884b269217eb69667d5308.s1.eu.hivemq.cloud";
const int   mqtt_port     = 8883;
const char* mqtt_user     = "Rheostat";
const char* mqtt_pass     = "Admin123";

#define PUB_PATH_SOS      "ground/sos"
#define PUB_PATH_TRACKING "ground/tracking"

#define LED_DATA_RX_PIN   5 
#define LED_SOS_ALARM_PIN 4
#define LORA_RX           39
#define LORA_TX           37

// ==================== OBJECTS ====================
WiFiClientSecure secure_wifi;
PubSubClient mqtt(secure_wifi);
HardwareSerial LoRaSerial(1); 
SemaphoreHandle_t mqttMutex;

// ==================== WIFI TASK ====================
void taskWiFi(void* pv) {
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

// ==================== MQTT RECONNECT ====================
void reconnect() {
    while (!mqtt.connected()) {
        if (mqtt.connect("GS_Physical_Node", mqtt_user, mqtt_pass)) {
            Serial.println("[MQTT] Connected to HiveMQ");
        } else {
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

// ==================== LORA BRIDGE TASK ====================
void taskLoRaListener(void* pv) {
    for (;;) {
        if (LoRaSerial.available()) {
            String incoming = LoRaSerial.readStringUntil('\n');
            incoming.trim();

            if (incoming.length() > 0) {
                Serial.printf("[LoRa RX] %s\n", incoming.c_str());
                
                digitalWrite(LED_DATA_RX_PIN, HIGH);
                
                const char* targetTopic = PUB_PATH_TRACKING;
                if (incoming.indexOf("FALL") >= 0 || incoming.indexOf("MANUAL") >= 0) {
                    targetTopic = PUB_PATH_SOS;
                    // Trigger Alarm LED
                    digitalWrite(LED_SOS_ALARM_PIN, HIGH);
                }

                if (mqtt.connected()) {
                    xSemaphoreTake(mqttMutex, portMAX_DELAY);
                    mqtt.publish(targetTopic, incoming.c_str());
                    xSemaphoreGive(mqttMutex);
                    Serial.println("[BRIDGE] Forwarded to Cloud");
                }
                
                vTaskDelay(pdMS_TO_TICKS(500));
                digitalWrite(LED_DATA_RX_PIN, LOW);
                digitalWrite(LED_SOS_ALARM_PIN, LOW);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
    
    pinMode(LED_DATA_RX_PIN, OUTPUT);
    pinMode(LED_SOS_ALARM_PIN, OUTPUT);

    secure_wifi.setInsecure();
    mqtt.setServer(mqtt_url, mqtt_port);
    mqttMutex = xSemaphoreCreateMutex();

    xTaskCreate(taskWiFi, "WiFi", 4096, NULL, 5, NULL);
    xTaskCreate(taskLoRaListener, "LoRa", 4096, NULL, 4, NULL);
}

void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!mqtt.connected()) reconnect();
        mqtt.loop();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}