#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>
#include <DFRobot_BMI160.h>
#include <DFRobot_GNSS.h>

// ==================== CONFIG ====================
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* mqtt_server = "7b4f38c681884b269217eb69667d5308.s1.eu.hivemq.cloud";

#define DEVICE_ID "HIKER_001"
#define SDA_PIN 33
#define SCL_PIN 35
#define SOS_PIN 4
#define BUZZER_PIN 1
#define LORA_RX 39
#define LORA_TX 37
#define LORA_M0 2
#define LORA_M1 3

// ==================== GLOBALS ====================
DFRobot_BMI160 bmi160;
DFRobot_GNSS_I2C gnss(&Wire, 0x20);
HardwareSerial LoRaSerial(1);

volatile uint32_t g_steps = 0;
volatile bool g_fallDetected = false;
volatile bool g_sosBtnPressed = false;
char g_activity[16] = "IDLE";

struct GPSData {
    float lat = -7.7956f;
    float lng = 110.3695f;
    bool valid = false;
} currentPos;

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
SemaphoreHandle_t commsMutex;

// ==================== HELPERS ====================
void setLoraMode(int mode) {
    if (mode == 0) {
        digitalWrite(LORA_M0, LOW);
        digitalWrite(LORA_M1, LOW);
    } else { 
        digitalWrite(LORA_M0, HIGH);
        digitalWrite(LORA_M1, HIGH);
    }
}

// ==================== TASKS ====================

void TaskIMU(void* pv) {
    bool inFF = false; uint32_t ffStart = 0;
    bool waitImpact = false; uint32_t impactDeadline = 0;
    bool stepHigh = false; uint32_t lastStepMs = 0;
    uint32_t windowStart = millis();
    uint32_t stepsInWindow = 0;

    for (;;) {
        int16_t accel[3];
        bmi160.getAccelData(accel);
        
        float ax = accel[0] / 16384.0f;
        float ay = accel[1] / 16384.0f;
        float az = accel[2] / 16384.0f;
        float totalG = sqrtf(ax*ax + ay*ay + az*az);
        uint32_t now = millis();

        // Fall Logic
        if (!inFF && !waitImpact && totalG < 0.5f) { inFF = true; ffStart = now; }
        else if (inFF && totalG >= 0.5f) {
            if ((now - ffStart) >= 60) { waitImpact = true; impactDeadline = now + 400; }
            inFF = false;
        } else if (waitImpact) {
            if (totalG > 2.2f) { g_fallDetected = true; waitImpact = false; }
            else if (now > impactDeadline) { waitImpact = false; }
        }

        if (!stepHigh && totalG > 1.25f) { 
            if ((now - lastStepMs) > 300) {
                g_steps++; stepsInWindow++; lastStepMs = now;
            }
            stepHigh = true;
        } else if (stepHigh && totalG < 0.95f) stepHigh = false;

        // Activity Monitor
        if ((now - windowStart) >= 10000) {
            uint32_t spm = stepsInWindow * 6;
            if (spm >= 100) strcpy(g_activity, "RUNNING");
            else if (spm >= 20) strcpy(g_activity, "WALKING");
            else strcpy(g_activity, "IDLE");
            stepsInWindow = 0; windowStart = now;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void TaskGNSS(void* pv) {
    for (;;) {
        sLonLat_t latData = gnss.getLat();
        sLonLat_t lonData = gnss.getLon();
        if (latData.latitude != 0) {
            currentPos.lat = (float)latData.latitude / 10000000.0f;
            currentPos.lng = (float)lonData.lonitude / 10000000.0f;
            currentPos.valid = true;
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void TaskComms(void* pv) {
    for (;;) {
        char payload[150];
        bool sendAlert = false;
        String status = "NORMAL";

        // Handle SOS Button & Fall Alert
        if (g_sosBtnPressed || g_fallDetected) {
            status = g_fallDetected ? "FALL" : "MANUAL";
            sendAlert = true;
            
            // Local Alarm
            for(int i=0; i<5; i++) {
                digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
                digitalWrite(BUZZER_PIN, LOW); vTaskDelay(pdMS_TO_TICKS(100));
            }
        }

        snprintf(payload, sizeof(payload), 
            "{\"id\":\"%s\",\"lat\":%.6f,\"lng\":%.6f,\"steps\":%lu,\"status\":\"%s\"}",
            DEVICE_ID, currentPos.lat, currentPos.lng, (unsigned long)g_steps, status.c_str());

        // Via LoRa
        xSemaphoreTake(commsMutex, portMAX_DELAY);
        LoRaSerial.println(payload);
        xSemaphoreGive(commsMutex);

        // Via MQTT
        if (WiFi.status() == WL_CONNECTED && mqtt.connected()) {
            xSemaphoreTake(commsMutex, portMAX_DELAY);
            mqtt.publish("hiker/data", payload);
            xSemaphoreGive(commsMutex);
        }

        g_fallDetected = false;
        g_sosBtnPressed = false;
        
        vTaskDelay(pdMS_TO_TICKS(sendAlert ? 5000 : 30000));
    }
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
    Wire.begin(SDA_PIN, SCL_PIN);
    
    pinMode(LORA_M0, OUTPUT); pinMode(LORA_M1, OUTPUT);
    pinMode(SOS_PIN, INPUT_PULLUP); pinMode(BUZZER_PIN, OUTPUT);
    setLoraMode(0);

    while (bmi160.I2cInit() != 0) { delay(1000); }
    
    gnss.begin();
    gnss.enablePower();

    secureClient.setInsecure();
    mqtt.setServer(mqtt_server, 8883);
    commsMutex = xSemaphoreCreateMutex();

    xTaskCreate(TaskIMU, "IMU", 4096, NULL, 3, NULL);
    xTaskCreate(TaskGNSS, "GPS", 4096, NULL, 2, NULL);
    xTaskCreate(TaskComms, "COM", 4096, NULL, 2, NULL);
}

void loop() {
    if (digitalRead(SOS_PIN) == LOW) {
        delay(50); 
        if (digitalRead(SOS_PIN) == LOW) g_sosBtnPressed = true;
    }
}