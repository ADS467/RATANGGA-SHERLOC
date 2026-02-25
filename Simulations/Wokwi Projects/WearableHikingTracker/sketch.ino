#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "7b4f38c681884b269217eb69667d5308.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "Rheostat";
const char* mqtt_pass = "Admin123";

#define DEVICE_ID "hiker_001"
#define SDA_PIN 8
#define SCL_PIN 9
#define SOS_PIN 4
#define MPU_INT_PIN 6 
#define LORA_M0 2
#define LORA_M1 3
#define BUZZER_PIN 1

#define MPU_ADDR 0x68
#define MPU_PWR_MGMT1 0x6B
#define MPU_ACCEL_OUT 0x3B

char TOPIC_TRACKING[64];
char TOPIC_SOS[64];

volatile float g_ax = 0, g_ay = 0, g_az = 0;
volatile uint32_t g_steps = 0;
volatile bool g_fallDetected = false;
volatile bool g_sosBtnPressed = false;
char g_activity[16] = "IDLE";

float base_lat = -7.7956f;
float base_lng = 110.3695f;

WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
SemaphoreHandle_t mqttMutex;

void IRAM_ATTR onMpuInterrupt() {}

void setLoraMode(int mode) {
  if (mode == 0) {
    digitalWrite(LORA_M0, LOW);
    digitalWrite(LORA_M1, LOW);
  } else {
    digitalWrite(LORA_M0, HIGH);
    digitalWrite(LORA_M1, HIGH);
  }
  vTaskDelay(pdMS_TO_TICKS(20)); 
}

void TaskWiFi(void* pv) {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  vTaskDelete(NULL);
}

void mqttReconnect() {
  while (!mqtt.connected()) {
    if (mqtt.connect(DEVICE_ID, mqtt_user, mqtt_pass)) break;
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void TaskMQTT(void* pv) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!mqtt.connected()) mqttReconnect();
      mqtt.loop();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT1);
  Wire.write(0x00);
  Wire.endTransmission();
  pinMode(MPU_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), onMpuInterrupt, RISING);
}

bool mpuRead(float& ax, float& ay, float& az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() < 6) return false;
  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  ax = raw_ax / 16384.0f;
  ay = raw_ay / 16384.0f;
  az = raw_az / 16384.0f;
  return true;
}

void TaskIMU(void* pv) {
  bool inFreeFall = false;
  uint32_t ffStart = 0;
  bool waitingImpact = false;
  uint32_t impactDeadline = 0;
  bool stepHigh = false;
  uint32_t lastStepMs = 0;
  uint32_t windowStart = millis();
  uint32_t stepsInWindow = 0;

  for (;;) {
    float ax, ay, az;
    if (mpuRead(ax, ay, az)) {
      float totalG = sqrtf(ax * ax + ay * ay + az * az);
      uint32_t now = millis();
      if (!inFreeFall && !waitingImpact && totalG < 0.5f) {
        inFreeFall = true;
        ffStart = now;
      } else if (inFreeFall && totalG >= 0.5f) {
        if ((now - ffStart) >= 60) {
          waitingImpact = true;
          impactDeadline = now + 400;
        }
        inFreeFall = false;
      } else if (waitingImpact) {
        if (totalG > 2.2f) {
          g_fallDetected = true;
          waitingImpact = false;
        } else if (now > impactDeadline) {
          waitingImpact = false;
        }
      }
      if (!stepHigh && totalG > 1.12f) {
        if ((now - lastStepMs) > 250) {
          g_steps++;
          stepsInWindow++;
          lastStepMs = now;
        }
        stepHigh = true;
      } else if (stepHigh && totalG < 0.88f) stepHigh = false;
      if ((now - windowStart) >= 10000) {
        uint32_t spm = stepsInWindow * 6;
        if (spm >= 100) strncpy(g_activity, "RUNNING", 16);
        else if (spm >= 30) strncpy(g_activity, "WALKING", 16);
        else strncpy(g_activity, "IDLE", 16);
        stepsInWindow = 0;
        windowStart = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void TaskSOS(void* pv) {
  pinMode(SOS_PIN, INPUT);
  bool lastState = LOW;
  uint32_t pressStart = 0;
  for (;;) {
    bool state = digitalRead(SOS_PIN);
    if (state == HIGH && lastState == LOW) pressStart = millis();
    if (state == LOW && lastState == HIGH) {
      if ((millis() - pressStart) > 100) g_sosBtnPressed = true;
    }
    lastState = state;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void TaskTracking(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(8000));
  for (;;) {
    if (mqtt.connected()) {
      base_lat += 0.00009f;
      base_lng += (sin(millis() / 5000.0f) * 0.00003f);
      char payload[128];
      snprintf(payload, sizeof(payload), "{\"lat\":%.6f,\"lng\":%.6f,\"steps\":%lu,\"act\":\"%s\"}",
               base_lat, base_lng, (unsigned long)g_steps, g_activity);
      setLoraMode(0);
      xSemaphoreTake(mqttMutex, portMAX_DELAY);
      mqtt.publish(TOPIC_TRACKING, payload);
      xSemaphoreGive(mqttMutex);
      setLoraMode(3);
    }
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

void TaskSOSAlert(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(8000));
  for (;;) {
    if (g_fallDetected) {
      g_fallDetected = false;
      bool cancelled = false;
      uint32_t countdownStart = millis();
      
      while (millis() - countdownStart < 15000) {
        digitalWrite(BUZZER_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(BUZZER_PIN, LOW);
        vTaskDelay(pdMS_TO_TICKS(400));
        
        if (g_sosBtnPressed) {
          g_sosBtnPressed = false;
          cancelled = true;
          break;
        }
      }
      
      if (!cancelled && mqtt.connected()) {
        char payload[128];
        snprintf(payload, sizeof(payload), "{\"status\":\"fall\",\"lat\":%.6f,\"lng\":%.6f}", base_lat, base_lng);
        setLoraMode(0);
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        mqtt.publish(TOPIC_SOS, payload);
        xSemaphoreGive(mqttMutex);
        setLoraMode(3);
      }
    }

    if (g_sosBtnPressed) {
      g_sosBtnPressed = false;
      if (mqtt.connected()) {
        char payload[128];
        snprintf(payload, sizeof(payload), "{\"status\":\"manual\",\"lat\":%.6f,\"lng\":%.6f}", base_lat, base_lng);
        setLoraMode(0);
        xSemaphoreTake(mqttMutex, portMAX_DELAY);
        mqtt.publish(TOPIC_SOS, payload);
        xSemaphoreGive(mqttMutex);
        setLoraMode(3);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  setLoraMode(3);
  snprintf(TOPIC_TRACKING, 64, "tracking/%s", DEVICE_ID);
  snprintf(TOPIC_SOS, 64, "sos/%s", DEVICE_ID);
  secureClient.setInsecure();
  mqtt.setServer(mqtt_server, mqtt_port);
  mpuInit();
  mqttMutex = xSemaphoreCreateMutex();
  xTaskCreate(TaskWiFi, "WiFi", 4096, NULL, 5, NULL);
  xTaskCreate(TaskMQTT, "MQTT", 4096, NULL, 4, NULL);
  xTaskCreate(TaskIMU, "IMU", 4096, NULL, 3, NULL);
  xTaskCreate(TaskSOS, "SOS", 2048, NULL, 3, NULL);
  xTaskCreate(TaskTracking, "Track", 4096, NULL, 2, NULL);
  xTaskCreate(TaskSOSAlert, "SOSAlert", 3072, NULL, 3, NULL);
}

void loop() {}