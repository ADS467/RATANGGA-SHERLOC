/*
  ================================================================
  CUBESAT PAYLOAD NODE - ESP32-C3
  Wokwi Simulator

  Hardware:
    - MPU6050  : SDA=8, SCL=9
    - SOS BTN  : GPIO4 (pull-up via 3V3)

  MQTT Topics Published:
    cubesat/tracking  -> {id, lat, lng, alt, steps, activity}
    cubesat/sos       -> {id, lat, lng, alt, status}
    cubesat/imu       -> {id, ax, ay, az, roll, pitch, yaw}

  Interval:
    tracking : 60 detik
    imu      : 5 detik
    sos      : real-time (fall / manual)
  ================================================================
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <math.h>

// ==================== WIFI ====================
const char* ssid     = "Wokwi-GUEST";
const char* password = "";

// ==================== MQTT ====================
// please put your broker credentials here, in order to set up refer to the README
const char* mqtt_server = "7b4f38c681884b269217eb69667d5308.s1.eu.hivemq.cloud";
const int   mqtt_port   = 8883;
const char* mqtt_user   = "Rheostat";
const char* mqtt_pass   = "Admin123";
const char* clientID    = "cubesat_payload_01";

#define DEVICE_ID "SAT-01"

// ==================== TOPICS ====================
#define TOPIC_TRACKING "cubesat/tracking"
#define TOPIC_SOS      "cubesat/sos"
#define TOPIC_IMU      "cubesat/imu"

// ==================== PINS ====================
#define SDA_PIN  8
#define SCL_PIN  9
#define SOS_PIN  4

// ==================== MPU6050 ====================
#define MPU_ADDR      0x68
#define MPU_PWR_MGMT1 0x6B
#define MPU_ACCEL_OUT 0x3B
#define MPU_GYRO_OUT  0x43

// ==================== FALL DETECTION ====================
#define FALL_FF_THRESHOLD  0.5f
#define FALL_FF_MIN_MS     60
#define FALL_IMPACT_THR    2.2f
#define FALL_IMPACT_WIN_MS 400

// ==================== STEP DETECTION ====================
#define STEP_HIGH 1.12f
#define STEP_LOW  0.88f
#define STEP_MIN_INTERVAL_MS 250

// ==================== SHARED STATE ====================
volatile float g_ax = 0, g_ay = 0, g_az = 0;
volatile float g_gx = 0, g_gy = 0, g_gz = 0;
volatile float g_roll = 0, g_pitch = 0;
volatile float g_totalG = 1.0f;
volatile uint32_t g_steps = 0;
volatile bool g_fallDetected  = false;
volatile bool g_sosBtnPressed = false;
char g_activity[16] = "IDLE";

// Dummy orbit position (LEO simulasi)
float sat_lat = -7.7956f;
float sat_lng = 110.3695f;
float sat_alt = 400.0f; // km (LEO)

// ==================== MQTT ====================
WiFiClientSecure secureClient;
PubSubClient mqtt(secureClient);
SemaphoreHandle_t mqttMutex;

// ==================== WIFI ====================
void TaskWiFi(void* pv) {
  Serial.println("[WIFI] Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  vTaskDelete(NULL);
}

// ==================== MQTT ====================
void mqttReconnect() {
  while (!mqtt.connected()) {
    Serial.print("[MQTT] Connecting...");
    if (mqtt.connect(clientID, mqtt_user, mqtt_pass)) {
      Serial.println(" OK");
    } else {
      Serial.printf(" FAILED rc=%d\n", mqtt.state());
      vTaskDelay(pdMS_TO_TICKS(3000));
    }
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

bool safePublish(const char* topic, const char* payload) {
  if (!mqtt.connected()) return false;
  xSemaphoreTake(mqttMutex, portMAX_DELAY);
  bool ok = mqtt.publish(topic, payload);
  xSemaphoreGive(mqttMutex);
  Serial.printf("[PUB] %s -> %s\n", topic, ok ? "OK" : "FAIL");
  return ok;
}

// ==================== MPU6050 ====================
void mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(150);
  Serial.println("[IMU] MPU6050 ready");
}

bool mpuRead(float& ax, float& ay, float& az,
             float& gx, float& gy, float& gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) return false;

  int16_t rax = (Wire.read()<<8)|Wire.read();
  int16_t ray = (Wire.read()<<8)|Wire.read();
  int16_t raz = (Wire.read()<<8)|Wire.read();
  Wire.read(); Wire.read(); // temp skip
  int16_t rgx = (Wire.read()<<8)|Wire.read();
  int16_t rgy = (Wire.read()<<8)|Wire.read();
  int16_t rgz = (Wire.read()<<8)|Wire.read();

  ax = rax / 16384.0f;
  ay = ray / 16384.0f;
  az = raz / 16384.0f;
  gx = rgx / 131.0f;
  gy = rgy / 131.0f;
  gz = rgz / 131.0f;
  return true;
}

// ==================== IMU TASK ====================
void TaskIMU(void* pv) {
  bool inFF = false;
  uint32_t ffStart = 0;
  bool waitImpact = false;
  uint32_t impactDeadline = 0;

  bool stepHigh = false;
  uint32_t lastStepMs = 0;

  uint32_t windowStart = millis();
  uint32_t stepsInWindow = 0;

  for (;;) {
    float ax, ay, az, gx, gy, gz;
    if (!mpuRead(ax, ay, az, gx, gy, gz)) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    float totalG = sqrtf(ax*ax + ay*ay + az*az);
    float roll   = atan2f(ay, az) * 180.0f / M_PI;
    float pitch  = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;

    g_ax = ax; g_ay = ay; g_az = az;
    g_gx = gx; g_gy = gy; g_gz = gz;
    g_roll = roll; g_pitch = pitch;
    g_totalG = totalG;

    uint32_t now = millis();

    // Fall detection
    if (!inFF && !waitImpact) {
      if (totalG < FALL_FF_THRESHOLD) { inFF = true; ffStart = now; }
    } else if (inFF) {
      if (totalG >= FALL_FF_THRESHOLD) {
        if ((now - ffStart) >= FALL_FF_MIN_MS) {
          waitImpact = true;
          impactDeadline = now + FALL_IMPACT_WIN_MS;
        }
        inFF = false;
      }
    } else if (waitImpact) {
      if (totalG > FALL_IMPACT_THR) {
        g_fallDetected = true;
        waitImpact = false;
      } else if (now > impactDeadline) {
        waitImpact = false;
      }
    }

    // Step count
    if (!stepHigh && totalG > STEP_HIGH) {
      if ((now - lastStepMs) > STEP_MIN_INTERVAL_MS) {
        g_steps++;
        stepsInWindow++;
        lastStepMs = now;
      }
      stepHigh = true;
    } else if (stepHigh && totalG < STEP_LOW) {
      stepHigh = false;
    }

    // Activity
    if ((now - windowStart) >= 10000) {
      uint32_t spm = stepsInWindow * 6;
      if (spm >= 100)     strncpy(g_activity, "RUNNING", 16);
      else if (spm >= 30) strncpy(g_activity, "WALKING", 16);
      else                strncpy(g_activity, "IDLE",    16);
      stepsInWindow = 0;
      windowStart = now;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ==================== SOS BUTTON TASK ====================
void TaskSOSBtn(void* pv) {
  pinMode(SOS_PIN, INPUT);
  bool last = LOW;
  uint32_t pressStart = 0;
  for (;;) {
    bool state = digitalRead(SOS_PIN);
    if (state == HIGH && last == LOW) pressStart = millis();
    if (state == LOW && last == HIGH && (millis()-pressStart) > 100) {
      g_sosBtnPressed = true;
      Serial.println("[SOS] Manual button pressed!");
    }
    last = state;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ==================== ORBIT SIMULATION ====================
void updateOrbit() {
  // Simulasi orbit LEO: maju ~0.1 derajat/menit ke timur
  sat_lng += 0.1f;
  if (sat_lng > 180.0f) sat_lng = -180.0f;
  // slight lat oscillation
  sat_lat = -7.7956f + sinf(millis() / 30000.0f) * 51.6f; // ISS-like inclination
  // alt varies slightly
  sat_alt = 400.0f + sinf(millis() / 10000.0f) * 5.0f;
}

// ==================== TRACKING TASK ====================
void TaskTracking(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(8000));
  for (;;) {
    if (mqtt.connected()) {
      updateOrbit();
      char payload[256];
      snprintf(payload, sizeof(payload),
        "{\"id\":\"%s\",\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,\"steps\":%lu,\"activity\":\"%s\"}",
        DEVICE_ID, sat_lat, sat_lng, sat_alt,
        (unsigned long)g_steps, g_activity
      );
      safePublish(TOPIC_TRACKING, payload);
    }
    vTaskDelay(pdMS_TO_TICKS(60000));
  }
}

// ==================== IMU PUBLISH TASK ====================
void TaskIMUPublish(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(9000));
  for (;;) {
    if (mqtt.connected()) {
      char payload[256];
      snprintf(payload, sizeof(payload),
        "{\"id\":\"%s\",\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
        "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
        "\"roll\":%.2f,\"pitch\":%.2f}",
        DEVICE_ID,
        (float)g_ax, (float)g_ay, (float)g_az,
        (float)g_gx, (float)g_gy, (float)g_gz,
        (float)g_roll, (float)g_pitch
      );
      safePublish(TOPIC_IMU, payload);
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// ==================== SOS ALERT TASK ====================
void TaskSOSAlert(void* pv) {
  vTaskDelay(pdMS_TO_TICKS(8000));
  for (;;) {
    char statusStr[16] = "";
    bool doSend = false;

    if (g_fallDetected)  { g_fallDetected = false;  strncpy(statusStr,"fall",16);   doSend=true; }
    if (g_sosBtnPressed) { g_sosBtnPressed = false; strncpy(statusStr,"manual",16); doSend=true; }

    if (doSend && mqtt.connected()) {
      char payload[256];
      snprintf(payload, sizeof(payload),
        "{\"id\":\"%s\",\"lat\":%.6f,\"lng\":%.6f,\"alt\":%.1f,\"status\":\"%s\"}",
        DEVICE_ID, sat_lat, sat_lng, sat_alt, statusStr
      );
      safePublish(TOPIC_SOS, payload);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== CUBESAT PAYLOAD NODE ===");
  Serial.printf("Device : %s\n", DEVICE_ID);

  secureClient.setInsecure();
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setBufferSize(512);

  mpuInit();
  mqttMutex = xSemaphoreCreateMutex();

  xTaskCreate(TaskWiFi,       "WiFi",    4096, NULL, 5, NULL);
  xTaskCreate(TaskMQTT,       "MQTT",    4096, NULL, 4, NULL);
  xTaskCreate(TaskIMU,        "IMU",     4096, NULL, 3, NULL);
  xTaskCreate(TaskSOSBtn,     "SOSBtn",  2048, NULL, 3, NULL);
  xTaskCreate(TaskTracking,   "Track",   4096, NULL, 2, NULL);
  xTaskCreate(TaskIMUPublish, "IMUPub",  4096, NULL, 2, NULL);
  xTaskCreate(TaskSOSAlert,   "SOS",     3072, NULL, 3, NULL);
}

void loop() {}
