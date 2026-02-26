#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <DFRobot_GNSS.h> 
#include <math.h>

// ==================== PINS ====================
#define I2C_SDA 33
#define I2C_SCL 35
#define SOS_PIN 11
#define LORA_RX 39
#define LORA_TX 37

// ==================== SETTINGS ====================
#define DEVICE_ID "SAT-01"
#define FALL_FF_THRESHOLD  0.5f
#define FALL_FF_MIN_MS     60
#define FALL_IMPACT_THR    2.2f
#define FALL_IMPACT_WIN_MS 400

// ==================== GLOBALS ====================
DFRobot_BMI160 bmi160;
DFRobot_GNSS_I2C gnss(&Wire, 0x20); 
HardwareSerial LoRaSerial(1); 

volatile bool g_fallDetected = false;
volatile bool g_sosManual    = false;

struct GPSData {
  float lat, lng, alt;
  bool valid;
} currentPos;

void setup() {
  Serial.begin(115200);
  LoRaSerial.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(SOS_PIN, INPUT_PULLUP);

  // FIX: Changed I2CInit to I2cInit (case sensitive)
  if (bmi160.I2cInit() != 0) {
    Serial.println("[IMU] BMI160 Init Failed");
  } else {
    Serial.println("[IMU] BMI160 Ready");
  }
  
  if (!gnss.begin()) {
    Serial.println("[GNSS] DFR1103 Init Failed");
  } else {
    gnss.enablePower();
    Serial.println("[GNSS] DFR1103 Ready");
  }

  xTaskCreate(TaskIMU,     "IMU",   4096, NULL, 3, NULL);
  xTaskCreate(TaskGPS,     "GPS",   4096, NULL, 2, NULL);
  xTaskCreate(TaskLoRaOut, "LoRa",  4096, NULL, 2, NULL);
}

void TaskIMU(void* pv) {
  bool inFF = false;
  uint32_t ffStart = 0;
  bool waitImpact = false;
  uint32_t impactDeadline = 0;

  for (;;) {
    int16_t accel[3]; 
    bmi160.getAccelData(accel);
    
    float gX = accel[0] / 16384.0f;
    float gY = accel[1] / 16384.0f;
    float gZ = accel[2] / 16384.0f;
    float totalG = sqrtf(gX*gX + gY*gY + gZ*gZ);
    
    uint32_t now = millis();

    if (!inFF && !waitImpact && totalG < FALL_FF_THRESHOLD) {
      inFF = true; ffStart = now;
    } else if (inFF && totalG >= FALL_FF_THRESHOLD) {
      if ((now - ffStart) >= FALL_FF_MIN_MS) {
        waitImpact = true;
        impactDeadline = now + FALL_IMPACT_WIN_MS;
      }
      inFF = false;
    } else if (waitImpact) {
      if (totalG > FALL_IMPACT_THR) {
        g_fallDetected = true;
        waitImpact = false;
      } else if (now > impactDeadline) {
        waitImpact = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}void TaskGPS(void* pv) {
  for (;;) {
    sLonLat_t latData = gnss.getLat();
    sLonLat_t lonData = gnss.getLon();
    
    if (latData.latitude != 0) {
      currentPos.lat = (float)latData.latitude / 10000000.0f;
      currentPos.lng = (float)lonData.lonitude / 10000000.0f;

      if (latData.latDirection == 'S') currentPos.lat = -abs(currentPos.lat);
      if (lonData.lonDirection == 'W') currentPos.lng = -abs(currentPos.lng);

      currentPos.alt = gnss.getAlt();
      currentPos.valid = true;
    } else {
      currentPos.valid = false;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
void TaskLoRaOut(void* pv) {
  for (;;) {
    if (digitalRead(SOS_PIN) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(50)); 
      if (digitalRead(SOS_PIN) == LOW) g_sosManual = true;
    }

    if (g_fallDetected || g_sosManual) {
      String status = g_fallDetected ? "FALL" : "MANUAL";
      String packet = "{\"id\":\"" + String(DEVICE_ID) + 
                      "\",\"lat\":" + String(currentPos.lat, 6) + 
                      ",\"lng\":" + String(currentPos.lng, 6) + 
                      ",\"status\":\"" + status + "\"}";

      LoRaSerial.println(packet); 
      g_fallDetected = false;
      g_sosManual = false;
      vTaskDelay(pdMS_TO_TICKS(5000)); 
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void loop() {}