/*
  ================================================================
  GROUND STATION NODE - ESP32-C3
  Wokwi Simulator

  Fungsi:
    Subscribe : cubesat/tracking, cubesat/sos, cubesat/imu
    Forward   : ground/tracking,  ground/sos,  ground/imu

  Hardware: ESP32-C3 DevKitM-1 only (no sensors needed)
  LED indikator:
    GPIO5 = biru  (data received)
    GPIO4 = merah (SOS received)
  ================================================================
*/

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <string.h>

// ==================== WIFI CONFIG ====================
const char* wifi_ssid = "Wokwi-GUEST";
const char* wifi_password = "";

// ==================== MQTT CONFIG ====================
const char* mqtt_broker_url = "7b4f38c681884b269217eb69667d5308.s1.eu.hivemq.cloud";
const int mqtt_broker_port = 8883;
const char* mqtt_username = "Rheostat";
const char* mqtt_password_key = "Admin123";
const char* mqtt_client_id = "ground_station_01";

// ==================== TOPICS ====================
// Subscribe from CubeSat
#define SUB_PATH_TRACKING "cubesat/tracking"
#define SUB_PATH_SOS      "cubesat/sos"
#define SUB_PATH_IMU      "cubesat/imu"

// Forward / Publish to Ground
#define PUB_PATH_TRACKING "ground/tracking"
#define PUB_PATH_SOS      "ground/sos"
#define PUB_PATH_IMU      "ground/imu"

// ==================== HARDWARE PINS ====================
#define LED_DATA_RX_PIN 5   // blue indicator
#define LED_SOS_ALARM_PIN 4 // red indicator

// ==================== COMMS OBJECTS ====================
WiFiClientSecure secure_wifi_client;
PubSubClient mqtt_client(secure_wifi_client);
SemaphoreHandle_t mqtt_resource_mutex;

// ==================== LED HELPERS ====================
void trigger_led_blink(int pin, int blink_count, int delay_ms) {
    for (int i = 0; i < blink_count; i++) {
        digitalWrite(pin, HIGH);
        delay(delay_ms);
        digitalWrite(pin, LOW);
        delay(delay_ms);
    }
}

// ==================== MQTT CALLBACK ====================
void handle_received_message(char* incoming_topic, byte* raw_payload, unsigned int payload_length) {
    char message_buffer[512] = {0};
    if (payload_length >= sizeof(message_buffer)) payload_length = sizeof(message_buffer) - 1;
    memcpy(message_buffer, raw_payload, payload_length);
    message_buffer[payload_length] = '\0';

    Serial.printf("\n[RX] Topic : %s\n", incoming_topic);
    Serial.printf("[RX] Payload: %s\n", message_buffer);

    const char* forward_destination_topic = NULL;

    if (strcmp(incoming_topic, SUB_PATH_TRACKING) == 0) {
        forward_destination_topic = PUB_PATH_TRACKING;
        
        xSemaphoreTake(mqtt_resource_mutex, portMAX_DELAY);
        mqtt_client.publish(forward_destination_topic, message_buffer);
        xSemaphoreGive(mqtt_resource_mutex);
        
        digitalWrite(LED_DATA_RX_PIN, HIGH);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_DATA_RX_PIN, LOW);
        Serial.printf("[FWD] %s -> %s : OK\n", incoming_topic, forward_destination_topic);
    }
    else if (strcmp(incoming_topic, SUB_PATH_SOS) == 0) {
        forward_destination_topic = PUB_PATH_SOS;
        
        xSemaphoreTake(mqtt_resource_mutex, portMAX_DELAY);
        mqtt_client.publish(forward_destination_topic, message_buffer);
        xSemaphoreGive(mqtt_resource_mutex);
        
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_SOS_ALARM_PIN, HIGH); delay(200);
            digitalWrite(LED_SOS_ALARM_PIN, LOW);  delay(200);
        }
        Serial.printf("[FWD] %s -> %s : OK *** SOS ALERT ***\n", incoming_topic, forward_destination_topic);
    }
    // Logic for Inertial Data
    else if (strcmp(incoming_topic, SUB_PATH_IMU) == 0) {
        forward_destination_topic = PUB_PATH_IMU;
        
        xSemaphoreTake(mqtt_resource_mutex, portMAX_DELAY);
        mqtt_client.publish(forward_destination_topic, message_buffer);
        xSemaphoreGive(mqtt_resource_mutex);
        
        Serial.printf("[FWD] %s -> %s : OK\n", incoming_topic, forward_destination_topic);
    }
}

// ==================== WIFI TASK ====================
void task_establish_wifi(void* parameter) {
    Serial.println("[WIFI] Connecting...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    Serial.printf("\n[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    vTaskDelete(NULL);
}

// ==================== MQTT MANAGEMENT ====================
void reconnect_mqtt_broker() {
    while (!mqtt_client.connected()) {
        Serial.print("[MQTT] Connecting...");
        if (mqtt_client.connect(mqtt_client_id, mqtt_username, mqtt_password_key)) {
            Serial.println(" OK");
            // Register subscriptions
            mqtt_client.subscribe(SUB_PATH_TRACKING);
            mqtt_client.subscribe(SUB_PATH_SOS);
            mqtt_client.subscribe(SUB_IMU);
            Serial.println("[MQTT] Subscribed to cubesat topics");
        } else {
            Serial.printf(" FAILED rc=%d\n", mqtt_client.state());
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

void task_manage_mqtt(void* parameter) {
    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!mqtt_client.connected()) reconnect_mqtt_broker();
            mqtt_client.loop();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== SYSTEM MONITORING ====================
void task_system_heartbeat(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    uint32_t heartbeat_counter = 0;
    for (;;) {
        Serial.printf("[GS] Heartbeat #%lu | MQTT: %s | WiFi: %s\n",
            ++heartbeat_counter,
            mqtt_client.connected() ? "OK" : "DISCONNECTED",
            WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED"
        );
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(1500);
    Serial.println("\n=== CUBESAT GROUND STATION ===");

    pinMode(LED_DATA_RX_PIN, OUTPUT);
    pinMode(LED_SOS_ALARM_PIN, OUTPUT);
    digitalWrite(LED_DATA_RX_PIN, LOW);
    digitalWrite(LED_SOS_ALARM_PIN, LOW);

    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_DATA_RX_PIN, HIGH); delay(100);
        digitalWrite(LED_SOS_ALARM_PIN, HIGH); delay(100);
        digitalWrite(LED_DATA_RX_PIN, LOW); delay(100);
        digitalWrite(LED_SOS_ALARM_PIN, LOW); delay(100);
    }

    secure_wifi_client.setInsecure();
    mqtt_client.setServer(mqtt_broker_url, mqtt_broker_port);
    mqtt_client.setBufferSize(512);
    mqtt_client.setCallback(handle_received_message);

    mqtt_resource_mutex = xSemaphoreCreateMutex();

    // Launch background tasks
    xTaskCreate(task_establish_wifi, "WiFi_Task", 4096, NULL, 5, NULL);
    xTaskCreate(task_manage_mqtt, "MQTT_Task", 4096, NULL, 4, NULL);
    xTaskCreate(task_system_heartbeat, "Monitor_Task", 2048, NULL, 1, NULL);
}

void loop() {
    // Handled by FreeRTOS tasks
}