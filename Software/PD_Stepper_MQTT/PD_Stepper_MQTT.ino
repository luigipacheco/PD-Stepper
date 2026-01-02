/*
 * PD Stepper MQTT Control Example:
 * 
 *    How to Use:
 * 1. On first boot, connect to WiFi Network named "PD Stepper Setup"
 * 2. On a browser visit 192.168.4.1
 * 3. Configure WiFi and MQTT broker settings
 * 4. After configuration, device will connect to WiFi and MQTT
 * 
 * MQTT Topics:
 * - motor/mode: "velocity", "move", or "position"
 * - motor/rpm: speed in RPM (when mode is velocity)
 * - motor/direction: "cw" or "ccw"
 * - motor/home: true sets current position as home, then resets to false
 * - motor/conf: JSON config (steps_per_rotation, distance_per_step, acceleration)
 * - motor/enable: true/false to enable/disable motor (allows manual rotation when disabled)
 * - motor/position: publishes current position (distance from home)
 * - motor/target: target distance (can be negative/positive), optionally with velocity like "20 mm/s"
 * 
 * Modes:
 * - "velocity": Continuous rotation at set RPM
 * - "move": One-time movement to target position, stops when reached
 * - "position": Continuously maintains target position, actively corrects if moved
 * 
 * For more info and to purchase PD Stepper kits visit:
 * https://thingsbyjosh.com
*/

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <PubSubClient.h>
#include <TMC2209.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "config_html.h"

Preferences preferences;

// WiFi and MQTT configuration
String wifi_ssid = "";
String wifi_password = "";
String mqtt_server = "";
int mqtt_port = 1883;
String mqtt_username = "";
String mqtt_password = "";
String mqtt_client_id = "PD_Stepper_" + String((uint32_t)ESP.getEfuseMac(), HEX);
String mqtt_base_topic = "motor";

// Network objects
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
AsyncWebServer server(80);

// Access Point for initial setup
const char *ap_ssid = "PD Stepper Setup";
const char *ap_password = "";

// TMC2209 setup
TMC2209 stepper_driver;
HardwareSerial & serial_stream = Serial2;
const long SERIAL_BAUD_RATE = 115200;
const uint8_t RUN_CURRENT_PERCENT = 100;

// Pin definitions (same as other examples)
#define TMC_EN  21
#define STEP    5
#define DIR     6
#define MS1     1
#define MS2     2
#define SPREAD  7
#define TMC_TX  17
#define TMC_RX  18
#define DIAG    16
#define INDEX   11
#define PG      15
#define CFG1    38
#define CFG2    48
#define CFG3    47
#define VBUS    4
#define NTC     7
#define LED1    10
#define LED2    12
#define SW1     35
#define SW2     36
#define SW3     37
#define AUX1    14
#define AUX2    13

// AS5600 Hall Effect Encoder
#define SCL     9
#define SDA     8
#include <Wire.h>
#define AS5600_ADDRESS 0x36
signed long total_encoder_counts = 0;
signed long home_position = 0; // Encoder position when home is set
unsigned long lastEncRead = 0;
int mainFreq = 10; // 100Hz for encoder reading

// Global variables
bool PGState = 0;
bool enabledState = 0;
bool state = 0;
int set_speed = 0;

// Motor configuration
String enabled1 = "enabled";
String setVoltage = "12";
String microsteps = "32";
String current = "30";
String stallThreshold = "10";
String standstillMode = "NORMAL";

// Motor control variables
String motor_mode = "velocity"; // "velocity", "move", or "position"
float motor_rpm = 0.0;
String motor_direction = "cw"; // "cw" or "ccw"
bool home_requested = false;
bool mqtt_enabled = true; // MQTT enable/disable control (true = enabled, false = disabled for manual rotation)

// Position control variables
float current_position_mm = 0.0; // Current position in mm from home
float target_position_mm = 0.0; // Target position in mm
float position_velocity_mm_s = 10.0; // Default velocity for position moves
bool position_moving = false; // For move mode: true when moving to target
bool position_mode_active = false; // For position mode: true when actively maintaining position

// Motor configuration
int steps_per_rotation = 200;
float distance_per_step_mm = 0.0; // Distance per step in mm (calculated from distance_per_revolution)
float distance_per_revolution_mm = 200.0; // Total distance per full rotation (default: 200mm per revolution)
float acceleration_mm_s2 = 100.0; // Acceleration in mm/s²

// Position control timing
unsigned long lastStep = 0;
signed long CurrentPosition = 0; // Position in microsteps
signed long setPoint = 0; // Target position in microsteps

// Voltage reading
float VBusVoltage = 0;
const float DIV_RATIO = 0.1189427313;

// MQTT status
unsigned long lastMqttReconnect = 0;
unsigned long lastPositionPublish = 0;
const unsigned long POSITION_PUBLISH_INTERVAL = 500; // Publish position every 500ms

// Setup mode flag
bool setup_mode = false;

void setup() {
  // PD Trigger Setup
  pinMode(PG, INPUT);
  pinMode(CFG1, OUTPUT);
  pinMode(CFG2, OUTPUT);
  pinMode(CFG3, OUTPUT);
  digitalWrite(CFG1, LOW);
  digitalWrite(CFG2, LOW);
  digitalWrite(CFG3, HIGH);

  // General pins
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);

  // TMC2209 setup
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(TMC_EN, OUTPUT);
  pinMode(DIAG, INPUT);
  digitalWrite(TMC_EN, LOW);
  digitalWrite(MS2, LOW);

  // AS5600 Encoder Setup
  Wire.begin(SDA, SCL);

  // ADC Setup
  analogSetPinAttenuation(VBUS, ADC_11db);

  // Read settings from flash
  readSettings();

  // Initialize TMC2209
  stepper_driver.setup(serial_stream, SERIAL_BAUD_RATE, TMC2209::SERIAL_ADDRESS_0, TMC_RX, TMC_TX);
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableAutomaticCurrentScaling();
  stepper_driver.enableStealthChop();
  stepper_driver.setCoolStepDurationThreshold(5000);
  stepper_driver.disable();

  configureSettings();

  delay(200);
  Serial.begin(115200);
  Serial.println("PD Stepper MQTT Starting...");

  // Setup web server routes (available in both setup and normal mode)
  setupWebServer();
  
  // Check if WiFi credentials are configured
  if (wifi_ssid.length() == 0 || mqtt_server.length() == 0) {
    // Start in setup mode (Access Point)
    setup_mode = true;
    startSetupMode();
  } else {
    // Connect to WiFi and MQTT
    setup_mode = false;
    connectToWiFi();
    setupMQTT();
    // Start web server on WiFi network too
    server.begin();
    Serial.println("Web server started on WiFi network");
  }

  digitalWrite(LED1, HIGH);
  delay(200);
  digitalWrite(LED1, LOW);
}

void loop() {
  // Handle serial commands (available in both modes)
  handleSerialCommands();
  
  if (setup_mode) {
    // In setup mode, just handle web server
    // Web server runs asynchronously, nothing to do here
    delay(100);
    return;
  }

  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    connectToWiFi();
  }

  // Handle MQTT
  if (!mqtt_client.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnect > 5000) {
      lastMqttReconnect = now;
      reconnectMQTT();
    }
  } else {
    mqtt_client.loop();
  }

  // Main control loop (100Hz)
  if (millis() - lastEncRead >= mainFreq) {
    lastEncRead = millis();
    readEncoder();

    // Update current position in mm
    updateCurrentPosition();

    // Publish position periodically
    if (millis() - lastPositionPublish >= POSITION_PUBLISH_INTERVAL) {
      lastPositionPublish = millis();
      publishPosition();
    }

    // Power Good check and enable/disable control
    PGState = digitalRead(PG);
    // Enable motor if: power is good, web config is enabled, MQTT is enabled, and currently disabled
    if (PGState == LOW && enabled1 == "enabled" && mqtt_enabled && enabledState == 0) {
      stepper_driver.enable();
      enabledState = 1;
    } 
    // Disable motor if: power is bad, web config is disabled, MQTT is disabled, or any combination
    else if ((PGState == HIGH || enabled1 == "disabled" || !mqtt_enabled) && enabledState == 1) {
      stepper_driver.disable();
      enabledState = 0;
    }

    digitalWrite(LED2, digitalRead(DIAG)); // Stall detection
  }

  // Handle motor control (only if enabled)
  if (mqtt_enabled && enabled1 == "enabled") {
    if (motor_mode == "velocity") {
      handleVelocityControl();
      position_moving = false; // Stop move mode
      position_mode_active = false; // Stop position mode
    } else if (motor_mode == "move") {
      handleMoveControl(); // One-time movement to target
    } else if (motor_mode == "position") {
      handlePositionControl(); // Continuously maintain target position
    }
  } else {
    // Motor is disabled - stop any movement
    stepper_driver.moveAtVelocity(0);
    position_moving = false;
    position_mode_active = false;
  }

  // Handle home request
  if (home_requested) {
    home_position = total_encoder_counts;
    current_position_mm = 0.0;
    target_position_mm = 0.0;
    setPoint = 0;
    CurrentPosition = 0;
    home_requested = false;
    // If in position mode, update setpoint to maintain at home
    if (motor_mode == "position") {
      position_mode_active = true;
    }
    // Publish home=false back to MQTT
    mqtt_client.publish((mqtt_base_topic + "/home").c_str(), "false", true);
  }
}

void setupWebServer() {
  // Setup web server routes (used in both setup and normal mode)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", config_html, processor);
  });

  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    bool config_saved = false;
    
    if (request->hasParam("wifi_ssid", true)) {
      wifi_ssid = request->getParam("wifi_ssid", true)->value();
      config_saved = true;
    }
    if (request->hasParam("wifi_password", true)) {
      wifi_password = request->getParam("wifi_password", true)->value();
    }
    if (request->hasParam("mqtt_server", true)) {
      mqtt_server = request->getParam("mqtt_server", true)->value();
      config_saved = true;
    }
    if (request->hasParam("mqtt_port", true)) {
      mqtt_port = request->getParam("mqtt_port", true)->value().toInt();
    }
    if (request->hasParam("mqtt_username", true)) {
      mqtt_username = request->getParam("mqtt_username", true)->value();
    }
    if (request->hasParam("mqtt_password", true)) {
      mqtt_password = request->getParam("mqtt_password", true)->value();
    }
    if (request->hasParam("mqtt_base_topic", true)) {
      mqtt_base_topic = request->getParam("mqtt_base_topic", true)->value();
    }

    // Save motor settings
    if (request->hasParam("enabled1", true)) {
      enabled1 = "enabled";
    } else {
      enabled1 = "disabled";
    }
    if (request->hasParam("setvoltage", true)) {
      setVoltage = request->getParam("setvoltage", true)->value();
    }
    if (request->hasParam("microsteps", true)) {
      microsteps = request->getParam("microsteps", true)->value();
    }
    if (request->hasParam("current", true)) {
      current = request->getParam("current", true)->value();
    }
    if (request->hasParam("stall_threshold", true)) {
      stallThreshold = request->getParam("stall_threshold", true)->value();
    }
    if (request->hasParam("standstill_mode", true)) {
      standstillMode = request->getParam("standstill_mode", true)->value();
    }

    if (config_saved) {
      writeSettings();
      request->send(200, "text/html", "<h1>Configuration Saved!</h1><p>Device will restart and connect to WiFi/MQTT.</p><script>setTimeout(function(){window.location.href='/';}, 3000);</script>");
      delay(1000);
      ESP.restart();
    } else {
      writeSettings();
      request->redirect("/");
    }
  });
}

void startSetupMode() {
  Serial.println("Starting setup mode (Access Point)");
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(ip);

  // Start web server
  server.begin();
  Serial.println("Setup web server started");
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(wifi_ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("WiFi connection failed. Starting setup mode...");
    setup_mode = true;
    startSetupMode();
  }
}

void setupMQTT() {
  mqtt_client.setServer(mqtt_server.c_str(), mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  reconnectMQTT();
}

void reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  Serial.print("Attempting MQTT connection...");
  
  if (mqtt_username.length() > 0 && mqtt_password.length() > 0) {
    if (mqtt_client.connect(mqtt_client_id.c_str(), mqtt_username.c_str(), mqtt_password.c_str())) {
      Serial.println("connected");
      subscribeToTopics();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
    }
  } else {
    if (mqtt_client.connect(mqtt_client_id.c_str())) {
      Serial.println("connected");
      subscribeToTopics();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

void subscribeToTopics() {
  mqtt_client.subscribe((mqtt_base_topic + "/mode").c_str());
  mqtt_client.subscribe((mqtt_base_topic + "/rpm").c_str());
  mqtt_client.subscribe((mqtt_base_topic + "/direction").c_str());
  mqtt_client.subscribe((mqtt_base_topic + "/home").c_str());
  mqtt_client.subscribe((mqtt_base_topic + "/conf").c_str());
  mqtt_client.subscribe((mqtt_base_topic + "/target").c_str());
  mqtt_client.subscribe((mqtt_base_topic + "/enable").c_str());
  Serial.println("Subscribed to MQTT topics");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topic_str = String(topic);
  String payload_str = "";
  
  for (unsigned int i = 0; i < length; i++) {
    payload_str += (char)payload[i];
  }
  
  payload_str.trim();

  Serial.print("MQTT Message [");
  Serial.print(topic_str);
  Serial.print("]: ");
  Serial.println(payload_str);

  // Handle different topics
  if (topic_str == mqtt_base_topic + "/mode") {
    if (payload_str == "velocity" || payload_str == "move" || payload_str == "position") {
      motor_mode = payload_str;
      if (motor_mode == "move" || motor_mode == "position") {
        // Stop velocity mode when switching to move/position mode
        stepper_driver.moveAtVelocity(0);
      }
      if (motor_mode == "position") {
        // Activate position mode - will continuously maintain target
        position_mode_active = true;
      } else {
        position_mode_active = false;
      }
    }
  }
  else if (topic_str == mqtt_base_topic + "/rpm") {
    motor_rpm = payload_str.toFloat();
  }
  else if (topic_str == mqtt_base_topic + "/direction") {
    if (payload_str == "cw" || payload_str == "ccw") {
      motor_direction = payload_str;
    }
  }
  else if (topic_str == mqtt_base_topic + "/home") {
    if (payload_str == "true" || payload_str == "1") {
      home_requested = true;
    }
  }
  else if (topic_str == mqtt_base_topic + "/conf") {
    parseConfiguration(payload_str);
  }
  else if (topic_str == mqtt_base_topic + "/target") {
    parseTarget(payload_str);
  }
  else if (topic_str == mqtt_base_topic + "/enable") {
    // Handle enable/disable
    if (payload_str == "true" || payload_str == "1" || payload_str == "enabled") {
      mqtt_enabled = true;
      Serial.println("Motor enabled via MQTT");
    } else if (payload_str == "false" || payload_str == "0" || payload_str == "disabled") {
      mqtt_enabled = false;
      // Stop any movement when disabling
      stepper_driver.moveAtVelocity(0);
      position_moving = false;
      position_mode_active = false;
      stepper_driver.disable();
      enabledState = 0;
      Serial.println("Motor disabled via MQTT (can now be manually rotated)");
    }
  }
}

void parseConfiguration(String config_json) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, config_json);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("steps_per_rotation")) {
    steps_per_rotation = doc["steps_per_rotation"];
  }
  if (doc.containsKey("distance_per_revolution")) {
    distance_per_revolution_mm = doc["distance_per_revolution"];
  }
  if (doc.containsKey("distance_per_step")) {
    // Allow direct specification of distance_per_step
    distance_per_step_mm = doc["distance_per_step"];
    if (distance_per_step_mm > 0 && steps_per_rotation > 0 && microsteps.toInt() > 0) {
      distance_per_revolution_mm = distance_per_step_mm * steps_per_rotation * microsteps.toInt();
    }
  } else if (doc.containsKey("distance_per_revolution")) {
    // Calculate distance per step from distance_per_revolution
    if (steps_per_rotation > 0 && microsteps.toInt() > 0) {
      distance_per_step_mm = distance_per_revolution_mm / (steps_per_rotation * microsteps.toInt());
    }
  }
  if (doc.containsKey("acceleration")) {
    acceleration_mm_s2 = doc["acceleration"];
  }
  
  // Calculate distance_per_step if distance_per_revolution was provided
  if (doc.containsKey("distance_per_revolution") && !doc.containsKey("distance_per_step")) {
    if (steps_per_rotation > 0 && microsteps.toInt() > 0) {
      distance_per_step_mm = distance_per_revolution_mm / (steps_per_rotation * microsteps.toInt());
    }
  }
  // Calculate distance_per_revolution if distance_per_step was provided
  else if (doc.containsKey("distance_per_step")) {
    if (steps_per_rotation > 0 && microsteps.toInt() > 0) {
      distance_per_revolution_mm = distance_per_step_mm * steps_per_rotation * microsteps.toInt();
    }
  }
  
  // Save configuration to flash
  preferences.begin("settings", false);
  preferences.putInt("steps_per_rotation", steps_per_rotation);
  preferences.putFloat("distance_per_step", distance_per_step_mm);
  preferences.putFloat("distance_per_revolution", distance_per_revolution_mm);
  preferences.putFloat("acceleration", acceleration_mm_s2);
  preferences.end();

  Serial.print("Configuration updated: steps_per_rotation=");
  Serial.print(steps_per_rotation);
  Serial.print(", distance_per_revolution=");
  Serial.print(distance_per_revolution_mm);
  Serial.print("mm, acceleration=");
  Serial.print(acceleration_mm_s2);
  Serial.println("mm/s²");
}

void parseTarget(String target_str) {
  bool json_parsed = false;
  
  // Try to parse as JSON array first: [target, velocity]
  if (target_str.startsWith("[") && target_str.endsWith("]")) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, target_str);
    
    if (!error && doc.is<JsonArray>() && doc.size() >= 1) {
      // JSON array format: [target, velocity]
      target_position_mm = doc[0].as<float>();
      if (doc.size() >= 2) {
        position_velocity_mm_s = doc[1].as<float>();
        if (position_velocity_mm_s <= 0) {
          position_velocity_mm_s = 10.0; // Default
        }
      } else {
        position_velocity_mm_s = 10.0; // Default if not specified
      }
      json_parsed = true;
    } else {
      // JSON parse failed, fall through to string parsing
      Serial.print("JSON parse error for target: ");
      Serial.println(error.c_str());
    }
  }
  
  // If JSON parsing failed or not JSON format, parse as string
  if (!json_parsed) {
    // Parse as string format - can be just a number or "number mm/s"
    // Extract the first number (target distance)
    int space_pos = target_str.indexOf(' ');
    String distance_str = target_str;
    if (space_pos > 0) {
      distance_str = target_str.substring(0, space_pos);
      // Check if there's a velocity specified
      String remaining = target_str.substring(space_pos + 1);
      remaining.trim();
      if (remaining.indexOf("mm/s") >= 0 || remaining.indexOf("mm/ s") >= 0) {
        // Extract velocity number
        String vel_str = remaining;
        vel_str.replace("mm/s", "");
        vel_str.replace("mm/ s", "");
        vel_str.replace(" ", "");
        vel_str.trim();
        position_velocity_mm_s = vel_str.toFloat();
        if (position_velocity_mm_s <= 0) {
          position_velocity_mm_s = 10.0; // Default
        }
      }
    }
    target_position_mm = distance_str.toFloat();
  }
  
  // Convert target position to microsteps
  // Use encoder-based position if available, otherwise use step counting
  if (distance_per_revolution_mm > 0) {
    // Calculate target in encoder counts
    float target_revolutions = target_position_mm / distance_per_revolution_mm;
    signed long target_encoder_counts = home_position + (signed long)(target_revolutions * 4096.0);
    
    // Calculate how many microsteps to move
    signed long encoder_diff = target_encoder_counts - total_encoder_counts;
    if (distance_per_step_mm > 0 && steps_per_rotation > 0 && microsteps.toInt() > 0) {
      // Convert encoder difference to microsteps
      float revolutions_to_move = (float)encoder_diff / 4096.0;
      signed long microsteps_to_move = (signed long)(revolutions_to_move * steps_per_rotation * microsteps.toInt());
      setPoint = CurrentPosition + microsteps_to_move;
    } else {
      // Fallback: use distance directly
      float distance_to_move = target_position_mm - current_position_mm;
      if (distance_per_step_mm > 0) {
        signed long microsteps_to_move = (signed long)(distance_to_move / distance_per_step_mm);
        setPoint = CurrentPosition + microsteps_to_move;
      }
    }
    position_moving = true;
  } else if (distance_per_step_mm > 0) {
    // Fallback to step-based calculation
    float distance_to_move = target_position_mm - current_position_mm;
    signed long microsteps_to_move = (signed long)(distance_to_move / distance_per_step_mm);
    setPoint = CurrentPosition + microsteps_to_move;
    position_moving = true; // For move mode
    if (motor_mode == "position") {
      position_mode_active = true; // For position mode
    }
  } else {
    Serial.println("Error: Motor configuration not set. Please configure motor/conf first.");
    return;
  }
  
  Serial.print("Target set: ");
  Serial.print(target_position_mm);
  Serial.print("mm, velocity: ");
  Serial.print(position_velocity_mm_s);
  Serial.println("mm/s");
}

void handleVelocityControl() {
  // Calculate velocity in microsteps per period
  // Formula: (RPM / 60) * steps_per_rotation * microsteps / 0.715
  int32_t microsteps_per_period = ((motor_rpm / 60.0) * steps_per_rotation * microsteps.toInt()) / 0.715;
  
  // Apply direction
  if (motor_direction == "ccw") {
    microsteps_per_period = -microsteps_per_period;
  }
  
  stepper_driver.moveAtVelocity(microsteps_per_period);
}

void handleMoveControl() {
  // Move mode: one-time movement to target, stops when reached
  if (!position_moving) {
    return;
  }

  // Calculate step delay based on velocity
  unsigned long step_delay_us = getStepDelay(position_velocity_mm_s);
  
  if (step_delay_us == 0) {
    step_delay_us = 1000; // Default minimum delay
  }

  // Move towards setpoint
  if (setPoint > CurrentPosition) {
    // Need to move in positive direction (increase position)
    if (micros() - lastStep > step_delay_us) {
      digitalWrite(DIR, motor_direction == "cw" ? HIGH : LOW);
      state = !state;
      digitalWrite(STEP, state);
      CurrentPosition += (256 / microsteps.toInt());
      lastStep = micros();
    }
  } else if (setPoint < CurrentPosition) {
    // Need to move in negative direction (decrease position)
    if (micros() - lastStep > step_delay_us) {
      digitalWrite(DIR, motor_direction == "cw" ? LOW : HIGH);
      state = !state;
      digitalWrite(STEP, state);
      CurrentPosition -= (256 / microsteps.toInt());
      lastStep = micros();
    }
  } else {
    // Reached target - stop moving
    position_moving = false;
    stepper_driver.moveAtVelocity(0);
  }
}

void handlePositionControl() {
  // Position mode: continuously maintain target position, actively corrects if moved
  if (!position_mode_active) {
    return;
  }

  // Recalculate setpoint from current target (in case target changed or position drifted)
  if (distance_per_revolution_mm > 0) {
    float target_revolutions = target_position_mm / distance_per_revolution_mm;
    signed long target_encoder_counts = home_position + (signed long)(target_revolutions * 4096.0);
    signed long encoder_diff = target_encoder_counts - total_encoder_counts;
    if (distance_per_step_mm > 0 && steps_per_rotation > 0 && microsteps.toInt() > 0) {
      float revolutions_to_move = (float)encoder_diff / 4096.0;
      signed long microsteps_to_move = (signed long)(revolutions_to_move * steps_per_rotation * microsteps.toInt());
      setPoint = CurrentPosition + microsteps_to_move;
    }
  } else if (distance_per_step_mm > 0) {
    float distance_to_move = target_position_mm - current_position_mm;
    signed long microsteps_to_move = (signed long)(distance_to_move / distance_per_step_mm);
    setPoint = CurrentPosition + microsteps_to_move;
  }

  // Calculate step delay based on velocity
  unsigned long step_delay_us = getStepDelay(position_velocity_mm_s);
  
  if (step_delay_us == 0) {
    step_delay_us = 1000; // Default minimum delay
  }

  // Check if we need to move (with small deadband to prevent jitter)
  signed long position_error = setPoint - CurrentPosition;
  const signed long deadband = 256 / microsteps.toInt(); // 1 microstep deadband

  if (abs(position_error) > deadband) {
    // Need to correct position
    if (position_error > 0) {
      // Need to move in positive direction
      if (micros() - lastStep > step_delay_us) {
        digitalWrite(DIR, motor_direction == "cw" ? HIGH : LOW);
        state = !state;
        digitalWrite(STEP, state);
        CurrentPosition += (256 / microsteps.toInt());
        lastStep = micros();
      }
    } else {
      // Need to move in negative direction
      if (micros() - lastStep > step_delay_us) {
        digitalWrite(DIR, motor_direction == "cw" ? LOW : HIGH);
        state = !state;
        digitalWrite(STEP, state);
        CurrentPosition -= (256 / microsteps.toInt());
        lastStep = micros();
      }
    }
  } else {
    // Within deadband - position is maintained
    stepper_driver.moveAtVelocity(0);
  }
}

unsigned long getStepDelay(float velocity_mm_s) {
  if (velocity_mm_s <= 0 || distance_per_step_mm <= 0) {
    return 0;
  }
  
  // Calculate steps per second needed
  float steps_per_second = velocity_mm_s / distance_per_step_mm;
  
  if (steps_per_second <= 0) {
    return 0;
  }
  
  // Delay between steps in microseconds
  unsigned long delay_us = (unsigned long)((1.0 / steps_per_second) * 1000000.0);
  
  return delay_us;
}

void updateCurrentPosition() {
  // Calculate current position in mm from home
  if (distance_per_revolution_mm > 0) {
    // Use encoder position relative to home (most accurate)
    signed long encoder_offset = total_encoder_counts - home_position;
    // Convert encoder counts to distance (assuming 4096 counts per revolution)
    float revolutions = (float)encoder_offset / 4096.0;
    current_position_mm = revolutions * distance_per_revolution_mm;
    
    // Sync CurrentPosition with encoder for step-based control
    if (distance_per_step_mm > 0 && steps_per_rotation > 0 && microsteps.toInt() > 0) {
      float current_revolutions = (float)encoder_offset / 4096.0;
      CurrentPosition = (signed long)(current_revolutions * steps_per_rotation * microsteps.toInt());
    }
  } else if (distance_per_step_mm > 0) {
    // Fallback to microstep-based position
    current_position_mm = (float)CurrentPosition * distance_per_step_mm;
  }
}

void publishPosition() {
  if (mqtt_client.connected()) {
    char position_str[32];
    dtostrf(current_position_mm, 1, 2, position_str);
    mqtt_client.publish((mqtt_base_topic + "/position").c_str(), position_str, true);
  }
}

void readEncoder() {
  int raw_counts;
  static int prev_raw_counts = 0;
  static signed long revolutions = 0;

  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    raw_counts = Wire.read() << 8 | Wire.read();
  }

  // Check for wrap around
  if (prev_raw_counts > 3000 && raw_counts < 1000) {
    revolutions++;
  } else if (prev_raw_counts < 1000 && raw_counts > 3000) {
    revolutions--;
  }

  prev_raw_counts = raw_counts;
  total_encoder_counts = raw_counts + (4096 * revolutions);
}

String processor(const String& var) {
  if (var == "wifi_ssid") return wifi_ssid;
  if (var == "mqtt_server") return mqtt_server;
  if (var == "mqtt_port") return String(mqtt_port);
  if (var == "mqtt_username") return mqtt_username;
  if (var == "mqtt_base_topic") return mqtt_base_topic;
  if (var == "enabled1") {
    return (enabled1 == "enabled") ? "checked" : "";
  }
  if (var == "microsteps") return microsteps;
  if (var == "voltage") return setVoltage;
  if (var == "current") return current;
  if (var == "stall_threshold") return stallThreshold;
  if (var == "standstill_mode") return standstillMode;
  return String();
}

void configureSettings() {
  if (setVoltage == "5") {
    digitalWrite(CFG1, HIGH);
  } else if (setVoltage == "9") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, LOW);
    digitalWrite(CFG3, LOW);
  } else if (setVoltage == "12") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, LOW);
    digitalWrite(CFG3, HIGH);
  } else if (setVoltage == "15") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, HIGH);
    digitalWrite(CFG3, HIGH);
  } else if (setVoltage == "20") {
    digitalWrite(CFG1, LOW);
    digitalWrite(CFG2, HIGH);
    digitalWrite(CFG3, LOW);
  }

  stepper_driver.setRunCurrent(current.toInt());
  stepper_driver.setMicrostepsPerStep(microsteps.toInt());
  stepper_driver.setStallGuardThreshold(stallThreshold.toInt());

  if (standstillMode == "NORMAL") {
    stepper_driver.setStandstillMode(stepper_driver.NORMAL);
  } else if (standstillMode == "FREEWHEELING") {
    stepper_driver.setStandstillMode(stepper_driver.FREEWHEELING);
  } else if (standstillMode == "BRAKING") {
    stepper_driver.setStandstillMode(stepper_driver.BRAKING);
  } else if (standstillMode == "STRONG_BRAKING") {
    stepper_driver.setStandstillMode(stepper_driver.STRONG_BRAKING);
  }
}

void readSettings() {
  preferences.begin("settings", false);

  wifi_ssid = preferences.getString("wifi_ssid", "");
  wifi_password = preferences.getString("wifi_password", "");
  mqtt_server = preferences.getString("mqtt_server", "");
  mqtt_port = preferences.getInt("mqtt_port", 1883);
  mqtt_username = preferences.getString("mqtt_username", "");
  mqtt_password = preferences.getString("mqtt_password", "");
  mqtt_base_topic = preferences.getString("mqtt_base_topic", "motor");

  enabled1 = preferences.getString("enable", "");
  if (enabled1 == "") {
    preferences.end();
    enabled1 = "enabled";
    setVoltage = "12";
    microsteps = "32";
    current = "30";
    stallThreshold = "10";
    standstillMode = "NORMAL";
    writeSettings();
  } else {
    setVoltage = preferences.getString("voltage", "12");
    microsteps = preferences.getString("microsteps", "32");
    current = preferences.getString("current", "30");
    stallThreshold = preferences.getString("stallThreshold", "10");
    standstillMode = preferences.getString("standstillMode", "NORMAL");
    preferences.end();
  }

  // Read motor configuration
  preferences.begin("settings", false);
  steps_per_rotation = preferences.getInt("steps_per_rotation", 200);
  distance_per_step_mm = preferences.getFloat("distance_per_step", 0.0);
  distance_per_revolution_mm = preferences.getFloat("distance_per_revolution", 200.0); // Default: 200mm per revolution
  acceleration_mm_s2 = preferences.getFloat("acceleration", 100.0);
  
  // Set defaults if not configured
  if (distance_per_step_mm <= 0 && distance_per_revolution_mm <= 0) {
    // Default: 200mm per revolution
    distance_per_revolution_mm = 200.0;
    if (steps_per_rotation > 0 && microsteps.toInt() > 0) {
      distance_per_step_mm = distance_per_revolution_mm / (steps_per_rotation * microsteps.toInt());
    }
    // Save default to preferences
    preferences.putFloat("distance_per_step", distance_per_step_mm);
    preferences.putFloat("distance_per_revolution", distance_per_revolution_mm);
  }
  // Calculate distance_per_revolution from distance_per_step if not set
  else if (distance_per_step_mm > 0 && distance_per_revolution_mm <= 0 && steps_per_rotation > 0 && microsteps.toInt() > 0) {
    distance_per_revolution_mm = distance_per_step_mm * steps_per_rotation * microsteps.toInt();
  }
  // Or calculate distance_per_step from distance_per_revolution if that's what's stored
  else if (distance_per_revolution_mm > 0 && distance_per_step_mm <= 0 && steps_per_rotation > 0 && microsteps.toInt() > 0) {
    distance_per_step_mm = distance_per_revolution_mm / (steps_per_rotation * microsteps.toInt());
  }

  preferences.end();
}

void writeSettings() {
  preferences.begin("settings", false);

  preferences.putString("wifi_ssid", wifi_ssid);
  preferences.putString("wifi_password", wifi_password);
  preferences.putString("mqtt_server", mqtt_server);
  preferences.putInt("mqtt_port", mqtt_port);
  preferences.putString("mqtt_username", mqtt_username);
  preferences.putString("mqtt_password", mqtt_password);
  preferences.putString("mqtt_base_topic", mqtt_base_topic);

  preferences.putString("enable", enabled1);
  preferences.putString("voltage", setVoltage);
  preferences.putString("microsteps", microsteps);
  preferences.putString("current", current);
  preferences.putString("stallThreshold", stallThreshold);
  preferences.putString("standstillMode", standstillMode);

  preferences.putInt("steps_per_rotation", steps_per_rotation);
  preferences.putFloat("distance_per_step", distance_per_step_mm);
  preferences.putFloat("distance_per_revolution", distance_per_revolution_mm);
  preferences.putFloat("acceleration", acceleration_mm_s2);

  Serial.println("Settings saved to flash");
  preferences.end();
  
  configureSettings();
}

void handleSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase(); // Make command case-insensitive

    if (cmd == "reset" || cmd == "reset wifi" || cmd == "reset config") {
      Serial.println("Resetting WiFi and MQTT configuration...");
      resetWiFiMQTTSettings();
      Serial.println("Configuration cleared. Device will restart in setup mode.");
      delay(1000);
      ESP.restart();
    }
    else if (cmd == "help" || cmd == "?") {
      Serial.println("\n=== PD Stepper MQTT Serial Commands ===");
      Serial.println("reset          - Reset WiFi and MQTT settings (returns to setup mode)");
      Serial.println("help or ?      - Show this help message");
      Serial.println("");
    }
    else if (cmd.length() > 0) {
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      Serial.println("Type 'help' for available commands");
    }
  }
}

void resetWiFiMQTTSettings() {
  preferences.begin("settings", false);
  
  // Clear WiFi and MQTT settings
  preferences.remove("wifi_ssid");
  preferences.remove("wifi_password");
  preferences.remove("mqtt_server");
  preferences.remove("mqtt_port");
  preferences.remove("mqtt_username");
  preferences.remove("mqtt_password");
  preferences.remove("mqtt_base_topic");
  
  preferences.end();
  
  // Clear variables
  wifi_ssid = "";
  wifi_password = "";
  mqtt_server = "";
  mqtt_port = 1883;
  mqtt_username = "";
  mqtt_password = "";
  mqtt_base_topic = "motor";
}

