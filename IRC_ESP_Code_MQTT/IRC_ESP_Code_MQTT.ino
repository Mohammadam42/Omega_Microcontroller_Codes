// Code V1.1: Trolley 1 Car - MQTT Version
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_task_wdt.h"
#include <ArduinoOTA.h>
#include <ModbusMaster.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <INA226.h>
ModbusMaster node;

#define RXD2 16
#define TXD2 17
#define Dstart 4096  
#define Mstart 2048
#define Xstart 1024


unsigned long lastMessageTime = 0;
const unsigned long MESSAGE_COOLDOWN = 2000; // 2 seconds cooldown
String lastMessage = "";

// ================== ROBOT IDENTIFICATION Trolley[No] - Robot car/main ==================
const char* ROBOT_NAME = "Robot";
const char* ROBOT_SECTION = "car";
const char* ROBOT_DISPLAY_NAME = "Robot car";
const char* ROBOT_CLIENT_PREFIX = "Robot-car";

// MQTT Topics
const char* topic_sub = "Omega/Robot/car";
const char* topic_pub = "Omega/web";

// ================== WATCHDOG CONFIGURATION ==================
#define WDT_TIMEOUT 800  // 3 seconds timeout

// ================== WIFI CONFIGURATION ==================
س

// ================== MQTT CONFIGURATION ==================
const char* mqtt_server = "885dce4dddce4977b1959e2fd8d937b1.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "Mohammad";
const char* mqtt_pass = "Omega@2022";



// ================== MQTT COMMAND QUEUE ==================
String commandQueue = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 5000;

// ================== HTML PAGE ================== 
WebServer HTML(80);

// ================== MQTT CLIENT ==================
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// ================== TIME CONTROL VARIABLES ==================
unsigned long lastVoltageCheckTime = 0;
unsigned long startTime = 0;
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; // 5 seconds

// ================== MOVEMENT & MODE CONTROL ==================
int cycleCount = 0;
float totalCycles = 0;
String currentMode = "Stopped";
String lastDirection = "";

// ================== LIMITS AND THRESHOLDS ==================
const unsigned long movementTimeout = 28 * 60 * 1000;
const unsigned long voltageCheckInterval = 20 * 60 * 2000;
float volt = 25.2;

// ================== MQTT COMMAND FLAGS ==================
volatile bool stopCommandFlag = false;
volatile bool restartCommandFlag = false;

// ================== FUNCTION DECLARATIONS ==================
void connectToWiFi();
void setupMQTT();
bool reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool handleCommand(String command);
void startManualMode();
bool receiveMessage();
void sendMessage(const char* message, const char* type);
void sendStatus(float voltage);
void sendNotification(const char* message);
void sendAlert(const char* message);
void sendCycleUpdate(const char* message);
void setupWDT();
void setupOTA();
String Robot_HTML();
void restartESP();

// Modbus Functions
uint8_t readRun();
uint8_t readStart();
uint8_t readStop();
uint8_t readManualAuto();
uint8_t readNoMotion();
uint8_t readForwardLimit();
uint8_t readReverseLimit();
uint8_t readEmergencyButton();

// PLC Command Functions
void setPLCTime(uint16_t year, uint16_t weekday, uint16_t month, uint16_t day, uint16_t hour, uint16_t minute, uint16_t second);
void Start_Robot();
void Stop_Robot();
void Backward_Robot();
void set_Wind_speed_return(float value);
void set_Wind_speed_Start(float value);
void setSchedule(uint8_t hour, uint8_t minute, bool days[7]);
float readVoltage();


INA226 ina(0x40); // INA226 current sensor object
// ================== OTA ==================
bool otaReady = false;

// =================== BOARD INITIALIZATION ===================
void setup() {
  Serial.begin(115200);
  setupWDT();
  delay(1000);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  node.begin(1, Serial2);

  // =================== VOLTAGE SENSOR ===================
  // Note: INA226 is optional - system will work without it
  Wire.begin(21, 22);
  Serial.println("⚠️ INA226 sensor not connected - using default voltage readings");


  if (!ina.begin()) {
    Serial.println("INA226 init failed!");
  } else {
    Serial.println("INA226 init Success!");
  }

  ina.setMaxCurrentShunt(10.0, 0.1);  // Set max current to 10A with 0.1Ω shunt
  connectToWiFi();
  
  if (WiFi.status() == WL_CONNECTED) {
    setupMQTT();
    setupOTA();
  } else {
    otaReady = false;
  }

  // ================== HTML PAGE ================== 
  HTML.on("/", []() {
    HTML.send(200, "text/html", Robot_HTML());
  });

  HTML.begin();
  Serial.println("[HTTP] Server started.");
}

// =================== MAIN LOOP ===================
void loop() {
  esp_task_wdt_reset();  // CRITICAL: Reset watchdog in main loop
  
  ArduinoOTA.handle();
  HTML.handleClient();

  // ---------- MQTT Connection Maintenance (BLOCKING until connected) ----------
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      Serial.println("🔗 MQTT not connected - attempting to connect...");
      unsigned long mqttConnectStart = millis();
      
      // BLOCKING: Keep trying to connect to MQTT until successful
      while (!mqttClient.connected()) {
        esp_task_wdt_reset(); // Reset watchdog during connection attempts
        
        if (reconnectMQTT()) {
          Serial.println("✅ MQTT connected successfully!");
          break; // Exit the while loop when connected
        }
        
        Serial.println("❌ MQTT connection failed, retrying in 3 seconds...");
        delay(3000); // Wait 3 seconds before retrying
        
        // Optional: Add a timeout to prevent infinite blocking
        if (millis() - mqttConnectStart > 60000) { // 60 seconds timeout
          Serial.println("⚠️ MQTT connection timeout after 60 seconds, continuing without MQTT");
          break;
        }
      }
    } else {
      // If connected, process MQTT messages
      mqttClient.loop();
    }
  } else {
    // WiFi reconnect logic (non-blocking)
    static unsigned long wifiRetryStart = 0;
    if (wifiRetryStart == 0) {
      wifiRetryStart = millis();
      WiFi.begin(ssid, password);
      Serial.println("📡 WiFi disconnected - attempting to reconnect...");
    }

    if (millis() - wifiRetryStart > 30000) { // 30 seconds timeout
      wifiRetryStart = 0; // Reset to try again later
      Serial.println("⚠️ WiFi reconnection timeout");
    }
    
    // Don't continue with other operations if WiFi is not connected
    delay(1000);
    return;
  }

  // ---------- Only continue if MQTT is connected ----------
  if (!mqttClient.connected()) {
    Serial.println("⏸️ Waiting for MQTT connection before continuing...");
    delay(1000);
    return; // Skip the rest of the loop until MQTT is connected
  }

  // ---------- Voltage monitoring ----------
  static unsigned long lastVoltageCheck = 0;
  unsigned long now = millis();
  if (now - lastVoltageCheck > 60000) { // Check every minute
    lastVoltageCheck = now;
    float voltage = readVoltage();
    if (voltage < volt) {
      sendAlert("⚠️ Warning: Voltage below 25V!");
    }
  }

  // ---------- OTA  ----------
  if (WiFi.status() == WL_CONNECTED) {
    if (!otaReady) setupOTA();
    ArduinoOTA.handle();
  } else {
    otaReady = false;
  }

  // ---------- Robot operation (only if MQTT connected) ----------
  if(readRun() == 1){
    startManualMode();
  }

  // Small delay to prevent overwhelming the CPU
  delay(10);
}


void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 1000000000000000; // 30 seconds timeout
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    esp_task_wdt_reset(); // Reset watchdog during WiFi connection
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n⚠️ WiFi connection timeout. Continuing without WiFi.");
  }
}

void setupMQTT() {
  // Use insecure connection for testing (bypass certificate verification)
  espClient.setInsecure();
  
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  mqttClient.setKeepAlive(60); // 60 seconds keepalive
}

bool reconnectMQTT() {
  Serial.print("🔗 Attempting MQTT connection to ");
  Serial.print(mqtt_server);
  Serial.print(":");
  Serial.print(mqtt_port);
  Serial.println("...");
  
  // Check WiFi first
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected, cannot connect to MQTT");
    return false;
  }
  
  String clientId = String(ROBOT_CLIENT_PREFIX) + "-";
  clientId += String(random(0xffff), HEX);
  
  Serial.print("📋 Client ID: ");
  Serial.println(clientId);
  
  // Set shorter timeouts for faster failure detection
  mqttClient.setSocketTimeout(10);
  
  if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
    Serial.println("✅ Connected to MQTT Broker!");
    
    // Subscribe to topic
    if (mqttClient.subscribe(topic_sub)) {
      Serial.println("📡 Subscribed to: " + String(topic_sub));
    } else {
      Serial.println("❌ Failed to subscribe to topic");
    }
    
    // Send connection notification
    String connectionMsg = "🤖 ";
    connectionMsg += ROBOT_DISPLAY_NAME;
    connectionMsg += " connected to MQTT";
    sendNotification(connectionMsg.c_str());
    
    return true;
  } else {
    int mqttState = mqttClient.state();
    Serial.print("❌ MQTT connection failed, rc=");
    Serial.print(mqttState);
    
    // Print human-readable error
    switch (mqttState) {
      case -4: Serial.println(" (MQTT_CONNECTION_TIMEOUT)"); break;
      case -3: Serial.println(" (MQTT_CONNECTION_LOST)"); break;
      case -2: Serial.println(" (MQTT_CONNECT_FAILED)"); break;
      case -1: Serial.println(" (MQTT_DISCONNECTED)"); break;
      case 1: Serial.println(" (MQTT_CONNECT_BAD_PROTOCOL)"); break;
      case 2: Serial.println(" (MQTT_CONNECT_BAD_CLIENT_ID)"); break;
      case 3: Serial.println(" (MQTT_CONNECT_UNAVAILABLE)"); break;
      case 4: Serial.println(" (MQTT_CONNECT_BAD_CREDENTIALS)"); break;
      case 5: Serial.println(" (MQTT_CONNECT_UNAUTHORIZED)"); break;
      default: Serial.println(" (Unknown error)"); break;
    }
    
    return false;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  esp_task_wdt_reset(); // Reset watchdog when processing MQTT messages
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  // Serial.println("📩 MQTT Message received on topic: " + String(topic));
  // Serial.println("Message: " + message);
  
  // Store command in queue for receiveMessage to process
  commandQueue = message;
  lastCommandTime = millis();
  
  // Also set flags for immediate interrupt commands
  message.trim();
  handleCommand(message);
}

// =================== Command Handling Function ===================
bool handleCommand(String command) {
  esp_task_wdt_reset(); // Reset watchdog when handling commands
  
  command.trim();
  float voltage = readVoltage();
  
  if (voltage > volt) {
    if (command == "start") {
      // Serial.println("start");
      Start_Robot();
      startManualMode();
      return true;
    } 
    else if (command == "stop") {
      Stop_Robot();
      currentMode = "Stopped";
      // Serial.println("stop");
      sendNotification("🛑 Robot stopped");
      return true;
    }
    else if (command == "forward") {
      // Serial.println("forward");
      sendNotification("The robot in forward mode.");
      Start_Robot();
      startManualMode();
      return true;
    }
    else if (command == "back") {
      Backward_Robot();
      sendNotification("The robot in backward mode.");
      return true;
    }
    else if (command == "status") {
      sendStatus(voltage);
      return true;
    }
    else if (command == "restart") {
      sendNotification("🔄 Restarting robot");
      delay(100);
      sendNotification("Please wait 5 minutes before operating.");
      restartESP();
      return true;
    }
    else if (command.startsWith("schedule_")) {
      command = command.substring(9);
      esp_task_wdt_reset();
      uint8_t parts[9] = {0};
      int index = 0;

      while (command.length() > 0 && index < 9) {
        esp_task_wdt_reset();
        int sepIndex = command.indexOf('_');
        String token;

        if (sepIndex == -1) {
          token = command;
          command = "";
        } else {
          token = command.substring(0, sepIndex);
          command = command.substring(sepIndex + 1);
        }

        token.trim();
        int val = token.toInt();
        parts[index++] = (val >= 0) ? val : 0;
      }

      if (index == 9) {
        bool days[7];
        for (int i = 0; i < 7; i++) {
          days[i] = (parts[2 + i] > 0);
        }
        setSchedule(parts[0], parts[1], days);
      } else {
        Serial.println("❌ Error: Invalid schedule format. Expected 9 values.");
      }
    }
    else if (command.startsWith("set_time_")) {
      command = command.substring(9);
      uint16_t timeValues[7] = {0, 0, 0, 0, 0, 0, 0};
      int index = 0;

      while (command.length() > 0 && index < 7) {
        int sepIndex = command.indexOf('_');
        String token;
        esp_task_wdt_reset();
        if (sepIndex == -1) {
          token = command;
          command = "";
        } else {
          token = command.substring(0, sepIndex);
          command = command.substring(sepIndex + 1);
        }
        esp_task_wdt_reset();
        token.trim();
        if (token.length() > 0 && token.toInt() >= 0) {
          timeValues[index] = token.toInt();
        } else {
          timeValues[index] = 0;
        }
        index++;
      }

      while (index < 7) {
        timeValues[index++] = 0;
      }

      setPLCTime(timeValues[0], timeValues[1], timeValues[2], timeValues[3], timeValues[4], timeValues[5], timeValues[6]);
    }
  } else {
    String message = "⚠️ Low charge: " + String(voltage, 3);
    sendAlert(message.c_str());
  }
  return false;
}

// =================== Manual Mode Function ===================
void startManualMode() {
  // Serial.println("Start3");
  esp_task_wdt_reset();

  
  stopCommandFlag = false;
  restartCommandFlag = false;

  if (!readEmergencyButton()){
    sendAlert("❗❗🚨⚠️\n\nError: Emergency button is activated.\n\n🚨⚠️❗❗");
    return;
  }
  // Serial.println("Start4");
  
  float time = millis();
    if (readRun() == 226) {
        sendAlert("🚨CRITICAL: Robot control check required! Run status: 226ST - Please inspect robot immediately 🔧🔍");
    }
    if (readRun() == 1) {
        sendNotification("Time to power up our energy savings! 🚀");
        currentMode = "Running.";
    }
  // Serial.println(readRun());
  while(readRun() == 1){
    esp_task_wdt_reset(); // Reset watchdog frequently in manual mode
    
    if((millis() - time) >= movementTimeout){
      sendAlert("❗❗🚨⚠️\n\nWarning: Movement time out and the robot not in the charging point.\n\n🚨⚠️❗❗");
      return;
    }
    
    if(readNoMotion() == 1){
      sendAlert("❗❗🚨⚠️\n\nWarning: Robot is unable to continue movement.🚨⚠️❗❗");
      return;
    }   
    
    if(!receiveMessage()){
      // Serial.println("🛑 Manual mode interrupted by command or emergency");
      return;
    }
    
    delay(10); // Small delay to prevent overwhelming the CPU
        if(readStop() == 1){
          sendNotification("Stop button is activated.✅✅");
          delay(100);
          sendCycleUpdate("+0.5");
          break;
        }
        else if (!readEmergencyButton()){
          sendNotification("Emergency button is activated.✅✅");
          delay(100);
          sendCycleUpdate("+0.5");
          break;
        }
        else if (!readReverseLimit()){
          sendNotification("The robot has completed one cycle.✅✅");
          delay(10);
          sendCycleUpdate("+1");
          // sendCycleUpdate("+0.5");
          // delay(10);
          // sendCycleUpdate("+0.5");
          currentMode = "Stopped";
          break;
        }

  }
currentMode = "Stopped";
sendNotification("This Robot Out from Runnning Mode to Stopped Mode.");
}

// =================== Receive Message Function ===================
bool receiveMessage() {
  esp_task_wdt_reset(); 
  
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
  
  if (commandQueue != "" && (millis() - lastCommandTime < COMMAND_TIMEOUT)) {
    String receivedCommand = commandQueue;
    commandQueue = "";
    
    receivedCommand.trim();
    // Serial.println("Processing queued command: " + receivedCommand);
    
    if (receivedCommand == "start" || receivedCommand == "stop" ||
        receivedCommand == "forward" || receivedCommand == "backward" ||
        receivedCommand == "restart") {
      handleCommand(receivedCommand);
      return false;
    } 
    else if (receivedCommand == "status") {
      float voltage = readVoltage();
      sendStatus(voltage);
      return true;
    } 
  }
  
  if (stopCommandFlag) {
    stopCommandFlag = false;
    handleCommand("stop");
    return false;
  }
  
  if (restartCommandFlag) {
    restartCommandFlag = false;
    handleCommand("restart");
    return false;
  }
  
  if (!readEmergencyButton()) {
    // Serial.println("🛑 Emergency button activated during manual mode");
    return false;
  }
  
  return true;
}

//=================== Voltage Reader ====================
// float readVoltage() {
//   // Always return a safe default voltage
//   return 26.2;
// }

float readVoltage() {
  esp_task_wdt_reset(); 
  if (!ina.begin()) {
    return 26.2;
  }
  esp_task_wdt_reset(); 
  delay(20);
  return ina.getBusVoltage();
}

//=================== Modbus Functions ====================
uint8_t readRun() {
  esp_task_wdt_reset(); 
  if (node.readCoils(Mstart , 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readStart() {
  esp_task_wdt_reset();
  if (node.readCoils(Mstart + 11, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readStop() {
  esp_task_wdt_reset();
  if (node.readCoils(Mstart + 13, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readManualAuto() {
  esp_task_wdt_reset();
  if (node.readCoils(Mstart + 515, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readNoMotion() {
  esp_task_wdt_reset();
  if (node.readCoils(Mstart + 71, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readForwardLimit() {
  esp_task_wdt_reset();
  if (node.readDiscreteInputs(Xstart + 3, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readReverseLimit() {
  esp_task_wdt_reset();
  if (node.readDiscreteInputs(Xstart + 4, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readEmergencyButton() {
  esp_task_wdt_reset();
  if (node.readDiscreteInputs(Xstart + 4, 0) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

//=================== PLC Command Functions ====================
void setPLCTime(uint16_t year, uint16_t weekday, uint16_t month, uint16_t day, uint16_t hour, uint16_t minute, uint16_t second) {
  esp_task_wdt_reset();
  uint16_t dataToWrite[7] = {year, weekday, month, day, hour, minute, second};

  for (uint8_t i = 0; i < 7; i++) {
    node.setTransmitBuffer(i, dataToWrite[i]);
  }

  node.writeMultipleRegisters(Dstart + 60, 7);
  node.writeSingleCoil(Mstart + 100, 1);
  delay(500);
  node.writeSingleCoil(Mstart + 100, 0);
}

void Start_Robot() {
  esp_task_wdt_reset();
  int start_coil = Mstart + 11;
  node.writeSingleCoil(start_coil, 1);
  delay(100);
  node.writeSingleCoil(start_coil, 0);
  // Serial.println("Start2");
}

void Stop_Robot() {
  esp_task_wdt_reset();
  int stop_coil = Mstart + 13;
  node.writeSingleCoil(stop_coil, 1);
  delay(500);
  node.writeSingleCoil(stop_coil, 0);
  if (readRun() == 226) {
      sendAlert("🚨CRITICAL: Robot control check required! Run status: 226SP - Please inspect robot immediately 🔧🔍");
  }
}

void Backward_Robot() {
  // First call Start_Robot()
  esp_task_wdt_reset();
  Start_Robot();
  
  // Wait 1 second (1000 ms)
  // delay(10);
  esp_task_wdt_reset();
  // Write 0 to Xstart + 4
  node.writeSingleCoil(Mstart + 16, 1);
  delay(10);
  node.writeSingleCoil(Mstart + 16, 0);
 
}

void set_Wind_speed_return(float value) {
  esp_task_wdt_reset();
  node.writeSingleRegister(Dstart + 512 ,value);
}

void set_Wind_speed_Start(float value) {
  esp_task_wdt_reset();
  node.writeSingleRegister(Dstart + 510 ,value);
}
void setSchedule(uint8_t hour, uint8_t minute, bool days[7]) {
  // 1. Set hour to D500

  uint8_t result = node.writeSingleRegister(Dstart + 500, hour);
  if (result != node.ku8MBSuccess) {
    Serial.println("❌ Failed to write Hour (D500)");
  }

  // 2. Set minute to D501
  result = node.writeSingleRegister(Dstart + 501, minute);
  if (result != node.ku8MBSuccess) {
    Serial.println("❌ Failed to write Minute (D501)");
  }

  // 3. Set day flags from M520 (Sunday) to M526 (Saturday)
  for (uint8_t i = 0; i < 7; i++) {
    uint16_t coilValue = days[i] ? 1 : 0;  // ON/OFF
    result = node.writeSingleCoil(Mstart + 520 + i, coilValue);

    if (result != node.ku8MBSuccess) {
      Serial.print("❌ Failed to set day M");
      Serial.println(520 + i);
    }
  }

  // Serial.println("✅ Schedule set successfully.");
}

//=================== MQTT Message Functions ====================
void sendMessage(const char* message, const char* type) {
  esp_task_wdt_reset(); 
  
  // Deduplication - prevent sending the same message multiple times
  static String lastMessage = "";
  static String lastType = "";
  static unsigned long lastMessageTime = 0;
  
  // Create current message identifier
  String currentMessage = String(message);
  String currentType = String(type);
  
  // Check if this is the same message as the last one sent within a short time window
  if (currentMessage == lastMessage && currentType == lastType) {
    if (millis() - lastMessageTime < 3000) { // 3 second cooldown for identical messages
      // Serial.println("🔄 Skipping duplicate message: " + currentMessage);
      return;
    }
  }
  
  if (mqttClient.connected()) {
    DynamicJsonDocument doc(512);
    doc["name"] = ROBOT_NAME;
    doc["section"] = ROBOT_SECTION;
    doc["type"] = type;
    doc["message"] = message;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    mqttClient.publish(topic_pub, jsonString.c_str());
    // Serial.println("📤 MQTT Message sent (" + String(type) + "): " + String(message));
    
    // Update last sent message info
    lastMessage = currentMessage;
    lastType = currentType;
    lastMessageTime = millis();
  } 
  // else {
  //   Serial.println("❌ MQTT not connected, message not sent: " + String(message));
  // }
}

void sendStatus(float voltage) {
  esp_task_wdt_reset();
  voltage = readVoltage();
  
  DynamicJsonDocument doc(512);
  doc["name"] = ROBOT_NAME;
  doc["section"] = ROBOT_SECTION;
  doc["type"] = "status";
  
  String statusString = "Voltage: " + String(voltage, 3) + "V | ";
  statusString += "Mode: " + currentMode;
  
  if (currentMode == "Just move" && lastDirection != "") {
    statusString += " | Direction: " + lastDirection;
  }
  
  doc["status"] = statusString;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  if (mqttClient.connected()) {
    mqttClient.publish(topic_pub, jsonString.c_str());
    // Serial.println("📊 Status sent: " + jsonString);
  }
}

void sendNotification(const char* message) {
  sendMessage(message, "notification");
}

void sendAlert(const char* message) {
  sendMessage(message, "alert");
}

void sendCycleUpdate(const char* message) {
  sendMessage(message, "cycle_update");
}

void restartESP() {
  delay(1000);
  ESP.restart();
}

void setupWDT() {
  static bool wdtInitialized = false;
  
  if (!wdtInitialized) {
    const esp_task_wdt_config_t wdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true
    };
    
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    wdtInitialized = true;
    // Serial.println("✅ Watchdog initialized");
  }
}

static const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Omega Robots - QAIA</title>
  <style>
    html,body{height:100%;margin:0}
    body{display:grid;place-items:center;font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,Cantarell,Arial,sans-serif}
    h1{color:Blue;font-size:clamp(2rem,6vw,5rem);margin:0;text-align:center}
  </style>
</head>
<body>
  <h1>Omega Robots on IRC</h1>
</body>
</html>
)HTML";

String Robot_HTML() {
  return FPSTR(PAGE_HTML);
}

void setupOTA() {
  ArduinoOTA.setHostname(ROBOT_CLIENT_PREFIX);
  esp_task_wdt_reset(); 

  ArduinoOTA
    .onStart([]() {
      esp_task_wdt_delete(NULL);
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      // Serial.println("\nEnd");
      esp_task_wdt_add(NULL);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      esp_task_wdt_reset();
      // Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR)     Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  otaReady = true;
  Serial.printf("[OTA] Ready on %s:3232  Hostname: %s\n", WiFi.localIP().toString().c_str(), ROBOT_CLIENT_PREFIX);
}