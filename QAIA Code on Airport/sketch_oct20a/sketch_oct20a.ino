// Code V1.1: 
// I move the function Start_Robot() from startManualMode() to handleCommand function
// I Fix function readVoltage() 
// I have fix OTA to check go to the ip address 
// 
#include <Wire.h>
#include <INA226.h>
#include <WiFi.h>
#include "esp_task_wdt.h"  // Watchdog library
#include <ArduinoOTA.h>
#include <ModbusMaster.h>
#include <WebServer.h>
ModbusMaster node;

#define RXD2 16
#define TXD2 17
#define Dstart 4096  
#define Mstart 2048
#define Xstart 1024
// ================== WATCHDOG CONFIGURATION ==================
#define WDT_TIMEOUT 1000  // Watchdog timeout in seconds

// ================== WIFI CONFIGURATION ==================
const char* ssid = "Omega_KR_QAIA2.4G";
const char* password = "Omega@2022";

// const char* ssid = "Zain Fiber_98AE";
// const char* password = "9BRJnmKJ";


// Static IP Configuration
IPAddress local_IP(192, 168, 20, 129); // <- Change to 193 for the second device
IPAddress gateway(192, 168, 20, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// ================== HTML PAGE ================== 
WebServer HTML(80);

// ================== TCP COMMUNICATION ==================
WiFiServer server(8888);
WiFiClient clientToPi;

// Raspberry Pi IP and Port
const char* raspi_ip = "192.168.20.128"; // Change if different
const int raspi_port = 8888;

// ================== TIME CONTROL VARIABLES ==================
unsigned long lastVoltageCheckTime = 0;
unsigned long startTime = 0;

// ================== MOVEMENT & MODE CONTROL ==================
int cycleCount = 0;        // Tracks number of cycles in auto mode
float totalCycles = 0;     // Tracks total cycles completed
String currentMode = "stopped"; // Modes: stopped, auto, manual, moving
String lastDirection = "";      // Tracks last movement direction

// ================== LIMITS AND THRESHOLDS ==================

const unsigned long movementTimeout = 28 * 60 * 1000; // Movement timeout (10 minutes)
const unsigned long voltageCheckInterval = 20 * 60 * 2000; // Voltage check interval
float volt = 21; // Voltage threshold (adjust as needed)

// ================== SENSOR ==================
INA226 ina(0x40); // INA226 current sensor object

// ================== FUNCTION DECLARATIONS ==================
void connectToWiFi();

// ================== OTA ==================
bool otaReady = false;
// =================== BOARD INITIALIZATION ===================
void setup() {
  Serial.begin(115200);
  setupWDT();
  delay(1000);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  node.begin(1, Serial2);

  connectToWiFi();  // Waits for WiFi to connect before OTA setup
  server.begin(); // TCP server

    if (WiFi.status() == WL_CONNECTED) {
    setupOTA();        // ✅ فعّل OTA فقط إذا متصل
  } else {
    otaReady = false;  // بدون WiFi: لا تفعّل OTA الآن
  }


  // =================== INA226 SENSOR ===================
  Wire.begin(21, 22);  // Initialize I2C with custom pins

  if (!ina.begin()) {
    Serial.println("INA226 init failed!");
  } else {
    Serial.println("INA226 init Success!");
  }

  ina.setMaxCurrentShunt(10.0, 0.1);  // Set max current to 10A with 0.1Ω shunt

  // ================== HTML PAGE ================== 
  
    HTML.on("/", []() {
    HTML.send(200, "text/html", Robot_HTML());
  });

  HTML.begin();
  Serial.println("[HTTP] Server started.");

  
}

// =================== MAIN LOOP ===================
void loop() {
  esp_task_wdt_reset();  // ✅ Keep watchdog alive
  ArduinoOTA.handle();
  HTML.handleClient();


  // ---------- Handle incoming TCP command ----------
  WiFiClient client = server.available();
  if (client && client.connected()) {
    if (client.available()) {
      String msg = client.readStringUntil('\n');
      handleCommand(msg); 
      client.stop();  // Close connection after reading
    }
  }

  // ---------- Voltage monitoring ----------
  float voltage = readVoltage();
  if (voltage < volt) {
    unsigned long currentMillis = millis();
    if (currentMillis - lastVoltageCheckTime >= voltageCheckInterval) {
      sendMessage("123/");  // Send low voltage alert
      lastVoltageCheckTime = currentMillis;
    }
  }
    // ---------- OTA  ----------
    if (WiFi.status() == WL_CONNECTED) {
        if (!otaReady) setupOTA();  // ✅ تهيئة تلقائية بعد reconnect
        ArduinoOTA.handle();
      } else {
        otaReady = false;           // سقط الاتصال؟ فعّل تهيئة عند رجوعه
      }
  // ---------- WiFi reconnect logic ----------
  static unsigned long wifiRetryStart = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiRetryStart == 0)
      wifiRetryStart = millis();

    if (millis() - wifiRetryStart < 2 * 60 * 1000) {
      WiFi.begin(ssid, password);
      esp_task_wdt_reset(); 
      delay(1000);  // Short wait before retry
    } 
  } 
  else {
    wifiRetryStart = 0;  // Reset retry timer when connected
  }


  if(readRun() == 1){
      startManualMode();
  }

}

void connectToWiFi() {
  // if (!WiFi.config(local_IP, gateway, subnet, dns)) {
  //   Serial.println("❌ Failed to configure static IP");
  // }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long startAttemptTime = millis();
  const unsigned long wifiTimeout = 2 * 60 * 1000;  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeout) {
    esp_task_wdt_reset();
    delay(500);
    Serial.print(".");
    
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n⚠️ WiFi connection timeout after 4 minutes. Continuing without WiFi.");
  }
}

// =================== Command Handling Function ===================
bool handleCommand(String command) {
  command.trim();
  // float voltage = readVoltage();
  float voltage = readVoltage();
  if (voltage > volt) {
    esp_task_wdt_reset();
    if (command == "start") {
      Serial.println("start");
      Start_Robot();
      startManualMode();
      esp_task_wdt_reset();
      return true;
    } 
    else if (command == "stop") {
      Stop_Robot();
      currentMode = "Stopped";
      Serial.println("stop");
      sendMessage("7891/");
      return true;
    }
    else if (command == "forward") {
      Serial.println("Forward");
      return true;
    }
    else if (command == "backward") {
      Serial.println("Backward");
      return true;
    }
    else if (command == "status") {
      sendStatus(voltage); // Send status
      return true;
    }
    else if (command == "restart") {
      sendMessage("3343/");
      delay(100);
      sendMessage("Please wait 5 minutes before operating.");
      restartESP();
      return true;
    }
    else if (command.startsWith("schedule_")) {
      command = command.substring(9);  // Remove "schedule_"
    // schedule_9_15_1_1_1_1_1_0_0
      uint8_t parts[9] = {0};  // HH, MM, SUN to SAT
      int index = 0;

      while (command.length() > 0 && index < 9) {
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

        parts[index++] = (val >= 0) ? val : 0;  // Default to 0 if invalid
      }

      // If valid format
      if (index == 9) {
        bool days[7];
        for (int i = 0; i < 7; i++) {
          days[i] = (parts[2 + i] > 0);  // Convert to bool
        }

        setSchedule(parts[0], parts[1], days);  // hour, minute, days
      } else {
        Serial.println("❌ Error: Invalid schedule format. Expected 9 values.");
      }
     }
       else if (command.startsWith("set_time_")) {
        command = command.substring(9);  // Remove "set_time_" prefix

        uint16_t timeValues[7] = {0, 0, 0, 0, 0, 0, 0};  // Defaults to 0
        int index = 0;

        while (command.length() > 0 && index < 7) {
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

          // Convert to integer, default to 0 if invalid
          if (token.length() > 0 && token.toInt() >= 0) {
            timeValues[index] = token.toInt();
          } else {
            timeValues[index] = 0;
          }

          index++;
        }

        // Fill missing values with 0 if less than 7 parts
        while (index < 7) {
          timeValues[index++] = 0;
        }

        // Set time on PLC
        setPLCTime(
          timeValues[0], // Year
          timeValues[1], // Weekday
          timeValues[2], // Month
          timeValues[3], // Day
          timeValues[4], // Hour
          timeValues[5], // Minute
          timeValues[6]  // Second
        );
      }
      
      } 
      else{
        String message = "55556/" + String(voltage, 3);
        sendMessage(message.c_str());
      }
      return false;
}



// =================== Manual Mode Function ===================
void startManualMode() {
    // Serial.println("Start3");
  esp_task_wdt_reset();
  if (!readEmergencyButton()){
    sendMessage("❗❗🚨⚠️\n\nError: Emergency button is activated.\n\n🚨⚠️❗❗");
    return ;
  }
    // Serial.println("Start4");
  
  float time = millis();
  // Serial.println(readRun());
  while(readRun() == 1){
    esp_task_wdt_reset();
      // Serial.println("Start5");
        esp_task_wdt_reset(); 
        if((millis() - time) >= movementTimeout){
            sendMessage("❗❗🚨⚠️\n\nWarning: Movement time out and the robot not in the charging point.\n\n🚨⚠️❗❗");
            return ;
        }
        if(readNoMotion() == 1){
            sendMessage("❗❗🚨⚠️\n\nWarning: Robot is unable to continue movement.🚨⚠️❗❗");
            return ;
        }   
        if(!receiveMessage()){
            return ;
        }
        esp_task_wdt_reset();
    }
    currentMode = "Stoped.";
    if(readStop() == 1){
        sendMessage("Stop button is activated.✅✅");
        delay(100);
        sendMessage("@1");
    }
    else if (!readEmergencyButton() == 1){
        sendMessage("Emergency button is activated.✅✅");
        delay(100);
        sendMessage("@1");
    }
    else if (!readReverseLimit() == 1){
        sendMessage("The robot has completed one cycle.✅✅");
        delay(100);
        sendMessage("@1");
        delay(100);
        sendMessage("@1");
        currentMode = "Stopped";
    }
    esp_task_wdt_reset();
}


//=================== التحكم في المحرك ====================

// --------------------------------------------------
// Modbus Functions
// --------------------------------------------------
uint8_t readRun() {
  esp_task_wdt_reset(); 
  if (node.readCoils(Mstart , 1) == node.ku8MBSuccess) {
    
    uint8_t val = node.getResponseBuffer(0) & 0x01;

    return (val == 0 || val == 1) ? val : 226;

  }

  return 226;

}


uint8_t readStart() {
  if (node.readCoils(Mstart + 11, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readStop() {
  if (node.readCoils(Mstart + 13, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}


uint8_t readManualAuto() {
  if (node.readCoils(Mstart + 515, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readNoMotion() {
  if (node.readCoils(Mstart + 71, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readForwardLimit() {
  if (node.readDiscreteInputs(Xstart + 3, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

uint8_t readReverseLimit() {
  if (node.readDiscreteInputs(Xstart + 4, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}

// Reads X2 - Emergency Button (Discrete Input)
uint8_t readEmergencyButton() {
  if (node.readDiscreteInputs(Xstart + 2, 1) == node.ku8MBSuccess) {
    uint8_t val = node.getResponseBuffer(0) & 0x01;
    return (val == 0 || val == 1) ? val : 226;
  }
  return 226;
}
// --------------------------------------------------
// PLC Command Functions
// --------------------------------------------------

void setPLCTime(uint16_t year, uint16_t weekday, uint16_t month,
                uint16_t day, uint16_t hour, uint16_t minute, uint16_t second) {
  uint16_t dataToWrite[7] = {
    year,     // D60
    weekday,  // D61 (1 = Monday, 7 = Sunday)
    month,    // D62
    day,      // D63
    hour,     // D64
    minute,   // D65
    second    // D66
  };

  // Load values into transmit buffer
  for (uint8_t i = 0; i < 7; i++) {
    node.setTransmitBuffer(i, dataToWrite[i]);
  }

  // Write the 7 time values to D60–D66
  node.writeMultipleRegisters(Dstart + 60, 7);

  // Pulse M100 to apply the time
  node.writeSingleCoil(Mstart + 100, 1);
  delay(500);
  node.writeSingleCoil(Mstart + 100, 0);
}



void Start_Robot() {
  int start_coil = Mstart + 11;
  node.writeSingleCoil(start_coil, 1);
  delay(500);
  node.writeSingleCoil(start_coil, 0);
  // Serial.println("Start2");
}

void Stop_Robot() {
  int stop_coil = Mstart + 13;
  node.writeSingleCoil(stop_coil, 1);
  delay(500);
  node.writeSingleCoil(stop_coil, 0);
}

void Backward_Robot() {
  int backward_coil = Mstart + 1;
  node.writeSingleCoil(backward_coil, 1);
  delay(500);
  node.writeSingleCoil(backward_coil, 1);
}

void set_Wind_speed_return(float value) {
  node.writeSingleRegister(Dstart + 512 ,value);
}

void set_Wind_speed_Start(float value) {
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
    uint16_t coilValue = days[i] ? 0xFF00 : 0x0000;  // ON/OFF
    result = node.writeSingleCoil(Mstart + 520 + i, coilValue);

    if (result != node.ku8MBSuccess) {
      Serial.print("❌ Failed to set day M");
      Serial.println(520 + i);
    }
  }

  Serial.println("✅ Schedule set successfully.");
}

//=================== Voltage Reader ====================
// float readVoltage() {
//   esp_task_wdt_reset();
//   delay(2);
//   esp_task_wdt_reset();
//   return ina.getBusVoltage();
// }
float readVoltage() {
  
  if (!ina.begin()) {
    return 26.2;
  }
  delay(20);
  return ina.getBusVoltage();
}
//=================== إرسال الرسائل ====================
void sendMessage(const char* message) {
  esp_task_wdt_reset(); 
  if (clientToPi.connect(raspi_ip, raspi_port)) {
    clientToPi.println(message);
 
    clientToPi.stop();
  } 
}


bool receiveMessage() {
  esp_task_wdt_reset(); 
  WiFiClient client = server.available();  // Check for incoming client

  if (client && client.available()) {
    String receivedCommand = client.readStringUntil('\n');
    receivedCommand.trim();
    if (receivedCommand == "start" || receivedCommand == "stop" ||
        receivedCommand == "forward" || receivedCommand == "backward" ||
        receivedCommand == "restart") {
        handleCommand(receivedCommand);
      return false  ;// Return true if command handled successfully
    } 
    else if (receivedCommand == "status") {
      float voltage = readVoltage();
      sendStatus(voltage);
      return true;
    } 
    else {
      return true;  
    }
  }

  return true;  // No client or no message — no problem
}


void sendStatus(float voltage) {
  voltage = ina.getBusVoltage();
  char message[128];
  snprintf(message, sizeof(message), "1/%.3f2/%s", voltage, currentMode.c_str());


  char cycleInfo[32];
  cycleInfo[0] = '4';
  cycleInfo[1] = '/';
  cycleInfo[2] = '\0'; // null-terminator
  strncat(message, cycleInfo, sizeof(message) - strlen(message) - 1);

  if (currentMode == "Just move") {
    char dirInfo[32];
    snprintf(dirInfo, sizeof(dirInfo), "5/%s", lastDirection.c_str());
    strncat(message, dirInfo, sizeof(message) - strlen(message) - 1);
  }

  sendMessage(message);
}

void restartESP() {
  delay(1000);
  ESP.restart();
}

void setupWDT() {
  const esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // All cores
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);  // Pass struct pointer
  esp_task_wdt_add(NULL);  // Add current task (main loop)
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
  <h1>Omega Robots on QAIA</h1>
</body>
</html>
)HTML";

// Function that returns the HTML page
String Robot_HTML() {
  return FPSTR(PAGE_HTML); // safely construct a String from PROGMEM
}

void setupOTA() {
  ArduinoOTA.setHostname("Omega_Robot");
  esp_task_wdt_reset(); 
  // ArduinoOTA.setPassword("your_secure_password"); // أنصح بها على شبكة مشتركة
  // ArduinoOTA.setPort(3232); // اختياري

  ArduinoOTA
    .onStart([]() {
      // أثناء OTA: فكّ مراقبة WDT عن اللوب لتجنّب الريست
      esp_task_wdt_delete(NULL);
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      // بعد الانتهاء: أعد إضافة المهمة إلى WDT
      esp_task_wdt_add(NULL);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      // أطعم الـWDT باستمرار أثناء النقل
      esp_task_wdt_reset();
      Serial.printf("Progress: %u%%\r", (progress * 100) / total);
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
  Serial.printf("[OTA] Ready on %s:3232  Hostname: Robot\n",
                WiFi.localIP().toString().c_str());
}
