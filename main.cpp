#define BLYNK_TEMPLATE_ID "TMPL6hW-Y-n7x"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "lHjkAQlbq8psFUqCrGsuOCTTKqVUTn_X"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Initialize LCD (address 0x27, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x27, 16, 2);

char ssid[] = "SLT_FIBRE";
char pass[] = "abcd1234";

// Wireless Communication Setup
WebServer server(80);              // HTTP server on port 80
WiFiUDP udp;                       // UDP for faster gesture commands
unsigned int localUdpPort = 4210;  // Local port for UDP
char incomingPacket[255];          // Buffer for incoming UDP packets
IPAddress pythonClientIP;          // Store Python client IP
bool pythonClientConnected = false;

#define FAN_PWM_PIN 25    // Blue wire (PWM control)
#define FAN_POWER_PIN 26  // Controls relay for fan power
#define LCD_SDA_PIN 21    // LCD SDA (I2C Data)
#define LCD_SCL_PIN 22    // LCD SCL (I2C Clock)

// BUZZER PIN
#define BUZZER_PIN 13     // Buzzer positive pin (connect negative to GND)

// LED 1 - System Status LED (WiFi + Blynk connection)
#define LED1_RED_PIN 32
#define LED1_GREEN_PIN 33
#define LED1_BLUE_PIN 4

// LED 2 - Fan Speed Activity LED
#define LED2_RED_PIN 16
#define LED2_GREEN_PIN 17
#define LED2_BLUE_PIN 18

// KY-040 Rotary Encoder pins
#define ENCODER_CLK_PIN 19   // CLK (Clock)
#define ENCODER_DT_PIN 23    // DT (Data)
#define ENCODER_SW_PIN 5     // SW (Switch/Button)

int pwmValue = 0;
int currentFanSpeed = 0;
bool systemOnline = false;
unsigned long fanActivityTime = 0;

// Rotary encoder variables
volatile int encoderPos = 0;
volatile bool encoderChanged = false;
int lastEncoderPos = 0;
bool lastCLK = HIGH;
bool buttonPressed = false;
unsigned long lastButtonPress = 0;
bool encoderControlActive = false;
unsigned long lastEncoderActivity = 0;

// Control source tracking (now includes GESTURE)
enum ControlSource {
  CONTROL_BLYNK,
  CONTROL_ENCODER,
  CONTROL_GESTURE
};
ControlSource lastControlSource = CONTROL_BLYNK;

// Connection status tracking for buzzer feedback
bool wasWiFiConnected = false;
bool wasBlynkConnected = false;

// GESTURE CONTROL VARIABLES
String gestureBuffer = "";
unsigned long lastGestureTime = 0;
bool gestureSystemActive = false;

// DS18B20 Temperature Sensor Setup
#define ONE_WIRE_BUS 15  // GPIO 15 connected to S pin of DS18B20 module
#define TEMPERATURE_PRECISION 12

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Timer for sending temperature data
BlynkTimer timer;

// WIRELESS COMMUNICATION FUNCTIONS
void setupWebServer() {
  // Handle gesture commands via HTTP POST
  server.on("/gesture", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      DynamicJsonDocument doc(1024);
      
      if (deserializeJson(doc, body) == DeserializationError::Ok) {
        if (doc.containsKey("gesture") && doc.containsKey("speed")) {
          String gesture = doc["gesture"];
          int speed = doc["speed"];
          
          // Store Python client IP
          pythonClientIP = server.client().remoteIP();
          pythonClientConnected = true;
          
          processGestureCommand(speed);
          
          // Send response
          DynamicJsonDocument response(512);
          response["status"] = "success";
          response["gesture"] = gesture;
          response["speed"] = speed;
          response["timestamp"] = millis();
          
          String responseStr;
          serializeJson(response, responseStr);
          
          server.send(200, "application/json", responseStr);
          
          Serial.print("📡 HTTP Gesture: ");
          Serial.print(gesture);
          Serial.print(" → ");
          Serial.print(speed);
          Serial.print("% from ");
          Serial.println(pythonClientIP);
        } else {
          server.send(400, "application/json", "{\"error\":\"Invalid gesture data\"}");
        }
      } else {
        server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"No data received\"}");
    }
  });

  // Handle status requests
  server.on("/status", HTTP_GET, []() {
    DynamicJsonDocument doc(512);
    doc["fanSpeed"] = currentFanSpeed;
    doc["systemOnline"] = systemOnline;
    doc["blynkConnected"] = Blynk.connected();
    doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
    doc["gestureSystemActive"] = gestureSystemActive;
    doc["lastControlSource"] = (lastControlSource == CONTROL_GESTURE) ? "GESTURE" : 
                              (lastControlSource == CONTROL_ENCODER) ? "ENCODER" : "BLYNK";
    doc["uptime"] = millis();
    doc["ip"] = WiFi.localIP().toString();
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });

  // Handle CORS preflight requests
  server.on("/gesture", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(200);
  });

  server.on("/status", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(200);
  });

  // Add CORS headers to all responses
  server.onNotFound([]() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Not Found");
  });

  server.begin();
  Serial.println("🌐 HTTP Server started on port 80");
  Serial.print("📡 Access at: http://");
  Serial.println(WiFi.localIP());
}

void setupUDP() {
  udp.begin(localUdpPort);
  Serial.print("🔗 UDP Server started on port ");
  Serial.println(localUdpPort);
}

void handleUDPGestures() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    
    // Store Python client IP
    pythonClientIP = udp.remoteIP();
    pythonClientConnected = true;
    
    // Parse UDP message: "GESTURE:speed" format
    String message = String(incomingPacket);
    if (message.startsWith("GESTURE:")) {
      int colonIndex = message.indexOf(':');
      if (colonIndex != -1) {
        String speedStr = message.substring(colonIndex + 1);
        int speed = speedStr.toInt();
        
        if (speed >= 0 && speed <= 100) {
          processGestureCommand(speed);
          
          // Send UDP acknowledgment
          String ack = "ACK:" + String(speed);
          udp.beginPacket(udp.remoteIP(), udp.remotePort());
          udp.print(ack);
          udp.endPacket();
          
          Serial.print("🔗 UDP Gesture: ");
          Serial.print(speed);
          Serial.print("% from ");
          Serial.println(udp.remoteIP());
        }
      }
    }
  }
}

void sendWirelessHeartbeat() {
  static unsigned long lastHeartbeat = 0;
  if (pythonClientConnected && (millis() - lastHeartbeat > 10000)) { // Every 10 seconds
    lastHeartbeat = millis();
    
    // Send UDP heartbeat
    String heartbeat = "HEARTBEAT:" + String(currentFanSpeed) + ":" + 
                      (Blynk.connected() ? "ONLINE" : "OFFLINE");
    udp.beginPacket(pythonClientIP, 4211); // Send to Python on port 4211
    udp.print(heartbeat);
    udp.endPacket();
  }
}

// BUZZER FUNCTIONS
void buzzerBeep(int duration = 100) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void buzzerDoubleBeep() {
  buzzerBeep(80);
  delay(50);
  buzzerBeep(80);
}

void buzzerTripleBeep() {
  buzzerBeep(60);
  delay(40);
  buzzerBeep(60);
  delay(40);
  buzzerBeep(60);
}

void buzzerSuccessSound() {
  // Rising tone pattern for success
  buzzerBeep(100);
  delay(50);
  buzzerBeep(150);
}

void buzzerErrorSound() {
  // Lower tone pattern for errors/disconnections
  buzzerBeep(200);
  delay(100);
  buzzerBeep(200);
}

void buzzerStartupSound() {
  // Startup melody
  buzzerBeep(80);
  delay(100);
  buzzerBeep(80);
  delay(100);
  buzzerBeep(150);
}

void buzzerGestureSound() {
  // Special sound for gesture recognition
  buzzerBeep(50);
  delay(30);
  buzzerBeep(50);
  delay(30);
  buzzerBeep(100);
}

// LED control functions
void setLED1Color(int red, int green, int blue) {
  analogWrite(LED1_RED_PIN, red);
  analogWrite(LED1_GREEN_PIN, green);
  analogWrite(LED1_BLUE_PIN, blue);
}

void setLED2Color(int red, int green, int blue) {
  analogWrite(LED2_RED_PIN, red);
  analogWrite(LED2_GREEN_PIN, green);
  analogWrite(LED2_BLUE_PIN, blue);
}

// Rotary encoder interrupt handler
void IRAM_ATTR encoderISR() {
  bool currentCLK = digitalRead(ENCODER_CLK_PIN);
  bool currentDT = digitalRead(ENCODER_DT_PIN);
  
  if (currentCLK != lastCLK && currentCLK == LOW) {
    if (currentDT != currentCLK) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    encoderChanged = true;
    lastEncoderActivity = millis();
  }
  lastCLK = currentCLK;
}

// GESTURE COMMAND PROCESSING
void processGestureCommand(int speed) {
  lastControlSource = CONTROL_GESTURE;
  int oldSpeed = currentFanSpeed;
  currentFanSpeed = speed;
  fanActivityTime = millis();
  lastGestureTime = millis();
  
  // BUZZER FEEDBACK for gesture control
  buzzerGestureSound();
  
  // Sync encoder position to match gesture control
  encoderPos = currentFanSpeed / 5;
  
  // Determine gesture type for logging
  String gestureType = "Unknown";
  if (speed == 0) gestureType = "Rock (OFF)";
  else if (speed == 25) gestureType = "Scissors (25%)";
  else if (speed == 50) gestureType = "L-shape (50%)";
  else if (speed == 75) gestureType = "Paper (75%)";
  else if (speed == 100) gestureType = "Index Up (100%)";
  
  Serial.print("👋 GESTURE CONTROL (");
  Serial.print(Blynk.connected() ? "ONLINE" : "OFFLINE");
  Serial.print("): ");
  Serial.print(gestureType);
  Serial.print(" → Fan Speed set to ");
  Serial.print(currentFanSpeed);
  Serial.println("%");
  
  updateFanSpeed();
  
  // Sync to Blynk app if connected
  if (Blynk.connected()) {
    Blynk.virtualWrite(V7, currentFanSpeed);
    Serial.println("   ↗ Synced to Blynk app");
  }
  
  updateLCD();
  updateFanActivityLED();
}

// WIRELESS COMMUNICATION HANDLER FOR GESTURE COMMANDS
void handleWirelessInput() {
  // Handle HTTP requests
  server.handleClient();
  
  // Handle UDP messages
  handleUDPGestures();
  
  // Send periodic heartbeat
  sendWirelessHeartbeat();
}

// Handle rotary encoder input
void handleEncoder() {
  if (encoderChanged) {
    encoderChanged = false;
    lastControlSource = CONTROL_ENCODER;
    
    // Map encoder position to fan speed (0-100%)
    int newSpeed = constrain(encoderPos * 5, 0, 100); // 5% per step
    
    if (newSpeed != currentFanSpeed) {
      currentFanSpeed = newSpeed;
      fanActivityTime = millis();
      
      // BUZZER FEEDBACK for speed change
      buzzerBeep(50);  // Short beep for encoder speed change
      
      Serial.print("🔄 ENCODER CONTROL (");
      Serial.print(Blynk.connected() ? "ONLINE" : "OFFLINE");
      Serial.print("): Fan Speed set to ");
      Serial.print(currentFanSpeed);
      Serial.println("%");
      
      // Update fan based on encoder input
      updateFanSpeed();
      
      // Sync to Blynk app if connected (keeps app in sync)
      if (Blynk.connected()) {
        Blynk.virtualWrite(V7, currentFanSpeed);
        Serial.println("   ↗ Synced to Blynk app");
      }
      
      updateLCD();
      updateFanActivityLED();
    }
  }
  
  // Handle encoder button press
  bool currentButton = digitalRead(ENCODER_SW_PIN);
  if (!currentButton && !buttonPressed && (millis() - lastButtonPress > 500)) {
    buttonPressed = true;
    lastButtonPress = millis();
    
    // Button press toggles between off and last speed, or resets to 50% if off
    if (currentFanSpeed == 0) {
      encoderPos = 10; // 50% speed
      currentFanSpeed = 50;
      // BUZZER FEEDBACK for fan ON
      buzzerDoubleBeep();  // Double beep for turning ON
    } else {
      encoderPos = 0;  // Turn off
      currentFanSpeed = 0;
      // BUZZER FEEDBACK for fan OFF
      buzzerBeep(150);  // Longer beep for turning OFF
    }
    
    fanActivityTime = millis();
    lastControlSource = CONTROL_ENCODER;
    
    Serial.print("🔘 ENCODER BUTTON (");
    Serial.print(Blynk.connected() ? "ONLINE" : "OFFLINE");
    Serial.print("): Fan Speed toggled to ");
    Serial.print(currentFanSpeed);
    Serial.println("%");
    
    updateFanSpeed();
    
    // Sync to Blynk app if connected
    if (Blynk.connected()) {
      Blynk.virtualWrite(V7, currentFanSpeed);
      Serial.println("   ↗ Synced to Blynk app");
    }
    
    updateLCD();
    updateFanActivityLED();
  } else if (currentButton) {
    buttonPressed = false;
  }
}

// Update fan speed based on current settings
void updateFanSpeed() {
  if (currentFanSpeed == 0) {
    // Turn off relay (cuts 12V power to fan)
    digitalWrite(FAN_POWER_PIN, HIGH);  // HIGH = Relay OFF for active LOW modules
    ledcWrite(0, 0);
    pwmValue = 0;
    Serial.println("Fan: OFF (Power Cut by Relay)");
  } else {
    // Turn on relay (provides 12V power to fan)
    digitalWrite(FAN_POWER_PIN, LOW);   // LOW = Relay ON for active LOW modules
    delay(100); // Small delay for relay to switch
    
    // Set PWM speed (1-100% mapped to 80-255 for good range)
    pwmValue = map(currentFanSpeed, 1, 100, 80, 255);
    ledcWrite(0, pwmValue);
    
    Serial.print("Fan Speed: ");
    Serial.print(currentFanSpeed);
    Serial.print("% → PWM: ");
    Serial.print(pwmValue);
    Serial.println(" (Power ON + PWM)");
  }
}

// LED status functions with connection monitoring for buzzer
void updateSystemStatusLED() {
  bool currentWiFiStatus = (WiFi.status() == WL_CONNECTED);
  bool currentBlynkStatus = Blynk.connected();
  
  // Check for WiFi connection changes
  if (currentWiFiStatus && !wasWiFiConnected) {
    // WiFi just connected
    buzzerSuccessSound();
    Serial.println("🔊 BUZZER: WiFi Connected!");
  } else if (!currentWiFiStatus && wasWiFiConnected) {
    // WiFi just disconnected
    buzzerErrorSound();
    Serial.println("🔊 BUZZER: WiFi Disconnected!");
  }
  
  // Check for Blynk connection changes
  if (currentBlynkStatus && !wasBlynkConnected && currentWiFiStatus) {
    // Blynk just connected (and WiFi is connected)
    buzzerTripleBeep();
    Serial.println("🔊 BUZZER: Blynk Connected!");
  } else if (!currentBlynkStatus && wasBlynkConnected) {
    // Blynk just disconnected
    buzzerBeep(200);
    Serial.println("🔊 BUZZER: Blynk Disconnected!");
  }
  
  // Update status tracking
  wasWiFiConnected = currentWiFiStatus;
  wasBlynkConnected = currentBlynkStatus;
  
  // Set LED colors with gesture system indicator
  if (currentWiFiStatus && currentBlynkStatus) {
    if (gestureSystemActive && (millis() - lastGestureTime < 5000)) {
      // Purple flash when gesture system is active
      int brightness = (sin(millis() * 0.02) + 1) * 127;
      setLED1Color(brightness, 0, brightness);  // Purple - Gesture active
    } else {
      setLED1Color(0, 255, 0);  // Green - System fully online
    }
    systemOnline = true;
  } else if (currentWiFiStatus) {
    setLED1Color(255, 255, 0);  // Yellow - WiFi connected, Blynk disconnected
    systemOnline = false;
  } else {
    setLED1Color(255, 0, 0);  // Red - WiFi disconnected
    systemOnline = false;
  }
}

void updateFanActivityLED() {
  // Check if fan speed was recently changed (within last 3 seconds)
  if (millis() - fanActivityTime < 3000) {
    // Different colors for different control sources
    if (lastControlSource == CONTROL_GESTURE) {
      int brightness = (sin(millis() * 0.015) + 1) * 127;  // Pulsing effect
      setLED2Color(brightness, brightness, 0);  // Yellow - Gesture control
    } else if (lastControlSource == CONTROL_ENCODER) {
      int brightness = (sin(millis() * 0.01) + 1) * 127;  // Pulsing effect
      setLED2Color(brightness, 0, brightness);  // Purple - Encoder control
    } else {
      int brightness = (sin(millis() * 0.01) + 1) * 127;  // Pulsing effect
      setLED2Color(0, 0, brightness);  // Blue - Blynk control
    }
  } else {
    // Show fan status based on speed
    if (currentFanSpeed == 0) {
      setLED2Color(0, 0, 0);  // Off - Fan is off
    } else if (currentFanSpeed < 33) {
      setLED2Color(0, 255, 0);  // Green - Low speed
    } else if (currentFanSpeed < 66) {
      setLED2Color(255, 255, 0);  // Yellow - Medium speed
    } else {
      setLED2Color(255, 0, 0);  // Red - High speed
    }
  }
}

// Function to update LCD display
void updateLCD() {
  lcd.clear();
  
  // Line 1: Show connection status and control source
  lcd.setCursor(0, 0);
  if (Blynk.connected()) {
    lcd.print("ON");
  } else {
    lcd.print("OF");
  }
  
  // Show control source with gesture indicator
  if (lastControlSource == CONTROL_GESTURE) {
    lcd.print(" 👋G");
  } else if (lastControlSource == CONTROL_ENCODER) {
    lcd.print(" 🔄E");
  } else {
    lcd.print(" 📱A");
  }
  
  // Show fan status
  if (currentFanSpeed == 0) {
    lcd.print(":OFF");
  } else {
    lcd.print(":ON");
  }
  
  // Show gesture system status
  if (gestureSystemActive && (millis() - lastGestureTime < 10000)) {
    lcd.print(" G✓");
  }
  
  // Line 2: Show fan speed and PWM value
  lcd.setCursor(0, 1);
  lcd.print("S:");
  lcd.print(currentFanSpeed);
  lcd.print("% P:");
  lcd.print(pwmValue);
  
  // Show last control time indicator
  unsigned long timeSinceControl = millis() - fanActivityTime;
  if (timeSinceControl < 1000) {
    lcd.print(" •");
  }
}

BLYNK_WRITE(V7) {
  Serial.println("🔥 BLYNK_WRITE(V7) TRIGGERED! 🔥");
  int percent = param.asInt();
  
  // Only update if this isn't an echo from encoder or gesture control
  if ((lastControlSource != CONTROL_ENCODER && lastControlSource != CONTROL_GESTURE) || 
      abs(percent - currentFanSpeed) > 5) {
    int oldSpeed = currentFanSpeed;
    currentFanSpeed = percent;
    fanActivityTime = millis();
    lastControlSource = CONTROL_BLYNK;
    
    // BUZZER FEEDBACK for Blynk app control
    if (currentFanSpeed == 0 && oldSpeed > 0) {
      // Fan turned OFF via app
      buzzerBeep(120);
    } else if (currentFanSpeed > 0 && oldSpeed == 0) {
      // Fan turned ON via app
      buzzerDoubleBeep();
    } else if (abs(currentFanSpeed - oldSpeed) > 10) {
      // Significant speed change via app
      buzzerBeep(60);
    }
    
    // Sync encoder position to match Blynk control
    encoderPos = currentFanSpeed / 5;
    
    updateFanSpeed();
    updateLCD();
    updateFanActivityLED();
  }
}

BLYNK_CONNECTED() {
  Serial.println("Connected to Blynk server!");
  // Sync current fan speed to Blynk
  Blynk.virtualWrite(V7, currentFanSpeed);
  updateLCD();
  updateSystemStatusLED();
}

BLYNK_DISCONNECTED() {
  Serial.println("Disconnected from Blynk server!");
  updateLCD();
  updateSystemStatusLED();
}

// Temperature sensor function
void sendTemperature() {
  sensors.requestTemperatures(); // Request temperature reading
  
  if (sensors.getDeviceCount() > 0) {
    float tempC = sensors.getTempCByIndex(0); // Get temperature in Celsius
    
    // Check if reading is valid
    if (tempC != DEVICE_DISCONNECTED_C) {
      float tempF = tempC * 9.0 / 5.0 + 32.0; // Convert to Fahrenheit
      
      // Send temperature to Blynk virtual pin V8
      Blynk.virtualWrite(V8, tempC);
      
      // Print to serial monitor
      Serial.print("Temperature: ");
      Serial.print(tempC);
      Serial.print("°C (");
      Serial.print(tempF);
      Serial.println("°F)");
    } else {
      Serial.println("Error: Could not read temperature data");
      Blynk.virtualWrite(V8, -999); // Send error value
    }
  } else {
    Serial.println("No DS18B20 sensors found!");
    Blynk.virtualWrite(V8, -999); // Send error value
  }
}

// Handle manual temperature reading from Blynk app
BLYNK_WRITE(V1) {
  int buttonState = param.asInt();
  if (buttonState == 1) {
    // Manual temperature reading trigger
    sendTemperature();
    Serial.println("Manual temperature reading triggered");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting ESP32 Fan Controller with Gesture, Encoder, LEDs, LCD and Buzzer...");
  
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);  // Start with buzzer off
  
  // Play startup sound
  delay(500);
  buzzerStartupSound();
  Serial.println("🔊 BUZZER: System Starting!");
  
  // Initialize LED pins
  pinMode(LED1_RED_PIN, OUTPUT);
  pinMode(LED1_GREEN_PIN, OUTPUT);
  pinMode(LED1_BLUE_PIN, OUTPUT);
  pinMode(LED2_RED_PIN, OUTPUT);
  pinMode(LED2_GREEN_PIN, OUTPUT);
  pinMode(LED2_BLUE_PIN, OUTPUT);
  
  // Initialize rotary encoder pins
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  
  // Attach interrupt for rotary encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), encoderISR, CHANGE);
  
  // Set initial LED states (LED1 = Red, LED2 = Off)
  setLED1Color(255, 0, 0);  // Red - System starting
  setLED2Color(0, 0, 0);    // Off - No fan activity
  
  // Initialize I2C for LCD
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Show startup message
  lcd.setCursor(0, 0);
  lcd.print("ESP32 Fan Ctrl");
  lcd.setCursor(0, 1);
  lcd.print("Gesture+Enc+App");
  delay(2000);
  
  // Setup relay control pin
  pinMode(FAN_POWER_PIN, OUTPUT);
  digitalWrite(FAN_POWER_PIN, HIGH); // Start with fan off
  
  // Setup PWM
  ledcSetup(0, 25000, 8);
  ledcAttachPin(FAN_PWM_PIN, 0);
  ledcWrite(0, 0);
  
  // Initialize DS18B20 temperature sensor
  sensors.begin();
  sensors.setResolution(TEMPERATURE_PRECISION);
  Serial.println("DS18B20 Temperature Sensor initialized");
  
  Serial.println("Relay, PWM, Encoder, Gesture, Temperature and Buzzer configured");
  
  // Show WiFi connecting message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  
  // Connect to WiFi and Blynk
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    updateSystemStatusLED();
    handleEncoder(); // Allow encoder control even during WiFi connection
    // Note: Wireless communication not available until WiFi is connected
  }
  Serial.println("\nWiFi connected!");
  updateSystemStatusLED();
  
  // Show Blynk connecting message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting Blynk");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  // Setup timer to send temperature every 2 seconds
  timer.setInterval(2000L, sendTemperature);
  Serial.println("Temperature timer configured (2s interval)");
  
  Serial.println("Setup complete!");
  updateSystemStatusLED();
  
  // Setup wireless communication
  setupWebServer();
  setupUDP();
  
  Serial.println("\n🌐 === WIRELESS COMMUNICATION READY ===");
  Serial.print("📡 ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println("🔗 HTTP Endpoint: http://" + WiFi.localIP().toString() + "/gesture");
  Serial.print("🔗 UDP Port: ");
  Serial.println(localUdpPort);
  Serial.println("📱 Python app can now connect wirelessly!");
  
  // Show ready message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  lcd.setCursor(0, 1);
  lcd.print("3-Way Control");
  delay(2000);
  
  // Final ready beep
  buzzerSuccessSound();
  Serial.println("🔊 BUZZER: System Ready!");
  
  // Print control methods
  Serial.println("\n🎮 === CONTROL METHODS ===");
  Serial.println("📱 Blynk App: Remote control via internet");
  Serial.println("🎛 Rotary Encoder: Local physical control");
  Serial.println("👋 Hand Gestures: Computer vision control");
  Serial.println("\n👋 === GESTURE COMMANDS ===");
  Serial.println("✊ Rock: Turn OFF (0%)");
  Serial.println("🤟 L-shape: Turn ON (50%)");  
  Serial.println("✌️ Scissors: Low Speed (25%)");
  Serial.println("✋ Paper: High Speed (75%)");
  Serial.println("👆 Index Up: Max Speed (100%)");
  Serial.println("\n📡 Waiting for gesture commands via WiFi...");
  
  // Initial updates
  updateLCD();
  Serial.println("🎛 Rotary Encoder ready for offline control!");
  Serial.println("🔊 Buzzer ready for audio feedback!");
  Serial.println("   Turn: Adjust speed (5% per click) + beep");
  Serial.println("   Press: Toggle ON/OFF + different beeps");
}

void loop() {
  Blynk.run();
  timer.run(); // Run temperature sensor timer
  
  // Handle wireless gesture commands from computer
  handleWirelessInput();
  
  // Handle rotary encoder input (works offline!)
  handleEncoder();
  
  // Update LCD and LEDs every 3 seconds
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 3000) {
    lastUpdate = millis();
    updateLCD();
    updateSystemStatusLED();  // This includes buzzer feedback for connection changes
  }
  
  // Update fan activity LED continuously for smooth effects
  updateFanActivityLED();
  
  // Reset gesture system active flag after 30 seconds of inactivity
  if (gestureSystemActive && (millis() - lastGestureTime > 30000)) {
    gestureSystemActive = false;
    Serial.println("👋 Gesture system inactive (30s timeout)");
  }
  
  delay(20);  // Small delay for smooth operation
}