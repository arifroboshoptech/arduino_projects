// AIoT Smart Home Project for Arduino Uno R4 WiFi

// ------------------- Library Includes -------------------
// Make sure to install these libraries via the Arduino IDE Library Manager
// 1. DHT sensor library by Adafruit
// 2. Adafruit Unified Sensor by Adafruit
// 3. LiquidCrystal_I2C by Frank de Brabander
// 4. MFRC522 by GithubCommunity
// 5. HuskyLens by DFRobot
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>
#include "HUSKYLENS.h"

// ------------------- Pin Definitions -------------------
// Sensor and Actuator Pinout
#define DHTPIN 2          // DHT11 Data Pin
#define PIR_PIN 8         // PIR Motion Sensor Output Pin
#define SERVO_PIN 6       // Servo Motor Signal Pin
#define BUZZER_PIN 7      // Buzzer Pin
#define GAS_SENSOR_PIN A2  // Gas Sensor Analog Pin
#define RAIN_SENSOR_PIN A1 // Rain Sensor Analog Pin

// L293D Motor Driver Pins for Fan
#define ENA 5             // L293D Enable A Pin (controls speed)
#define IN1 3             // L293D Input 1
#define IN2 4             // L293D Input 2

// MFRC522 RFID Reader Pins (Standard SPI pins for Uno R4 WiFi)
// SCK: 13, MISO: 12, MOSI: 11
#define SS_PIN 10         // RFID SDA (SS) Pin
#define RST_PIN 9         // RFID Reset Pin

// ------------------- Global Constants -------------------
const float TEMP_THRESHOLD = 35.0; // Temperature threshold in Celsius
const int RAIN_THRESHOLD = 100;    // Adjust based on sensor calibration
const int GAS_THRESHOLD = 500;     // Adjust based on sensor calibration
const int FACE_ACCESS_ID = 1;      // The Face ID that is granted access

// ------------------- Object Initializations -------------------
// Initialize for a 16x2 LCD. Address might be 0x27 or 0x3F.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Initialize DHT sensor
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Initialize RFID reader
MFRC522 mfrc522(SS_PIN, RST_PIN);

// Initialize Servo motor
Servo doorServo;

// Initialize HuskyLens on the I2C bus
HUSKYLENS huskylens;

// ------------------- Global Variables -------------------
String grantedUID = "7C D9 4A 06"; 
bool motionDetected = false;
bool gasDetected = false;
bool rainDetected = false;
String huskyLensStatus = "No Target";

// --- NEW: Variables for rotating LCD display ---
unsigned long lastDisplayUpdateTime = 0;
int displayState = 0;
const long DISPLAY_INTERVAL = 4000; // Rotate display every 4 seconds

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for LCD and HuskyLens
  Wire.begin();
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("AIoT Smart Home");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize HuskyLens
  while (!huskylens.begin(Wire)) {
    lcd.setCursor(0, 0); // Use line 0 for error on 16x2
    lcd.print("HuskyLens Error!");
    delay(100);
  }
  // Set the HuskyLens algorithm to Face Recognition on startup
  huskylens.writeAlgorithm(ALGORITHM_FACE_RECOGNITION); 

  // Initialize other components
  dht.begin();
  SPI.begin();
  mfrc522.PCD_Init();
  doorServo.attach(SERVO_PIN);
  doorServo.write(0); // Lock door

  // Set pin modes
  pinMode(PIR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(GAS_SENSOR_PIN, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Ensure fan and buzzer are off
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(BUZZER_PIN, LOW);

  delay(2000);
  lcd.clear();
  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Read all sensor data first
  handleTemperatureAndFan();
  handleMotion();
  handleRain();
  handleGas();
  
  // Handle actuators and logic based on sensor data
  handleRFID();
  handleHuskyLens();
  handleBuzzer();
  
  // Update the display last
  updateLCD();

  delay(200); 
}

// ------------------- Function Implementations -------------------

void handleTemperatureAndFan() {
  float t = dht.readTemperature();
  if (isnan(t)) return;

  if (t >= TEMP_THRESHOLD) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 70); // Set fan speed
    delay(1000);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void handleRFID() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  String currentUID = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    currentUID += (mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    currentUID += String(mfrc522.uid.uidByte[i], HEX);
  }
  currentUID.toUpperCase();
  currentUID.trim();

  if (currentUID == grantedUID) {
    // Check if door is locked before acting
    if (doorServo.read() < 10) {
        doorServo.write(90); // Unlock
        digitalWrite(BUZZER_PIN, HIGH);
        delay(3000);
        doorServo.write(0);  // Lock
        digitalWrite(BUZZER_PIN, LOW);
    }
  } else {
    // --- Access Denied ---
    // Beep briefly only if a major alarm isn't already active
    if (!gasDetected && !rainDetected) {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(500);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}

void handleMotion() {
  motionDetected = (digitalRead(PIR_PIN) == HIGH);
}

void handleRain() {
  rainDetected = (analogRead(RAIN_SENSOR_PIN) > RAIN_THRESHOLD);
}

void handleGas() {
  gasDetected = (analogRead(GAS_SENSOR_PIN) > GAS_THRESHOLD);
}

void handleBuzzer() {
  // Continuous alarm for high-priority alerts like gas or rain
  if (gasDetected) {
    digitalWrite(BUZZER_PIN, HIGH);
  }
  else if (motionDetected){
      for (int count = 0; count < 5; count++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);  
     delay(100); 
  }

  }
  else if (rainDetected){
    digitalWrite(BUZZER_PIN, HIGH);
    delay(500);
    digitalWrite(BUZZER_PIN, LOW);
    delay(500);

  }
   else {
    digitalWrite(BUZZER_PIN, LOW); // Turn off if no alerts
  }
}

void handleHuskyLens() {
  if (!huskylens.request()) {
    huskyLensStatus = "No Target";
  } else if (!huskylens.isLearned()) {
    huskyLensStatus = "Unknown";
  } else if (!huskylens.available()) {
    huskyLensStatus = "No Target";
  } else {
    // Something is detected, read the result
    HUSKYLENSResult result = huskylens.read();
    
    // Update the status for the LCD
    huskyLensStatus = "ID:" + String(result.ID);

    // --- Face Recognition Door Access Logic ---
    // Check if the detected face has the authorized ID
    if (result.ID == FACE_ACCESS_ID) {
      // Check if the door is currently locked to prevent it from re-triggering
      if (doorServo.read() < 10) { 
        doorServo.write(90); // Unlock the door
        digitalWrite(BUZZER_PIN, HIGH);
        delay(3000);         // Wait for 3 seconds
        doorServo.write(0);  // Lock the door again
        digitalWrite(BUZZER_PIN, LOW); // Turn off if no alerts
      }
    }
  }
}

// *** ADJUSTED: Rewritten for 16x2 LCD with rotating status line ***
void updateLCD() {
  // --- Line 1: Temperature and Humidity (Always shown) ---
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  lcd.setCursor(0, 0);
  String tempStr = "T:" + (isnan(t) ? "ERR" : String(t, 1)) + (char)223 + "C";
  String humStr = "H:" + (isnan(h) ? "ERR" : String(h, 0)) + "%";
  lcd.print(tempStr);
  // Add padding to clear the rest of the line
  for (int i = tempStr.length(); i < (16 - humStr.length()); i++) {
    lcd.print(" ");
  }
  lcd.print(humStr);

  // --- Line 2: Priority Alerts or Rotating Status ---
  lcd.setCursor(0, 1);
  if (gasDetected) {
    lcd.print("!GAS DETECTED!  ");
    lastDisplayUpdateTime = millis(); // Reset timer during alert
  } else if (rainDetected) {
    lcd.print("!RAIN DETECTED! ");
    lastDisplayUpdateTime = millis(); // Reset timer during alert
  } else {
    // --- No alerts, so cycle through status screens ---
    if (millis() - lastDisplayUpdateTime > DISPLAY_INTERVAL) {
      displayState = (displayState + 1) % 2; // Cycle between state 0 and 1
      lastDisplayUpdateTime = millis();
    }

    switch (displayState) {
      case 0: {
        // Screen 0: Door and Camera Status
        String doorStatus = "Door:" + String(doorServo.read() > 45 ? "Open " : "Lock");
        String camStatus = "Cam:" + huskyLensStatus;
        lcd.print(doorStatus);
        // Add padding
        for (int i = doorStatus.length(); i < (16 - camStatus.length()); i++) {
          lcd.print(" ");
        }
        lcd.print(camStatus);
        break;
      }
      case 1: {
        // Screen 1: Motion and Fan Status
        float temp = dht.readTemperature();
        String motionStatus = "Motion:" + String(motionDetected ? "Yes" : "No ");
        String fanStatus = "Fan:" + String(temp >= TEMP_THRESHOLD ? "ON" : "OFF");
        lcd.print(motionStatus);
        // Add padding
        for (int i = motionStatus.length(); i < (16 - fanStatus.length()); i++) {
          lcd.print(" ");
        }
        lcd.print(fanStatus);
        break;
      }
    }
  }
}
