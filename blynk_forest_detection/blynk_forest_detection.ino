/*
  Project: IoT Sensor Hub with Arduino Uno R4 WiFi, LCD, and Alerts
  Board: Arduino Uno R4 WiFi
  Description:
  This code uses the onboard WiFi module of the Uno R4 WiFi.
  - It reads temperature, humidity, and smoke sensors.
  - Displays readings on an I2C LCD.
  - Sends data to the Blynk IoT platform.
  - Triggers a buzzer and sends Blynk notifications for high temperature or smoke.
  - NOTE: WiFi credentials must be entered into the code below.

  Libraries to install via Arduino IDE Library Manager:
  1. Blynk by Volodymyr Shymanskyy
  2. DHT sensor library by Adafruit
  3. Adafruit Unified Sensor by Adafruit
  4. LiquidCrystal_I2C by Frank de Brabander
*/

// -------------------
// Blynk Configuration
// -------------------
#define BLYNK_TEMPLATE_ID "TMPL6uCNqf93V"
#define BLYNK_TEMPLATE_NAME "Forest Detection IoT"
#define BLYNK_AUTH_TOKEN "MYE5AOEF_UzesQsZ02Mk67uvj9bBakcC"

// -------------------
// WiFi Configuration
// -------------------
char ssid[] = "change wifi username";
char pass[] = "wifi password";

// Define to print debug messages to the Serial Monitor
#define BLYNK_PRINT Serial

// ---------------------
// Library Inclusions
// ---------------------
#include <WiFiS3.h>                 // Uno R4 WiFi's native library
#include <BlynkSimpleWifi.h>      // Blynk library for Uno R4 WiFi
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// ---------------------
// Hardware Pin Definitions for Arduino Uno R4
// ---------------------
const int DHT_PIN = 2;              // DHT11 sensor connected to D2
const int MQ2_PIN = A1;             // MQ2 sensor analog output connected to A1
const int BUZZER_PIN = 11;          // Buzzer connected to D11
// Pin 13 is no longer needed for WiFiManager
// const int WIFI_CONFIG_PIN = 13;

// ---------------------
// Sensor & Component Setup
// ---------------------
// I2C LCD setup (address 0x27 is common, but can be 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Address, 16 columns, 2 rows

// DHT Sensor setup
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);

// Blynk Timer for sending data periodically
BlynkTimer timer;

// ---------------------
// Global Variables
// ---------------------
float temperature = 0;
float humidity = 0;
int smokeValue = 0;
int smokePercentage = 0; // New variable for percentage

// Alert thresholds
const int SMOKE_PERCENT_THRESHOLD = 40; // Alert when smoke is over 40%
const float TEMP_THRESHOLD = 40.0;

// Flags to prevent spamming notifications
bool tempNotifSent = false;
bool smokeNotifSent = false;


// This function is called every 5 seconds by the timer
void sendSensorData() {
  // --- Read DHT11 Sensor ---
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("DHT Error!");
    return;
  }

  // --- Read MQ2 Sensor and map to percentage ---
  smokeValue = analogRead(MQ2_PIN);
  smokePercentage = map(smokeValue, 0, 1023, 0, 100); // Map 0-1023 to 0-100

  // --- Send data to Blynk Virtual Pins ---
  Blynk.virtualWrite(V0, temperature);
  Blynk.virtualWrite(V1, humidity);
  Blynk.virtualWrite(V2, smokePercentage); // Send percentage to Blynk

  // --- Update the LCD Display ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print("C ");
  lcd.print("H:");
  lcd.print(humidity, 1);
  lcd.print("%");
  lcd.setCursor(0, 1);
  lcd.print("Smoke: ");
  lcd.print(smokePercentage); // Display percentage on LCD
  lcd.print("%");

  // --- Check for Alert Conditions ---
  // Temperature Alert
  if (temperature > TEMP_THRESHOLD) {
    if (!tempNotifSent) {
      digitalWrite(BUZZER_PIN, HIGH);
      tempNotifSent = true;
    }
  } else {
    tempNotifSent = false;
    if (!smokeNotifSent) {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  // Smoke Alert
  if (smokePercentage > SMOKE_PERCENT_THRESHOLD) { // Check against percentage threshold
    if (!smokeNotifSent) {
      digitalWrite(BUZZER_PIN, HIGH);
      smokeNotifSent = true;
    }
  } else {
    smokeNotifSent = false;
    if (!tempNotifSent) {
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}

void setup() {
  // Start Serial for debugging.
  Serial.begin(9600);
  delay(10);

  // Initialize hardware components
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize I2C LCD
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("IoT Sensor Hub");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  // Initialize DHT sensor
  dht.begin();
  delay(2000);

  // --- WiFi & Blynk Setup ---
  lcd.clear();
  lcd.print("Connecting to:");
  lcd.setCursor(0,1);
  lcd.print(ssid);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);
  int wifi_status = WiFi.status();
  while (wifi_status != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    wifi_status = WiFi.status();
  }
  
  Serial.println("\nConnected to WiFi!");
  lcd.clear();
  lcd.print("WiFi Connected!");
  delay(1000);

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  
  lcd.clear();
  lcd.print("Blynk Ready!");
  delay(1000);

  // Setup a timer to call the `sendSensorData` function every 5 seconds
  timer.setInterval(5000L, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();
}

