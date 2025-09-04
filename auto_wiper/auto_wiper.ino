/*
  Wiper Motor Control with BTS7960 Driver, Rain Sensor, and I2C LCD

  This sketch controls a motor using a BTS7960 motor driver based on readings
  from a rain sensor. The motor has four modes that are displayed on an I2C LCD.
  1. No Rain: Motor is stopped.
  2. Light Rain: Motor runs at 50% speed.
  3. Medium Rain: Motor runs at ~80% speed.
  4. Heavy Rain: Motor runs at 100% speed.

  Connections:
  - Arduino Pin 5  -> BTS7960 L_PWM
  - Arduino Pin 6  -> BTS7960 R_PWM
  - Arduino Pin 4  -> BTS7960 L_EN
  - Arduino Pin 3  -> BTS7960 R_EN
  - Arduino Pin A1 -> Rain Sensor Analog OUT (AO)
  - Arduino Pin A4 (SDA) -> I2C LCD SDA
  - Arduino Pin A5 (SCL) -> I2C LCD SCL

  - BTS7960 B+ -> Positive terminal of your Power Supply
  - BTS7960 B- -> Negative terminal of your Power Supply
  - BTS7960 M+ -> Positive terminal of the Motor
  - BTS7960 M- -> Negative terminal of the Motor

  - Rain Sensor VCC -> 5V on Arduino
  - Rain Sensor GND -> GND on Arduino
  
  - I2C LCD VCC -> 5V on Arduino
  - I2C LCD GND -> GND on Arduino
*/

// --- Include Libraries ---
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- Initialize LCD ---
// Set the I2C address to 0x27 for a 16 chars and 2 line display.
// Note: Your LCD's address might be different (e.g., 0x3F).
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Pin Definitions ---
// BTS7960 Motor Driver Pins
const int L_EN = 4;    // Left Enable (also serves as general enable)
const int R_EN = 3;    // Right Enable (also serves as general enable)
const int L_PWM = 5;   // Left PWM (controls speed for forward direction)
const int R_PWM = 6;  // Right PWM (controls speed for reverse direction)

// Rain Sensor Pin
const int RAIN_SENSOR_PIN = A1; // Analog input pin for the rain sensor

// --- Thresholds for Rain Detection ---
// These values may need calibration depending on your specific sensor.
// A lower analog value means more rain.
const int NO_RAIN_THRESHOLD = 750;   // ADC value when there is no rain (dry)
const int MEDIUM_RAIN_THRESHOLD = 500; // Threshold for medium rain
const int HEAVY_RAIN_THRESHOLD = 350; // Threshold for heavy rain

// --- Motor Speed Settings ---
const int MOTOR_SPEED_HALF = 127; // 50% speed
const int MOTOR_SPEED_MED = 200;  // ~80% speed
const int MOTOR_SPEED_FULL = 255; // 100% speed

// Variable to track the current state to prevent LCD flickering
String currentState = "";

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("Rain Sensor Wiper Control Initialized");

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Wiper Control");
  lcd.setCursor(0, 1);
  lcd.print("Initialized...");
  delay(2000); // Show startup message for 2 seconds
  lcd.clear();

  // Set motor driver pins as OUTPUT
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  // Enable the motor driver. Both EN pins need to be HIGH.
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
}

void loop() {
  // Read the analog value from the rain sensor
  int sensorValue = analogRead(RAIN_SENSOR_PIN);

  // Variables to hold the new status and speed for this loop iteration
  String newStatus = "";
  int motorSpeed = 0;
  
  // --- Logic to Determine New Status Based on Rain Level ---
  if (sensorValue > NO_RAIN_THRESHOLD) {
    newStatus = "No Rain";
    motorSpeed = 0;
  } else if (sensorValue < HEAVY_RAIN_THRESHOLD) {
    newStatus = "Heavy Rain";
    motorSpeed = MOTOR_SPEED_FULL;
  } else if (sensorValue < MEDIUM_RAIN_THRESHOLD) {
    newStatus = "Medium Rain";
    motorSpeed = MOTOR_SPEED_MED;
  } else {
    newStatus = "Light Rain";
    motorSpeed = MOTOR_SPEED_HALF;
  }

  // --- Update Displays and Motor only if the state has changed ---
  if (newStatus != currentState) {
    currentState = newStatus; // Update the state
    updateDisplayAndSerial(currentState, motorSpeed);
  }
  
  // Apply the correct speed to the motor
  if (motorSpeed > 0) {
    runMotor(motorSpeed);
  } else {
    stopMotor();
  }

  // Wait for a short period before the next reading
  delay(500);
}


// --- Helper Functions ---

/**
 * @brief Updates the LCD and Serial Monitor with the current status.
 * This function is called only when the status changes to prevent flickering.
 * @param status The current rain status string.
 * @param speed The current motor speed (0-255).
 */
void updateDisplayAndSerial(String status, int speed) {
  int speedPercent = map(speed, 0, 255, 0, 100);

  // Update Serial Monitor
  Serial.print("Status: ");
  Serial.print(status);
  Serial.print(" | Speed: ");
  Serial.print(speedPercent);
  Serial.println("%");

  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0); // Go to first line
  lcd.print("Status: ");
  lcd.print(status);
  
  lcd.setCursor(0, 1); // Go to second line
  lcd.print("Speed: ");
  if (speed > 0) {
    lcd.print(speedPercent);
    lcd.print("%   "); // Add padding to clear old characters
  } else {
    lcd.print("Stopped ");
  }
}

/**
 * @brief Runs the motor in one direction at a specified speed.
 * @param speed The PWM value for motor speed (0-255).
 */
void runMotor(int speed) {
  // To run the motor forward, we apply PWM to L_PWM and keep R_PWM low.
  analogWrite(L_PWM, speed);
  analogWrite(R_PWM, 0);
}

/**
 * @brief Stops the motor.
 */
void stopMotor() {
  // To stop the motor, we set both PWM pins to LOW.
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}
