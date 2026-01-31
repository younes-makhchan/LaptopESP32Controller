#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BleMouse.h>

// ==========================================
// ========== V3 TUNING PARAMETERS ==========
// ==========================================
// 1. SENSITIVITY
// Lower "Base" means easier to click small icons.
// Higher "Accel" means faster travel when you flick your wrist.
float MIN_SENSITIVITY  = 2.0;   // Sensitivity when moving slowly (Precision)
float MAX_SENSITIVITY  = 25.0;  // Sensitivity when moving fast (Travel)
float ACCEL_THRESHOLD  = 1.5;   // How hard you must flick to trigger max speed

// 2. SMOOTHING (Jitter Reduction)
// Range: 0.01 (Very slow/smooth) to 1.0 (Instant/Raw).
// 0.2 is a good "commercial feel" balance.
float SMOOTHING_ALPHA  = 0.2; 

// 3. DEADZONE
// Ignores tiny sensor noise when hand is "still"
float GYRO_DEADZONE    = 0.08; 

// 4. CLICK STABILITY
// Freezes cursor for X milliseconds after a click to prevent accidental drags
int CLICK_FREEZE_MS    = 150; 

// 5. ORIENTATION
bool VERTICAL_MODE = false;
bool INVERT_X      = true;
bool INVERT_Y      = true;
// ==========================================

// ----- Pins -----
#define LEFT_BTN  25
#define RIGHT_BTN 26

// ----- Objects -----
Adafruit_MPU6050 mpu;
BleMouse bleMouse;

// ----- State Variables -----
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float smoothX = 0, smoothY = 0; // Stores the filtered position
unsigned long freezeEndTime = 0; // Timer for click freezing

// ----- Button State Tracking -----
bool lastLeftState = HIGH;
bool lastRightState = HIGH;

void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(3);
  }
  gyroX_offset = sumX / samples;
  gyroY_offset = sumY / samples;
  gyroZ_offset = sumZ / samples;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(LEFT_BTN, INPUT_PULLUP);
  pinMode(RIGHT_BTN, INPUT_PULLUP);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  // --- V3 Sensor Setup ---
  // We use 2000deg/s for max headroom on fast flicks
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  
  // Set filter bandwidth to 21 Hz. 
  // This does hardware smoothing before our software smoothing.
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating... Keep STILL.");
  delay(1000);
  calibrateGyro();
  Serial.println("Go!");

  bleMouse.begin();
}

void loop() {
  if (!bleMouse.isConnected()) { 
    delay(100); 
    return; 
  }

  // 1. Read Raw Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rawX = g.gyro.x - gyroX_offset;
  float rawY = g.gyro.y - gyroY_offset;
  float rawZ = g.gyro.z - gyroZ_offset;

  // 2. Deadzone
  if (abs(rawX) < GYRO_DEADZONE) rawX = 0;
  if (abs(rawY) < GYRO_DEADZONE) rawY = 0;
  if (abs(rawZ) < GYRO_DEADZONE) rawZ = 0;

  // 3. Orient Data (Map Gyro to Screen X/Y)
  float activeX = 0;
  float activeY = 0;

  if (VERTICAL_MODE) {
    activeX = rawX; 
    activeY = rawZ; 
  } else {
    // FLAT MODE (Standard Remote)
    activeX = rawZ; // Yaw -> Left/Right
    activeY = rawX; // Pitch -> Up/Down
  }

  // 4. Calculate Dynamic Sensitivity (Acceleration)
  // We look at the magnitude of movement to decide speed.
  float velocity = sqrt(activeX*activeX + activeY*activeY);
  float sensitivity = MIN_SENSITIVITY;
  
  // If moving fast, ramp up sensitivity linearly
  if (velocity > 0.1) {
     sensitivity = map(velocity * 100, 0, ACCEL_THRESHOLD * 100, MIN_SENSITIVITY * 100, MAX_SENSITIVITY * 100) / 100.0;
     // Cap the sensitivity at max
     if (sensitivity > MAX_SENSITIVITY) sensitivity = MAX_SENSITIVITY;
  }

  float targetMoveX = activeX * sensitivity;
  float targetMoveY = activeY * sensitivity;

  // 5. Software Smoothing (Exponential Moving Average)
  // New = Alpha * Target + (1-Alpha) * Old
  smoothX = (SMOOTHING_ALPHA * targetMoveX) + ((1.0 - SMOOTHING_ALPHA) * smoothX);
  smoothY = (SMOOTHING_ALPHA * targetMoveY) + ((1.0 - SMOOTHING_ALPHA) * smoothY);

  // 6. Inversions
  int finalX = (int)smoothX;
  int finalY = (int)smoothY;
  
  if (INVERT_X) finalX = -finalX;
  if (INVERT_Y) finalY = -finalY;

  // 7. Handle Buttons & Click Freeze
  // We check buttons BEFORE moving. If a click happens, we pause movement.
  
  bool movementAllowed = true;
  if (millis() < freezeEndTime) movementAllowed = false;

  int currentLeft = digitalRead(LEFT_BTN);
  if (currentLeft != lastLeftState) {
    if (currentLeft == LOW) {
      bleMouse.press(MOUSE_LEFT);
      // Trigger Freeze on PRESS to stabilize the click
      freezeEndTime = millis() + CLICK_FREEZE_MS;
      movementAllowed = false;
    }
    else bleMouse.release(MOUSE_LEFT);
    lastLeftState = currentLeft;
  }

  int currentRight = digitalRead(RIGHT_BTN);
  if (currentRight != lastRightState) {
    if (currentRight == LOW) {
      bleMouse.press(MOUSE_RIGHT);
      freezeEndTime = millis() + CLICK_FREEZE_MS;
      movementAllowed = false;
    }
    else bleMouse.release(MOUSE_RIGHT);
    lastRightState = currentRight;
  }

  // 8. Execute Move
  if (movementAllowed && (finalX != 0 || finalY != 0)) {
    bleMouse.move(finalX, finalY);
  }

  delay(10); // Loop pacing
}