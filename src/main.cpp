#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BleMouse.h>

// ==========================================
// ========== USER CONFIGURATION ============
// ==========================================
// Update these global variables to tune the feel
float MOUSE_SENSITIVITY = 15.0; // Higher = faster mouse
float GYRO_DEADZONE = 0.1;      // Ignore tiny movements (jitter reduction)
bool INVERT_X = true;          // Set to true if Left/Right is swapped
bool INVERT_Y = true;           // Set to true if Up/Down is swapped
bool VERTICAL_MODE = true;
// ==========================================

// ----- Pins -----
#define LEFT_BTN  25
#define RIGHT_BTN 26

// ----- Objects -----
Adafruit_MPU6050 mpu;
BleMouse bleMouse;

// ----- Calibration Variables -----
float gyroX_offset = 0;
float gyroY_offset = 0;
float gyroZ_offset = 0;

// ----- Button State Tracking (to prevent spamming) -----
bool lastLeftState = HIGH;
bool lastRightState = HIGH;
// Function to calculate the "zero" error of the gyro
void calibrateGyro() {
  float sumX = 0, sumY = 0, sumZ = 0;
  int samples = 200;
  
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(5);
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

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  // Setup ranges for Mouse usage
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG); // Max range for fast flicks
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Calibrating Gyro... Keep the remote STILL!");
  delay(1000);
  calibrateGyro();
  Serial.println("Calibration Done.");

  bleMouse.begin();
}

void loop() {
  if (!bleMouse.isConnected()) {
    // Optional: Blink an LED here to show disconnected state
    delay(100);
    return;
  }

  // 1. Read Sensor Data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 2. Process Gyro Data (Radians/s)
  // We subtract the offset calculated in setup
  float gx = g.gyro.x - gyroX_offset;
  float gy = g.gyro.y - gyroY_offset;
  float gz = g.gyro.z - gyroZ_offset;

  // 3. Apply Deadzone (if movement is too small, ignore it)
  if (abs(gx) < GYRO_DEADZONE) gx = 0;
  if (abs(gy) < GYRO_DEADZONE) gy = 0;
  if (abs(gz) < GYRO_DEADZONE) gz = 0;

  // 4. Map Gyro to Mouse Movement
  // Standard Remote Mounting Assumption:
  // Rotation around Z axis (Yaw) -> Mouse Left/Right (X)
  // Rotation around X axis (Pitch) -> Mouse Up/Down (Y) (OR Y-axis depending on mounting)
  
  // Note: g.gyro returns Rad/s. We multiply by sensitivity to get pixels.
  float activeX_val = 0;
  float activeY_val = 0;
  if (VERTICAL_MODE) {
    // TRY THIS DEFAULT FOR VERTICAL:
    activeX_val = gx; // Rolling the board (twist) acts as Left/Right
    activeY_val = gz; // Pitching (chop) acts as Up/Down
    
    // If this feels wrong, swap 'gy' with 'gz' here.
  } 
  else {
    // --- FLAT MODE (Horizontal) ---
    // Your verified working setup:
    activeX_val = gz; // Yaw
    activeY_val = gx; // Pitch
  }

  // 4. Calculate movement
  int moveX = (int)(activeX_val * MOUSE_SENSITIVITY); 
  int moveY = (int)(activeY_val * MOUSE_SENSITIVITY);

  // Apply Inversions
  if( VERTICAL_MODE ) {
    // In vertical mode, we might want to invert Y differently
    moveX = -moveX;
    moveY = moveY;
  }
  else{
    if (INVERT_X) moveX = -moveX;
    if (INVERT_Y) moveY = -moveY;
  }


  // 5. Move Mouse
  if (moveX != 0 || moveY != 0) {
    bleMouse.move(moveX, moveY);
  }

  // 6. Handle Buttons (State Change Detection)
  // We only send the command when the button actually changes state
  int currentLeft = digitalRead(LEFT_BTN);
  if (currentLeft != lastLeftState) {
    if (currentLeft == LOW) bleMouse.press(MOUSE_LEFT);
    else bleMouse.release(MOUSE_LEFT);
    lastLeftState = currentLeft;
  }

  int currentRight = digitalRead(RIGHT_BTN);
  if (currentRight != lastRightState) {
    if (currentRight == LOW) bleMouse.press(MOUSE_RIGHT);
    else bleMouse.release(MOUSE_RIGHT);
    lastRightState = currentRight;
  }

  delay(10); // Short delay for stability
}

