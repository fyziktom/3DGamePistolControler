// HeadMouse with gyro-based "real mouse" behaviour on M5StickC (ESP32)
// Uses M5Unified for IMU and display, and BleMouse for Bluetooth HID mouse.

#include <M5Unified.h>
#include <BleMouse.h>
#include <math.h>

// Enable to show debug info directly on the M5StickC display.
#define ENABLE_DEBUG_DISPLAY false

static constexpr gpio_num_t HOLD_PIN = GPIO_NUM_4;

// Reduce I2C/IMU clock to improve stability on some PLUS2 boards.
static constexpr uint32_t IMU_CLOCK_HZ = 100000;
// Slow down sampling to reduce I2C stress (mouse still feels fine at 100-200 Hz).
static constexpr uint8_t LOOP_DELAY_MS  = 5;
static constexpr uint8_t CALIB_DELAY_MS = 5;

// External buttons used as mouse buttons.
const int LEFT_BUTTON  = 33;
const int RIGHT_BUTTON = 32;

// Gyro axis identifiers.
#define GYRO_AXIS_X 0
#define GYRO_AXIS_Y 1
#define GYRO_AXIS_Z 2

// Axis mapping: which gyro axes drive mouse X/Y velocity.
// You can override these from the build flags or before including this file.
#ifndef HEADMOUSE_GYRO_AXIS_X
#define HEADMOUSE_GYRO_AXIS_X GYRO_AXIS_Z   // Default: yaw (turning head) moves mouse horizontally.
#define HEADMOUSE_GYRO_AXIS_X_IS_DEFAULT 1
#else
#define HEADMOUSE_GYRO_AXIS_X_IS_DEFAULT 0
#endif

#ifndef HEADMOUSE_GYRO_AXIS_Y
#define HEADMOUSE_GYRO_AXIS_Y GYRO_AXIS_Y   // Default: pitch (nodding) moves mouse vertically.
#define HEADMOUSE_GYRO_AXIS_Y_IS_DEFAULT 1
#else
#define HEADMOUSE_GYRO_AXIS_Y_IS_DEFAULT 0
#endif

// Invert directions if needed (1 = invert, 0 = normal).
#ifndef HEADMOUSE_INVERT_X
#define HEADMOUSE_INVERT_X 1
#endif

#ifndef HEADMOUSE_INVERT_Y
#define HEADMOUSE_INVERT_Y 0
#endif


// Gyro tuning parameters.
float gyroDeadzoneDps = 1.5f;        // Angular speed in deg/s below which movement is ignored.
float gyroSensitivityX = 20.0f;      // Mouse pixels per (deg/s * second) on X.
float gyroSensitivityY = 20.0f;      // Mouse pixels per (deg/s * second) on Y.
float maxStepPerUpdate = 25.0f;      // Maximum mouse pixels per update on each axis.

// Gyro bias (offset) to compensate for sensor drift when device is stationary.
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Time bookkeeping for integration.
unsigned long lastUpdateMs = 0;

// Activation and button handling.
bool isActivated = false;            // When false, head movement does not move the mouse.
bool homeHoldTriggered = false;
unsigned long homePressStart = 0;

int left_button_last_state  = 0;
int right_button_last_state = 0;

// Simple text status on display.
String lcdText = " ";
String sensitivityStatusText = "";

// BLE mouse instance.
BleMouse bleMouse;

// Sensitivity presets toggled by side button (BtnB).
struct SensitivityProfile {
  float gainX;
  float gainY;
  float maxStep;
  const char* label;
  uint16_t color;
};

const SensitivityProfile kSensitivityProfiles[] = {
  {10.0f, 10.0f, 15.0f, "LOW",  BLUE},
  {20.0f, 20.0f, 25.0f, "MED",  GREEN},
  {35.0f, 35.0f, 30.0f, "HIGH", YELLOW},
};

const int kSensitivityProfileCount =
    sizeof(kSensitivityProfiles) / sizeof(kSensitivityProfiles[0]);
int currentSensitivityIndex = 1;   // Start with "MED" profile.

// ---------------------------------------------------------------------------
// Display helpers
// ---------------------------------------------------------------------------

void showSensitivityStatus(const char* label, uint16_t color) {
  String text = String("Sensitivity: ") + label;
  M5.Display.setTextColor(BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.println(sensitivityStatusText);
  M5.Display.setTextColor(color);
  M5.Display.setCursor(0, 0);
  M5.Display.println(text);
  sensitivityStatusText = text;
}

void applySensitivityProfile(int index) {
  if (index < 0 || index >= kSensitivityProfileCount) {
    return;
  }
  const auto& profile = kSensitivityProfiles[index];
  gyroSensitivityX = profile.gainX;
  gyroSensitivityY = profile.gainY;
  maxStepPerUpdate = profile.maxStep;
  currentSensitivityIndex = index;
  showSensitivityStatus(profile.label, profile.color);
}

void cycleSensitivityProfile() {
  int next = (currentSensitivityIndex + 1) % kSensitivityProfileCount;
  applySensitivityProfile(next);
}

void writeText(const String& text, int color) {
  M5.Display.setTextColor(BLACK);
  M5.Display.setCursor(0, 80);
  M5.Display.println(lcdText);
  M5.Display.setTextColor(color);
  M5.Display.setCursor(0, 80);
  M5.Display.println(text);
  lcdText = text;
}

void updateDebugDisplay(float rawRateX, float rawRateY, int dx, int dy) {
  if (!ENABLE_DEBUG_DISPLAY) {
    return;
  }
  M5.Display.fillRect(0, 0, 160, 90, BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextColor(WHITE);
  M5.Display.printf("rateX: %.2f dps\n", rawRateX);
  M5.Display.printf("rateY: %.2f dps\n", rawRateY);
  M5.Display.printf("dx/dy: %d / %d\n", dx, dy);
  M5.Display.printf("active: %d\n", isActivated ? 1 : 0);
}

// ---------------------------------------------------------------------------
// Gyro calibration and head-mouse logic
// ---------------------------------------------------------------------------

bool ensureImuReady() {
  // If already initialized by M5.begin(cfg), just accept it.
  if (M5.Imu.isEnabled()) return true;

  // Try explicit init on internal I2C with the selected board.
  // Note: board type helps choose IMU driver (MPU6886/SH200Q/BMI270...).
  bool ok = M5.Imu.begin(&M5.In_I2C, M5.getBoard());
  return ok && M5.Imu.isEnabled();
}

bool calibrateGyroBias() {
  // Calibrate with progress + time limit to avoid "stuck forever" feel.
  const int targetSamples = 200;
  const uint32_t maxCalMs = 5000;   // Hard limit (ms)
  int got = 0;

  float sumX = 0, sumY = 0, sumZ = 0;
  uint32_t start = millis();
  uint32_t lastUi = 0;

  writeText("Calibrating...", YELLOW);

  while (got < targetSamples && (millis() - start) < maxCalMs) {
    float gx, gy, gz;
    if (M5.Imu.getGyro(&gx, &gy, &gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      got++;
    }

    // Show progress every ~20 samples so user sees it's alive.
    if (millis() - lastUi > 200) {
      lastUi = millis();
      M5.Display.setCursor(0, 100);
      M5.Display.setTextColor(WHITE, BLACK);
      M5.Display.printf("IMU type: %d\n", (int)M5.Imu.getType());
      M5.Display.printf("Samples: %d/%d\n", got, targetSamples);
      Serial.printf("IMU type: %d\n", (int)M5.Imu.getType());
      Serial.printf("Samples: %d/%d\n", got, targetSamples);
    }

    M5.update();
    delay(CALIB_DELAY_MS);
  }

  if (got < 20) {
    // Not enough valid samples -> treat as failure.
    return false;
  }

  gyroBiasX = sumX / got;
  gyroBiasY = sumY / got;
  gyroBiasZ = sumZ / got;
  lastUpdateMs = millis();
  return true;
}

// Main head-mouse update: convert gyro rotation speed into mouse movement.
void updateMouseFromHead() {
  float gx, gy, gz;
  if (!M5.Imu.getGyro(&gx, &gy, &gz)) {
    return;
  }

  // Remove bias.
  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

  // Time delta in seconds.
  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt <= 0.0f) {
    dt = 0.001f;
  } else if (dt > 0.05f) {
    // Clamp dt to avoid huge jumps if something stalls.
    dt = 0.05f;
  }
  lastUpdateMs = now;

  // Pack into array so we can choose axes dynamically.
  float g[3] = { gx, gy, gz };

  float rateX = g[HEADMOUSE_GYRO_AXIS_X];
  float rateY = g[HEADMOUSE_GYRO_AXIS_Y];

  // Small deadzone to avoid jitter when the head is still.
  if (fabsf(rateX) < gyroDeadzoneDps) {
    rateX = 0.0f;
  }
  if (fabsf(rateY) < gyroDeadzoneDps) {
    rateY = 0.0f;
  }

  // Convert angular rate into mouse step. This is basically integrating
  // velocity: if you rotate and then stop, the cursor also stops.
  float stepX = gyroSensitivityX * rateX * dt;
  float stepY = gyroSensitivityY * rateY * dt;

  // Limit maximum movement per update for stability.
  if (stepX > maxStepPerUpdate) stepX = maxStepPerUpdate;
  if (stepX < -maxStepPerUpdate) stepX = -maxStepPerUpdate;
  if (stepY > maxStepPerUpdate) stepY = maxStepPerUpdate;
  if (stepY < -maxStepPerUpdate) stepY = -maxStepPerUpdate;

  int dx = (int)lrintf(stepX);
  int dy = (int)lrintf(stepY);

#if HEADMOUSE_INVERT_X
  dx = -dx;
#endif
#if HEADMOUSE_INVERT_Y
  dy = -dy;
#endif

  updateDebugDisplay(rateX, rateY, dx, dy);

  if (bleMouse.isConnected() && isActivated && (dx != 0 || dy != 0)) {
    // HID: positive X moves right; positive Y moves down.
    bleMouse.move(dx, dy, 0);
  }
}

// ---------------------------------------------------------------------------
// Button handling
// ---------------------------------------------------------------------------

void handleHomeButton() {
  if (M5.BtnA.wasPressed()) {
    homePressStart = millis();
    homeHoldTriggered = false;
  }

  if (M5.BtnA.isPressed() && !homeHoldTriggered &&
      (millis() - homePressStart) > 1000) {
    // Long press -> recalibrate gyro bias while assuming the head is still.
    homeHoldTriggered = true;
    calibrateGyroBias();
  }

  if (M5.BtnA.wasReleased()) {
    if (!homeHoldTriggered) {
      // Short press -> toggle head-mouse activation.
      isActivated = !isActivated;
      writeText("HeadMouse", isActivated ? WHITE : RED);
    }
    homeHoldTriggered = false;
  }
}

void handleSensitivityButton() {
  if (M5.BtnB.wasReleased()) {
    cycleSensitivityProfile();
  }
}

void handleExternalMouseButtons() {
  if (!bleMouse.isConnected()) {
    return;
  }
  int left_state  = digitalRead(LEFT_BUTTON);
  int right_state = digitalRead(RIGHT_BUTTON);

  // Simple edge detection; buttons assumed to be wired as digital inputs.
  if (left_state != left_button_last_state) {
    if (left_state == HIGH) {
      bleMouse.press(MOUSE_LEFT);
      bleMouse.release(MOUSE_LEFT);
    } else {
      bleMouse.release(MOUSE_LEFT);
    }
    left_button_last_state = left_state;
  }

  if (right_state != right_button_last_state) {
    if (right_state == HIGH) {
      bleMouse.press(MOUSE_RIGHT);
      bleMouse.release(MOUSE_RIGHT);
    } else {
      bleMouse.release(MOUSE_RIGHT);
    }
    right_button_last_state = right_state;
  }
}

// ---------------------------------------------------------------------------
// Setup and loop
// ---------------------------------------------------------------------------

void setup() {
  
  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(HOLD_PIN, HIGH);
  delay(10);

  Serial.begin(115200);
  Serial.setTxBufferSize(1024);
  delay(200);
  Serial.println();
  Serial.println("=== M5StickC M5Unified Head Mouse ===");

  auto cfg = M5.config();
  cfg.internal_imu  = true;
  cfg.internal_rtc  = true;
  cfg.internal_mic = false;   // Microphone not used in this version.
  cfg.internal_spk = false;
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.fallback_board = m5::board_t::board_M5StickCPlus2;

  // Disable external displays.
  cfg.external_display.module_display = false;
  cfg.external_display.atom_display   = false;
  cfg.external_display.unit_glass     = false;
  cfg.external_display.unit_glass2    = false;
  cfg.external_display.unit_oled      = false;
  cfg.external_display.unit_mini_oled = false;
  cfg.external_display.unit_lcd       = false;
  cfg.external_display.unit_rca       = false;
  cfg.external_display.module_rca     = false;

  M5.begin(cfg);

  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);

  M5.Display.setRotation(2);
  M5.Display.setBrightness(200);
  M5.Display.fillScreen(BLACK);
  writeText("HeadMouse", isActivated ? WHITE : RED);
  applySensitivityProfile(currentSensitivityIndex);

  if (!ensureImuReady()) {
    M5.Display.fillScreen(BLACK);
    writeText("Cannot init IMU!", RED);
    delay(2000);
  }

  // Lower IMU clock for stability.
  M5.Imu.setClock(IMU_CLOCK_HZ);

  Serial.println("Starting Calibration");
  // Initial gyro calibration; keep your head still during boot.
  if (!calibrateGyroBias()) {
    M5.Display.fillScreen(BLACK);
    writeText("Cannot calibrate IMU!", RED);
    delay(2000);
  }
  

 Serial.println("Calibration Done.");
  M5.Display.fillScreen(BLACK);
  writeText("Done. Starting...", WHITE);
  

  bleMouse.begin();
  left_button_last_state  = digitalRead(LEFT_BUTTON);
  right_button_last_state = digitalRead(RIGHT_BUTTON);

  delay(2000);
}

void loop() {
  M5.update();

  handleHomeButton();
  handleSensitivityButton();
  updateMouseFromHead();

  if (bleMouse.isConnected() && isActivated) {
    handleExternalMouseButtons();
  }

  delay(LOOP_DELAY_MS);  // Small delay to keep loop timing reasonable.
}
