// HeadMouse with gyro-based "real mouse" behaviour on M5StickC-family (ESP32)
// Uses M5Unified for IMU+display and BleMouse for BLE HID mouse.
// Includes BLE advertising recovery after disconnect and a friendly status HUD.
// NOTE: All comments are intentionally in English.

#include <M5Unified.h>
#include <BleMouse.h>
#include <BLEDevice.h>   // BLEDevice::startAdvertising()
#include <math.h>
#include "WhistleCommands.h"

// ------------------------------------------------------------
// User toggles
// ------------------------------------------------------------
#define ENABLE_DEBUG_DISPLAY false

// ------------------------------------------------------------
// Pins / HW
// ------------------------------------------------------------
// External buttons used as mouse buttons (Grove port on StickC-family).
const int LEFT_BUTTON  = 33;
const int RIGHT_BUTTON = 32;

// Reduce I2C/IMU clock to improve stability on some boards.
static constexpr uint32_t IMU_CLOCK_HZ = 100000;

// Loop speed (100–200 Hz is enough for a mouse and reduces I2C stress).
static constexpr uint8_t LOOP_DELAY_MS  = 5;
static constexpr uint8_t CALIB_DELAY_MS = 5;

// Optional: Plus2 "HOLD" pin (keeps power on after wake). Safe to set only on Plus2.
static constexpr uint8_t PLUS2_HOLD_PIN = 4;

// ------------------------------------------------------------
// Axis mapping
// ------------------------------------------------------------
#define GYRO_AXIS_X 0
#define GYRO_AXIS_Y 1
#define GYRO_AXIS_Z 2

#ifndef HEADMOUSE_GYRO_AXIS_X
#define HEADMOUSE_GYRO_AXIS_X GYRO_AXIS_Z   // Default: yaw -> mouse X
#endif

#ifndef HEADMOUSE_GYRO_AXIS_Y
#define HEADMOUSE_GYRO_AXIS_Y GYRO_AXIS_Y   // Default: pitch -> mouse Y
#endif

#ifndef HEADMOUSE_INVERT_X
#define HEADMOUSE_INVERT_X 1
#endif

#ifndef HEADMOUSE_INVERT_Y
#define HEADMOUSE_INVERT_Y 0
#endif

// ------------------------------------------------------------
// Tone commands
// ------------------------------------------------------------
WhistleCommands g_whistle;

// ------------------------------------------------------------
// Mouse tuning
// ------------------------------------------------------------
float gyroDeadzoneDps  = 1.5f;
float gyroSensitivityX = 20.0f;
float gyroSensitivityY = 20.0f;
float maxStepPerUpdate = 25.0f;

// Gyro bias
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Integration time
unsigned long lastUpdateMs = 0;

// Activation state
bool isActivated = false;

// External button state
int left_button_last_state  = 0;
int right_button_last_state = 0;

// BLE mouse instance
BleMouse bleMouse;

// ------------------------------------------------------------
// Sensitivity presets
// ------------------------------------------------------------
struct SensitivityProfile {
  float gainX;
  float gainY;
  float maxStep;
  const char* label;
};

const SensitivityProfile kSensitivityProfiles[] = {
  {10.0f, 10.0f, 15.0f, "LOW"},
  {20.0f, 20.0f, 25.0f, "MED"},
  {35.0f, 35.0f, 30.0f, "HIGH"},
};
const int kSensitivityProfileCount =
    sizeof(kSensitivityProfiles) / sizeof(kSensitivityProfiles[0]);
int currentSensitivityIndex = 1;

// ------------------------------------------------------------
// HUD / UI state
// ------------------------------------------------------------
enum BleUiState : uint8_t {
  BLE_UI_BOOTING = 0,
  BLE_UI_ADVERTISING = 1,
  BLE_UI_CONNECTED = 2
};

BleUiState bleUiState = BLE_UI_BOOTING;

bool lastBleConnected = false;
unsigned long bleStartMs = 0;
unsigned long lastAdvKickMs = 0;

static constexpr unsigned long BLE_BOOT_GRACE_MS     = 2000;  // Let BLE task init before kicking advertising
static constexpr unsigned long ADV_KICK_PERIOD_MS    = 4000;  // Periodic advertising kick while disconnected
static constexpr unsigned long HUD_REFRESH_MS        = 150;   // UI refresh interval
static constexpr unsigned long EVENT_DEFAULT_MS      = 1500;  // Default event message lifetime

// Calibration status for HUD
bool g_isCalibrating = false;
int  g_calibGot = 0;
int  g_calibTarget = 0;

// Event line
String g_eventText = "";
uint16_t g_eventColor = WHITE;
unsigned long g_eventUntilMs = 0;

unsigned long g_lastHudDrawMs = 0;
bool g_hudDirty = true;

// ------------------------------------------------------------
// Helpers: sensitivity / activation / events
// ------------------------------------------------------------
void applySensitivityProfile(int index) {
  if (index < 0 || index >= kSensitivityProfileCount) return;
  const auto& p = kSensitivityProfiles[index];
  gyroSensitivityX = p.gainX;
  gyroSensitivityY = p.gainY;
  maxStepPerUpdate = p.maxStep;
  currentSensitivityIndex = index;
  g_hudDirty = true;
}

void cycleSensitivityProfile() {
  int next = (currentSensitivityIndex + 1) % kSensitivityProfileCount;
  applySensitivityProfile(next);
  // Event message
  g_eventText = String("Sensitivity: ") + kSensitivityProfiles[currentSensitivityIndex].label;
  g_eventColor = CYAN;
  g_eventUntilMs = millis() + EVENT_DEFAULT_MS;
  g_hudDirty = true;
}

void toggleActivation() {
  isActivated = !isActivated;

  g_eventText = isActivated ? "Movement: ACTIVE" : "Movement: INACTIVE";
  g_eventColor = isActivated ? GREEN : ORANGE;
  g_eventUntilMs = millis() + EVENT_DEFAULT_MS;
  g_hudDirty = true;
}

void setEvent(const String& text, uint16_t color, unsigned long durationMs = EVENT_DEFAULT_MS) {
  g_eventText = text;
  g_eventColor = color;
  g_eventUntilMs = (durationMs == 0) ? 0 : (millis() + durationMs);
  g_hudDirty = true;
}

// ------------------------------------------------------------
// HUD drawing
// ------------------------------------------------------------
void configureFontForScreen() {
  // Use bigger font on wider displays, smaller on narrow StickC.
  if (M5.Display.width() < 120) {
    M5.Display.setFont(&fonts::Font0);
    M5.Display.setTextSize(1);
  } else {
    M5.Display.setFont(&fonts::Font0);
    //M5.Display.setFont(&fonts::AsciiFont8x16);
    M5.Display.setTextSize(1);
  }
}

const char* boardShortName(m5::board_t b) {
  switch (b) {
    case m5::board_t::board_M5StickC:       return "StickC";
    case m5::board_t::board_M5StickCPlus:   return "Plus";
    case m5::board_t::board_M5StickCPlus2:  return "Plus2";
    default:                                return "M5";
  }
}

void drawHud(bool force = false) {
  unsigned long now = millis();
  if (!force && !g_hudDirty && (now - g_lastHudDrawMs) < HUD_REFRESH_MS) return;
  g_lastHudDrawMs = now;
  g_hudDirty = false;

  configureFontForScreen();

  int w = M5.Display.width();
  int h = M5.Display.height();
  int lineH = M5.Display.fontHeight() + 2;

  // Reserve a compact HUD area at the top
  int hudLines = 5;
  int hudH = hudLines * lineH + 4;

  // Clear HUD area
  M5.Display.fillRect(0, 0, w, hudH, BLACK);

  int x = 2;
  int y = 2;

  // Line 1: Title + board
  M5.Display.setCursor(x, y);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.printf("HeadMouse (%s)", boardShortName(M5.getBoard()));
  y += lineH;

  // Line 2: BLE state
  M5.Display.setCursor(x, y);
  if (bleUiState == BLE_UI_CONNECTED) {
    M5.Display.setTextColor(GREEN, BLACK);
    M5.Display.print("BLE: CONNECTED");
  } else if (bleUiState == BLE_UI_ADVERTISING) {
    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.print("BLE: ADVERTISING");
  } else {
    M5.Display.setTextColor(ORANGE, BLACK);
    M5.Display.print("BLE: BOOTING");
  }
  y += lineH;

  // Line 3: Movement active + connection hint
  M5.Display.setCursor(x, y);
  if (isActivated) {
    M5.Display.setTextColor(CYAN, BLACK);
    M5.Display.print("MOVE: ACTIVE");
  } else {
    M5.Display.setTextColor(WHITE, BLACK);
    M5.Display.print("MOVE: INACTIVE");
  }

  // Small hint if user is active but not connected
  if (isActivated && bleUiState != BLE_UI_CONNECTED) {
    M5.Display.setTextColor(ORANGE, BLACK);
    M5.Display.print(" (connect)");
  }
  y += lineH;

  // Line 4: Sensitivity
  M5.Display.setCursor(x, y);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.printf("SENS: %s", kSensitivityProfiles[currentSensitivityIndex].label);
  y += lineH;

  // Line 5: Calibration progress or quick controls
  M5.Display.setCursor(x, y);
  if (g_isCalibrating) {
    M5.Display.setTextColor(YELLOW, BLACK);
    M5.Display.printf("CAL: %d/%d", g_calibGot, g_calibTarget);
  } else {
    M5.Display.setTextColor(DARKGREY, BLACK);
    M5.Display.print("A: toggle | A-hold: calib | B: sens");
  }

  // Event message line near bottom (non-intrusive)
  int eventY = h - lineH - 2;
  M5.Display.fillRect(0, eventY - 2, w, lineH + 4, BLACK);

  if (g_eventText.length() > 0 && (g_eventUntilMs == 0 || now <= g_eventUntilMs)) {
    M5.Display.setCursor(2, eventY);
    M5.Display.setTextColor(g_eventColor, BLACK);
    M5.Display.print(g_eventText);
  }
}

// ------------------------------------------------------------
// IMU + calibration
// ------------------------------------------------------------
bool ensureImuReady() {
  if (M5.Imu.isEnabled()) return true;
  bool ok = M5.Imu.begin(&M5.In_I2C, M5.getBoard());
  return ok && M5.Imu.isEnabled();
}

bool calibrateGyroBias() {
  const int targetSamples = 200;
  const uint32_t maxCalMs = 5000;

  g_isCalibrating = true;
  g_calibGot = 0;
  g_calibTarget = targetSamples;
  setEvent("Calibrating gyro...", YELLOW, 0);

  float sumX = 0, sumY = 0, sumZ = 0;
  uint32_t start = millis();
  uint32_t lastUi = 0;

  while (g_calibGot < targetSamples && (millis() - start) < maxCalMs) {
    float gx, gy, gz;
    if (M5.Imu.getGyro(&gx, &gy, &gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      g_calibGot++;
    }

    // Refresh HUD periodically
    if (millis() - lastUi > 100) {
      lastUi = millis();
      g_hudDirty = true;
      drawHud(true);
      Serial.printf("[CAL] IMU type=%d samples=%d/%d\n", (int)M5.Imu.getType(), g_calibGot, targetSamples);
    }

    M5.update();
    delay(CALIB_DELAY_MS);
  }

  g_isCalibrating = false;

  if (g_calibGot < 20) {
    setEvent("Calibration FAILED", RED, 2500);
    g_hudDirty = true;
    return false;
  }

  gyroBiasX = sumX / g_calibGot;
  gyroBiasY = sumY / g_calibGot;
  gyroBiasZ = sumZ / g_calibGot;

  lastUpdateMs = millis();

  setEvent("Recalibrated", GREEN, 1500);
  g_hudDirty = true;
  return true;
}

// ------------------------------------------------------------
// Mouse motion
// ------------------------------------------------------------
void updateMouseFromHead() {
  float gx, gy, gz;
  if (!M5.Imu.getGyro(&gx, &gy, &gz)) return;

  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  else if (dt > 0.05f) dt = 0.05f;
  lastUpdateMs = now;

  float g[3] = { gx, gy, gz };
  float rateX = g[HEADMOUSE_GYRO_AXIS_X];
  float rateY = g[HEADMOUSE_GYRO_AXIS_Y];

  if (fabsf(rateX) < gyroDeadzoneDps) rateX = 0.0f;
  if (fabsf(rateY) < gyroDeadzoneDps) rateY = 0.0f;

  float stepX = gyroSensitivityX * rateX * dt;
  float stepY = gyroSensitivityY * rateY * dt;

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

  if (ENABLE_DEBUG_DISPLAY) {
    // If you want debug output, enable this flag and add your debug drawing here.
  }

  if (bleMouse.isConnected() && isActivated && (dx != 0 || dy != 0)) {
    bleMouse.move((signed char)dx, (signed char)dy, 0);
  }
}

// ------------------------------------------------------------
// BLE advertising recovery
// ------------------------------------------------------------
void kickAdvertising(const char* reason) {
  if (millis() - bleStartMs < BLE_BOOT_GRACE_MS) return;
  BLEDevice::startAdvertising();
  lastAdvKickMs = millis();
  setEvent(String("ADV restart: ") + reason, YELLOW, 1200);
  bleUiState = BLE_UI_ADVERTISING;
  g_hudDirty = true;
}

void handleBleRecovery() {
  bool connected = bleMouse.isConnected();

  if (!lastBleConnected && connected) {
    bleUiState = BLE_UI_CONNECTED;
    setEvent("BLE Connected", GREEN, 1200);
    g_hudDirty = true;
  }

  if (lastBleConnected && !connected) {
    // Restart advertising after disconnect so other hosts can find it again.
    kickAdvertising("disconnect");
  }

  // While disconnected, keep advertising alive (some stacks stop advertising).
  if (!connected) {
    bleUiState = (millis() - bleStartMs < BLE_BOOT_GRACE_MS) ? BLE_UI_BOOTING : BLE_UI_ADVERTISING;

    if ((millis() - lastAdvKickMs) > ADV_KICK_PERIOD_MS) {
      kickAdvertising("watchdog");
    }
  }

  lastBleConnected = connected;
}

// ------------------------------------------------------------
// Buttons
// ------------------------------------------------------------
void handleButtons() {
  // A hold -> recalibrate
  if (M5.BtnA.wasHold()) {
    calibrateGyroBias();
    return;
  }

  // A click -> toggle activation
  if (M5.BtnA.wasClicked()) {
    toggleActivation();
  }

  // B click -> cycle sensitivity
  if (M5.BtnB.wasClicked()) {
    cycleSensitivityProfile();
  }
}

// External mouse buttons (GPIO32/33)
void handleExternalMouseButtons() {
  if (!bleMouse.isConnected()) return;

  int left_state  = digitalRead(LEFT_BUTTON);
  int right_state = digitalRead(RIGHT_BUTTON);

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

//-------------------------------------------------------------
// Callbacks for tone commands
//-------------------------------------------------------------
static void onRecalibrate(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Recalibrate");
  Serial.println("[CMD] Calibrating gyro bias...");
  calibrateGyroBias();
  if (!isActivated) {
    toggleActivation();
  }
}

static void onDeactivate(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Deactivate");
    if (isActivated) {
    toggleActivation();
  }
}

static void onLeftClick(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Left click");
  bleMouse.press(MOUSE_LEFT);
  bleMouse.release(MOUSE_LEFT);
}

static void onRightClick(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Right click");
  bleMouse.press(MOUSE_RIGHT);
  bleMouse.release(MOUSE_RIGHT);
}

static void onCopy(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Ctrl+C");
}

static void onPaste(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Ctrl+V");
}


// ------------------------------------------------------------
// Tone commands mapping
// ------------------------------------------------------------

static const WhistleCommandDef kCommands[] = {
  { 1, "Recalibrate", "BUU",  onRecalibrate },
  { 2, "Deactivate",  "BDD",  onDeactivate  },
  { 3, "LeftClick",  "BB",   onLeftClick  },
  { 4, "RightClick",  "BU",   onRightClick  },
  { 5, "Ctrl+C",      "BUBU", onCopy        },
  { 6, "Ctrl+V",      "BDBD", onPaste        },
};

// ------------------------------------------------------------
// Setup / loop
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.setTxBufferSize(1024);
  delay(200);

  Serial.println();
  Serial.println("=== M5Unified HeadMouse (HUD + BLE recovery) ===");

  auto cfg = M5.config();
  cfg.internal_imu  = true;
  cfg.internal_mic  = true;
  cfg.internal_spk  = false;
  cfg.clear_display = true;
  cfg.output_power  = true;

  // Keep this as StickC if you build for StickC.
  // For Plus2 you can change it to: m5::board_t::board_M5StickCPlus2
  cfg.fallback_board = m5::board_t::board_M5StickC;

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

  // Plus2: keep power on after wake (safe to do only when detected).
  if (M5.getBoard() == m5::board_t::board_M5StickCPlus2) {
    pinMode(PLUS2_HOLD_PIN, OUTPUT);
    digitalWrite(PLUS2_HOLD_PIN, HIGH);
  }

  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);

  M5.Display.setRotation(1);
  M5.Display.setBrightness(200);
  M5.Display.fillScreen(BLACK);

  // Button thresholds (more robust click/hold behavior).
  M5.BtnA.setHoldThresh(1000);
  M5.BtnB.setHoldThresh(1000);

  applySensitivityProfile(currentSensitivityIndex);

  // Initial HUD draw
  setEvent("Booting...", WHITE, 800);
  bleUiState = BLE_UI_BOOTING;
  drawHud(true);

  if (!ensureImuReady()) {
    setEvent("IMU init FAILED", RED, 2500);
    drawHud(true);
    delay(2000);
  }

  M5.Imu.setClock(IMU_CLOCK_HZ);

  Serial.println("[BOOT] Calibrating gyro bias...");
  calibrateGyroBias();

  // Start BLE mouse (BLE stack runs in its own task).
  Serial.println("[BOOT] Starting BLE mouse...");
  bleStartMs = millis();
  lastAdvKickMs = bleStartMs;
  bleMouse.begin();
  bleUiState = BLE_UI_ADVERTISING;

  left_button_last_state  = digitalRead(LEFT_BUTTON);
  right_button_last_state = digitalRead(RIGHT_BUTTON);

    // Configure whistle commands to reuse existing init and disable graphs/overlay.
  WhistleCommandsConfig wc;
  wc.skipM5Begin   = true;   // Already initialized above.
  wc.callM5Update  = false;  // Main loop calls M5.update().
  wc.configureMic  = true;   // Let module set up/enable the mic.
  wc.enableGraphs  = false;  // Keep display free for the main UI.
  wc.enableOverlay = false;
  wc.display       = static_cast<LGFX_Device*>(&M5.Display);
  wc.commands      = kCommands;
  wc.commandCount  = sizeof(kCommands) / sizeof(kCommands[0]);
  wc.onCommand     = nullptr;  // Optional extra hook after per-command callbacks.
  wc.initSerial    = false;    // Serial already initialized.
  wc.enablePeriodicSerialDebug = false;
  if (!g_whistle.begin(wc)) {
    Serial.println("[BOOT] Whistle init failed, halting.");
    while (true) { delay(1000); }
  }

  setEvent("Ready", WHITE, 1200);
  drawHud(true);
}

void loop() {
  M5.update();

  g_whistle.update();

  // BLE state + advertising recovery
  handleBleRecovery();

  // Buttons
  handleButtons();

  // Mouse motion
  updateMouseFromHead();

  // External click buttons
  if (bleMouse.isConnected() && isActivated) {
    handleExternalMouseButtons();
  }

  // Refresh HUD (non-flickery; only if needed / timed)
  drawHud(false);

  delay(LOOP_DELAY_MS);
}
