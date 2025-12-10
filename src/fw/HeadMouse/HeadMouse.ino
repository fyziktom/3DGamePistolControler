#include <M5Unified.h>   // M5Unified for M5StickC hardware
#include <BleMouse.h>    // https://github.com/T-vK/ESP32-BLE-Mouse
#include <math.h>

#ifndef M5_LED
#define M5_LED 10
#endif

#define ENABLE_DEBUG_DISPLAY true

// External buttons used as mouse buttons.
const int LEFT_BUTTON = 33;
const int RIGHT_BUTTON = 32;

// Tunable parameters.
float complementaryAlpha = 0.98f;     // Blend factor between gyro integration and accelerometer tilt (0.95-0.99). Higher trusts gyro more.
float deadzonePitchDeg = 2.0f;        // Pitch deadzone in degrees (1-5 deg). Increase if jittery.
float deadzoneRollDeg = 2.0f;         // Roll deadzone in degrees (1-5 deg). Increase if jittery.
float sensitivityPitch = 0.6f;        // Non-linear scale for mouse speed on Y. Raise for faster response (0.3-1.0 recommended).
float sensitivityRoll = 0.6f;         // Non-linear scale for mouse speed on X. Raise for faster response (0.3-1.0 recommended).
float maxStepPerUpdate = 15.0f;       // Max pixels per loop for each axis. Reduce if motion feels too fast.

// Filtered orientation.
float pitch = 0.0f;       // Head tilt forward/back in degrees.
float roll = 0.0f;        // Head tilt left/right in degrees.
float centerPitch = 0.0f;
float centerRoll = 0.0f;

unsigned long lastUpdateMs = 0;

bool isActivated = true;
bool homeHoldTriggered = false;
unsigned long homePressStart = 0;

int left_button_last_state = 0;
int right_button_last_state = 0;

String lcdText = " ";

BleMouse bleMouse;

void writeText(const String &text, int color) {
  M5.Display.setTextColor(BLACK);
  M5.Display.setCursor(0, 80);
  M5.Display.println(lcdText);
  M5.Display.setTextColor(color);
  M5.Display.setCursor(0, 80);
  M5.Display.println(text);
  lcdText = text;
}

float computeAccelPitch(float ax, float ay, float az) {
  return atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
}

float computeAccelRoll(float ax, float ay, float az) {
  return atan2f(ay, az) * 180.0f / PI;
}

void initImuFilter() {
  float ax, ay, az;
  if (M5.Imu.getAccel(&ax, &ay, &az)) {
    pitch = computeAccelPitch(ax, ay, az);
    roll = computeAccelRoll(ax, ay, az);
  } else {
    pitch = 0.0f;
    roll = 0.0f;
  }
  lastUpdateMs = millis();
}

void updateOrientation() {
  float ax, ay, az;
  float gx, gy, gz;

  if (!M5.Imu.getAccel(&ax, &ay, &az)) {
    return;
  }
  if (!M5.Imu.getGyro(&gx, &gy, &gz)) {
    return;
  }

  unsigned long now = millis();
  float dt = (now - lastUpdateMs) / 1000.0f;
  if (dt <= 0.0f) {
    dt = 0.001f;
  }
  lastUpdateMs = now;

  float pitchAcc = computeAccelPitch(ax, ay, az);
  float rollAcc = computeAccelRoll(ax, ay, az);

  float pitchGyro = pitch + gy * dt;
  float rollGyro = roll + gx * dt;

  pitch = complementaryAlpha * pitchGyro + (1.0f - complementaryAlpha) * pitchAcc;
  roll = complementaryAlpha * rollGyro + (1.0f - complementaryAlpha) * rollAcc;
}

// Applies a quadratic response after removing a deadzone so small angles move slowly and larger angles accelerate.
float applyCurve(float deltaDeg, float deadzoneDeg, float sensitivity, float maxStep) {
  float magnitude = fabsf(deltaDeg);
  if (magnitude <= deadzoneDeg) {
    return 0.0f; // Inside deadzone, no movement.
  }

  magnitude -= deadzoneDeg;                // Strip deadzone to avoid a sudden jump.
  float curved = magnitude * magnitude;    // Quadratic response: small angles => very fine motion, larger angles accelerate.
  float step = curved * sensitivity;
  if (step > maxStep) {
    step = maxStep;
  }
  return (deltaDeg >= 0.0f ? step : -step); // Keep original sign. Flip sign here if you want to invert direction.
}

void updateDebugDisplay(float deltaPitch, float deltaRoll, int dx, int dy) {
  if (!ENABLE_DEBUG_DISPLAY) {
    return;
  }
  M5.Display.fillRect(0, 0, 160, 90, BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextColor(WHITE);
  M5.Display.printf("Pitch: %.2f\n", pitch);
  M5.Display.printf("Roll : %.2f\n", roll);
  M5.Display.printf("dP   : %.2f\n", deltaPitch);
  M5.Display.printf("dR   : %.2f\n", deltaRoll);
  M5.Display.printf("dx/dy: %d / %d\n", dx, dy);
}

void calibrateCenter() {
  const int samples = 200;
  float sumPitch = 0.0f;
  float sumRoll = 0.0f;

  for (int i = 0; i < samples; ++i) {
    updateOrientation();
    sumPitch += pitch;
    sumRoll += roll;
    delay(5);
  }

  centerPitch = sumPitch / samples;
  centerRoll = sumRoll / samples;
}

void handleHomeButton() {
  if (M5.BtnA.wasPressed()) {
    homePressStart = millis();
    homeHoldTriggered = false;
  }

  if (M5.BtnA.isPressed() && !homeHoldTriggered && (millis() - homePressStart) > 1000) {
    homeHoldTriggered = true;
    writeText("Calibrating...", YELLOW);
    calibrateCenter();
    writeText("3D Mouse", isActivated ? WHITE : RED);
  }

  if (M5.BtnA.wasReleased()) {
    if (!homeHoldTriggered) {
      isActivated = !isActivated;
      writeText("3D Mouse", isActivated ? WHITE : RED);
    }
    homeHoldTriggered = false;
  }
}

void updateMouseFromHead() {
  updateOrientation();

  const float pitchSign = 1.0f; // Set to -1.0f to invert forward/back cursor direction.
  const float rollSign = 1.0f;  // Set to -1.0f to invert left/right cursor direction.

  float deltaPitch = pitch - centerPitch; // Positive when tilting forward.
  float deltaRoll = roll - centerRoll;    // Positive when tilting right.

  float moveY = pitchSign * applyCurve(deltaPitch, deadzonePitchDeg, sensitivityPitch, maxStepPerUpdate);
  float moveX = rollSign * applyCurve(deltaRoll, deadzoneRollDeg, sensitivityRoll, maxStepPerUpdate);

  int dx = (int)lrintf(moveX);
  int dy = (int)lrintf(moveY);

  updateDebugDisplay(deltaPitch, deltaRoll, dx, dy);

  if (bleMouse.isConnected() && isActivated && (dx != 0 || dy != 0)) {
    // HID positive X moves right; positive Y moves down.
    bleMouse.move(dx, dy, 0);
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.fallback_board = m5::board_t::board_M5StickC; // Force StickC mapping if auto-detect fails.

  // !!! Vypnout externí displeje !!!
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

  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);

  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);

  M5.Display.setRotation(2);
  M5.Display.setBrightness(200);
  M5.Display.fillScreen(BLACK);
  //M5.Display.setTextSize(1);
  writeText("3D Mouse", WHITE);

  bleMouse.begin();
  left_button_last_state = digitalRead(LEFT_BUTTON);
  right_button_last_state = digitalRead(RIGHT_BUTTON);

  initImuFilter();
  calibrateCenter();
}

void loop() {
  M5.update();
  handleHomeButton();
  updateMouseFromHead();

  if (bleMouse.isConnected() && isActivated) {
    if (M5.BtnB.wasReleased()) {
      bleMouse.press(MOUSE_MIDDLE);
      bleMouse.release(MOUSE_MIDDLE);
    }

    int left_button_actual_state = digitalRead(LEFT_BUTTON);
    int right_button_actual_state = digitalRead(RIGHT_BUTTON);

    if (left_button_actual_state != left_button_last_state && left_button_actual_state == HIGH) {
      left_button_last_state = left_button_actual_state;
      bleMouse.press(MOUSE_LEFT);
      bleMouse.release(MOUSE_LEFT);
    } else if (left_button_actual_state != left_button_last_state && left_button_actual_state == LOW) {
      left_button_last_state = left_button_actual_state;
      bleMouse.release(MOUSE_LEFT);
    }

    if (right_button_actual_state != right_button_last_state && right_button_actual_state == HIGH) {
      right_button_last_state = right_button_actual_state;
      bleMouse.press(MOUSE_RIGHT);
      bleMouse.release(MOUSE_RIGHT);
    } else if (right_button_actual_state != right_button_last_state && right_button_actual_state == LOW) {
      right_button_last_state = right_button_actual_state;
      bleMouse.release(MOUSE_RIGHT);
    }
  }

  delay(2);
}
