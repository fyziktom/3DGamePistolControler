// HeadMouse with optional audio control on M5StickC (ESP32)
#include <M5Unified.h>   // IMU, display, board setup
#include <BleMouse.h>    // https://github.com/T-vK/ESP32-BLE-Mouse
#include <math.h>

#ifndef AudioStats_defined
#define AudioStats_defined
struct AudioStats {
  float rms;
  float peak;
};
#endif

#ifndef M5_LED
#define M5_LED 10
#endif

#define ENABLE_DEBUG_DISPLAY false
#define ENABLE_AUDIO_DEBUG false
#define ENABLE_AUDIO_SERIAL_DEBUG false
#define ALLOW_AUDIO_COMMANDS false

// External buttons used as mouse buttons.
const int LEFT_BUTTON = 33;
const int RIGHT_BUTTON = 32;

// Axis mapping helpers.
// Leave defaults for current behavior. If you mount the IMU differently:
// - Set HEADMOUSE_SWAP_TILT_AXES to 1 to swap which tilt drives X/Y.
// - Set HEADMOUSE_INVERT_X or HEADMOUSE_INVERT_Y to 1 to flip direction.
#define AXIS_PITCH 0
#define AXIS_ROLL  1
#define AXIS_YAW   2

#ifndef HEADMOUSE_SWAP_TILT_AXES
#define HEADMOUSE_SWAP_TILT_AXES 0
#endif
#ifndef HEADMOUSE_INVERT_X
#define HEADMOUSE_INVERT_X 1
#endif
#ifndef HEADMOUSE_INVERT_Y
#define HEADMOUSE_INVERT_Y 0
#endif
// Direct axis selection (overrides the swap flag if set).
#ifndef HEADMOUSE_AXIS_X
#define HEADMOUSE_AXIS_X AXIS_YAW   // Default: yaw (head turn) moves mouse X.
#define HEADMOUSE_AXIS_X_IS_DEFAULT 1
#else
#define HEADMOUSE_AXIS_X_IS_DEFAULT 0
#endif
#ifndef HEADMOUSE_AXIS_Y
#define HEADMOUSE_AXIS_Y AXIS_PITCH  // Default: pitch (nodding) moves mouse Y.
#define HEADMOUSE_AXIS_Y_IS_DEFAULT 1
#else
#define HEADMOUSE_AXIS_Y_IS_DEFAULT 0
#endif

// Tunable IMU parameters.
float complementaryAlpha = 0.98f;     // Blend factor between gyro integration and accelerometer tilt (0.95-0.99). Higher trusts gyro more.
float deadzonePitchDeg = 2.0f;        // Pitch deadzone in degrees (1-5 deg). Increase if jittery.
float deadzoneRollDeg = 2.0f;         // Roll deadzone in degrees (1-5 deg). Increase if jittery.
float deadzoneYawDeg = 3.0f;          // Yaw deadzone in degrees (gyro-derived, can drift). Increase if noisy.
float sensitivityPitch = 0.6f;        // Non-linear scale for mouse speed on Y. Raise for faster response (0.3-1.0 recommended).
float sensitivityRoll = 0.6f;         // Non-linear scale for mouse speed on X. Raise for faster response (0.3-1.0 recommended).
float sensitivityYaw = 0.4f;          // Lower default sensitivity for yaw to reduce drift impact.
float maxStepPerUpdate = 15.0f;       // Max pixels per loop for each axis. Reduce if motion feels too fast.

// Filtered orientation.
float pitch = 0.0f;       // Head tilt forward/back in degrees.
float roll = 0.0f;        // Head tilt left/right in degrees.
float yaw = 0.0f;         // Head turn left/right in degrees (gyro integrated, can drift).
float centerPitch = 0.0f;
float centerRoll = 0.0f;
float centerYaw = 0.0f;

// Gyro biases to reduce drift.
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

unsigned long lastUpdateMs = 0;

bool isActivated = false; // Starts not activated and you must press Home button to activate the reading of movements
bool homeHoldTriggered = false;
unsigned long homePressStart = 0;
bool audioInputReady = false;

int left_button_last_state = 0;
int right_button_last_state = 0;

String lcdText = " ";
String sensitivityStatusText = "";

BleMouse bleMouse;

// Sensitivity presets toggled by the side button.
struct SensitivityProfile {
  float pitch;
  float roll;
  float yaw;
  float maxStep;
  const char* label;
  uint16_t color;
};

const SensitivityProfile kSensitivityProfiles[] = {
  {0.15f, 0.15f, 0.10f, 8.0f, "LOW", BLUE},
  {0.30f, 0.30f, 0.20f, 12.0f, "MED", GREEN},
  {0.50f, 0.50f, 0.35f, 15.0f, "HIGH", YELLOW},
};
const int kSensitivityProfileCount = sizeof(kSensitivityProfiles) / sizeof(kSensitivityProfiles[0]);
int currentSensitivityIndex = 1; // Default to the middle (matches original tuning).

// ---------------------------------------------------------------------------
// Audio configuration and state
// ---------------------------------------------------------------------------
const int AUDIO_SAMPLE_RATE = 16000;
const size_t AUDIO_BLOCK_SAMPLES = 256; // 16 ms at 16 kHz

// Audio detection parameters (exposed for tuning).
float audioThreshold = 500.0f;              // Level above noiseFloor that starts an impulse.
unsigned long shortEventMaxMs = 200;        // Max duration for a short impulse.
unsigned long longEventMinMs = 500;         // Min duration for a long blow.
float noiseFloorAdaptRate = 0.003f;         // Lower values adapt slower but resist noise changes.
float toneMinRms = 1200.0f;                 // Minimum RMS before attempting tone detection.
float toneDominanceRatio = 20.0f;           // Tone peak must exceed this multiple of average power.

// Impulse detector.
enum AudioImpulseState {
  AUDIO_IMPULSE_IDLE,
  AUDIO_IMPULSE_ACTIVE
};
AudioImpulseState impulseState = AUDIO_IMPULSE_IDLE;
unsigned long impulseStartMs = 0;
float impulsePeakLevel = 0.0f;
unsigned long lastImpulseEventMs = 0;
unsigned long impulseRefractoryMs = 250;

bool scrollModeActive = false;
unsigned long scrollModeUntilMs = 0;
unsigned long scrollModeDurationMs = 1500; // Time after a long blow where pitch drives scroll.

// Frequency event history for relative tone sequences.
struct FreqEvent {
  float freq;
  unsigned long time;
};

FreqEvent freqHistory[8];
int freqHistoryCount = 0;
int freqHistoryIndex = 0;

// Audio levels.
float noiseFloor = 0.0f;
bool noiseFloorPrimed = false;
int noiseFloorPrimeBlocks = 0;
const int kNoisePrimeTargetBlocks = 60; // About 1 second at 16 ms per block.

// Forward declaration to satisfy Arduino's auto-prototyper.
struct AudioStats;

// Temporary audio buffer.
int16_t audioBuffer[AUDIO_BLOCK_SAMPLES];

// ---------------------------------------------------------------------------
// Utility
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
  sensitivityPitch = profile.pitch;
  sensitivityRoll = profile.roll;
  sensitivityYaw = profile.yaw;
  maxStepPerUpdate = profile.maxStep;
  currentSensitivityIndex = index;
  showSensitivityStatus(profile.label, profile.color);
}

void cycleSensitivityProfile() {
  int next = (currentSensitivityIndex + 1) % kSensitivityProfileCount;
  applySensitivityProfile(next);
}

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

void calibrateGyroBias() {
  const int samples = 200;
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (int i = 0; i < samples; ++i) {
    float gx, gy, gz;
    if (M5.Imu.getGyro(&gx, &gy, &gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
    }
    delay(2);
  }

  gyroBiasX = sumX / samples;
  gyroBiasY = sumY / samples;
  gyroBiasZ = sumZ / samples;
}

// ---------------------------------------------------------------------------
// IMU orientation filter
// ---------------------------------------------------------------------------
void initImuFilter() {
  float ax, ay, az;
  if (M5.Imu.getAccel(&ax, &ay, &az)) {
    pitch = computeAccelPitch(ax, ay, az);
    roll = computeAccelRoll(ax, ay, az);
  } else {
    pitch = 0.0f;
    roll = 0.0f;
  }
  yaw = 0.0f;
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

  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

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
  float yawGyro = yaw + gz * dt; // No accelerometer reference; expect slow drift.

  pitch = complementaryAlpha * pitchGyro + (1.0f - complementaryAlpha) * pitchAcc;
  roll = complementaryAlpha * rollGyro + (1.0f - complementaryAlpha) * rollAcc;
  yaw = yawGyro;
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
  M5.Display.printf("Yaw  : %.2f\n", yaw);
  M5.Display.printf("dP   : %.2f\n", deltaPitch);
  M5.Display.printf("dR   : %.2f\n", deltaRoll);
  M5.Display.printf("dx/dy: %d / %d\n", dx, dy);
}

void calibrateCenter() {
  calibrateGyroBias(); // Refresh bias; assume stationary during calibration.

  const int samples = 200;
  float sumPitch = 0.0f;
  float sumRoll = 0.0f;
  float sumYaw = 0.0f;

  for (int i = 0; i < samples; ++i) {
    updateOrientation();
    sumPitch += pitch;
    sumRoll += roll;
    sumYaw += yaw;
    delay(5);
  }

  centerPitch = sumPitch / samples;
  centerRoll = sumRoll / samples;
  centerYaw = sumYaw / samples;
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

void handleSensitivityButton() {
  if (M5.BtnB.wasReleased()) {
    cycleSensitivityProfile();
  }
}

void updateMouseFromHead() {
  updateOrientation();

  float deltaPitch = pitch - centerPitch; // Positive when tilting forward.
  float deltaRoll = roll - centerRoll;    // Positive when tilting right.
  float deltaYaw = yaw - centerYaw;       // Positive when turning right.

  float pitchStep = applyCurve(deltaPitch, deadzonePitchDeg, sensitivityPitch, maxStepPerUpdate);
  float rollStep = applyCurve(deltaRoll, deadzoneRollDeg, sensitivityRoll, maxStepPerUpdate);
  float yawStep = applyCurve(deltaYaw, deadzoneYawDeg, sensitivityYaw, maxStepPerUpdate);

  // Apply legacy swap flag unless explicit axis selection is provided.
  int axisX = HEADMOUSE_AXIS_X;
  int axisY = HEADMOUSE_AXIS_Y;
  if (HEADMOUSE_SWAP_TILT_AXES && HEADMOUSE_AXIS_X_IS_DEFAULT && HEADMOUSE_AXIS_Y_IS_DEFAULT) {
    axisX = AXIS_PITCH;
    axisY = AXIS_ROLL;
  }

  auto selectStep = [&](int axis) -> float {
    switch (axis) {
      case AXIS_PITCH: return pitchStep;
      case AXIS_ROLL:  return rollStep;
      case AXIS_YAW:   return yawStep;
      default:         return 0.0f;
    }
  };

  float moveX = selectStep(axisX);
  float moveY = selectStep(axisY);

#if HEADMOUSE_INVERT_X
  moveX = -moveX;
#endif
#if HEADMOUSE_INVERT_Y
  moveY = -moveY;
#endif

  int dx = (int)lrintf(moveX);
  int dy = (int)lrintf(moveY);

  updateDebugDisplay(deltaPitch, deltaRoll, dx, dy);

  if (bleMouse.isConnected() && isActivated && (dx != 0 || dy != 0)) {
    // HID positive X moves right; positive Y moves down.
    bleMouse.move(dx, dy, 0);
  }
}

// ---------------------------------------------------------------------------
// Audio feature extraction
// ---------------------------------------------------------------------------
AudioStats computeAudioStats(const int16_t* buffer, size_t count) {
  AudioStats stats{0.0f, 0.0f};
  if (count == 0) {
    return stats;
  }
  uint64_t sumSq = 0;
  int16_t peak = 0;
  for (size_t i = 0; i < count; ++i) {
    int16_t s = buffer[i];
    int32_t absVal = (s >= 0) ? s : -s;
    if (absVal > peak) {
      peak = (int16_t)absVal;
    }
    sumSq += (uint64_t)(absVal * absVal);
  }
  float rms = sqrtf((float)sumSq / (float)count);
  stats.rms = rms;
  stats.peak = (float)peak;
  return stats;
}

void updateNoiseFloor(float rms) {
  if (!noiseFloorPrimed) {
    noiseFloor = (noiseFloor * (float)noiseFloorPrimeBlocks + rms) / (float)(noiseFloorPrimeBlocks + 1);
    noiseFloorPrimeBlocks++;
    if (noiseFloorPrimeBlocks >= kNoisePrimeTargetBlocks) {
      noiseFloorPrimed = true;
    }
    return;
  }
  // Slow adaptation to follow environment changes.
  noiseFloor = (1.0f - noiseFloorAdaptRate) * noiseFloor + noiseFloorAdaptRate * rms;
}

// ---------------------------------------------------------------------------
// Goertzel-based tone detection
// ---------------------------------------------------------------------------
float goertzelPower(const int16_t* buffer, size_t count, float targetFreq) {
  float k = 0.5f + ((count * targetFreq) / (float)AUDIO_SAMPLE_RATE);
  int kInt = (int)k;
  float omega = (2.0f * PI * kInt) / (float)count;
  float coeff = 2.0f * cosf(omega);
  float s_prev = 0.0f;
  float s_prev2 = 0.0f;
  for (size_t i = 0; i < count; ++i) {
    float s = (float)buffer[i] + coeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }
  float power = s_prev2 * s_prev2 + s_prev * s_prev - coeff * s_prev * s_prev2;
  return power;
}

float detectDominantFrequency(const int16_t* buffer, size_t numSamples) {
  if (numSamples == 0) {
    return 0.0f;
  }
  const float startHz = 200.0f;
  const float endHz = 2000.0f;
  const float stepHz = 40.0f;

  float bestPower = 0.0f;
  float bestFreq = 0.0f;
  float sumPower = 0.0f;
  int binCount = 0;

  for (float f = startHz; f <= endHz; f += stepHz) {
    float p = goertzelPower(buffer, numSamples, f);
    sumPower += p;
    binCount++;
    if (p > bestPower) {
      bestPower = p;
      bestFreq = f;
    }
  }

  if (binCount == 0) {
    return 0.0f;
  }
  float avgPower = sumPower / (float)binCount;
  if (bestPower < toneDominanceRatio * avgPower) {
    return 0.0f; // Not tone-like enough.
  }
  return bestFreq;
}

// ---------------------------------------------------------------------------
// Tone segment handling and history
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Frequency command matcher (relative ascending/descending)
// ---------------------------------------------------------------------------
void onRecenterCommand() {
  writeText("Recenter", YELLOW);
  calibrateCenter();
  centerYaw = yaw; // Reset yaw drift reference.
  isActivated = true;
  scrollModeActive = false;
  writeText("3D Mouse", WHITE);
}

void onRightClickCommand() {
  if (!bleMouse.isConnected()) {
    return;
  }
  bleMouse.press(MOUSE_RIGHT);
  bleMouse.release(MOUSE_RIGHT);
}

void onDeactivateCommand() {
  isActivated = false;
  scrollModeActive = false;
  writeText("3D Mouse", RED);
}

void pushFreqEvent(float freq, unsigned long timeMs) {
  freqHistory[freqHistoryIndex] = { freq, timeMs };
  freqHistoryIndex = (freqHistoryIndex + 1) % (int)(sizeof(freqHistory) / sizeof(freqHistory[0]));
  if (freqHistoryCount < (int)(sizeof(freqHistory) / sizeof(freqHistory[0]))) {
    freqHistoryCount++;
  }
  if (ENABLE_AUDIO_SERIAL_DEBUG) {
    static unsigned long lastSerial = 0;
    if (millis() - lastSerial > 100) {
      Serial.printf("[FreqEvent] f=%.1f time=%lu\n", freq, timeMs);
      lastSerial = millis();
    }
  }
}

void checkFrequencyCommands() {
  if (freqHistoryCount < 2) {
    return;
  }

  auto getEvent = [&](int recentIdx) -> FreqEvent {
    int idx = (freqHistoryIndex - 1 - recentIdx + (int)(sizeof(freqHistory) / sizeof(freqHistory[0]))) % (int)(sizeof(freqHistory) / sizeof(freqHistory[0]));
    return freqHistory[idx];
  };

  auto withinSpacing = [&](const FreqEvent& newer, const FreqEvent& older) -> bool {
    return (newer.time - older.time) <= 800;
  };

  // Two-tone ascending for right click.
  if (freqHistoryCount >= 2) {
    FreqEvent e1 = getEvent(1);
    FreqEvent e0 = getEvent(0);
    if (withinSpacing(e0, e1) && e0.freq > e1.freq * 1.20f) { // Note: e1 is older than e0
      onRightClickCommand();
      if (ENABLE_AUDIO_SERIAL_DEBUG) {
        static unsigned long lastSerial = 0;
        if (millis() - lastSerial > 100) {
          Serial.println("[FreqCmd] Ascending 2-tone -> Right click");
          lastSerial = millis();
        }
      }
      freqHistoryCount = 0;
      freqHistoryIndex = 0;
      return;
    }
  }

  // Three-tone ascending for recenter; descending for deactivate.
  if (freqHistoryCount >= 3) {
    FreqEvent e2 = getEvent(2);
    FreqEvent e1 = getEvent(1);
    FreqEvent e0 = getEvent(0);
    bool spaced = withinSpacing(e1, e2) && withinSpacing(e0, e1);
    if (spaced) {
      bool ascending = (e1.freq > e2.freq * 1.20f) && (e0.freq > e1.freq * 1.20f);
      bool descending = (e1.freq < e2.freq * 0.80f) && (e0.freq < e1.freq * 0.80f);
      if (ascending) {
        onRecenterCommand();
        if (ENABLE_AUDIO_SERIAL_DEBUG) {
          static unsigned long lastSerial = 0;
          if (millis() - lastSerial > 100) {
            Serial.println("[FreqCmd] Ascending 3-tone -> Recenter+Activate");
            lastSerial = millis();
          }
        }
        freqHistoryCount = 0;
        freqHistoryIndex = 0;
        return;
      } else if (descending) {
        onDeactivateCommand();
        if (ENABLE_AUDIO_SERIAL_DEBUG) {
          static unsigned long lastSerial = 0;
          if (millis() - lastSerial > 100) {
            Serial.println("[FreqCmd] Descending 3-tone -> Deactivate");
            lastSerial = millis();
          }
        }
        freqHistoryCount = 0;
        freqHistoryIndex = 0;
        return;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Impulse detector for clicks and scroll activation
// ---------------------------------------------------------------------------
void handleShortImpulse(unsigned long now) {
  if (!bleMouse.isConnected() || !isActivated) return;
  bleMouse.press(MOUSE_LEFT);
  bleMouse.release(MOUSE_LEFT);
}

void handleLongBlow(unsigned long now) {
  scrollModeActive = true;
  scrollModeUntilMs = now + scrollModeDurationMs;
}

void processImpulse(float effectiveLevel, unsigned long now) {
  switch (impulseState) {
    case AUDIO_IMPULSE_IDLE:
      if (effectiveLevel > audioThreshold && (now - lastImpulseEventMs) > impulseRefractoryMs) {
        impulseState = AUDIO_IMPULSE_ACTIVE;
        impulseStartMs = now;
        impulsePeakLevel = effectiveLevel;
      }
      break;
    case AUDIO_IMPULSE_ACTIVE:
      if (effectiveLevel > impulsePeakLevel) {
        impulsePeakLevel = effectiveLevel;
      }
      if (effectiveLevel <= audioThreshold) {
        unsigned long duration = now - impulseStartMs;
        if (duration <= shortEventMaxMs) {
          handleShortImpulse(now);
        } else if (duration >= longEventMinMs) {
          handleLongBlow(now);
        }
        lastImpulseEventMs = now;
        impulseState = AUDIO_IMPULSE_IDLE;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Audio control main update
// ---------------------------------------------------------------------------
void updateAudioControl() {
  if (!audioInputReady || !M5.Mic.isEnabled()) {
    return;
  }
  size_t sampleCount = M5.Mic.record(audioBuffer, AUDIO_BLOCK_SAMPLES);
  if (sampleCount == 0) {
    return; // No audio available.
  }

  AudioStats stats = computeAudioStats(audioBuffer, sampleCount);
  updateNoiseFloor(stats.rms);
  float effectiveLevel = stats.rms - noiseFloor;
  if (effectiveLevel < 0.0f) {
    effectiveLevel = 0.0f;
  }

  unsigned long now = millis();
  float dominantFreq = 0.0f;
  if (stats.rms > toneMinRms) {
    dominantFreq = detectDominantFrequency(audioBuffer, sampleCount);
  }
  bool hasTone = (dominantFreq > 0.0f);

  if (ENABLE_AUDIO_SERIAL_DEBUG) {
    static unsigned long lastSerial = 0;
    if (millis() - lastSerial > 100) {
      Serial.printf("[Audio] rms=%.1f eff=%.1f nf=%.1f samples=%u freq=%.1f hasTone=%d\n",
                    stats.rms, effectiveLevel, noiseFloor, (unsigned)sampleCount, dominantFreq, hasTone ? 1 : 0);
      lastSerial = millis();
    }
  }

  if (hasTone) {
    pushFreqEvent(dominantFreq, now);
  }

  if (!hasTone) {
    processImpulse(effectiveLevel, now);
  } else {
    impulseState = AUDIO_IMPULSE_IDLE; // Suppress impulses during tones.
  }

  // Scroll mode: head pitch drives scroll while active.
  if (scrollModeActive) {
    if (now >= scrollModeUntilMs) {
      scrollModeActive = false;
    } else {
      float deltaPitch = pitch - centerPitch;
      float scrollStep = applyCurve(deltaPitch, deadzonePitchDeg, sensitivityPitch, maxStepPerUpdate);
      int scroll = (int)lrintf(scrollStep);
      if (bleMouse.isConnected() && isActivated && scroll != 0) {
        bleMouse.move(0, 0, scroll);
      }
    }
  }

  if (hasTone) {
    checkFrequencyCommands();
  }

  if (ENABLE_AUDIO_DEBUG && ENABLE_DEBUG_DISPLAY) {
    M5.Display.setCursor(0, 90);
    M5.Display.setTextColor(GREEN, BLACK);
    M5.Display.printf("RMS:%.1f eff:%.1f NF:%.1f\n", stats.rms, effectiveLevel, noiseFloor);
    M5.Display.printf("Freq:%.0f Imp:%d Scroll:%d\n", dominantFreq, (int)impulseState, scrollModeActive ? 1 : 0);
  }
}

// ---------------------------------------------------------------------------
// Setup and loop
// ---------------------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  cfg.internal_mic = true;  // Use internal mic via M5Unified.
  cfg.internal_spk = false; // Speaker unused.
  cfg.clear_display = true;
  cfg.output_power = true;
  cfg.fallback_board = m5::board_t::board_M5StickC; // Force StickC mapping if auto-detect fails.

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

  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);

  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);

  M5.Display.setRotation(2);
  M5.Display.setBrightness(200);
  M5.Display.fillScreen(BLACK);
  writeText("3D Mouse", isActivated ? WHITE : RED);
  applySensitivityProfile(currentSensitivityIndex);

  bleMouse.begin();
  left_button_last_state = digitalRead(LEFT_BUTTON);
  right_button_last_state = digitalRead(RIGHT_BUTTON);

  calibrateGyroBias();
  initImuFilter();
  calibrateCenter();

  // Configure and start microphone using M5Unified helper.
  auto micCfg = M5.Mic.config();
  micCfg.sample_rate = AUDIO_SAMPLE_RATE;
  micCfg.dma_buf_len = AUDIO_BLOCK_SAMPLES;
  micCfg.dma_buf_count = 4;
  M5.Mic.config(micCfg);
  audioInputReady = M5.Mic.begin();

  if (ENABLE_AUDIO_SERIAL_DEBUG) {
    Serial.begin(115200);
    Serial.setTxBufferSize(1024);
    Serial.println("[Init] Serial debug for audio enabled.");
  }
}

void loop() {
  M5.update();
  handleHomeButton();
  handleSensitivityButton();
  updateMouseFromHead();
  if (ALLOW_AUDIO_COMMANDS) {
    updateAudioControl(); // Audio gestures and tone commands.
  }

  if (bleMouse.isConnected() && isActivated) {
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
