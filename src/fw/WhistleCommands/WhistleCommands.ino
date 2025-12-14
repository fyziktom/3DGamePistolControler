#include <M5Unified.h>
#include "WhistleCommands.h"


// Optional: Plus2 "HOLD" pin (keeps power on after wake). Safe to set only on Plus2.
static constexpr uint8_t PLUS2_HOLD_PIN = 4;

// ---------------------------------------------------------------------------
// Example callbacks for detected tone sequences.
// Replace these with integration hooks (e.g., toggle activation, send clicks).
// ---------------------------------------------------------------------------
static void onRecalibrate(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Recalibrate");
}

static void onDeactivate(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Deactivate");
}

static void onRightClick(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Right click");
}

static void onCopy(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Ctrl+C");
}

static void onPaste(int, const char*, const char*, const float*, const float*, uint8_t) {
  Serial.println("[CMD] Ctrl+V");
}

// Map tone patterns to callbacks.
static const WhistleCommandDef kCommands[] = {
  { 1, "Recalibrate", "BUU",  onRecalibrate },
  { 2, "Deactivate",  "BDD",  onDeactivate  },
  { 3, "RightClick",  "BU",   onRightClick  },
  { 4, "Ctrl+C",      "BUBU", onCopy        },
  { 4, "Ctrl+V",      "BDBD", onPaste        },
};

WhistleCommands g_whistle;

void setup() {
  // Main sketch owns system setup (Serial, M5, display).
  Serial.begin(115200);
  Serial.println("[BOOT] Main setup starting...");

  auto m5cfg = M5.config();
  m5cfg.internal_mic   = true;
  m5cfg.internal_spk   = false;
  m5cfg.clear_display  = true;
  m5cfg.output_power   = true;
  m5cfg.fallback_board = m5::board_t::board_M5StickCPlus2;
  M5.begin(m5cfg);
  
  // Plus2: keep power on after wake (safe to do only when detected).
  if (M5.getBoard() == m5::board_t::board_M5StickCPlus2) {
    pinMode(PLUS2_HOLD_PIN, OUTPUT);
    digitalWrite(PLUS2_HOLD_PIN, HIGH);
  }

  M5.Display.setRotation(3);
  M5.Display.setBrightness(180);
  M5.Display.fillScreen(BLACK);

  // Configure whistle commands to reuse existing init and disable graphs/overlay.
  WhistleCommandsConfig wc;
  wc.skipM5Begin   = true;   // Already initialized above.
  wc.callM5Update  = false;  // Main loop calls M5.update().
  wc.configureMic  = true;   // Let module set up/enable the mic.
  wc.enableGraphs  = true;  // Keep display free for the main UI.
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

  Serial.println("[BOOT] Ready.");
}

void loop() {
  // Main sketch owns M5.update() when callM5Update=false.
  M5.update();
  g_whistle.update();
}
