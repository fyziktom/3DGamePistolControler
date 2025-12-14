#pragma once

#include <stddef.h>
#include <stdint.h>
#include <M5GFX.h>  // for lgfx::LGFX_Device

using LGFX_Device = lgfx::LGFX_Device;

using WhistleCommandCallback = void (*)(int cmd_id,
                                        const char* name,
                                        const char* pattern,
                                        const float* midi_seq,
                                        const float* freq_seq,
                                        uint8_t ntones);

struct WhistleCommandDef {
  int                     id;
  const char*             name;
  const char*             pattern;   // e.g. "BUU", "BDD", "BU", "BUUB"
  WhistleCommandCallback  cb;
};

struct WhistleCommandsConfig {
  bool    skipM5Begin                 = false;   // If true, caller must have initialized M5.
  bool    configureMic                = true;    // Apply mic config + begin.
  bool    enableGraphs                = true;    // Draw spectrogram/waveform if a display is provided.
  bool    enableOverlay               = true;    // Draw small HUD on top if a display is provided.
  bool    enablePeriodicSerialDebug   = true;    // Emit periodic serial debug lines.
  bool    ownDisplay                  = true;    // If true, this module can change rotation/brightness/startWrite.
  bool    callM5Update                = true;    // Call M5.update() internally (set false if the main sketch already does it).
  bool    initSerial                  = true;    // Initialize Serial in begin().
  uint32_t serialBaud                 = 115200;
  uint8_t displayRotation             = 3;
  uint8_t displayBrightness           = 200;

  LGFX_Device*             display    = nullptr; // Optional external display; defaults to M5.Display when available.
  WhistleCommandCallback   onCommand  = nullptr; // Additional hook fired after per-command callbacks.
  const WhistleCommandDef* commands   = nullptr; // Optional custom command list (must outlive this instance).
  size_t                   commandCount = 0;
};

class WhistleCommands {
public:
  WhistleCommands() = default;
  ~WhistleCommands() = default;

  bool begin(const WhistleCommandsConfig& cfg = WhistleCommandsConfig());
  void update();
  int  lastCommandId() const;
};
