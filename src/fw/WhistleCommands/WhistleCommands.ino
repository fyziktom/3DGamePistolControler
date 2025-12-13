#include <M5Unified.h>
#include <M5GFX.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// -----------------------------------------------------------------------------
// Serial debug
// -----------------------------------------------------------------------------
static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr bool ENABLE_PERIODIC_SERIAL_DEBUG = true;
static constexpr uint32_t SERIAL_DEBUG_INTERVAL_MS = 500;

// -----------------------------------------------------------------------------
// Simple heap helpers (no PSRAM tricks here, just standard malloc/free)
// -----------------------------------------------------------------------------
static void* memory_alloc(size_t size) {
  return malloc(size);
}

static void memory_free(void* ptr) {
  free(ptr);
}

// -----------------------------------------------------------------------------
// Ring buffer for raw audio
// -----------------------------------------------------------------------------
struct wav_data_t {
  int16_t* wav = nullptr;   // ring-buffer samples
  size_t   length = 0;      // total size of ring buffer
  size_t   latest_index = 0;

  // Find a near-zero crossing region (used by waveform drawer).
  size_t searchEdge(size_t offset, size_t search_length) const {
    int mem_position = (int)latest_index + (int)offset;
    if ((size_t)mem_position >= length) {
      mem_position -= (int)length;
    }
    int mem_difference = 0;
    int position = -1;
    uint32_t counter[2] = {0, 0};
    bool prev_sign = false;

    for (size_t i = 0; i < search_length; ++i) {
      size_t idx = latest_index + i;
      if (idx >= length) {
        idx -= length;
      }
      int value = wav[idx];
      bool sign = (value < 0);
      if (prev_sign != sign) {
        prev_sign = sign;
        if (sign) {
          // When changing from positive to negative.
          if (position >= 0) {
            int diff = (int)counter[0] + (int)counter[1];
            if (mem_difference < diff) {
              mem_difference = diff;
              mem_position   = position;
            }
          }
        }
        counter[sign] = 0;
        if (i >= offset) {
          int pidx = (idx ? (int)idx : (int)length) - 1;
          int cv   = abs(wav[idx]);
          int pv   = abs(wav[pidx]);
          position = (cv < pv) ? (int)idx : pidx;
        }
      }
      uint32_t v = (uint32_t)(value * value);
      counter[sign] += v >> 7;
    }

    mem_position -= (int)offset;
    if (mem_position < 0) {
      mem_position += (int)length;
    }
    return (size_t)mem_position;
  }
};

// -----------------------------------------------------------------------------
// FFT data storage (magnitude spectrum)
// -----------------------------------------------------------------------------
struct fft_data_t {
  float*      fdata = nullptr;    // magnitudes
  size_t      length = 0;         // number of bins used
  size_t      sample_rate = 0;
  wav_data_t* wav_data = nullptr;
  uint8_t     fft_size_bits = 0;

  // Map bins to display pixels, using max-over-range if needed.
  float getDataByPixel(uint16_t x, uint16_t width) const {
    if (length <= width) {
      int index = (int)x * (int)length / (int)width;
      if (index >= (int)length) {
        index = (int)length - 1;
      }
      return fdata[index];
    }

    int index0 = (int)x * (int)length / (int)width;
    int index1 = (int)(x + 1) * (int)length / (int)width;
    if (index0 >= (int)length) { index0 = (int)length - 1; }
    if (index1 >= (int)length) { index1 = (int)length - 1; }

    float value = 0.0f;
    for (int i = index0; i < index1; ++i) {
      if (value < fdata[i]) {
        value = fdata[i];
      }
    }
    return value;
  }
};

// -----------------------------------------------------------------------------
// FFT engine (ported from M5Unified Mic_FFT example)
// -----------------------------------------------------------------------------
class fft_function_t {
public:
  bool setup(uint8_t max_fft_size_bits) {
    close();
    int FFT_SIZE = 1 << max_fft_size_bits;

    int need_size = 1
                  + (FFT_SIZE * 3 / 4) * (int)sizeof(float)
                  + (FFT_SIZE * (int)sizeof(uint16_t));
    _work_area = memory_alloc(need_size);
    if (_work_area == nullptr) {
      return false;
    }

    _max_fft_size_bits         = max_fft_size_bits;
    _initialized_fft_size_bits = 0;

    fi = (float*)memory_alloc(sizeof(float) * FFT_SIZE + 1);
    fr = (float*)memory_alloc(sizeof(float) * FFT_SIZE + 1);
    return (fi != nullptr && fr != nullptr);
  }

  void close(void) {
    if (_work_area) {
      memory_free(_work_area);
      _work_area = nullptr;
    }
    if (fi) { memory_free(fi); fi = nullptr; }
    if (fr) { memory_free(fr); fr = nullptr; }

    _max_fft_size_bits          = 0;
    _initialized_fft_size_bits  = 0;
  }

  // Heavy FFT core, compiled with -O3 for speed.
  __attribute((optimize("-O3")))
  bool update(fft_data_t* fft_data) {
    if (_work_area == nullptr
        || fft_data == nullptr
        || fft_data->fdata == nullptr
        || fft_data->fft_size_bits == 0) {
      return false;
    }

    if (_initialized_fft_size_bits != fft_data->fft_size_bits) {
      if (!_init(fft_data->fft_size_bits)) {
        return false;
      }
    }

    int FFT_SIZE = 1 << fft_data->fft_size_bits;
    uint16_t* br = (uint16_t*)_work_area;

    // Reorder real input into bit-reversal order.
    {
      auto src = fft_data->wav_data->wav;
      for (int i = 0; i < FFT_SIZE; ++i) {
        float lv = (float)src[i];
        fr[br[i]] = lv;
      }
      memset(fi, 0, sizeof(float) * FFT_SIZE);
    }

    int s4 = FFT_SIZE / 4;
    float* wi = (float*)(&br[FFT_SIZE]);

    size_t s  = 1;
    size_t i  = 0;
    size_t je = (size_t)FFT_SIZE;

    // Iterative Cooley–Tukey.
    do {
      size_t ke = s;
      s <<= 1;
      je >>= 1;
      size_t j = 0;
      do {
        size_t k = 0;
        size_t m = ke * ((j << 1) + 1);
        size_t l = s * j;

        auto frm_p = &fr[m];
        auto fim_p = &fi[m];
        auto frl_p = &fr[l];
        auto fil_p = &fi[l];
        auto wi_p  = &wi[0];
        auto wr_p  = &wi[s4];

        do {
          float wi_val = *wi_p;
          float wr_val = *wr_p;
          float frm    = *frm_p;
          float fim    = *fim_p;

          float Wxmr = frm * wr_val + fim * wi_val;
          float Wxmi = fim * wr_val - frm * wi_val;

          float frl  = *frl_p;
          float fil  = *fil_p;

          *frm_p++ = frl - Wxmr;
          *frl_p++ = frl + Wxmr;
          *fim_p++ = fil - Wxmi;
          *fil_p++ = fil + Wxmi;

          wi_p += je;
          wr_p += je;
        } while (++k < ke);
      } while (++j < je);
    } while (++i < _initialized_fft_size_bits);

    // Convert complex result to magnitude.
    int loop_end = (int)fft_data->length;
    if (loop_end > FFT_SIZE) {
      loop_end = FFT_SIZE;
    }

    float max_value = 0.0f;
    for (int i2 = 0; i2 < loop_end; ++i2) {
      float vr = fr[i2];
      float vi = fi[i2];
      float v  = sqrtf(vr * vr + vi * vi);
      fft_data->fdata[i2] = v;
      if (max_value < v) {
        max_value = v;
      }
    }

    // Normalize peak to ~65536, with simple limiter.
    if (max_value <= 0.0f) {
      return true;
    }

    float k = 65536.0f / max_value;
    if (k > 0.03125f) {
      // Do not over-amplify noise floor.
      k = 0.03125f;
    } else {
      for (int i2 = 0; i2 < loop_end; ++i2) {
        float tmp = fft_data->fdata[i2];
        tmp      *= k;
        fft_data->fdata[i2] = tmp;
      }
    }
    return true;
  }

private:
  bool _init(uint8_t size_bits) {
    if (_max_fft_size_bits < size_bits) {
      return false;
    }

    _initialized_fft_size_bits = size_bits;
    uint16_t* br = (uint16_t*)_work_area;
    int FFT_SIZE = 1 << size_bits;
    float* wi    = (float*)(&br[FFT_SIZE]);

    // Bit-reversal table.
    size_t je = 1;
    br[0] = 0;
    br[1] = FFT_SIZE >> 1;
    for (size_t i = 0; i < size_bits - 1; ++i) {
      br[je << 1] = br[je] >> 1;
      je = je << 1;
      for (size_t j = 1; j < je; ++j) {
        br[je + j] = br[je] + br[j];
      }
    }

    // Twiddle factors.
    float omega = 2.0f * (float)M_PI / (float)FFT_SIZE;
    int s2 = FFT_SIZE >> 1;
    int s4 = FFT_SIZE >> 2;

    wi[0]   = 0.0f;
    wi[s2]  = 0.0f;
    wi[s4]  = 1.0f;

    for (int i = 1; i < s4; ++i) {
      float f = cosf(omega * (float)i);
      wi[s4 + i]      = f;
      wi[s4 - i]      = f;
      wi[s4 + s2 - i] = -f;
    }

    return true;
  }

  float* fi = nullptr;
  float* fr = nullptr;
  void*  _work_area = nullptr;
  uint8_t _max_fft_size_bits = 0;
  uint8_t _initialized_fft_size_bits = 0;
};

// -----------------------------------------------------------------------------
// Simple rectangle helper
// -----------------------------------------------------------------------------
struct rect_t {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

// -----------------------------------------------------------------------------
// Waveform drawer (bottom band)
// -----------------------------------------------------------------------------
class wav_drawer_t {
  LGFX_Device* _gfx = nullptr;
  int16_t* prev_y   = nullptr;
  int16_t* prev_h   = nullptr;
  rect_t   draw_rect{0, 0, 0, 0};

  uint32_t bg_color   = 0x000000u;
  uint32_t fg_color   = 0xFFFFFFu;
  uint32_t line_color = 0x303030u;

public:
  bool setup(LGFX_Device* gfx, const rect_t& rect) {
    if (gfx == nullptr) return false;
    _gfx = gfx;
    draw_rect = rect;

    gfx->fillRect(rect.x, rect.y, rect.w, rect.h, bg_color);
    gfx->drawFastVLine(rect.x + (rect.w >> 1), rect.y, rect.h, line_color);

    int width = rect.w;
    prev_y = (int16_t*)memory_alloc(width * sizeof(int16_t));
    prev_h = (int16_t*)memory_alloc(width * sizeof(int16_t));
    if (!prev_y || !prev_h) return false;

    memset(prev_y, 0, width * sizeof(int16_t));
    memset(prev_h, 0, width * sizeof(int16_t));
    return true;
  }

  bool update(const wav_data_t& wav_data) {
    auto gfx       = _gfx;
    int32_t width  = draw_rect.w;
    int32_t height = draw_rect.h;

    int wav_count  = (int)wav_data.length;
    int wav_index  = (int)wav_data.searchEdge(draw_rect.w >> 1, wav_count / 3);
    auto wav       = wav_data.wav;

    // Find max absolute value for auto scaling.
    int32_t max_value = 1;
    int wi = wav_index;
    for (int i = 0; i < width; ++i) {
      int32_t tmp = abs(wav[wi]);
      if (max_value < tmp) {
        max_value = tmp;
      }
      if (++wi >= wav_count) {
        wi = 0;
      }
    }

    int new_k = (int)((height << 15) / max_value);
    if (new_k > 65536) {
      new_k = 65536;
    }

    static int k = 0;
    if (k > new_k) {
      k = new_k;
    } else {
      k = (k * 127 + new_k) >> 7;
    }

    int32_t value1 = (32768 - wav[wav_index] * k) >> 16;
    int32_t value2 = value1;
    int32_t base_y = draw_rect.y + (height >> 1);

    for (int i = 0; i < width; ++i) {
      if (++wav_index >= wav_count) {
        wav_index = 0;
      }

      int32_t x = i + draw_rect.x;
      int32_t y = prev_y[i];
      int32_t h = prev_h[i];

      // Erase previous column.
      gfx->setColor(i == (width >> 1) ? line_color : bg_color);
      gfx->drawFastVLine(x, base_y + y, h);

      int32_t value0   = value1;
      value1           = value2;
      value2           = (32768 - wav[wav_index] * k) >> 16;
      int32_t value_01 = (value0 + value1) >> 1;
      int32_t value_12 = (value1 + value2) >> 1;

      int32_t y_min = (value_01 < value_12) ? value_01 : value_12;
      if (y_min > value1) y_min = value1;
      int32_t y_max = (value_01 > value_12) ? value_01 : value_12;
      if (y_max < value1) y_max = value1;

      y = y_min;
      h = y_max + 1 - y;

      prev_y[i] = y;
      prev_h[i] = h;

      gfx->drawPixel(x, base_y, line_color);
      gfx->drawFastVLine(x, base_y + y, h, fg_color);
    }
    return true;
  }
};

// -----------------------------------------------------------------------------
// Single-frame FFT drawer (top band)
// -----------------------------------------------------------------------------
class fft_drawer_t {
  LGFX_Device* _gfx = nullptr;
  uint16_t* prev_y  = nullptr;
  uint16_t* prev_h  = nullptr;
  rect_t    draw_rect{0, 0, 0, 0};

  uint32_t bg_color = 0x000066u;
  uint32_t fg_color = 0x00FF00u;

public:
  bool setup(LGFX_Device* gfx, const rect_t& rect) {
    if (gfx == nullptr) return false;
    _gfx = gfx;
    draw_rect = rect;

    gfx->fillRect(rect.x, rect.y, rect.w, rect.h, bg_color);

    int width = rect.w;
    prev_y = (uint16_t*)memory_alloc(width * sizeof(uint16_t));
    prev_h = (uint16_t*)memory_alloc(width * sizeof(uint16_t));
    if (!prev_y || !prev_h) return false;

    memset(prev_y, 0, width * sizeof(uint16_t));
    memset(prev_h, 0, width * sizeof(uint16_t));
    return true;
  }

  bool update(const fft_data_t& fft_data) {
    auto gfx       = _gfx;
    int32_t width  = draw_rect.w;
    int32_t height = draw_rect.h - 1;

    int32_t value1 = height - (((int32_t)(fft_data.getDataByPixel(0, width) * height)) >> 16);
    if (value1 < 0) value1 = 0;
    int32_t value2 = value1;

    for (int i = 0; i < width; ++i) {
      int32_t x = i + draw_rect.x;
      int32_t y = prev_y[i];
      int32_t h = prev_h[i];

      // Erase old bar.
      gfx->drawFastVLine(x, draw_rect.y + y, h, bg_color);

      int32_t value0   = value1;
      value1           = value2;
      value2           = height - (((int32_t)(fft_data.getDataByPixel(i + 1, width) * height)) >> 16);
      if (value2 < 0) value2 = 0;

      int32_t value_01 = (value0 + value1) >> 1;
      int32_t value_12 = (value1 + value2) >> 1;

      int32_t y_min = (value_01 < value_12) ? value_01 : value_12;
      if (y_min > value1) y_min = value1;
      int32_t y_max = (value_01 > value_12) ? value_01 : value_12;
      if (y_max < value1) y_max = value1;

      y = y_min;
      h = y_max + 1 - y;

      prev_y[i] = (uint16_t)y;
      prev_h[i] = (uint16_t)h;

      gfx->drawFastVLine(x, draw_rect.y + y, h, fg_color);
    }
    return true;
  }
};

// -----------------------------------------------------------------------------
// Spectrogram (grayscale heatmap, middle band)
// -----------------------------------------------------------------------------
class fft_history_t {
  LGFX_Device* _gfx = nullptr;
  uint8_t*     color_map = nullptr;
  rect_t       draw_rect{0, 0, 0, 0};

  uint32_t bg_color = 0x000033u;
  uint32_t fg_color = 0xFFFF00u;

  int step = 0;

public:
  bool setup(LGFX_Device* gfx, const rect_t& rect) {
    if (gfx == nullptr) return false;
    _gfx = gfx;
    draw_rect = rect;

    int width  = rect.w;
    int height = rect.h;

    color_map = (uint8_t*)memory_alloc(width * height * sizeof(uint8_t));
    if (!color_map) return false;

    memset(color_map, 0, width * height * sizeof(uint8_t));
    return true;
  }

  bool update(const fft_data_t& fft_data) {
    int32_t width  = draw_rect.w;
    int32_t height = draw_rect.h;

    // Scroll one row after a few frames.
    if (--step < 0) {
      step = 7;
      memmove(&color_map[width],
              color_map,
              width * (height - 1) * sizeof(uint8_t));
      memset(color_map, 0, width * sizeof(uint8_t));
    }

    // Update top row intensity from current FFT.
    for (int i = 0; i < width; ++i) {
      int v = (int)(fft_data.getDataByPixel(i, width) / 256.0f);
      if (v > 255) v = 255;
      if (v < 0)   v = 0;
      // Max-hold in that pixel.
      color_map[i] = (color_map[i] > v) ? color_map[i] : (uint8_t)v;
    }

    // Draw only the band corresponding to current step.
    int y0 = ( step      * draw_rect.h) >> 3;
    int y1 = ((step + 1) * draw_rect.h) >> 3;

    _gfx->pushGrayscaleImage(
      draw_rect.x,
      draw_rect.y + y0,
      draw_rect.w,
      y1 - y0,
      &color_map[y0 * width],
      m5gfx::color_depth_t::grayscale_8bit,
      fg_color,
      bg_color
    );

    return true;
  }
};

// -----------------------------------------------------------------------------
// Tone sequence detection
// -----------------------------------------------------------------------------
typedef void (*CommandCallback)(int cmd_id,
                                const char* name,
                                const char* pattern,
                                const float* midi_seq,
                                const float* freq_seq,
                                uint8_t ntones);

struct CommandDef {
  int             id;
  const char*     name;
  const char*     pattern;   // e.g. "BUU", "BDD", "BU", "BUUB"
  CommandCallback cb;
};

static void onCommandPrint(int cmd_id,
                           const char* name,
                           const char* pattern,
                           const float* midi_seq,
                           const float* freq_seq,
                           uint8_t ntones) {
  Serial.printf("\n=== CMD %d: %s  pattern=%s  n=%u ===\n", cmd_id, name, pattern, (unsigned)ntones);
  for (uint8_t i = 0; i < ntones; ++i) {
    Serial.printf("  #%u: %7.1f Hz  midi=%6.2f\n", (unsigned)(i + 1), freq_seq[i], midi_seq[i]);
  }
  Serial.println("====================================\n");
}

static const CommandDef COMMANDS[] = {
  { 1, "Recalibrate", "BUU",  onCommandPrint },
  { 2, "Deactivate",  "BDD",  onCommandPrint },
  { 3, "RightClick",  "BU",   onCommandPrint },
  { 4, "Ctrl+C",      "BUUB", onCommandPrint },
};

class ToneSequencer {
public:
  void begin(const CommandDef* cmds, size_t cmd_count) {
    _cmds = cmds;
    _cmd_count = cmd_count;
    resetAll();
    _last_candidate_ms = millis();
  }

  void updateFromFFT(const fft_data_t& fft) {
    uint32_t now_ms = millis();
    PitchFrame pf = analyzePitchFrame(fft);
    updateToneState(pf, now_ms);
    evaluateSequenceIfEnded(now_ms);
    periodicDebug(now_ms);
  }

  int lastCommandId() const { return _last_cmd_id; }
  uint8_t sequenceLength() const { return _seq_len; }
  bool isToneActive() const { return _tone_active; }

  float liveFreqHz() const { return _live_freq_hz; }
  float liveSNR() const { return _live_snr; }
  float livePeak2() const { return _live_peak2; }
  float liveConc() const { return _live_conc; }
  bool  liveCandidate() const { return _live_candidate; }

private:
  struct PitchFrame {
    float freq_hz = 0.0f;
    float peak_mag = 0.0f;
    float avg_mag = 0.0f;
    float snr = 0.0f;
    float peak2_ratio = 0.0f;
    float conc = 0.0f;
    bool  tone_candidate = false;
  };

  // Detection band.
  static constexpr float DET_FMIN_HZ = 300.0f;
  static constexpr float DET_FMAX_HZ = 3200.0f;

  // Candidate thresholds.
  static constexpr float DET_MIN_SNR = 8.0f;
  static constexpr float DET_MIN_PEAK2_RATIO = 2.2f;
  static constexpr float DET_MIN_CONC = 0.18f;

  static constexpr int   DET_PEAK_NEIGHBOR_BINS = 3;
  static constexpr int   DET_CONC_BAND_BINS = 2;

  static constexpr float DET_PEAK_OVER_NOISE = 4.0f;
  static constexpr float DET_ABS_MIN_PEAK = 800.0f;
  static constexpr float DET_NOISE_UPDATE_SNR_MAX = 2.0f;

  // Segmentation / timing.
  static constexpr uint8_t START_FRAMES = 2;
  static constexpr uint8_t END_FRAMES   = 4;

  static constexpr uint32_t MIN_TONE_MS = 170;
  static constexpr uint32_t PRE_SILENCE_MS = 250;
  static constexpr uint32_t SEQ_END_GAP_MS = 550;
  static constexpr uint32_t POST_SILENCE_MS = 450;
  static constexpr uint32_t SEQ_HARD_RESET_MS = 2500;

  // Tone stability / quality.
  static constexpr float TONE_MAX_DRIFT_ST = 2.0f;
  static constexpr float TONE_MIN_AVG_SNR  = 8.0f;
  static constexpr float TONE_MIN_AVG_P2   = 2.2f;
  static constexpr float TONE_MIN_AVG_CONC = 0.18f;

  // Pattern tolerances (semitones).
  static constexpr float REL_UP_MIN_ST   = 0.9f;
  static constexpr float REL_DOWN_MIN_ST = 0.9f;
  static constexpr float REL_SAME_TOL_ST = 0.8f;
  static constexpr float REL_BASE_TOL_ST = 2.0f;

  static constexpr uint8_t MAX_SEQ_TONES = 8;

  const CommandDef* _cmds = nullptr;
  size_t _cmd_count = 0;

  // Noise estimate.
  float _noise_avg_mag = 0.0f;
  bool  _noise_primed  = false;

  // Live metrics.
  float _live_freq_hz = 0.0f;
  float _live_snr     = 0.0f;
  float _live_peak2   = 0.0f;
  float _live_conc    = 0.0f;
  float _live_peak    = 0.0f;
  float _live_avg     = 0.0f;
  bool  _live_candidate = false;

  // Last candidate timestamp.
  uint32_t _last_candidate_ms = 0;

  // Tone segmentation state.
  bool     _tone_active = false;
  uint8_t  _present_frames = 0;
  uint8_t  _absent_frames  = 0;
  uint32_t _tone_start_ms  = 0;

  float _tone_freq_smooth = 0.0f;
  float _tone_midi_smooth = 0.0f;

  // Tone quality accumulators.
  float _tone_midi_min = 0.0f;
  float _tone_midi_max = 0.0f;
  float _tone_snr_sum  = 0.0f;
  float _tone_p2_sum   = 0.0f;
  float _tone_conc_sum = 0.0f;
  uint16_t _tone_metric_frames = 0;

  // Sequence buffer.
  float    _seq_midi[MAX_SEQ_TONES];
  float    _seq_freq[MAX_SEQ_TONES];
  uint8_t  _seq_len = 0;
  uint32_t _last_tone_end_ms = 0;

  // Last fired command.
  int _last_cmd_id = 0;
  const char* _last_cmd_name = "None";
  const char* _last_cmd_pattern = "";

  // Periodic debug.
  uint32_t _last_dbg_ms = 0;

  void periodicDebug(uint32_t now_ms) {
    if (!ENABLE_PERIODIC_SERIAL_DEBUG) return;
    if (now_ms - _last_dbg_ms < SERIAL_DEBUG_INTERVAL_MS) return;
    _last_dbg_ms = now_ms;

    Serial.printf("[DBG] cand=%u tone=%u seq=%u  f=%6.0fHz snr=%4.1f r=%4.2f c=%4.2f  peak=%7.0f avg=%7.0f\n",
                  (unsigned)_live_candidate,
                  (unsigned)_tone_active,
                  (unsigned)_seq_len,
                  _live_freq_hz,
                  _live_snr,
                  _live_peak2,
                  _live_conc,
                  _live_peak,
                  _live_avg);
  }

  void resetAll() {
    _noise_avg_mag = 0.0f;
    _noise_primed  = false;

    _tone_active = false;
    _present_frames = 0;
    _absent_frames  = 0;
    _tone_start_ms  = 0;

    _tone_freq_smooth = 0.0f;
    _tone_midi_smooth = 0.0f;

    _tone_midi_min = 0.0f;
    _tone_midi_max = 0.0f;
    _tone_snr_sum  = 0.0f;
    _tone_p2_sum   = 0.0f;
    _tone_conc_sum = 0.0f;
    _tone_metric_frames = 0;

    _seq_len = 0;
    _last_tone_end_ms = 0;

    _last_cmd_id = 0;
    _last_cmd_name = "None";
    _last_cmd_pattern = "";

    _last_dbg_ms = 0;
  }

  static float hzToMidi(float hz) {
    return 69.0f + 12.0f * log2f(hz / 440.0f);
  }

  PitchFrame analyzePitchFrame(const fft_data_t& fft) {
    PitchFrame pf;

    const int FFT_SIZE = 1 << fft.fft_size_bits;
    const float bin_hz = (float)fft.sample_rate / (float)FFT_SIZE;

    int k0 = (int)ceilf(DET_FMIN_HZ / bin_hz);
    int k1 = (int)floorf(DET_FMAX_HZ / bin_hz);

    if (k0 < 1) k0 = 1;
    if (k1 > (int)fft.length - 2) k1 = (int)fft.length - 2;
    if (k1 <= k0) return pf;

    float sum = 0.0f;
    int count = 0;

    int peak_k = k0;
    float peak = 0.0f;

    for (int k = k0; k <= k1; ++k) {
      float v = fft.fdata[k];
      sum += v;
      ++count;
      if (v > peak) {
        peak = v;
        peak_k = k;
      }
    }

    float avg_all = (count > 0) ? (sum / (float)count) : 0.0f;
    float avg_wo_peak = (count > 1) ? ((sum - peak) / (float)(count - 1)) : avg_all;
    float snr = peak / (avg_wo_peak + 1e-6f);

    // Second peak outside neighborhood.
    float second_peak = 0.0f;
    for (int k = k0; k <= k1; ++k) {
      if (abs(k - peak_k) <= DET_PEAK_NEIGHBOR_BINS) continue;
      float v = fft.fdata[k];
      if (v > second_peak) second_peak = v;
    }
    float peak2 = peak / (second_peak + 1e-6f);

    // Concentration around peak.
    float band_sum = 0.0f;
    int b0 = peak_k - DET_CONC_BAND_BINS;
    int b1 = peak_k + DET_CONC_BAND_BINS;
    if (b0 < k0) b0 = k0;
    if (b1 > k1) b1 = k1;
    for (int k = b0; k <= b1; ++k) band_sum += fft.fdata[k];
    float conc = band_sum / (sum + 1e-6f);

    // Parabolic interpolation.
    float peak_kf = (float)peak_k;
    if (peak_k > k0 && peak_k < k1) {
      float m1 = fft.fdata[peak_k - 1];
      float m2 = fft.fdata[peak_k];
      float m3 = fft.fdata[peak_k + 1];
      float denom = (m1 - 2.0f * m2 + m3);
      if (fabsf(denom) > 1e-6f) {
        float delta = 0.5f * (m1 - m3) / denom;
        if (delta > -0.5f && delta < 0.5f) {
          peak_kf = (float)peak_k + delta;
        }
      }
    }

    float freq = peak_kf * bin_hz;

    pf.freq_hz = freq;
    pf.peak_mag = peak;
    pf.avg_mag = avg_wo_peak;
    pf.snr = snr;
    pf.peak2_ratio = peak2;
    pf.conc = conc;

    // Live metrics for HUD/debug.
    _live_freq_hz = freq;
    _live_snr = snr;
    _live_peak2 = peak2;
    _live_conc = conc;
    _live_peak = peak;
    _live_avg = avg_wo_peak;

    // Update noise baseline only when spectrum is flat.
    if (snr < DET_NOISE_UPDATE_SNR_MAX) {
      if (!_noise_primed) {
        _noise_avg_mag = avg_wo_peak;
        _noise_primed = true;
      } else {
        const float a = 0.02f;
        _noise_avg_mag = (1.0f - a) * _noise_avg_mag + a * avg_wo_peak;
      }
    }

    float noise_ref = _noise_primed ? _noise_avg_mag : avg_wo_peak;
    float min_peak = noise_ref * DET_PEAK_OVER_NOISE;
    if (min_peak < DET_ABS_MIN_PEAK) min_peak = DET_ABS_MIN_PEAK;

    pf.tone_candidate = (freq >= DET_FMIN_HZ && freq <= DET_FMAX_HZ
                        && snr >= DET_MIN_SNR
                        && peak >= min_peak
                        && peak2 >= DET_MIN_PEAK2_RATIO
                        && conc >= DET_MIN_CONC);

    _live_candidate = pf.tone_candidate;
    return pf;
  }

  static uint8_t patternLength(const char* pattern) {
    uint8_t n = 0;
    for (const char* p = pattern; *p; ++p) {
      if (*p == ' ') continue;
      ++n;
    }
    return n;
  }

  static bool matchPattern(const char* pattern, const float* midi_seq, uint8_t ntones) {
    if (ntones == 0) return false;

    float base = midi_seq[0];
    float prev = midi_seq[0];

    uint8_t idx = 0;
    for (const char* p = pattern; *p; ++p) {
      if (*p == ' ') continue;
      if (idx >= ntones) return false;

      char c = *p;
      float cur = midi_seq[idx];

      if (idx == 0) {
        base = cur;
        prev = cur;
        ++idx;
        continue;
      }

      switch (c) {
        case 'B': {
          if (fabsf(cur - base) > REL_BASE_TOL_ST) return false;
        } break;

        case 'U': {
          if ((cur - prev) < REL_UP_MIN_ST) return false;
        } break;

        case 'D': {
          if ((prev - cur) < REL_DOWN_MIN_ST) return false;
        } break;

        case 'S': {
          if (fabsf(cur - prev) > REL_SAME_TOL_ST) return false;
        } break;

        default:
          return false;
      }

      prev = cur;
      ++idx;
    }

    return (idx == ntones);
  }

  void pushToneToSequence(float midi, float freq, uint32_t now_ms) {
    if (_seq_len >= MAX_SEQ_TONES) {
      // Drop oldest if full.
      for (uint8_t i = 1; i < MAX_SEQ_TONES; ++i) {
        _seq_midi[i - 1] = _seq_midi[i];
        _seq_freq[i - 1] = _seq_freq[i];
      }
      _seq_len = MAX_SEQ_TONES - 1;
    }

    _seq_midi[_seq_len] = midi;
    _seq_freq[_seq_len] = freq;
    ++_seq_len;

    _last_tone_end_ms = now_ms;

    Serial.printf("[TONE] freq=%7.1f Hz  midi=%6.2f  seq_len=%u\n", freq, midi, (unsigned)_seq_len);
  }

  void startTone(const PitchFrame& pf, uint32_t now_ms) {
    _tone_active = true;
    _tone_start_ms = now_ms;
    _absent_frames = 0;

    _tone_freq_smooth = pf.freq_hz;
    _tone_midi_smooth = hzToMidi(pf.freq_hz);

    _tone_midi_min = _tone_midi_smooth;
    _tone_midi_max = _tone_midi_smooth;
    _tone_snr_sum  = 0.0f;
    _tone_p2_sum   = 0.0f;
    _tone_conc_sum = 0.0f;
    _tone_metric_frames = 0;
  }

  void updateToneState(const PitchFrame& pf, uint32_t now_ms) {
    uint32_t since_prev_candidate = now_ms - _last_candidate_ms;
    if (pf.tone_candidate) {
      _last_candidate_ms = now_ms;
    }

    if (!_tone_active) {
      if (pf.tone_candidate) {
        // IMPORTANT FIX:
        // Apply PRE_SILENCE only at the very beginning of a candidate run,
        // otherwise START_FRAMES can never be reached.
        if (_seq_len == 0 && _present_frames == 0 && since_prev_candidate < PRE_SILENCE_MS) {
          _present_frames = 0;
          return;
        }

        if (++_present_frames >= START_FRAMES) {
          // Reset sequence if previous ended too long ago.
          if (_seq_len > 0 && (now_ms - _last_tone_end_ms) > SEQ_HARD_RESET_MS) {
            _seq_len = 0;
          }
          startTone(pf, now_ms);
          _present_frames = 0;
        }
      } else {
        _present_frames = 0;
      }
      return;
    }

    // Tone active.
    if (pf.tone_candidate) {
      _absent_frames = 0;

      // Smooth during tone.
      const float a = 0.20f;
      float midi = hzToMidi(pf.freq_hz);
      _tone_freq_smooth = (1.0f - a) * _tone_freq_smooth + a * pf.freq_hz;
      _tone_midi_smooth = (1.0f - a) * _tone_midi_smooth + a * midi;

      // Quality metrics accumulation.
      if (_tone_midi_smooth < _tone_midi_min) _tone_midi_min = _tone_midi_smooth;
      if (_tone_midi_smooth > _tone_midi_max) _tone_midi_max = _tone_midi_smooth;

      _tone_snr_sum  += pf.snr;
      _tone_p2_sum   += pf.peak2_ratio;
      _tone_conc_sum += pf.conc;
      _tone_metric_frames++;
    } else {
      if (++_absent_frames >= END_FRAMES) {
        _tone_active = false;
        _absent_frames = 0;

        uint32_t dur = now_ms - _tone_start_ms;
        if (dur < MIN_TONE_MS) {
          Serial.println("[TONE] Ignored short chirp.");
          return;
        }

        if (_tone_metric_frames == 0) {
          Serial.println("[TONE] Rejected (no metric frames).");
          return;
        }

        float drift = _tone_midi_max - _tone_midi_min;
        float avg_snr  = _tone_snr_sum  / (float)_tone_metric_frames;
        float avg_p2   = _tone_p2_sum   / (float)_tone_metric_frames;
        float avg_conc = _tone_conc_sum / (float)_tone_metric_frames;

        bool ok = true;
        if (drift > TONE_MAX_DRIFT_ST) ok = false;
        if (avg_snr < TONE_MIN_AVG_SNR) ok = false;
        if (avg_p2  < TONE_MIN_AVG_P2)  ok = false;
        if (avg_conc < TONE_MIN_AVG_CONC) ok = false;

        if (!ok) {
          Serial.printf("[TONE] Rejected: dur=%ums drift=%.2fst snr=%.1f p2=%.2f conc=%.2f\n",
                        (unsigned)dur, drift, avg_snr, avg_p2, avg_conc);
          return;
        }

        pushToneToSequence(_tone_midi_smooth, _tone_freq_smooth, now_ms);
      }
    }
  }

  void evaluateSequenceIfEnded(uint32_t now_ms) {
    if (_tone_active) return;
    if (_seq_len == 0) return;

    if (now_ms - _last_tone_end_ms > SEQ_HARD_RESET_MS) {
      Serial.println("[SEQ] Hard reset (timeout).");
      _seq_len = 0;
      return;
    }

    if (now_ms - _last_tone_end_ms < SEQ_END_GAP_MS) {
      return;
    }

    if (now_ms - _last_candidate_ms < POST_SILENCE_MS) {
      return;
    }

    const CommandDef* best = nullptr;
    uint8_t best_len = 0;

    for (size_t ci = 0; ci < _cmd_count; ++ci) {
      const CommandDef& cmd = _cmds[ci];
      uint8_t plen = patternLength(cmd.pattern);
      if (plen == 0) continue;
      if (plen > _seq_len) continue;

      const float* midi_tail = &_seq_midi[_seq_len - plen];

      if (matchPattern(cmd.pattern, midi_tail, plen)) {
        if (plen > best_len) {
          best = &cmd;
          best_len = plen;
        }
      }
    }

    if (best) {
      const float* midi_tail = &_seq_midi[_seq_len - best_len];
      const float* freq_tail = &_seq_freq[_seq_len - best_len];

      _last_cmd_id = best->id;
      _last_cmd_name = best->name;
      _last_cmd_pattern = best->pattern;

      if (best->cb) {
        best->cb(best->id, best->name, best->pattern, midi_tail, freq_tail, best_len);
      }
    } else {
      Serial.println("[SEQ] No command matched.");
    }

    _seq_len = 0;
  }
};

// -----------------------------------------------------------------------------
// Global configuration
// -----------------------------------------------------------------------------
static constexpr const size_t SAMPLE_RATE      = 24000;
static constexpr const size_t FFT_BITS         = 10;        // 2^10 = 1024 samples
static constexpr const size_t WAVE_BLOCK_SIZE  = 256;
static constexpr const size_t FFT_SIZE         = 1u << FFT_BITS;
static constexpr const size_t WAVE_BLOCK_COUNT = 3 + FFT_SIZE / WAVE_BLOCK_SIZE;
static constexpr const size_t WAVE_TOTAL_SIZE  = WAVE_BLOCK_SIZE * WAVE_BLOCK_COUNT;

static fft_function_t fft_function;
static fft_data_t     fft_data;
static wav_data_t     wav_data;
static wav_drawer_t   wav_drawer;
static fft_drawer_t   fft_drawer;
static fft_history_t  fft_history;

static ToneSequencer  tone_seq;

// -----------------------------------------------------------------------------
// Small overlay: last command ID + live metrics
// -----------------------------------------------------------------------------
static void drawOverlay() {
  static uint32_t last_draw_ms = 0;
  uint32_t now = millis();
  if (now - last_draw_ms < 100) return;
  last_draw_ms = now;

  auto& d = M5.Display;
  int w = d.width();

  const int HUD_H = 20;

  d.fillRect(0, 0, w, HUD_H, 0x000000u);

  d.setTextColor(0xFFFF00u, 0x000000u);
  d.setTextSize(2);
  d.setCursor(0, 0);

  int cmd = tone_seq.lastCommandId();
  if (cmd > 0) {
    d.printf("%02d", cmd);
  } else {
    d.print("--");
  }

  // Candidate / tone indicators.
  d.setTextSize(1);
  d.setTextColor(0xFFFFFFu, 0x000000u);
  d.setCursor(0, 12);
  d.print(tone_seq.liveCandidate() ? "C" : ".");
  d.print(tone_seq.isToneActive()  ? "T" : ".");

  d.setCursor(30, 4);
  d.printf("SEQ:%u", (unsigned)tone_seq.sequenceLength());

  d.setCursor(78, 4);
  if (tone_seq.isToneActive()) {
    d.printf("%4.0fHz", tone_seq.liveFreqHz());
  } else {
    d.print("----Hz");
  }

  d.setCursor(120, 4);
  d.printf("S%.0f", tone_seq.liveSNR());

  d.setCursor(30, 12);
  d.printf("R%.1f", tone_seq.livePeak2());

  d.setCursor(78, 12);
  d.printf("C%.2f", tone_seq.liveConc());
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup(void) {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.println();
  Serial.println("=== M5Unified Mic FFT + Tone Sequence Commands (fixed PRE_SILENCE) ===");

  auto cfg = M5.config();
  cfg.internal_mic   = true;
  cfg.internal_spk   = false;
  cfg.clear_display  = true;
  cfg.output_power   = true;
  cfg.fallback_board = m5::board_t::board_M5StickC;

  M5.begin(cfg);

  // Re-init serial after M5.begin (some environments reconfigure UART/USB).
  Serial.begin(SERIAL_BAUD);
  delay(50);

  // Use landscape orientation.
  M5.Display.setRotation(3);
  M5.Display.setBrightness(200);
  M5.Display.fillScreen(0x000000U);

  // Configure microphone via M5Unified.
  {
    auto mc = M5.Mic.config();
    mc.dma_buf_count      = 3;
    mc.dma_buf_len        = WAVE_BLOCK_SIZE;
    mc.over_sampling      = 1;
    mc.noise_filter_level = 0;
    mc.sample_rate        = SAMPLE_RATE;
    mc.magnification      = mc.use_adc ? 16 : 1;
    M5.Mic.config(mc);

    Serial.printf("Mic cfg: Fs=%u, dma_len=%u, dma_cnt=%u, over=%u, nf=%u, mag=%u, use_adc=%u\n",
                  (unsigned)mc.sample_rate,
                  (unsigned)mc.dma_buf_len,
                  (unsigned)mc.dma_buf_count,
                  (unsigned)mc.over_sampling,
                  (unsigned)mc.noise_filter_level,
                  (unsigned)mc.magnification,
                  (unsigned)mc.use_adc);
  }

  if (!M5.Mic.isEnabled()) {
    M5.Display.setCursor(0, 0);
    M5.Display.setTextColor(0xFFFF00U, 0x000000U);
    M5.Display.println("Microphone not available");
    for (;;) { delay(500); }
  }

  // Ensure speaker is off to avoid I2S conflict.
  M5.Speaker.end();
  M5.Mic.begin();

  // Display: continuous write for speed.
  M5.Display.startWrite();

  int16_t w = M5.Display.width();
  int16_t h = M5.Display.height() / 3;
  int16_t y = 0;

  rect_t rect_fft_drawer   = {0, y, w, h}; y += h;
  rect_t rect_fft_history  = {0, y, w, h}; y += h;
  rect_t rect_wav_drawer   = {0, y, w, h};

  fft_function.setup(FFT_BITS);
  fft_drawer.setup(&M5.Display, rect_fft_drawer);
  fft_history.setup(&M5.Display, rect_fft_history);
  wav_drawer.setup(&M5.Display, rect_wav_drawer);

  M5.Display.setFont(&fonts::AsciiFont8x16);
  M5.Display.setTextSize(1);

  // Allocate FFT + WAV buffers.
  fft_data.fft_size_bits = FFT_BITS;
  fft_data.sample_rate   = SAMPLE_RATE;
  fft_data.wav_data      = &wav_data;
  fft_data.length        = (1 << (fft_data.fft_size_bits - 1)) + 1;
  fft_data.fdata         = (float*)memory_alloc(fft_data.length * sizeof(float));

  wav_data.length        = WAVE_TOTAL_SIZE;
  wav_data.wav           = (int16_t*)memory_alloc(WAVE_TOTAL_SIZE * sizeof(int16_t));
  wav_data.latest_index  = 0;

  memset(wav_data.wav, 0, WAVE_TOTAL_SIZE * sizeof(int16_t));

  tone_seq.begin(COMMANDS, sizeof(COMMANDS) / sizeof(COMMANDS[0]));

  drawOverlay();
  Serial.println("[BOOT] Ready.");
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop(void) {
  M5.update();

  static int step = -1;

  if (M5.Mic.isEnabled()) {
    int wav_idx = (int)wav_data.latest_index;

    // Pull enough audio blocks from M5Unified internal queue.
    while (M5.Mic.isRecording() < 2) {
      M5.Mic.record(&(wav_data.wav[wav_idx]),
                    WAVE_BLOCK_SIZE,
                    SAMPLE_RATE,
                    false);  // mono
      wav_idx += (int)WAVE_BLOCK_SIZE;
      if ((size_t)wav_idx >= WAVE_TOTAL_SIZE) {
        wav_idx = 0;
      }
      wav_data.latest_index = (size_t)wav_idx;
    }

    // Spread heavy work over frames.
    switch (++step) {
      default:
      case 0:
        step = 0;
        M5.Display.display();

        fft_function.update(&fft_data);
        tone_seq.updateFromFFT(fft_data);
        break;

      case 1:
        wav_drawer.update(wav_data);
        break;

      case 2:
        fft_drawer.update(fft_data);
        break;

      case 3:
        fft_history.update(fft_data);
        break;
    }
  }

  // HUD on top of graphs.
  drawOverlay();
}
