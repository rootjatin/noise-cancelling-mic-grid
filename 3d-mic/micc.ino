#include "BluetoothA2DPSource.h"
#include "driver/i2s.h"
#include <math.h>

BluetoothA2DPSource a2dp_source;
const char *speaker_name = "Airdopes 411ANC";

// ---------------- I2S MIC PINS ----------------
#define I2S_PORT I2S_NUM_0
#define I2S_WS   25
#define I2S_SCK  26
#define I2S_SD   33

// If L/R pin is connected to GND, use ONLY_LEFT
// If L/R pin is connected to 3.3V, change this to ONLY_RIGHT
#define MIC_CHANNEL_FORMAT I2S_CHANNEL_FMT_ONLY_LEFT

// ---------------- AUDIO BUFFER ----------------
const int BUFFER_SIZE = 16384;
volatile int16_t sampleBuffer[BUFFER_SIZE];
volatile int writeIndex = 0;
volatile int readIndex  = 0;

// ---------------- USER TUNING ----------------
// Input level
float micGain = 11.5f;
int noiseGate = 10;

// Cinematic tone
float rumbleCutHz   = 75.0f;    // removes boom/handling noise
float bassCutoffHz  = 150.0f;   // low body
float mudCutoffHz   = 320.0f;   // reduce boxiness
float presenceHz    = 1800.0f;  // bring clarity

float bassBoost     = 0.55f;    // 0.35..0.75
float mudCut        = 0.22f;    // 0.10..0.35
float presenceBoost = 0.16f;    // 0.08..0.24

// Harmonics
float evenDrive = 1.35f;        // 2nd harmonic drive
float evenMix   = 0.16f;        // warmth

float oddMix    = 0.14f;        // 3rd harmonic density
float finalDrive = 1.20f;       // final saturation / seriousness

// Compression
float compThreshold = 0.26f;    // lower = more compression
float compRatio     = 2.8f;     // 2.0..4.0
float compAttack    = 0.08f;
float compRelease   = 0.003f;

float outputGain    = 1.05f;    // final level

// ---------------- DSP STATE ----------------
float bassLP = 0.0f;
float mudLP  = 0.0f;
float presLP = 0.0f;

float bassAlpha = 0.0f;
float mudAlpha  = 0.0f;
float presAlpha = 0.0f;

// High-pass (rumble cut) state
float hp_x1 = 0.0f;
float hp_y1 = 0.0f;
float hpA   = 0.0f;

// DC blockers
float dc1_x1 = 0.0f, dc1_y1 = 0.0f;
float dc2_x1 = 0.0f, dc2_y1 = 0.0f;
const float dcR = 0.995f;

// Compressor envelope
float compEnv = 0.0f;

void pushSample(int16_t s) {
  int next = (writeIndex + 1) % BUFFER_SIZE;
  if (next != readIndex) {
    sampleBuffer[writeIndex] = s;
    writeIndex = next;
  }
}

int16_t popSample() {
  if (readIndex == writeIndex) return 0;
  int16_t s = sampleBuffer[readIndex];
  readIndex = (readIndex + 1) % BUFFER_SIZE;
  return s;
}

float onePoleAlpha(float cutoffHz, float fs) {
  return (2.0f * PI * cutoffHz) / (fs + 2.0f * PI * cutoffHz);
}

float highPassCoeff(float cutoffHz, float fs) {
  return fs / (fs + 2.0f * PI * cutoffHz);
}

float dcBlock1(float x) {
  float y = x - dc1_x1 + dcR * dc1_y1;
  dc1_x1 = x;
  dc1_y1 = y;
  return y;
}

float dcBlock2(float x) {
  float y = x - dc2_x1 + dcR * dc2_y1;
  dc2_x1 = x;
  dc2_y1 = y;
  return y;
}

// gentle, fast saturator
float softClip(float x) {
  if (x > 1.5f) x = 1.5f;
  if (x < -1.5f) x = -1.5f;
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

// very light compressor for spoken voice
float compressSample(float x) {
  float level = fabsf(x);

  if (level > compEnv) {
    compEnv += compAttack * (level - compEnv);
  } else {
    compEnv += compRelease * (level - compEnv);
  }

  float gain = 1.0f;
  if (compEnv > compThreshold) {
    float desired = compThreshold + (compEnv - compThreshold) / compRatio;
    gain = desired / compEnv;
  }

  return x * gain;
}

int16_t processVoiceSample(int32_t raw) {
  // 1) Convert 32-bit I2S slot to useful sample
  float x = (float)(raw >> 14);

  // 2) Noise gate
  if (x < noiseGate && x > -noiseGate) {
    x = 0.0f;
  }

  // 3) Mic gain
  x *= micGain;

  // 4) Remove DC before normalization
  x = dcBlock1(x);

  // 5) Normalize to [-1, 1] roughly
  float xn = x / 32768.0f;

  // 6) Remove low rumble / handling noise
  float hp = hpA * (hp_y1 + xn - hp_x1);
  hp_x1 = xn;
  hp_y1 = hp;

  // 7) Bass / body
  bassLP += bassAlpha * (hp - bassLP);
  float body = hp + bassBoost * bassLP;

  // 8) Remove muddy low-mid band
  mudLP += mudAlpha * (body - mudLP);
  float mudBand = mudLP - bassLP;         // approx low-mid region
  float cleaned = body - mudCut * mudBand;

  // 9) Add presence / clarity
  presLP += presAlpha * (cleaned - presLP);
  float presenceBand = cleaned - presLP;  // upper content
  float voiced = cleaned + presenceBoost * presenceBand;

  // 10) Add 2nd harmonic (warmth) using asymmetry
  float evenGen = softClip((voiced + 0.35f * voiced * voiced) * evenDrive)
                  - softClip(voiced * evenDrive);

  // 11) Add 3rd harmonic (serious density)
  float oddGen = voiced * voiced * voiced;

  float harmonicVoice = voiced + evenMix * evenGen + oddMix * oddGen;

  // 12) Remove DC again because even-harmonic generation can add offset
  harmonicVoice = dcBlock2(harmonicVoice);

  // 13) Light compression for stable narration feel
  harmonicVoice = compressSample(harmonicVoice);

  // 14) Final drive / cinematic thickness
  float y = softClip(harmonicVoice * finalDrive);

  // 15) Final trim
  y *= outputGain;

  // 16) Clamp
  if (y > 1.0f) y = 1.0f;
  if (y < -1.0f) y = -1.0f;

  return (int16_t)(y * 32767.0f);
}

void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = MIC_CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);

  const float fs = 44100.0f;
  bassAlpha = onePoleAlpha(bassCutoffHz, fs);
  mudAlpha  = onePoleAlpha(mudCutoffHz, fs);
  presAlpha = onePoleAlpha(presenceHz, fs);
  hpA       = highPassCoeff(rumbleCutHz, fs);
}

void micTask(void *param) {
  const int BLOCK_SAMPLES = 256;
  int32_t rawSamples[BLOCK_SAMPLES];
  size_t bytesRead = 0;

  while (true) {
    esp_err_t result = i2s_read(
                         I2S_PORT,
                         (void *)rawSamples,
                         sizeof(rawSamples),
                         &bytesRead,
                         portMAX_DELAY
                       );

    if (result == ESP_OK && bytesRead > 0) {
      int count = bytesRead / sizeof(int32_t);

      for (int i = 0; i < count; i++) {
        int16_t s = processVoiceSample(rawSamples[i]);
        pushSample(s);
      }
    }
  }
}

// A2DP callback: must provide stereo 16-bit PCM
int32_t get_audio_data(uint8_t *data, int32_t len) {
  int16_t *out = (int16_t *)data;
  int frames = len / 4;  // 4 bytes per stereo frame

  for (int i = 0; i < frames; i++) {
    int16_t s = popSample();

    // same mono sample to L + R
    out[2 * i]     = s;
    out[2 * i + 1] = s;
  }

  return frames * 4;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting I2S mic...");
  setupI2SMic();

  xTaskCreatePinnedToCore(
    micTask,
    "MicTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("Starting Bluetooth A2DP source...");
  a2dp_source.set_auto_reconnect(true);
  a2dp_source.set_volume(127);
  a2dp_source.set_data_callback(get_audio_data);
  a2dp_source.start(speaker_name);

  Serial.println("Ready. Put your Bluetooth device in pairing mode.");
}

void loop() {
  delay(10);
}
