#include "BluetoothA2DPSource.h"
#include "driver/i2s.h"

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

// Mic tuning
int micGain   = 8;   // increase if voice is low: 8, 12, 16
int noiseGate = 10;  // increase if too much hiss/noise

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
        // INMP441-style mics often give 24-bit audio in a 32-bit slot.
        // Shift down to usable range, then apply gain.
        int32_t s = rawSamples[i] >> 14;

        if (s < noiseGate && s > -noiseGate) {
          s = 0;
        }

        s *= micGain;

        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;

        pushSample((int16_t)s);
      }
    }
  }
}

// A2DP callback: must provide stereo 16-bit PCM
int32_t get_audio_data(uint8_t *data, int32_t len) {
  int16_t *out = (int16_t *)data;
  int frames = len / 4;  // 4 bytes per stereo frame (L+R, 16-bit each)

  for (int i = 0; i < frames; i++) {
    int16_t s = popSample();

    // Send same mono mic sample to both L and R
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
  a2dp_source.set_volume(100);
  a2dp_source.set_data_callback(get_audio_data);
  a2dp_source.start(speaker_name);

  Serial.println("Ready. Put your Bluetooth device in pairing mode.");
}

void loop() {
  delay(10);
}
