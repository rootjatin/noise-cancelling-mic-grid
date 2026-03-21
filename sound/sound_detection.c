//no recording
#include "BluetoothA2DPSource.h"

BluetoothA2DPSource a2dp_source;
const char *speaker_name = "Airdopes 411ANC";

const int MIC_PIN = 34;
const int BUFFER_SIZE = 8192;

volatile int16_t sampleBuffer[BUFFER_SIZE];
volatile int writeIndex = 0;
volatile int readIndex = 0;

float dcOffset = 2048.0f;

// Increase these
int gainValue = 120;   // was low before
int noiseGate = 3;     // keep very small so speech is not removed

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

void micTask(void *param) {
  while (true) {
    int raw = analogRead(MIC_PIN);

    // track center slowly
    dcOffset = dcOffset * 0.9995f + raw * 0.0005f;

    float centered = raw - dcOffset;

    // very small noise gate
    if (centered < noiseGate && centered > -noiseGate) {
      centered = 0;
    }

    int sample = (int)(centered * gainValue);

    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;

    pushSample((int16_t)sample);

    delayMicroseconds(8);
  }
}

int32_t get_audio_data(uint8_t *data, int32_t len) {
  int16_t *out = (int16_t *)data;
  int frames = len / 4;

  for (int i = 0; i < frames; i++) {
    int16_t s = popSample();
    out[2 * i] = s;
    out[2 * i + 1] = s;
  }

  return frames * 4;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  xTaskCreatePinnedToCore(
    micTask,
    "MicTask",
    4096,
    NULL,
    1,
    NULL,
    1
  );

  a2dp_source.set_auto_reconnect(true);
  a2dp_source.set_volume(100);
  a2dp_source.set_data_callback(get_audio_data);
  a2dp_source.start(speaker_name);
}

void loop() {
  delay(10);
}
