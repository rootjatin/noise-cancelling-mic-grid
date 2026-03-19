/*
  5-mic adaptive noise reduction demo for ESP32 (Arduino core)

  Layout:
    - 1 main mic   = desired speech + noise
    - 4 ref mics   = ambient noise references

  IMPORTANT:
    - Use AO pins only
    - DO pins are NOT used
    - This is NOT true ANC
    - This is NOT true beamforming
    - This is a multi-reference adaptive noise reduction experiment

  Serial Plotter output:
    rawMain rawRefAvg cleaned

  Suggested placement:
    - MAIN mic: 2 to 5 cm from mouth
    - REF mics: around headset / room-facing, farther from mouth

  Better results:
    - Match gain on all modules as closely as possible
    - Use short wires, good grounding
    - Keep ref mics away from direct mouth sound
*/

#define MAIN_MIC_PIN 34

const int REF_PINS[4] = {35, 32, 33, 36};
const int NUM_REF = 4;

// Set to 1 only on classic ESP32 boards with dacWrite() on GPIO25/26
#define USE_DAC_OUTPUT 1
#define DAC_PIN 25

const uint32_t SAMPLE_RATE = 8000;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

// More taps = better modeling, but more CPU and slower adaptation
const int TAPS = 12;

// Adaptive filter weights and histories
float w[NUM_REF][TAPS];
float xHist[NUM_REF][TAPS];

// DC tracking
float dcMain = 2048.0f;
float dcRef[NUM_REF] = {2048.0f, 2048.0f, 2048.0f, 2048.0f};
const float DC_ALPHA = 0.002f;

// Tuning
// Lower MU if unstable or "chirpy"
// Increase MU slightly if cancellation is too weak
const float MU = 0.08f;
const float LEAK = 0.9998f;

// Simple speech-protection gate:
// if main mic energy is much larger than reference energy,
// pause adaptation so the filter does not "learn" your voice too much.
const float SPEECH_HOLD_FACTOR = 3.0f;

// Optional output limiting
const float LIMIT_E = 1400.0f;

void setup() {
  Serial.begin(921600);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(MAIN_MIC_PIN, INPUT);
  for (int r = 0; r < NUM_REF; r++) {
    pinMode(REF_PINS[r], INPUT);
  }

#if USE_DAC_OUTPUT
  pinMode(DAC_PIN, OUTPUT);
#endif

  for (int r = 0; r < NUM_REF; r++) {
    for (int i = 0; i < TAPS; i++) {
      w[r][i] = 0.0f;
      xHist[r][i] = 0.0f;
    }
  }

  Serial.println("rawMain rawRefAvg cleaned");
}

void loop() {
  static uint32_t nextSample = micros();

  if ((int32_t)(micros() - nextSample) < 0) {
    return;
  }
  nextSample += SAMPLE_PERIOD_US;

  // -----------------------------
  // 1) Read all microphones
  // -----------------------------
  int rawMain = analogRead(MAIN_MIC_PIN);

  int rawRef[NUM_REF];
  for (int r = 0; r < NUM_REF; r++) {
    rawRef[r] = analogRead(REF_PINS[r]);
  }

  // -----------------------------
  // 2) Remove slow DC offset
  // -----------------------------
  dcMain += DC_ALPHA * ((float)rawMain - dcMain);
  float d = (float)rawMain - dcMain;   // desired speech + noise

  float refAvgRaw = 0.0f;

  for (int r = 0; r < NUM_REF; r++) {
    dcRef[r] += DC_ALPHA * ((float)rawRef[r] - dcRef[r]);
    float x = (float)rawRef[r] - dcRef[r];  // reference noise
    refAvgRaw += rawRef[r];

    // Shift history
    for (int i = TAPS - 1; i > 0; i--) {
      xHist[r][i] = xHist[r][i - 1];
    }
    xHist[r][0] = x;
  }

  refAvgRaw /= NUM_REF;

  // -----------------------------
  // 3) Estimate noise from all refs
  // -----------------------------
  float yhat = 0.0f;
  float power = 1.0f;
  float refNowPower = 1.0f;

  for (int r = 0; r < NUM_REF; r++) {
    refNowPower += xHist[r][0] * xHist[r][0];

    for (int i = 0; i < TAPS; i++) {
      yhat += w[r][i] * xHist[r][i];
      power += xHist[r][i] * xHist[r][i];
    }
  }

  // Residual = cleaned output
  float e = d - yhat;

  // -----------------------------
  // 4) Adapt weights (NLMS)
  // -----------------------------
  float mainNowPower = d * d;
  bool allowAdapt = (mainNowPower < (SPEECH_HOLD_FACTOR * refNowPower));

  if (allowAdapt) {
    float step = MU / power;

    for (int r = 0; r < NUM_REF; r++) {
      for (int i = 0; i < TAPS; i++) {
        w[r][i] = LEAK * w[r][i] + step * e * xHist[r][i];
      }
    }
  } else {
    // Leak only, so filter slowly forgets old state without learning speech
    for (int r = 0; r < NUM_REF; r++) {
      for (int i = 0; i < TAPS; i++) {
        w[r][i] *= LEAK;
      }
    }
  }

  // -----------------------------
  // 5) Soft limit
  // -----------------------------
  if (e > LIMIT_E) e = LIMIT_E;
  if (e < -LIMIT_E) e = -LIMIT_E;

#if USE_DAC_OUTPUT
  // Classic ESP32 DAC is 8-bit
  int dacValue = (int)(e * 0.09f) + 128;
  if (dacValue < 0) dacValue = 0;
  if (dacValue > 255) dacValue = 255;
  dacWrite(DAC_PIN, dacValue);
#endif

  // -----------------------------
  // 6) Serial Plotter
  // -----------------------------
  static uint8_t plotDivider = 0;
  plotDivider++;
  if (plotDivider >= 8) {
    plotDivider = 0;

    int cleanedPlot = (int)e + 2048;
    if (cleanedPlot < 0) cleanedPlot = 0;
    if (cleanedPlot > 4095) cleanedPlot = 4095;

    Serial.print(rawMain);
    Serial.print(' ');
    Serial.print((int)refAvgRaw);
    Serial.print(' ');
    Serial.println(cleanedPlot);
  }
}
