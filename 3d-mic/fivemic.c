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
    - This is a multi-reference adaptive noise-reduction experiment

  Serial Plotter output:
    rawMain rawRefAvg cleaned

  Notes:
    - LM393 mic modules are not ideal for good cancellation
    - ESP32 ADC channels are sampled one by one, not simultaneously
    - Best results happen when reference mics capture noise much more than speech
*/

#define MAIN_MIC_PIN 34

constexpr int NUM_REF = 4;
const int REF_PINS[NUM_REF] = {35, 32, 33, 36};

// Set to 1 only on classic ESP32 boards with DAC on GPIO25/26
#define USE_DAC_OUTPUT 1
#define DAC_PIN 25

constexpr uint32_t SAMPLE_RATE = 8000;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

// Filter length
constexpr int TAPS = 12;

// Adaptive filter state
float w[NUM_REF][TAPS];
float xHist[NUM_REF][TAPS];

// DC trackers
float dcMain = 2048.0f;
float dcRef[NUM_REF] = {2048.0f, 2048.0f, 2048.0f, 2048.0f};

// Running energy trackers for normalization
float refRms2[NUM_REF] = {2000.0f, 2000.0f, 2000.0f, 2000.0f};
float mainRms2 = 2000.0f;

// Output smoothing
float eSmooth = 0.0f;

// Tunables
constexpr float DC_ALPHA = 0.0020f;
constexpr float RMS_ALPHA = 0.0100f;

// Lower MU if unstable. Raise slightly if effect is too weak.
constexpr float MU = 0.05f;
constexpr float LEAK = 0.9998f;
constexpr float EPSILON = 1.0f;

// Adapt only when reference noise is present
constexpr float REF_ACTIVITY_THRESHOLD = 80.0f * 80.0f;

// Speech protection: if main energy is much larger than ref energy,
// freeze adaptation to reduce voice cancellation.
constexpr float SPEECH_HOLD_FACTOR = 3.5f;

// Output limiting and smoothing
constexpr float LIMIT_E = 1300.0f;
constexpr float OUTPUT_SMOOTH_ALPHA = 0.25f;

void resetState() {
  for (int r = 0; r < NUM_REF; r++) {
    for (int i = 0; i < TAPS; i++) {
      w[r][i] = 0.0f;
      xHist[r][i] = 0.0f;
    }
  }
}

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

  resetState();

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
  // 2) DC removal
  // -----------------------------
  dcMain += DC_ALPHA * ((float)rawMain - dcMain);
  float d = (float)rawMain - dcMain;

  float refAvgRaw = 0.0f;
  float refInstantPower = 0.0f;

  for (int r = 0; r < NUM_REF; r++) {
    dcRef[r] += DC_ALPHA * ((float)rawRef[r] - dcRef[r]);
    float x = (float)rawRef[r] - dcRef[r];

    // Track ref RMS power
    refRms2[r] += RMS_ALPHA * (x * x - refRms2[r]);

    // Normalize each reference so one hot mic does not dominate
    float xNorm = x / sqrtf(refRms2[r] + EPSILON);

    // Shift history
    for (int i = TAPS - 1; i > 0; i--) {
      xHist[r][i] = xHist[r][i - 1];
    }
    xHist[r][0] = xNorm;

    refAvgRaw += rawRef[r];
    refInstantPower += x * x;
  }

  refAvgRaw /= NUM_REF;

  // Track main RMS power
  mainRms2 += RMS_ALPHA * (d * d - mainRms2);

  // -----------------------------
  // 3) Estimate noise from refs
  // -----------------------------
  float yhat = 0.0f;
  float totalPower = 1.0f;

  for (int r = 0; r < NUM_REF; r++) {
    for (int i = 0; i < TAPS; i++) {
      yhat += w[r][i] * xHist[r][i];
      totalPower += xHist[r][i] * xHist[r][i];
    }
  }

  // Residual = cleaned signal
  float e = d - yhat;

  // -----------------------------
  // 4) Controlled adaptation
  // -----------------------------
  bool refActive = (refInstantPower > REF_ACTIVITY_THRESHOLD);

  // Compare main power to reference power:
  // if main looks much stronger, likely speech is dominant
  bool speechDominant = (mainRms2 > (SPEECH_HOLD_FACTOR * (refInstantPower / NUM_REF + EPSILON)));

  if (refActive && !speechDominant) {
    float step = MU / totalPower;

    for (int r = 0; r < NUM_REF; r++) {
      for (int i = 0; i < TAPS; i++) {
        w[r][i] = LEAK * w[r][i] + step * e * xHist[r][i];
      }
    }
  } else {
    // Slow leak only
    for (int r = 0; r < NUM_REF; r++) {
      for (int i = 0; i < TAPS; i++) {
        w[r][i] *= LEAK;
      }
    }
  }

  // -----------------------------
  // 5) Output limiting + smoothing
  // -----------------------------
  if (e > LIMIT_E) e = LIMIT_E;
  if (e < -LIMIT_E) e = -LIMIT_E;

  eSmooth += OUTPUT_SMOOTH_ALPHA * (e - eSmooth);

#if USE_DAC_OUTPUT
  // Classic ESP32 DAC is 8-bit, centered around 128
  int dacValue = (int)(eSmooth * 0.09f) + 128;
  if (dacValue < 0) dacValue = 0;
  if (dacValue > 255) dacValue = 255;
  dacWrite(DAC_PIN, dacValue);
#endif

  // -----------------------------
  // 6) Serial Plotter output
  // -----------------------------
  static uint8_t plotDivider = 0;
  plotDivider++;
  if (plotDivider >= 8) {
    plotDivider = 0;

    int cleanedPlot = (int)eSmooth + 2048;
    if (cleanedPlot < 0) cleanedPlot = 0;
    if (cleanedPlot > 4095) cleanedPlot = 4095;

    Serial.print(rawMain);
    Serial.print(' ');
    Serial.print((int)refAvgRaw);
    Serial.print(' ');
    Serial.println(cleanedPlot);
  }
}
