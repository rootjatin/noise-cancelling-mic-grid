/*
  Two LM393 mic modules -> limited adaptive noise reduction demo
  Board: ESP32 (Arduino core)

  IMPORTANT:
  - Use AO pins only
  - DO pins are NOT used
  - This is NOT true ANC and NOT real beamforming
  - It is a simple adaptive noise-reduction experiment

  Mic 1 = "signal mic" (near mouth / desired sound)
  Mic 2 = "noise mic"  (captures ambient noise)

  Serial Plotter output:
  rawSignal rawNoise cleaned

  Optional:
  On classic ESP32 boards, you can enable DAC output on GPIO25.
*/

#define MIC_SIGNAL_PIN 34
#define MIC_NOISE_PIN  35

// Set to 1 only on classic ESP32 boards that support dacWrite(GPIO25/26)
#define USE_DAC_OUTPUT 1
#define DAC_PIN 25

const uint32_t SAMPLE_RATE = 8000;
const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

const int TAPS = 16;          // adaptive filter length
float w[TAPS];
float xHist[TAPS];

float dcSignal = 2048.0f;
float dcNoise  = 2048.0f;
const float DC_ALPHA = 0.002f;

// Adaptive speed.
// If unstable/noisy, reduce MU.
// If effect is too weak, increase MU slightly.
const float MU   = 0.25f;
const float LEAK = 0.9999f;

void setup() {
  Serial.begin(921600);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // wider input range

  pinMode(MIC_SIGNAL_PIN, INPUT);
  pinMode(MIC_NOISE_PIN, INPUT);

#if USE_DAC_OUTPUT
  pinMode(DAC_PIN, OUTPUT);
#endif

  for (int i = 0; i < TAPS; i++) {
    w[i] = 0.0f;
    xHist[i] = 0.0f;
  }

  Serial.println("rawSignal rawNoise cleaned");
}

void loop() {
  static uint32_t nextSample = micros();

  if ((int32_t)(micros() - nextSample) < 0) {
    return;
  }
  nextSample += SAMPLE_PERIOD_US;

  // Read both microphones
  int rawSignal = analogRead(MIC_SIGNAL_PIN);
  int rawNoise  = analogRead(MIC_NOISE_PIN);

  // Remove DC / slow offset
  dcSignal += DC_ALPHA * ((float)rawSignal - dcSignal);
  dcNoise  += DC_ALPHA * ((float)rawNoise  - dcNoise);

  float d = (float)rawSignal - dcSignal;  // desired + noise
  float x = (float)rawNoise  - dcNoise;   // reference noise

  // Shift noise history
  for (int i = TAPS - 1; i > 0; i--) {
    xHist[i] = xHist[i - 1];
  }
  xHist[0] = x;

  // Estimate correlated noise from reference mic
  float yhat = 0.0f;
  float power = 1.0f;
  for (int i = 0; i < TAPS; i++) {
    yhat  += w[i] * xHist[i];
    power += xHist[i] * xHist[i];
  }

  // Residual = cleaned signal
  float e = d - yhat;

  // NLMS update
  float step = MU / power;
  for (int i = 0; i < TAPS; i++) {
    w[i] = LEAK * w[i] + step * e * xHist[i];
  }

  // Soft limit for viewing/output
  if (e > 1400.0f) e = 1400.0f;
  if (e < -1400.0f) e = -1400.0f;

#if USE_DAC_OUTPUT
  // Classic ESP32 DAC is 8-bit, center at midscale
  int dacValue = (int)(e * 0.09f) + 128;
  if (dacValue < 0) dacValue = 0;
  if (dacValue > 255) dacValue = 255;
  dacWrite(DAC_PIN, dacValue);
#endif

  // Send slower data to Serial Plotter
  static uint8_t plotDivider = 0;
  plotDivider++;
  if (plotDivider >= 8) {
    plotDivider = 0;

    int cleanedPlot = (int)e + 2048;
    if (cleanedPlot < 0) cleanedPlot = 0;
    if (cleanedPlot > 4095) cleanedPlot = 4095;

    Serial.print(rawSignal);
    Serial.print(' ');
    Serial.print(rawNoise);
    Serial.print(' ');
    Serial.println(cleanedPlot);
  }
}
