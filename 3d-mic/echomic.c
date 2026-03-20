/*
  ESP32 3-Microphone Echo Effect Demo (Arduino)

  What it does:
    - Reads 3 analog microphones
    - Removes DC offset
    - Mixes them into one voice signal
    - Adds echo using a delay buffer
    - Outputs to ESP32 DAC

  Hardware notes:
    - Use AO pins from analog mic modules
    - DO pins are NOT used
    - Best on classic ESP32 with DAC output on GPIO25 or GPIO26
    - LM393 mic modules are low quality for real audio, but okay for experiments

  Serial Plotter output:
    mic1 mic2 mic3 mixed echoed
*/

#define MIC1_PIN 34
#define MIC2_PIN 35
#define MIC3_PIN 32

// Set to 1 only for classic ESP32 boards with DAC
#define USE_DAC_OUTPUT 1
#define DAC_PIN 25

constexpr uint32_t SAMPLE_RATE = 8000;
constexpr uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE;

// -----------------------------
// Echo settings
// -----------------------------
constexpr float ECHO_DELAY_MS = 220.0f;     // echo delay
constexpr float ECHO_FEEDBACK = 0.45f;      // how much echo repeats
constexpr float ECHO_MIX = 0.55f;           // how much delayed signal is added
constexpr float INPUT_GAIN = 1.2f;          // gain on mic mix
constexpr float LIMIT_LEVEL = 1400.0f;      // limiter

// Delay buffer size
constexpr int DELAY_SAMPLES = (int)(SAMPLE_RATE * ECHO_DELAY_MS / 1000.0f);
float delayBuffer[DELAY_SAMPLES];
int delayIndex = 0;

// DC trackers
float dc1 = 2048.0f;
float dc2 = 2048.0f;
float dc3 = 2048.0f;

constexpr float DC_ALPHA = 0.0020f;

// Smoothing
float outSmooth = 0.0f;
constexpr float OUTPUT_SMOOTH_ALPHA = 0.22f;

void setup() {
  Serial.begin(921600);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(MIC1_PIN, INPUT);
  pinMode(MIC2_PIN, INPUT);
  pinMode(MIC3_PIN, INPUT);

#if USE_DAC_OUTPUT
  pinMode(DAC_PIN, OUTPUT);
#endif

  for (int i = 0; i < DELAY_SAMPLES; i++) {
    delayBuffer[i] = 0.0f;
  }

  Serial.println("mic1 mic2 mic3 mixed echoed");
}

void loop() {
  static uint32_t nextSample = micros();

  if ((int32_t)(micros() - nextSample) < 0) {
    return;
  }
  nextSample += SAMPLE_PERIOD_US;

  // -----------------------------
  // 1) Read microphones
  // -----------------------------
  int raw1 = analogRead(MIC1_PIN);
  int raw2 = analogRead(MIC2_PIN);
  int raw3 = analogRead(MIC3_PIN);

  // -----------------------------
  // 2) Remove DC offset
  // -----------------------------
  dc1 += DC_ALPHA * ((float)raw1 - dc1);
  dc2 += DC_ALPHA * ((float)raw2 - dc2);
  dc3 += DC_ALPHA * ((float)raw3 - dc3);

  float s1 = (float)raw1 - dc1;
  float s2 = (float)raw2 - dc2;
  float s3 = (float)raw3 - dc3;

  // -----------------------------
  // 3) Mix 3 mics
  // -----------------------------
  float mixed = (s1 + s2 + s3) / 3.0f;
  mixed *= INPUT_GAIN;

  // -----------------------------
  // 4) Echo effect
  // -----------------------------
  float delayed = delayBuffer[delayIndex];

  // output = dry + wet
  float echoed = mixed + (ECHO_MIX * delayed);

  // store with feedback for repeating echo
  delayBuffer[delayIndex] = mixed + (delayed * ECHO_FEEDBACK);

  delayIndex++;
  if (delayIndex >= DELAY_SAMPLES) {
    delayIndex = 0;
  }

  // -----------------------------
  // 5) Limiter + smoothing
  // -----------------------------
  if (echoed > LIMIT_LEVEL) echoed = LIMIT_LEVEL;
  if (echoed < -LIMIT_LEVEL) echoed = -LIMIT_LEVEL;

  outSmooth += OUTPUT_SMOOTH_ALPHA * (echoed - outSmooth);

#if USE_DAC_OUTPUT
  // Classic ESP32 DAC is 8-bit: 0..255, centered at 128
  int dacValue = (int)(outSmooth * 0.09f) + 128;
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

    int mixedPlot = (int)mixed + 2048;
    int echoPlot  = (int)outSmooth + 2048;

    if (mixedPlot < 0) mixedPlot = 0;
    if (mixedPlot > 4095) mixedPlot = 4095;

    if (echoPlot < 0) echoPlot = 0;
    if (echoPlot > 4095) echoPlot = 4095;

    Serial.print(raw1);
    Serial.print(' ');
    Serial.print(raw2);
    Serial.print(' ');
    Serial.print(raw3);
    Serial.print(' ');
    Serial.print(mixedPlot);
    Serial.print(' ');
    Serial.println(echoPlot);
  }
}
