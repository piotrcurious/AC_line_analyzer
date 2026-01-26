// AC Line Analyzer — robust PLL + Goertzel phase sync for 50 Hz
// Target: ESP32 (adc1_get_raw, dac_output_voltage). Adapt if on different platform.

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <math.h>

// ----- Configuration -----
static constexpr float TARGET_FREQ = 50.0f;         // Hz we want to lock to
static constexpr float FS = 10000.0f;               // sampling frequency (Hz)
static constexpr float TS = 1.0f / FS;              // sampling period (s)
static constexpr int BUFFER_SIZE = 200;             // samples (200 @10kHz == 20ms == 1 cycle)
static constexpr int CORRELATION_UPDATE_INTERVAL = 1000; // samples (100ms)
static constexpr float ADC_VREF = 3.6f;             // measured ADC full-scale (set to your board ADC ref)
static constexpr float DAC_VREF = 3.3f;             // ESP32 DAC typical full-scale
static constexpr int ADC_CHANNEL = ADC1_CHANNEL_0;  // GPIO36
static constexpr dac_channel_t DAC_CHANNEL = DAC_CHANNEL_1; // GPIO25

// PLL loop gains (tune for stability & lock speed)
static constexpr float PLL_KP = 500.0f;   // proportional gain (rad/s per rad phase error)
static constexpr float PLL_KI = 1000.0f;  // integral gain (rad/s per rad * s)
static constexpr float OMEGA_NOMINAL = 2.0f * M_PI * TARGET_FREQ; // rad/s nominal (50Hz)
static constexpr float OMEGA_MIN = 2.0f * M_PI * 45.0f;
static constexpr float OMEGA_MAX = 2.0f * M_PI * 55.0f;

// Output amplitude (peak) and offsets
static constexpr float VOUT_AMP = 1.6f;   // V peak for generated sine (tune)
static constexpr float ADC_OFFSET = ADC_VREF / 2.0f; // approximate DC midpoint seen by ADC

// Simple 1-pole IIR smoothing for phase error
static constexpr float PHASE_ERR_ALPHA = 0.2f; // [0..1] higher -> faster response

// ----- Buffers & state -----
float vgrid_buffer[BUFFER_SIZE];
float vout_buffer[BUFFER_SIZE];
int buffer_index = 0;
unsigned long samples_collected = 0;
int correlation_counter = 0;

volatile float theta = 0.0f;          // NCO phase (rad)
volatile float omega = OMEGA_NOMINAL; // NCO angular frequency (rad/s)
float integrator = 0.0f;              // PI integrator for frequency control

// Smoothed phase error from Goertzel
float smoothed_phase_error = 0.0f;

// Timing
unsigned long nextMicros = 0;

// ----- Utility: wrap to -PI..PI -----
static inline float wrapPI(float x) {
  while (x > M_PI) x -= 2.0f * M_PI;
  while (x <= -M_PI) x += 2.0f * M_PI;
  return x;
}

// ----- Goertzel algorithm for single target freq -----
// Computes magnitude and phase of the target frequency in "data" of length N.
// Phase returned is in radians, relative to the first sample (index 0).
void goertzel(const float *data, int N, float targetFreq, float Fs, float &mag, float &phase) {
  if (N <= 0) { mag = 0.0f; phase = 0.0f; return; }
  int k = (int)(0.5f + ((float)N * targetFreq) / Fs);
  float omega_g = (2.0f * M_PI * k) / (float)N;
  float coeff = 2.0f * cosf(omega_g);

  float s_prev = 0.0f, s_prev2 = 0.0f;
  for (int i = 0; i < N; ++i) {
    float s = data[i] + coeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev = s;
  }

  float real = s_prev - s_prev2 * cosf(omega_g);
  float imag = s_prev2 * sinf(omega_g);
  mag = sqrtf(real * real + imag * imag);
  phase = atan2f(imag, real); // phase of the complex phasor
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  delay(100);

  // ADC init
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

  // DAC init
  dac_output_enable(DAC_CHANNEL);

  // Zero-out buffers
  for (int i = 0; i < BUFFER_SIZE; ++i) {
    vgrid_buffer[i] = 0.0f;
    vout_buffer[i] = 0.0f;
  }

  nextMicros = micros();

  Serial.println("Robust AC Line Analyzer (DPLL + Goertzel) initialized");
  Serial.println("Columns: vgrid(V), vout(V), theta(rad), omega(rad/s), phi_err(rad), goertzel_mag(V), samples_collected");
}

// ----- Main loop (polling sampler timed with micros) -----
void loop() {
  // Wait until next sample time (busy wait — accurate enough for moderate sample rates).
  // This keeps consistent sampling interval without using delayMicroseconds which accumulates jitter.
  unsigned long now;
  do {
    now = micros();
    // If we fell too far behind (e.g. Serial blocking), resync
    if (now - nextMicros > 1000000UL) {
      nextMicros = now;
      break;
    }
  } while (now < nextMicros);

  // schedule next sample
  nextMicros += (unsigned long)(TS * 1e6f + 0.5f); // in microseconds

  // ---- SAMPLE ADC ----
  int raw = adc1_get_raw(ADC_CHANNEL);
  // convert raw 12-bit to volts (approx). Tune ADC_VREF if necessary.
  float vgrid = ((float)raw) * ADC_VREF / 4095.0f;
  // center to approx zero (assumes half-scale bias in front-end)
  vgrid -= ADC_OFFSET;

  // store measured sample
  vgrid_buffer[buffer_index] = vgrid;

  // ---- NCO / generate reference ----
  // update theta by current omega
  theta += omega * TS;
  theta = wrapPI(theta);

  // create vout (sine centered at zero)
  float vout = VOUT_AMP * sinf(theta);
  vout_buffer[buffer_index] = vout;

  // write to DAC (map from [-Vout_max, +Vout_max] to [0..255])
  float dac_center = DAC_VREF / 2.0f;
  float dac_val_f = (vout + dac_center); // offset to 0..Vref
  // scale to 0..255
  int dac_int = (int)roundf(constrain(dac_val_f / DAC_VREF * 255.0f, 0.0f, 255.0f));
  dac_output_voltage(DAC_CHANNEL, (uint8_t)dac_int);

  // advance buffer pointer
  buffer_index = (buffer_index + 1) % BUFFER_SIZE;
  samples_collected++;
  correlation_counter++;

  // ---- Periodic Goertzel + PLL correction ----
  if (correlation_counter >= CORRELATION_UPDATE_INTERVAL && samples_collected >= (unsigned long)BUFFER_SIZE) {
    correlation_counter = 0;

    // Compute phasor for grid and our output (both over the same buffer)
    float mag_grid = 0.0f, phase_grid = 0.0f;
    float mag_out = 0.0f, phase_out = 0.0f;

    // Goertzel on measured grid buffer and output buffer
    goertzel(vgrid_buffer, BUFFER_SIZE, TARGET_FREQ, FS, mag_grid, phase_grid);
    goertzel(vout_buffer, BUFFER_SIZE, TARGET_FREQ, FS, mag_out, phase_out);

    // The difference is the phase error (phase_grid - phase_out)
    float raw_phase_error = wrapPI(phase_grid - phase_out);

    // Smooth the phase error to reduce jitter
    smoothed_phase_error = PHASE_ERR_ALPHA * raw_phase_error + (1.0f - PHASE_ERR_ALPHA) * smoothed_phase_error;

    // PLL PI controller to adjust omega (rad/s)
    // proportional term
    float pterm = PLL_KP * smoothed_phase_error;
    // integrate
    integrator += PLL_KI * smoothed_phase_error * (CORRELATION_UPDATE_INTERVAL * TS); // approximate dt
    // anti-windup
    if (integrator > (OMEGA_MAX - OMEGA_NOMINAL)) integrator = (OMEGA_MAX - OMEGA_NOMINAL);
    if (integrator < (OMEGA_MIN - OMEGA_NOMINAL)) integrator = (OMEGA_MIN - OMEGA_NOMINAL);

    // new omega
    omega = OMEGA_NOMINAL + pterm + integrator;
    // clamp
    if (omega > OMEGA_MAX) omega = OMEGA_MAX;
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;

    // Apply a small phase nudging to theta to reduce steady-state phase error.
    // We apply a fraction of smoothed_phase_error as a correction to the NCO phase.
    const float PHASE_NUDGE_GAIN = 0.05f; // small value, tune as needed
    theta = wrapPI(theta + PHASE_NUDGE_GAIN * smoothed_phase_error);

    // Diagnostics
    float rms_grid = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; ++i) rms_grid += vgrid_buffer[i] * vgrid_buffer[i];
    rms_grid = sqrtf(rms_grid / BUFFER_SIZE);

    Serial.print("GridV: ");
    Serial.print(rms_grid, 3);
    Serial.print(" V_rms, Mag: ");
    Serial.print(mag_grid, 4);
    Serial.print(", PhaseErr: ");
    Serial.print(smoothed_phase_error, 5);
    Serial.print(" rad, Omega: ");
    Serial.print(omega, 3);
    Serial.print(" rad/s (");
    Serial.print(omega / (2.0f * M_PI), 4);
    Serial.println(" Hz)");
  }

  // Periodic human-friendly printing (not every sample)
  static int print_counter = 0;
  print_counter++;
  if (print_counter >= 200) { // print every 200 samples (~20 ms)
    print_counter = 0;
    float rms_display = 0.0f;
    for (int i=0; i<BUFFER_SIZE; ++i) rms_display += vgrid_buffer[i]*vgrid_buffer[i];
    rms_display = sqrtf(rms_display / BUFFER_SIZE);
    Serial.print(vgrid + ADC_OFFSET, 3); // measured raw-ish value
    Serial.print(", ");
    Serial.print(vout + DAC_VREF/2.0f, 3);
    Serial.print(", ");
    Serial.print(theta, 4);
    Serial.print(", ");
    Serial.print(omega, 3);
    Serial.print(", ");
    Serial.print(smoothed_phase_error, 5);
    Serial.print(", bufferSamples: ");
    Serial.println(samples_collected);
  }
}
