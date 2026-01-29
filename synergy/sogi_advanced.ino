/*
 * ESP32 Advanced SOGI-PLL with Harmonic Subtraction
 * Extended features: THD calculation, inter-harmonic detection, 
 * exponential averaging, and enhanced diagnostics
 */

#include <Arduino.h>
#include <math.h>

// Configuration
#define ADC_PIN 34
#define NOMINAL_FREQ 50.0f
#define SAMPLING_FREQ_BASE 5000.0f
#define MAX_HARMONICS 20
#define DFT_UPDATE_RATE 10

// SOGI-PLL Parameters
#define SOGI_K 1.414f
#define PLL_KP 180.0f
#define PLL_KI 3200.0f

// ADC Configuration
#define ADC_SAMPLES_PER_WINDOW 200
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE 4095.0f
#define V_REF 3.3f

// Filtering
#define HARMONIC_ALPHA 0.1f  // Exponential averaging factor

// Thresholds
#define MIN_FUNDAMENTAL_MAGNITUDE 0.1f
#define HARMONIC_DETECTION_THRESHOLD 0.005f

// Sampling control
volatile float detected_frequency = NOMINAL_FREQ;
volatile uint16_t samples_per_window = ADC_SAMPLES_PER_WINDOW;
volatile float actual_sampling_freq = SAMPLING_FREQ_BASE;
volatile uint32_t sample_period_us = 200;

// SOGI-PLL State
struct SOGI_PLL {
  float x, qx, dx;
  float u_q, u_d;
  float theta, omega, freq;
  float sin_theta, cos_theta;
  float x_filtered;
  float Ts;
  
  // Enhanced metrics
  float v_alpha, v_beta;  // Alpha-beta frame
  float v_d, v_q;         // DQ frame
  float amplitude;        // Signal amplitude
  bool locked;            // PLL lock status
  float lock_indicator;   // Lock quality metric
} sogi_pll;

// Enhanced Harmonic Structure
struct Harmonic {
  float magnitude;
  float phase;
  float magnitude_filtered;  // Exponentially averaged
  bool active;
  uint32_t last_seen;       // Timestamp
};

Harmonic harmonics[MAX_HARMONICS + 1];
float harmonic_sum = 0.0f;

// Power Quality Metrics
struct PowerQuality {
  float thd;              // Total Harmonic Distortion
  float thd_r;            // THD referenced to RMS
  float thd_f;            // THD referenced to fundamental
  float rms;              // RMS voltage
  float crest_factor;     // Peak / RMS
  float fundamental_mag;  // Fundamental magnitude
  float dc_component;     // DC offset
  
  // Individual harmonic groups
  float odd_harmonics;    // Sum of odd harmonics
  float even_harmonics;   // Sum of even harmonics
  float triplen_harmonics; // 3rd, 9th, 15th, etc.
} pq_metrics;

// DFT Buffer
#define DFT_BUFFER_SIZE 256
float dft_buffer[DFT_BUFFER_SIZE];
uint16_t dft_buffer_index = 0;
uint16_t pll_cycle_count = 0;
bool dft_ready = false;

// Statistics
float min_voltage = 999.0f;
float max_voltage = 0.0f;
uint32_t zero_cross_count = 0;

// Timing
hw_timer_t *sampling_timer = NULL;
volatile bool sample_ready = false;
volatile float current_sample = 0.0f;
uint32_t loop_count = 0;
uint32_t last_stats_time = 0;

// Function Prototypes
void IRAM_ATTR onSamplingTimer();
void initSOGI_PLL(float nominal_freq, float sampling_freq);
void updateSOGI_PLL(float input);
void performDFT();
void calculatePowerQuality();
void updateHarmonicSubtraction();
void adjustSamplingRate();
float readADCVoltage();
void printDetailedStatistics();
void checkPLLLock();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 Advanced SOGI-PLL System ===");
  Serial.printf("Core: %s | CPU: %d MHz\n", 
                ARDUINO_ESP32_RELEASE, ESP.getCpuFreqMHz());
  
  // Initialize ADC
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_11db);
  pinMode(ADC_PIN, INPUT);
  
  // Initialize SOGI-PLL
  initSOGI_PLL(NOMINAL_FREQ, SAMPLING_FREQ_BASE);
  
  // Initialize harmonics
  for (int i = 0; i <= MAX_HARMONICS; i++) {
    harmonics[i].magnitude = 0.0f;
    harmonics[i].phase = 0.0f;
    harmonics[i].magnitude_filtered = 0.0f;
    harmonics[i].active = false;
    harmonics[i].last_seen = 0;
  }
  
  memset(dft_buffer, 0, sizeof(dft_buffer));
  memset(&pq_metrics, 0, sizeof(pq_metrics));
  
  // Setup timer
  sampling_timer = timerBegin(1000000);
  timerAttachInterrupt(sampling_timer, &onSamplingTimer);
  timerAlarm(sampling_timer, sample_period_us, true, 0);
  
  Serial.println("System ready. Monitoring...\n");
  last_stats_time = millis();
}

void loop() {
  if (sample_ready) {
    sample_ready = false;
    
    // Track min/max
    if (current_sample < min_voltage) min_voltage = current_sample;
    if (current_sample > max_voltage) max_voltage = current_sample;
    
    // Update SOGI-PLL
    updateSOGI_PLL(current_sample);
    
    // Store in DFT buffer
    dft_buffer[dft_buffer_index] = sogi_pll.x;
    dft_buffer_index++;
    
    // Detect zero crossings
    static float prev_sin = 0.0f;
    if (prev_sin < 0.0f && sogi_pll.sin_theta >= 0.0f) {
      pll_cycle_count++;
      zero_cross_count++;
      
      if (pll_cycle_count >= DFT_UPDATE_RATE) {
        pll_cycle_count = 0;
        dft_ready = true;
      }
    }
    prev_sin = sogi_pll.sin_theta;
    
    // Wrap buffer
    if (dft_buffer_index >= samples_per_window) {
      dft_buffer_index = 0;
    }
    
    loop_count++;
  }
  
  // Process DFT
  if (dft_ready) {
    dft_ready = false;
    performDFT();
    calculatePowerQuality();
    updateHarmonicSubtraction();
    adjustSamplingRate();
    checkPLLLock();
  }
  
  // Print statistics
  if (millis() - last_stats_time >= 3000) {
    last_stats_time = millis();
    printDetailedStatistics();
    loop_count = 0;
    zero_cross_count = 0;
    min_voltage = 999.0f;
    max_voltage = 0.0f;
  }
  
  delay(1);
}

void IRAM_ATTR onSamplingTimer() {
  current_sample = readADCVoltage();
  sample_ready = true;
}

void initSOGI_PLL(float nominal_freq, float sampling_freq) {
  memset(&sogi_pll, 0, sizeof(sogi_pll));
  
  sogi_pll.Ts = 1.0f / sampling_freq;
  sogi_pll.omega = 2.0f * PI * nominal_freq;
  sogi_pll.freq = nominal_freq;
  sogi_pll.locked = false;
  
  actual_sampling_freq = sampling_freq;
  sample_period_us = (uint32_t)(1000000.0f / sampling_freq);
  samples_per_window = (uint16_t)(sampling_freq / nominal_freq);
  
  if (samples_per_window > DFT_BUFFER_SIZE) {
    samples_per_window = DFT_BUFFER_SIZE;
  }
}

void updateSOGI_PLL(float input) {
  // Apply harmonic subtraction
  sogi_pll.x = input - harmonic_sum;
  sogi_pll.x_filtered = sogi_pll.x;
  
  // SOGI implementation
  float k = SOGI_K;
  float omega = sogi_pll.omega;
  float Ts = sogi_pll.Ts;
  
  float epsilon = sogi_pll.x - sogi_pll.dx;
  
  // Integrators
  sogi_pll.u_q += omega * epsilon * Ts;
  sogi_pll.qx = k * sogi_pll.u_q;
  
  sogi_pll.u_d += omega * sogi_pll.qx * Ts;
  sogi_pll.dx = sogi_pll.u_d;
  
  // Store alpha-beta frame
  sogi_pll.v_alpha = sogi_pll.dx;
  sogi_pll.v_beta = sogi_pll.qx;
  
  // Calculate amplitude
  sogi_pll.amplitude = sqrtf(sogi_pll.dx * sogi_pll.dx + 
                             sogi_pll.qx * sogi_pll.qx);
  
  // PLL
  sogi_pll.sin_theta = sinf(sogi_pll.theta);
  sogi_pll.cos_theta = cosf(sogi_pll.theta);
  
  // Park transform
  sogi_pll.v_d = sogi_pll.dx * sogi_pll.cos_theta + 
                 sogi_pll.qx * sogi_pll.sin_theta;
  sogi_pll.v_q = -sogi_pll.dx * sogi_pll.sin_theta + 
                 sogi_pll.qx * sogi_pll.cos_theta;
  
  // Phase error
  float phase_error = atanf(sogi_pll.v_q / (fabsf(sogi_pll.v_d) + 1e-6f));
  
  // PI controller
  static float integral = 0.0f;
  integral += PLL_KI * phase_error * Ts;
  integral = constrain(integral, -200.0f, 200.0f);
  
  float omega_correction = PLL_KP * phase_error + integral;
  sogi_pll.omega = 2.0f * PI * NOMINAL_FREQ + omega_correction;
  
  // Constrain
  float omega_min = 2.0f * PI * NOMINAL_FREQ * 0.8f;
  float omega_max = 2.0f * PI * NOMINAL_FREQ * 1.2f;
  sogi_pll.omega = constrain(sogi_pll.omega, omega_min, omega_max);
  
  sogi_pll.freq = sogi_pll.omega / (2.0f * PI);
  
  // Update phase
  sogi_pll.theta += sogi_pll.omega * Ts;
  while (sogi_pll.theta >= 2.0f * PI) sogi_pll.theta -= 2.0f * PI;
  while (sogi_pll.theta < 0.0f) sogi_pll.theta += 2.0f * PI;
  
  // Calculate lock indicator
  sogi_pll.lock_indicator = fabsf(sogi_pll.v_q) / 
                            (sogi_pll.amplitude + 1e-6f);
}

void performDFT() {
  uint16_t N = samples_per_window;
  if (N > DFT_BUFFER_SIZE) N = DFT_BUFFER_SIZE;
  
  // DC component
  float dc_sum = 0.0f;
  for (uint16_t n = 0; n < N; n++) {
    dc_sum += dft_buffer[n];
  }
  pq_metrics.dc_component = dc_sum / (float)N;
  
  // Harmonics
  for (int h = 1; h <= MAX_HARMONICS; h++) {
    float real_sum = 0.0f;
    float imag_sum = 0.0f;
    
    for (uint16_t n = 0; n < N; n++) {
      float angle = -2.0f * PI * h * n / (float)N;
      real_sum += dft_buffer[n] * cosf(angle);
      imag_sum += dft_buffer[n] * sinf(angle);
    }
    
    real_sum /= (float)N;
    imag_sum /= (float)N;
    
    float new_magnitude = 2.0f * sqrtf(real_sum * real_sum + 
                                       imag_sum * imag_sum);
    
    // Exponential averaging
    harmonics[h].magnitude_filtered = 
      HARMONIC_ALPHA * new_magnitude + 
      (1.0f - HARMONIC_ALPHA) * harmonics[h].magnitude_filtered;
    
    harmonics[h].magnitude = new_magnitude;
    harmonics[h].phase = atan2f(imag_sum, real_sum);
    harmonics[h].active = (harmonics[h].magnitude > 
                           HARMONIC_DETECTION_THRESHOLD);
    
    if (harmonics[h].active) {
      harmonics[h].last_seen = millis();
    }
  }
}

void calculatePowerQuality() {
  // Fundamental
  pq_metrics.fundamental_mag = harmonics[1].magnitude;
  
  if (pq_metrics.fundamental_mag < MIN_FUNDAMENTAL_MAGNITUDE) {
    pq_metrics.thd = 0.0f;
    return;
  }
  
  // Sum harmonics
  float harmonic_power = 0.0f;
  pq_metrics.odd_harmonics = 0.0f;
  pq_metrics.even_harmonics = 0.0f;
  pq_metrics.triplen_harmonics = 0.0f;
  
  for (int h = 2; h <= MAX_HARMONICS; h++) {
    if (harmonics[h].active) {
      float mag = harmonics[h].magnitude;
      harmonic_power += mag * mag;
      
      if (h % 2 == 1) {
        pq_metrics.odd_harmonics += mag;
      } else {
        pq_metrics.even_harmonics += mag;
      }
      
      if (h % 3 == 0) {
        pq_metrics.triplen_harmonics += mag;
      }
    }
  }
  
  // THD calculations
  float fund_power = pq_metrics.fundamental_mag * pq_metrics.fundamental_mag;
  pq_metrics.thd_f = 100.0f * sqrtf(harmonic_power) / 
                     pq_metrics.fundamental_mag;
  
  // RMS
  pq_metrics.rms = sqrtf(fund_power + harmonic_power);
  pq_metrics.thd_r = 100.0f * sqrtf(harmonic_power) / pq_metrics.rms;
  
  // Crest factor
  float peak = max_voltage - min_voltage;
  pq_metrics.crest_factor = peak / (2.0f * pq_metrics.rms);
}

void updateHarmonicSubtraction() {
  harmonic_sum = 0.0f;
  
  for (int h = 2; h <= MAX_HARMONICS; h++) {
    if (harmonics[h].active) {
      // Use filtered magnitude for smoother subtraction
      float harmonic_angle = h * sogi_pll.theta + harmonics[h].phase;
      harmonic_sum += harmonics[h].magnitude_filtered * 
                     cosf(harmonic_angle);
    }
  }
}

void adjustSamplingRate() {
  detected_frequency = sogi_pll.freq;
  
  float ideal_samples = SAMPLING_FREQ_BASE / detected_frequency;
  uint16_t new_samples = (uint16_t)(ideal_samples + 0.5f);
  
  new_samples = constrain(new_samples, 50, DFT_BUFFER_SIZE);
  
  if (abs((int)new_samples - (int)samples_per_window) > 2) {
    samples_per_window = new_samples;
    actual_sampling_freq = detected_frequency * samples_per_window;
    
    uint32_t new_period = (uint32_t)(1000000.0f / actual_sampling_freq);
    
    if (new_period != sample_period_us && 
        new_period > 50 && new_period < 10000) {
      sample_period_us = new_period;
      sogi_pll.Ts = 1.0f / actual_sampling_freq;
      timerAlarm(sampling_timer, sample_period_us, true, 0);
    }
  }
}

void checkPLLLock() {
  // Consider locked if phase error is small
  sogi_pll.locked = (sogi_pll.lock_indicator < 0.1f);
}

void printDetailedStatistics() {
  Serial.println("╔══════════════════════════════════════════════════╗");
  Serial.println("║        POWER QUALITY ANALYSIS REPORT             ║");
  Serial.println("╚══════════════════════════════════════════════════╝");
  
  // PLL Status
  Serial.printf("PLL Status: %s | Lock Quality: %.3f\n", 
                sogi_pll.locked ? "LOCKED" : "UNLOCKING",
                sogi_pll.lock_indicator);
  Serial.printf("Frequency: %.3f Hz | Phase: %.1f°\n",
                sogi_pll.freq, sogi_pll.theta * 180.0f / PI);
  Serial.printf("Zero Crossings: %lu | Loop Rate: %lu Hz\n",
                zero_cross_count, loop_count / 3);
  
  // Sampling Info
  Serial.printf("\nSampling: %.1f Hz | Period: %lu us | Samples/Window: %d\n",
                actual_sampling_freq, sample_period_us, samples_per_window);
  
  // Power Quality Metrics
  Serial.println("\n┌─── POWER QUALITY METRICS ───┐");
  Serial.printf("│ Fundamental: %.4f V       │\n", pq_metrics.fundamental_mag);
  Serial.printf("│ RMS Voltage: %.4f V       │\n", pq_metrics.rms);
  Serial.printf("│ THD-F:       %.2f%%         │\n", pq_metrics.thd_f);
  Serial.printf("│ THD-R:       %.2f%%         │\n", pq_metrics.thd_r);
  Serial.printf("│ Crest Factor: %.2f         │\n", pq_metrics.crest_factor);
  Serial.printf("│ DC Offset:   %.4f V       │\n", pq_metrics.dc_component);
  Serial.println("└─────────────────────────────┘");
  
  // Harmonic Groups
  Serial.println("\n┌─── HARMONIC GROUPS ───┐");
  Serial.printf("│ Odd:     %.4f V     │\n", pq_metrics.odd_harmonics);
  Serial.printf("│ Even:    %.4f V     │\n", pq_metrics.even_harmonics);
  Serial.printf("│ Triplen: %.4f V     │\n", pq_metrics.triplen_harmonics);
  Serial.println("└───────────────────────┘");
  
  // Harmonic Table
  Serial.println("\n╔═══════════════════════════════════════════════════════╗");
  Serial.println("║  H  │  Mag(V) │  Filt(V) │  Phase  │  %Fund │ Active ║");
  Serial.println("╠═════╪═════════╪══════════╪═════════╪════════╪════════╣");
  
  for (int h = 1; h <= min(10, MAX_HARMONICS); h++) {
    if (harmonics[h].active || h == 1) {
      float percent = (pq_metrics.fundamental_mag > MIN_FUNDAMENTAL_MAGNITUDE) ?
                     (harmonics[h].magnitude / pq_metrics.fundamental_mag * 100.0f) : 0.0f;
      
      Serial.printf("║ %2d  │ %7.4f │  %7.4f │ %6.1f° │ %5.1f%% │   %s   ║\n",
                    h,
                    harmonics[h].magnitude,
                    harmonics[h].magnitude_filtered,
                    harmonics[h].phase * 180.0f / PI,
                    percent,
                    harmonics[h].active ? "✓" : "✗");
    }
  }
  Serial.println("╚═══════════════════════════════════════════════════════╝\n");
}

float readADCVoltage() {
  int raw = analogRead(ADC_PIN);
  return ((float)raw / ADC_MAX_VALUE) * V_REF;
}
