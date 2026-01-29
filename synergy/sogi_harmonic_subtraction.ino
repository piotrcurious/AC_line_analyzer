/*
 * ESP32 SOGI-PLL with Harmonic Subtraction Strategy
 * Arduino Core 3.3.1
 * 
 * Features:
 * - SOGI-PLL for phase and frequency tracking
 * - DFT-based harmonic analysis with perfect window alignment
 * - Adaptive sampling tuned to detected frequency
 * - Harmonic subtraction to extract fundamental
 * 
 * Connections:
 * - ADC_PIN (GPIO34): Input signal
 */

#include <Arduino.h>
#include <math.h>

// Configuration
#define ADC_PIN 34
#define NOMINAL_FREQ 50.0f      // Nominal frequency (Hz) - change to 60.0f for 60Hz systems
#define SAMPLING_FREQ_BASE 5000.0f  // Base sampling frequency (Hz)
#define MAX_HARMONICS 15        // Number of harmonics to track
#define DFT_UPDATE_RATE 10      // Update DFT every N PLL cycles

// SOGI-PLL Parameters
#define SOGI_K 1.414f           // SOGI gain (sqrt(2) for optimal damping)
#define PLL_KP 180.0f           // PLL proportional gain
#define PLL_KI 3200.0f          // PLL integral gain

// ADC Configuration
#define ADC_SAMPLES_PER_WINDOW 200  // Base samples per window
#define ADC_RESOLUTION 12
#define ADC_MAX_VALUE 4095.0f
#define V_REF 3.3f

// Sampling control
volatile float detected_frequency = NOMINAL_FREQ;
volatile uint16_t samples_per_window = ADC_SAMPLES_PER_WINDOW;
volatile float actual_sampling_freq = SAMPLING_FREQ_BASE;
volatile uint32_t sample_period_us = 200;  // Microseconds between samples

// SOGI-PLL State Variables
struct SOGI_PLL {
  // SOGI states
  float x;          // Input
  float qx;         // Quadrature output
  float dx;         // Direct output (in-phase)
  float u_q;        // Integrator state for qx
  float u_d;        // Integrator state for dx
  
  // PLL states
  float theta;      // Phase angle
  float omega;      // Angular frequency (rad/s)
  float freq;       // Frequency (Hz)
  float sin_theta;  // sin(theta)
  float cos_theta;  // cos(theta)
  
  // Harmonic subtraction
  float x_filtered; // Input with harmonics removed
  
  float Ts;         // Sample time
} sogi_pll;

// Harmonic Information Structure
struct Harmonic {
  float magnitude;
  float phase;
  bool active;
};

// Harmonic storage
Harmonic harmonics[MAX_HARMONICS + 1];  // Index 0 unused, 1-15 for harmonics
float harmonic_sum = 0.0f;

// DFT Buffer
#define DFT_BUFFER_SIZE 256
float dft_buffer[DFT_BUFFER_SIZE];
uint16_t dft_buffer_index = 0;
uint16_t pll_cycle_count = 0;
bool dft_ready = false;

// Timing
hw_timer_t *sampling_timer = NULL;
volatile bool sample_ready = false;
volatile float current_sample = 0.0f;

// Statistics
uint32_t loop_count = 0;
uint32_t last_stats_time = 0;

// Function Prototypes
void IRAM_ATTR onSamplingTimer();
void initSOGI_PLL(float nominal_freq, float sampling_freq);
void updateSOGI_PLL(float input);
void performDFT();
void updateHarmonicSubtraction();
void adjustSamplingRate();
float readADCVoltage();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== ESP32 SOGI-PLL Harmonic Subtraction System ===");
  Serial.printf("Arduino Core Version: %s\n", ARDUINO_ESP32_RELEASE);
  Serial.printf("Nominal Frequency: %.1f Hz\n", NOMINAL_FREQ);
  Serial.printf("Base Sampling Frequency: %.1f Hz\n", SAMPLING_FREQ_BASE);
  Serial.printf("Max Harmonics: %d\n\n", MAX_HARMONICS);
  
  // Initialize ADC
  analogReadResolution(ADC_RESOLUTION);
  analogSetAttenuation(ADC_11db);  // 0-3.3V range
  pinMode(ADC_PIN, INPUT);
  
  // Initialize SOGI-PLL
  initSOGI_PLL(NOMINAL_FREQ, SAMPLING_FREQ_BASE);
  
  // Initialize harmonics
  for (int i = 0; i <= MAX_HARMONICS; i++) {
    harmonics[i].magnitude = 0.0f;
    harmonics[i].phase = 0.0f;
    harmonics[i].active = false;
  }
  
  // Initialize DFT buffer
  memset(dft_buffer, 0, sizeof(dft_buffer));
  
  // Setup sampling timer
  sampling_timer = timerBegin(1000000);  // 1MHz timer (1us resolution)
  timerAttachInterrupt(sampling_timer, &onSamplingTimer);
  timerAlarm(sampling_timer, sample_period_us, true, 0);
  
  Serial.println("Initialization complete. Starting acquisition...\n");
  last_stats_time = millis();
}

void loop() {
  if (sample_ready) {
    sample_ready = false;
    
    // Update SOGI-PLL with new sample
    updateSOGI_PLL(current_sample);
    
    // Store sample in DFT buffer
    dft_buffer[dft_buffer_index] = sogi_pll.x;
    dft_buffer_index++;
    
    // Check if we completed a PLL cycle (zero crossing detection)
    static float prev_sin = 0.0f;
    if (prev_sin < 0.0f && sogi_pll.sin_theta >= 0.0f) {
      pll_cycle_count++;
      
      // Perform DFT every N cycles
      if (pll_cycle_count >= DFT_UPDATE_RATE) {
        pll_cycle_count = 0;
        dft_ready = true;
      }
    }
    prev_sin = sogi_pll.sin_theta;
    
    // Wrap DFT buffer
    if (dft_buffer_index >= samples_per_window) {
      dft_buffer_index = 0;
    }
    
    loop_count++;
  }
  
  // Perform DFT analysis when ready
  if (dft_ready) {
    dft_ready = false;
    performDFT();
    updateHarmonicSubtraction();
    adjustSamplingRate();
  }
  
  // Print statistics every 2 seconds
  if (millis() - last_stats_time >= 2000) {
    last_stats_time = millis();
    
    Serial.println("=== System Status ===");
    Serial.printf("Detected Frequency: %.3f Hz (ω=%.2f rad/s)\n", 
                  sogi_pll.freq, sogi_pll.omega);
    Serial.printf("Phase: %.2f° | Samples/Window: %d\n", 
                  sogi_pll.theta * 180.0f / PI, samples_per_window);
    Serial.printf("Sampling Freq: %.1f Hz (Period: %d us)\n", 
                  actual_sampling_freq, sample_period_us);
    Serial.printf("Loop Rate: %lu Hz\n", loop_count / 2);
    
    Serial.println("\nHarmonic Content:");
    Serial.println("Harmonic | Magnitude | Phase | %Fund");
    Serial.println("---------|-----------|-------|-------");
    
    float fundamental = harmonics[1].magnitude;
    for (int h = 1; h <= MAX_HARMONICS; h++) {
      if (harmonics[h].active && harmonics[h].magnitude > 0.001f) {
        float percent = (fundamental > 0.001f) ? 
                       (harmonics[h].magnitude / fundamental * 100.0f) : 0.0f;
        Serial.printf("   %2d    | %7.4f   | %5.1f° | %5.1f%%\n", 
                      h, 
                      harmonics[h].magnitude,
                      harmonics[h].phase * 180.0f / PI,
                      percent);
      }
    }
    Serial.println();
    
    loop_count = 0;
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}

// ISR: Sampling timer interrupt
void IRAM_ATTR onSamplingTimer() {
  current_sample = readADCVoltage();
  sample_ready = true;
}

// Initialize SOGI-PLL
void initSOGI_PLL(float nominal_freq, float sampling_freq) {
  memset(&sogi_pll, 0, sizeof(sogi_pll));
  
  sogi_pll.Ts = 1.0f / sampling_freq;
  sogi_pll.omega = 2.0f * PI * nominal_freq;
  sogi_pll.freq = nominal_freq;
  sogi_pll.theta = 0.0f;
  
  actual_sampling_freq = sampling_freq;
  sample_period_us = (uint32_t)(1000000.0f / sampling_freq);
  
  // Calculate optimal samples per window for integer cycles
  samples_per_window = (uint16_t)(sampling_freq / nominal_freq);
  if (samples_per_window > DFT_BUFFER_SIZE) {
    samples_per_window = DFT_BUFFER_SIZE;
  }
}

// Update SOGI-PLL with new input sample
void updateSOGI_PLL(float input) {
  // Apply harmonic subtraction to input
  sogi_pll.x = input - harmonic_sum;
  sogi_pll.x_filtered = sogi_pll.x;
  
  // SOGI (Second Order Generalized Integrator)
  // State space implementation with bilinear transform
  float k = SOGI_K;
  float omega = sogi_pll.omega;
  float Ts = sogi_pll.Ts;
  
  // SOGI equations
  float epsilon = sogi_pll.x - sogi_pll.dx;
  
  // Update quadrature component (integrator)
  sogi_pll.u_q += omega * epsilon * Ts;
  sogi_pll.qx = k * sogi_pll.u_q;
  
  // Update direct component (integrator)
  sogi_pll.u_d += omega * sogi_pll.qx * Ts;
  sogi_pll.dx = sogi_pll.u_d;
  
  // PLL: Calculate phase error using Park transformation
  sogi_pll.sin_theta = sinf(sogi_pll.theta);
  sogi_pll.cos_theta = cosf(sogi_pll.theta);
  
  // Park transform to get d-q components
  float v_d = sogi_pll.dx * sogi_pll.cos_theta + sogi_pll.qx * sogi_pll.sin_theta;
  float v_q = -sogi_pll.dx * sogi_pll.sin_theta + sogi_pll.qx * sogi_pll.cos_theta;
  
  // Phase error (v_q should be zero when locked)
  float phase_error = atanf(v_q / (fabsf(v_d) + 1e-6f));
  
  // PI controller for frequency
  static float integral = 0.0f;
  integral += PLL_KI * phase_error * Ts;
  
  // Limit integral windup
  integral = constrain(integral, -200.0f, 200.0f);
  
  float omega_correction = PLL_KP * phase_error + integral;
  sogi_pll.omega = 2.0f * PI * NOMINAL_FREQ + omega_correction;
  
  // Constrain omega to reasonable range (±20% of nominal)
  float omega_min = 2.0f * PI * NOMINAL_FREQ * 0.8f;
  float omega_max = 2.0f * PI * NOMINAL_FREQ * 1.2f;
  sogi_pll.omega = constrain(sogi_pll.omega, omega_min, omega_max);
  
  // Update frequency
  sogi_pll.freq = sogi_pll.omega / (2.0f * PI);
  
  // Update phase
  sogi_pll.theta += sogi_pll.omega * Ts;
  
  // Wrap phase to [0, 2π]
  while (sogi_pll.theta >= 2.0f * PI) {
    sogi_pll.theta -= 2.0f * PI;
  }
  while (sogi_pll.theta < 0.0f) {
    sogi_pll.theta += 2.0f * PI;
  }
}

// Perform DFT on collected samples
void performDFT() {
  uint16_t N = samples_per_window;
  if (N > DFT_BUFFER_SIZE) N = DFT_BUFFER_SIZE;
  
  // Calculate DFT for each harmonic
  for (int h = 1; h <= MAX_HARMONICS; h++) {
    float real_sum = 0.0f;
    float imag_sum = 0.0f;
    
    // DFT calculation for harmonic h
    for (uint16_t n = 0; n < N; n++) {
      float angle = -2.0f * PI * h * n / (float)N;
      real_sum += dft_buffer[n] * cosf(angle);
      imag_sum += dft_buffer[n] * sinf(angle);
    }
    
    // Normalize
    real_sum /= (float)N;
    imag_sum /= (float)N;
    
    // Calculate magnitude and phase
    harmonics[h].magnitude = 2.0f * sqrtf(real_sum * real_sum + imag_sum * imag_sum);
    harmonics[h].phase = atan2f(imag_sum, real_sum);
    
    // Mark as active if magnitude is significant
    harmonics[h].active = (harmonics[h].magnitude > 0.01f);
  }
}

// Update harmonic subtraction signal
void updateHarmonicSubtraction() {
  // Reconstruct harmonic content (excluding fundamental)
  harmonic_sum = 0.0f;
  
  for (int h = 2; h <= MAX_HARMONICS; h++) {  // Start from 2nd harmonic
    if (harmonics[h].active) {
      // Reconstruct harmonic using detected magnitude and phase
      float harmonic_angle = h * sogi_pll.theta + harmonics[h].phase;
      harmonic_sum += harmonics[h].magnitude * cosf(harmonic_angle);
    }
  }
}

// Adjust sampling rate based on detected frequency
void adjustSamplingRate() {
  detected_frequency = sogi_pll.freq;
  
  // Calculate ideal samples per window for integer number of cycles
  // Target: exactly one fundamental period
  float ideal_samples = SAMPLING_FREQ_BASE / detected_frequency;
  
  // Round to nearest integer for perfect window alignment
  uint16_t new_samples = (uint16_t)(ideal_samples + 0.5f);
  
  // Constrain to buffer size
  if (new_samples < 50) new_samples = 50;
  if (new_samples > DFT_BUFFER_SIZE) new_samples = DFT_BUFFER_SIZE;
  
  // Only update if significantly different
  if (abs((int)new_samples - (int)samples_per_window) > 2) {
    samples_per_window = new_samples;
    
    // Recalculate actual sampling frequency for perfect window
    actual_sampling_freq = detected_frequency * samples_per_window;
    
    // Update sampling period
    uint32_t new_period = (uint32_t)(1000000.0f / actual_sampling_freq);
    
    if (new_period != sample_period_us && new_period > 50 && new_period < 10000) {
      sample_period_us = new_period;
      sogi_pll.Ts = 1.0f / actual_sampling_freq;
      
      // Update timer
      timerAlarm(sampling_timer, sample_period_us, true, 0);
    }
  }
}

// Read ADC and convert to voltage
float readADCVoltage() {
  int raw = analogRead(ADC_PIN);
  return ((float)raw / ADC_MAX_VALUE) * V_REF;
}
