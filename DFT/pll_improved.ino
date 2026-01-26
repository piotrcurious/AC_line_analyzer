#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>

// ================================================================
// SYSTEM CONFIGURATION
// ================================================================
#define SAMPLE_FREQ 10000.0        // 10 kHz sampling
#define SAMPLES_PER_CYCLE 200      // 200 samples @ 10kHz = 20ms (50Hz)
#define TS (1.0 / SAMPLE_FREQ)     // Sampling period
#define PI_2 (2.0 * PI)

// PIN DEFINITIONS
#define ADC_PIN 36 // GPIO36 (VP)
#define DAC_PIN 25 // GPIO25 (DAC1)

// ================================================================
// GLOBAL BUFFERS & VARIABLES
// ================================================================

// Circular Buffer for ADC readings
// Double buffering to prevent race conditions
volatile float input_buffer[2][SAMPLES_PER_CYCLE]; 
volatile int buffer_head = 0;
volatile int active_buffer = 0;
volatile bool new_cycle_available = false;

// Pre-computed Trig Tables (Optimization for DFT)
// Uses 2/N normalization factor built in
float ref_sin[SAMPLES_PER_CYCLE];
float ref_cos[SAMPLES_PER_CYCLE];

// PLL State (protected by critical sections where needed)
volatile double pll_freq = 50.0;           // Current locked frequency
volatile double pll_phase = 0.0;           // Current internal phase (0 to 2PI)
volatile double pll_phase_increment = 0;   // Phase step per sample

// PID Parameters for Locking
#define K_P 0.5      // Proportional Gain (reduced for stability)
#define K_I 3.0      // Integral Gain (reduced for stability)
#define K_D 0.1      // Derivative Gain (added for damping)
#define I_MAX 5.0    // Anti-windup limit
#define I_MIN -5.0

double integrator = 0.0;      // Frequency error integrator
double prev_error = 0.0;      // For derivative term

// Output Generation
const float OUTPUT_AMPLITUDE = 0.9; // Scale factor for output

// Debug / Telemetry
float debug_rms = 0;
float debug_phase_err = 0;
float debug_magnitude = 0;

// Hardware Timer
hw_timer_t *timer = NULL;

// Timing diagnostics
volatile unsigned long isr_max_time = 0;
volatile unsigned long loop_max_time = 0;

// ================================================================
// ISR: TIME CRITICAL (Must be lightweight)
// ================================================================
void IRAM_ATTR onTimer() {
  unsigned long start_time = micros();
  
  // 1. UPDATE PLL PHASE
  // Advance the internal phase by the current frequency estimate
  pll_phase += pll_phase_increment;
  
  // Normalize phase to [0, 2*PI) range
  while (pll_phase >= PI_2) pll_phase -= PI_2;
  while (pll_phase < 0) pll_phase += PI_2;

  // 2. GENERATE OUTPUT (Locked Sine Wave)
  // Output a clean sine wave based on internal LOCKED phase
  float out_val = OUTPUT_AMPLITUDE * sin(pll_phase);
  
  // Map -1..1 to 0..255 (DAC range) with proper clamping
  int dac_raw = constrain((int)((out_val + 1.0f) * 127.5f), 0, 255);
  dac_output_voltage(DAC_CHANNEL_1, dac_raw);

  // 3. READ INPUT (ADC)
  int adc_raw = adc1_get_raw(ADC1_CHANNEL_0);
  
  // Convert to voltage centered at 0V
  // ADC range: 0-4095 -> 0-3.3V, Center at ~1.65V (2048)
  float v_in = (adc_raw - 2048.0f) * (3.3f / 4095.0f);

  // 4. FILL BUFFER (Double buffering)
  input_buffer[active_buffer][buffer_head] = v_in;
  buffer_head++;
  
  // Handle cycle completion
  if (buffer_head >= SAMPLES_PER_CYCLE) {
    buffer_head = 0;
    
    // Swap buffers atomically
    active_buffer = 1 - active_buffer;
    new_cycle_available = true;
  }
  
  // Track ISR timing
  unsigned long elapsed = micros() - start_time;
  if (elapsed > isr_max_time) isr_max_time = elapsed;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("ESP32 Grid-Tied PLL with DFT Analysis");
  Serial.println("========================================\n");

  // Configure ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  
  // Configure DAC
  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128); // Start at mid-point

  // 1. PRE-COMPUTE REFERENCE VECTORS
  // Include 2/N normalization factor for proper DFT amplitude
  float normalization = 2.0f / SAMPLES_PER_CYCLE;
  
  for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
    float angle = (PI_2 * i) / SAMPLES_PER_CYCLE;
    ref_sin[i] = sin(angle) * normalization;
    ref_cos[i] = cos(angle) * normalization;
  }

  // Initial Frequency Step (50 Hz)
  pll_phase_increment = (PI_2 * 50.0) / SAMPLE_FREQ;

  // Initialize buffers
  for (int buf = 0; buf < 2; buf++) {
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
      input_buffer[buf][i] = 0;
    }
  }

  // Timer Setup (10kHz interrupt)
  // ESP32 timer: prescaler 80 -> 1MHz clock, 100 ticks = 100us = 10kHz
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true);
  timerAlarmEnable(timer);

  Serial.println("Initialization complete!");
  Serial.println("Monitoring grid synchronization...\n");
  Serial.println("Format: Freq(Hz) | Phase_Err(rad) | RMS(V) | Magnitude(V)");
  Serial.println("--------------------------------------------------------");
}

// ================================================================
// MAIN LOOP: HEAVY ANALYSIS
// ================================================================
void loop() {
  // Wait for a complete cycle of data from the ISR
  if (new_cycle_available) {
    unsigned long loop_start = millis();
    new_cycle_available = false;
    
    // Determine which buffer to process (the one not being written to)
    int process_buffer = 1 - active_buffer;

    // --- STEP 1: DC REMOVAL ---
    // Calculate mean to remove DC offset (crucial for accurate DFT)
    float sum = 0;
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
      sum += input_buffer[process_buffer][i];
    }
    float mean = sum / SAMPLES_PER_CYCLE;

    // --- STEP 2: FULL CYCLE DFT (CORRELATION) ---
    // Project the signal onto reference sine/cosine to extract fundamental
    // Real Part = In-phase component (I)
    // Imag Part = Quadrature component (Q)
    float sum_real = 0;  // I component
    float sum_imag = 0;  // Q component
    float sum_sq = 0;    // For RMS calculation

    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
      float val = input_buffer[process_buffer][i] - mean;
      
      // DFT Projection using pre-computed tables
      sum_real += val * ref_cos[i];
      sum_imag += val * ref_sin[i];
      
      sum_sq += val * val;
    }

    // --- STEP 3: PHASE AND MAGNITUDE EXTRACTION ---
    // The complex phasor (sum_real, sum_imag) represents the fundamental
    float magnitude = sqrt(sum_real * sum_real + sum_imag * sum_imag);
    float measured_phase = atan2(sum_imag, sum_real);
    
    // Calculate RMS
    debug_rms = sqrt(sum_sq / SAMPLES_PER_CYCLE);
    debug_magnitude = magnitude;

    // --- STEP 4: PLL FEEDBACK LOOP (PID Controller) ---
    // Phase error: difference between measured and expected phase
    // We want the phase to align with our internal PLL phase at buffer start
    float error = measured_phase;
    
    // Normalize error to [-PI, PI]
    while (error > PI) error -= PI_2;
    while (error < -PI) error += PI_2;
    
    debug_phase_err = error;

    // PID Controller
    // P term: immediate response to phase error
    float p_term = K_P * error;
    
    // I term: tracks frequency drift (with anti-windup)
    integrator += K_I * error * 0.02; // 0.02 = loop period (20ms)
    integrator = constrain(integrator, I_MIN, I_MAX);
    
    // D term: damping to prevent oscillation
    float d_term = K_D * (error - prev_error) / 0.02;
    prev_error = error;
    
    // Combine PID terms
    float freq_correction = p_term + integrator + d_term;
    
    // Update frequency with safety limits
    double new_freq = 50.0 + freq_correction;
    new_freq = constrain(new_freq, 45.0, 55.0);
    
    // Update PLL state atomically
    noInterrupts();
    pll_freq = new_freq;
    pll_phase_increment = (PI_2 * pll_freq) / SAMPLE_FREQ;
    interrupts();

    // --- DIAGNOSTICS ---
    static unsigned long last_print = 0;
    static int sample_count = 0;
    sample_count++;
    
    // Print every 200ms (10 cycles)
    if (millis() - last_print >= 200) {
      last_print = millis();
      
      Serial.print("Freq: ");
      Serial.print(pll_freq, 4);
      Serial.print(" Hz | Phase: ");
      Serial.print(debug_phase_err * 57.2958, 2); // Convert to degrees
      Serial.print(" deg | RMS: ");
      Serial.print(debug_rms, 3);
      Serial.print(" V | Mag: ");
      Serial.print(debug_magnitude, 3);
      Serial.print(" V | ISR: ");
      Serial.print(isr_max_time);
      Serial.println(" us");
      
      // Reset max timing
      isr_max_time = 0;
    }
    
    // Track loop timing
    unsigned long loop_time = millis() - loop_start;
    if (loop_time > loop_max_time) loop_max_time = loop_time;
  }
  
  // Small delay to prevent tight loop when no data available
  delay(1);
}
