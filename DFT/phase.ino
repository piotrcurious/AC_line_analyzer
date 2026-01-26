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
// We use a volatile index so the ISR can update it safely
volatile float input_buffer[SAMPLES_PER_CYCLE]; 
volatile int buffer_head = 0;
volatile bool new_cycle_available = false;

// Pre-computed Trig Tables (Optimization for "Symmetry Analysis")
// This saves us from calling expensive sin() 400 times per loop
float ref_sin[SAMPLES_PER_CYCLE];
float ref_cos[SAMPLES_PER_CYCLE];

// PLL State
double pll_freq = 50.0;           // Current locked frequency
double pll_phase = 0.0;           // Current internal phase (0 to 2PI)
double pll_phase_increment = 0;   // Phase step per sample
double phase_correction = 0;      // Correction from PID

// Output Generation
volatile float dac_output = 0;

// PID Parameters for Locking
#define K_P 0.8  // Proportional Gain (How fast we catch phase jumps)
#define K_I 5.0  // Integral Gain (How hard we track frequency drift)
double integrator = 50.0; // Starts at 50Hz

// Debug / Telemetry
float debug_rms = 0;
float debug_phase_err = 0;

// Hardware Timer
hw_timer_t *timer = NULL;

// ================================================================
// ISR: TIME CRITICAL (Must be lightweight)
// ================================================================
void IRAM_ATTR onTimer() {
  // 1. UPDATE PLL PHASE
  // We advance the "generated" phase by the current frequency estimate
  pll_phase += pll_phase_increment;
  if (pll_phase >= PI_2) pll_phase -= PI_2;

  // 2. GENERATE OUTPUT (Locked Sine Wave)
  // We output a clean sine wave based on our internal LOCKED phase
  // Map -1..1 to 0..255 (DAC range)
  float out_val = sin(pll_phase);
  int dac_raw = (int)((out_val + 1.0f) * 127.5f); 
  dac_output_voltage(DAC_CHANNEL_1, dac_raw);

  // 3. READ INPUT (ADC)
  // Fast raw read
  int adc_raw = adc1_get_raw(ADC1_CHANNEL_0);
  // Convert to roughly +/- Voltage (Assuming 1.8V bias)
  // 0-4095 -> 0-3.3V. Center is ~1.65V (2048)
  float v_in = (adc_raw - 2048) * (3.3 / 4095.0);

  // 4. FILL BUFFER
  input_buffer[buffer_head] = v_in;
  buffer_head++;
  
  // Handle Wrap-around
  if (buffer_head >= SAMPLES_PER_CYCLE) {
    buffer_head = 0;
    new_cycle_available = true; // Signal main loop to analyze
  }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // Configure ADC/DAC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  dac_output_enable(DAC_CHANNEL_1);

  // 1. PRE-COMPUTE REFERENCE VECTORS
  // This creates the "Perfect Template" we compare the distorted signal against.
  for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
    float angle = (PI_2 * i) / SAMPLES_PER_CYCLE;
    ref_sin[i] = sin(angle);
    ref_cos[i] = cos(angle);
  }

  // Initial Frequency Step
  pll_phase_increment = (PI_2 * 50.0) / SAMPLE_FREQ;

  // Timer Setup (10kHz interrupt)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100, true); // 100us = 10kHz
  timerAlarmEnable(timer);

  Serial.println("Full-Cycle DFT Analyzer Initialized");
  Serial.println("Use Serial Plotter: Blue=Freq, Red=PhaseError, Green=RMS");
}

// ================================================================
// MAIN LOOP: HEAVY ANALYSIS
// ================================================================
void loop() {
  // We wait for a flag from the ISR indicating 20ms of data is ready.
  if (new_cycle_available) {
    new_cycle_available = false; // Reset flag

    // --- STEP 1: DC REMOVAL ---
    // Calculate mean to remove DC offset (crucial for accurate DFT)
    float sum = 0;
    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) sum += input_buffer[i];
    float mean = sum / SAMPLES_PER_CYCLE;

    // --- STEP 2: FULL CYCLE DFT (CORRELATION) ---
    // We project the recorded buffer onto our reference Sin and Cos vectors.
    // This extracts the Fundamental 50Hz component and ignores all other distortion.
    // Real Part = Correlation with Cosine
    // Imag Part = Correlation with Sine
    float sum_real = 0;
    float sum_imag = 0;
    float sum_sq = 0; // For RMS calculation

    for (int i = 0; i < SAMPLES_PER_CYCLE; i++) {
      float val = input_buffer[i] - mean; // Remove DC
      
      // DFT Projection
      // Note: We use the pre-computed tables for speed
      sum_real += val * ref_cos[i];
      sum_imag += val * ref_sin[i];
      
      sum_sq += val * val;
    }

    // --- STEP 3: PHASE RECONSTRUCTION ---
    // The "Measured Phase" is the angle of the vector (Real, Imag).
    // This represents the phase of the grid relative to the START of the buffer.
    float measured_phase_offset = atan2(sum_real, sum_imag); 
    
    // Calculate RMS for display
    debug_rms = sqrt(sum_sq / SAMPLES_PER_CYCLE);

    // --- STEP 4: PLL FEEDBACK LOOP ---
    // We want the buffer start to align with Phase 0.
    // If measured_phase_offset is not 0, our internal clock is drifting relative to the grid.
    
    float error = measured_phase_offset;
    debug_phase_err = error;

    // PI Controller
    // Integral Term (Tracking Frequency)
    integrator += (K_I * error * 0.02); // 0.02 is the loop time (20ms)
    
    // Clamp Frequency (Safety)
    if (integrator > 55.0) integrator = 55.0;
    if (integrator < 45.0) integrator = 45.0;

    // Calculate new frequency
    pll_freq = integrator + (K_P * error);
    
    // Update the Phase Increment for the ISR
    // The ISR picks this up instantly
    noInterrupts();
    pll_phase_increment = (PI_2 * pll_freq) / SAMPLE_FREQ;
    interrupts();

    // --- DIAGNOSTICS (Every 100ms) ---
    static int print_prescaler = 0;
    if (print_prescaler++ > 5) {
      print_prescaler = 0;
      Serial.print("Freq:");
      Serial.print(pll_freq, 4);
      Serial.print(" , PhaseErr:");
      Serial.print(debug_phase_err, 4);
      Serial.print(" , RMS:");
      Serial.println(debug_rms, 3);
    }
  }
}
