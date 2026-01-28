#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>

// ================================================================
// SYSTEM CONFIGURATION
// ================================================================
#define SAMPLE_FREQ 12000.0        
#define PI_2 (2.0 * PI)

// Buffer sizing
#define MIN_FREQ 45.0
#define MAX_FREQ 55.0
#define MAX_BUFFER_SIZE 600        

// Fixed-Point Scaling
#define FP_SHIFT 16
#define FP_SCALE (1 << FP_SHIFT)
#define PI_2_FP ((uint32_t)(PI_2 * FP_SCALE))
#define ZERO_THRESHOLD (PI_2_FP / 20)

#define ADC_PIN 36 
#define DAC_PIN 25 

// ================================================================
// GLOBAL BUFFERS & VARIABLES
// ================================================================

// Circular buffer for continuous sampling
volatile int16_t input_buffer[MAX_BUFFER_SIZE]; 
volatile int16_t ref_sin_buffer[MAX_BUFFER_SIZE];
volatile int16_t ref_cos_buffer[MAX_BUFFER_SIZE];
volatile uint16_t write_index = 0;

// Zero-crossing detection
volatile uint16_t last_zero_crossing_index = 0;
volatile uint16_t current_zero_crossing_index = 0;
volatile bool new_cycle_ready = false;
volatile bool last_was_high = false;

// Sample counting for accurate rate measurement
volatile uint32_t total_samples = 0;
volatile uint32_t sample_count_at_last_second = 0;
volatile uint32_t last_second_millis = 0;

// ISR Sine Lookup Table (Fixed-Point)
#define SINE_LUT_SIZE 1024
int16_t sine_lut_fp[SINE_LUT_SIZE];

// PLL State (Fixed-Point for ISR)
volatile uint32_t pll_phase_fp = 0;           
volatile uint32_t pll_phase_inc_fp = 0;   

// Loop processing variables
float pll_freq = 50.0;
float integrator = 0.0;      
const float OUTPUT_AMPLITUDE = 0.9; 

// Sample rate calibration - use simple sample counting
float actual_sample_freq = SAMPLE_FREQ;
bool calibration_done = false;

// PLL Gains
const float KP = -1.0;
const float KI = -5.0;

// Debug
float debug_phase_err = 0;
float debug_magnitude = 0;
float debug_rms = 0;
uint16_t debug_samples_used = 0;

hw_timer_t *timer = NULL;
volatile uint32_t isr_max_time = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ================================================================
// ISR: CONTINUOUS SAMPLING WITH ZERO-CROSSING DETECTION
// ================================================================
void IRAM_ATTR onTimer() {
  uint32_t start_time = micros();
  
  portENTER_CRITICAL_ISR(&timerMux);
  
  total_samples++;  // Count every sample for calibration
  
  // 1. UPDATE PLL PHASE
  pll_phase_fp += pll_phase_inc_fp;
  if (pll_phase_fp >= PI_2_FP) {
    pll_phase_fp -= PI_2_FP;
  }
  
  // 2. DETECT ZERO CROSSING
  bool current_is_high = (pll_phase_fp > (PI_2_FP - ZERO_THRESHOLD));
  bool current_is_low = (pll_phase_fp < ZERO_THRESHOLD);
  
  if (last_was_high && current_is_low) {
    last_zero_crossing_index = current_zero_crossing_index;
    current_zero_crossing_index = write_index;
    new_cycle_ready = true;
  }
  
  last_was_high = current_is_high;

  // 3. GENERATE OUTPUT AND GET REFERENCE VALUES
  uint32_t lut_idx = (pll_phase_fp * (SINE_LUT_SIZE - 1)) / PI_2_FP;
  if (lut_idx >= SINE_LUT_SIZE) lut_idx = SINE_LUT_SIZE - 1;
  int16_t sine_val_fp = sine_lut_fp[lut_idx];
  
  // Cosine is sine shifted by 90 degrees
  uint32_t cos_idx = lut_idx + (SINE_LUT_SIZE / 4);
  if (cos_idx >= SINE_LUT_SIZE) cos_idx -= SINE_LUT_SIZE;
  int16_t cosine_val_fp = sine_lut_fp[cos_idx];
  
  portEXIT_CRITICAL_ISR(&timerMux);

  // 4. DAC OUTPUT
  int32_t out_scaled = (sine_val_fp * (int32_t)(OUTPUT_AMPLITUDE * 127)) >> 15;
  int dac_raw = out_scaled + 128;
  
  if (dac_raw > 255) dac_raw = 255;
  else if (dac_raw < 0) dac_raw = 0;
  dac_output_voltage(DAC_CHANNEL_1, (uint8_t)dac_raw);

  // 5. READ INPUT
  int adc_raw = adc1_get_raw(ADC1_CHANNEL_0);
  
  // 6. CONTINUOUS CIRCULAR BUFFER STORAGE
  portENTER_CRITICAL_ISR(&timerMux);
  input_buffer[write_index] = (int16_t)(adc_raw - 2048);
  ref_sin_buffer[write_index] = sine_val_fp;
  ref_cos_buffer[write_index] = cosine_val_fp;
  
  write_index++;
  if (write_index >= MAX_BUFFER_SIZE) {
    write_index = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
  
  uint32_t elapsed = micros() - start_time;
  if (elapsed > isr_max_time) {
    isr_max_time = elapsed;
  }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("ESP32 Grid PLL - SAMPLE RATE CALIBRATED");
  Serial.println("========================================\n");

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
  dac_output_enable(DAC_CHANNEL_1);

  // Pre-compute Fixed-Point LUT
  for (int i = 0; i < SINE_LUT_SIZE; i++) {
    float angle = (PI_2 * i) / (SINE_LUT_SIZE - 1);
    sine_lut_fp[i] = (int16_t)(sin(angle) * 32767.0);
  }

  // Initial Fixed-Point Phase Increment (50 Hz)
  pll_phase_inc_fp = (uint32_t)(((PI_2 * 50.0) / SAMPLE_FREQ) * FP_SCALE);

  Serial.printf("Nominal Sample Rate: %.0f Hz\n", SAMPLE_FREQ);
  Serial.printf("Expected samples per cycle: %.1f\n", SAMPLE_FREQ / 50.0);
  Serial.printf("Max Buffer Size: %d samples\n\n", MAX_BUFFER_SIZE);

  // Timer Setup
  uint32_t timer_period = (uint32_t)(1000000.0 / SAMPLE_FREQ);
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, timer_period, true, 0); 

  Serial.printf("Timer period: %u us\n\n", timer_period);
  
  last_second_millis = millis();
  
  Serial.println("Calibrating actual sample rate (3 seconds)...\n");
}

void loop() {
  // --- CALIBRATION PHASE ---
  if (!calibration_done) {
    uint32_t now = millis();
    if (now - last_second_millis >= 3000) {  // Measure over 3 seconds
      portENTER_CRITICAL(&timerMux);
      uint32_t samples_counted = total_samples - sample_count_at_last_second;
      portEXIT_CRITICAL(&timerMux);
      
      uint32_t elapsed_ms = now - last_second_millis;
      actual_sample_freq = (samples_counted * 1000.0) / elapsed_ms;
      
      calibration_done = true;
      
      Serial.println("========================================");
      Serial.printf("CALIBRATION COMPLETE!\n");
      Serial.printf("Nominal Sample Rate: %.0f Hz\n", SAMPLE_FREQ);
      Serial.printf("Actual Sample Rate:  %.2f Hz\n", actual_sample_freq);
      Serial.printf("Error: %.3f%% (%+.1f Hz)\n", 
                    ((actual_sample_freq - SAMPLE_FREQ) / SAMPLE_FREQ) * 100.0,
                    actual_sample_freq - SAMPLE_FREQ);
      Serial.printf("Measured over %u ms with %u samples\n", elapsed_ms, samples_counted);
      Serial.println("========================================\n");
      
      // Update phase increment with calibrated rate
      pll_phase_inc_fp = (uint32_t)(((PI_2 * 50.0) / actual_sample_freq) * FP_SCALE);
    }
    delay(100);
    return;
  }
  
  if (!new_cycle_ready) {
    delay(1);
    return;
  }
  
  // Get the indices for one complete cycle
  portENTER_CRITICAL(&timerMux);
  new_cycle_ready = false;
  uint16_t start_idx = last_zero_crossing_index;
  uint16_t end_idx = current_zero_crossing_index;
  portEXIT_CRITICAL(&timerMux);
  
  // Calculate number of samples in this cycle
  uint16_t num_samples;
  if (end_idx > start_idx) {
    num_samples = end_idx - start_idx;
  } else {
    num_samples = (MAX_BUFFER_SIZE - start_idx) + end_idx;
  }
  
  // Sanity check
  if (num_samples < 50 || num_samples > MAX_BUFFER_SIZE - 50) {
    return;
  }
  
  debug_samples_used = num_samples;

  // --- EXTRACT ONE CYCLE FROM CIRCULAR BUFFER ---
  float sum_real = 0;
  float sum_imag = 0;
  float sum_sq = 0;
  
  uint16_t idx = start_idx;
  for (uint16_t i = 0; i < num_samples; i++) {
    float val = input_buffer[idx] * (3.3f / 4095.0f);
    float ref_s = ref_sin_buffer[idx] / 32768.0f;
    float ref_c = ref_cos_buffer[idx] / 32768.0f;
    
    sum_real += val * ref_c;
    sum_imag += val * ref_s;
    sum_sq += val * val;
    
    idx++;
    if (idx >= MAX_BUFFER_SIZE) idx = 0;
  }

  // Normalize
  sum_real *= (2.0f / num_samples);
  sum_imag *= (2.0f / num_samples);

  float magnitude = sqrt(sum_real * sum_real + sum_imag * sum_imag);
  float phase_error = atan2(sum_imag, sum_real);
  
  debug_rms = sqrt(sum_sq / num_samples);
  debug_magnitude = magnitude;

  // Wrap to [-π, π]
  while (phase_error > PI) phase_error -= PI_2;
  while (phase_error < -PI) phase_error += PI_2;
  
  debug_phase_err = phase_error;

  // --- PI CONTROLLER ---
  float p_term = KP * phase_error;
  
  float tentative_freq = 50.0f + p_term + integrator;
  bool saturated = (tentative_freq >= 52.0 || tentative_freq <= 48.0);
  
  if (!saturated) {
    float dt = num_samples / actual_sample_freq;
    integrator += KI * phase_error * dt;
  } else {
    integrator *= 0.95;
  }
  
  integrator = constrain(integrator, -5.0, 5.0);
  
  float freq_correction = p_term + integrator;
  
  pll_freq = 50.0f + freq_correction;
  pll_freq = constrain(pll_freq, 48.0, 52.0);
  
  // --- UPDATE PLL PARAMETERS ---
  uint32_t next_inc = (uint32_t)(((PI_2 * pll_freq) / actual_sample_freq) * FP_SCALE);
  
  portENTER_CRITICAL(&timerMux);
  pll_phase_inc_fp = next_inc;
  portEXIT_CRITICAL(&timerMux);

  // --- TELEMETRY ---
  static unsigned long last_print = 0;
  if (millis() - last_print >= 500) {
    last_print = millis();
    
    bool locked = (abs(debug_phase_err * 57.2958) < 10.0);
    float expected_samples = actual_sample_freq / pll_freq;
    
    Serial.printf("Freq: %6.4f Hz | Err: %+6.2f° | Samples: %3d (exp: %.1f) | Mag: %.3f V %s\n",
                  pll_freq, 
                  debug_phase_err * 57.2958,
                  debug_samples_used,
                  expected_samples,
                  debug_magnitude,
                  locked ? "[LOCKED]" : "[TRACKING]");
  }
}
