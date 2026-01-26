#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <soc/sens_reg.h>
#include <soc/soc.h>

// ================================================================
// CONFIGURATION
// ================================================================
#define SAMPLING_FREQ 10000.0 // 10 kHz
#define TS (1.0 / SAMPLING_FREQ)
#define PI_2 (2.0 * PI)

// PLL Gains (Tuned for 50Hz)
#define SOGI_K 1.414  // SOGI Damping factor (1.414 is standard)
#define PLL_KP 2.0    // Proportional Gain
#define PLL_KI 15.0   // Integral Gain

// Pins
#define ADC_PIN 36
#define DAC_PIN 25

// ================================================================
// GLOBAL VARIABLES (VOLATILE for ISR)
// ================================================================

// SOGI State Variables
volatile float s_v_alpha = 0.0f; // Filtered Grid Voltage
volatile float s_v_beta = 0.0f;  // Quadrature Voltage (90 deg shift)

// PLL State Variables
volatile float pll_omega = 314.159f; // Angular freq (starts at 50Hz)
volatile float pll_theta = 0.0f;     // Grid Phase angle
volatile float pll_freq = 50.0f;     // Frequency in Hz

// DC Offset Removal
volatile float v_grid_offset = 1.85f; // Initial guess for DC offset
const float dc_filter_alpha = 0.001f; // Slow filter for DC offset

// Debugging / shared variables
volatile float debug_v_grid = 0;
volatile float debug_v_out = 0;
hw_timer_t *timer = NULL;

// ================================================================
// SOGI-PLL ALGORITHM (The "Magic")
// ================================================================
// This function runs inside the Interrupt Service Routine (ISR)
// It implements a Second Order Generalized Integrator (SOGI)
// to clean the signal and lock the phase.
void IRAM_ATTR onTimer() {
  
  // 1. READ ADC (Fast optimized read recommended)
  // ADC1 Channel 0 is GPIO 36. 
  // Raw 0-4095 mapped to roughly 0-3.3V (attenuation dependent)
  // For speed in ISR, we assume simple linear mapping here.
  int raw = adc1_get_raw(ADC1_CHANNEL_0);
  
  // Convert to Voltage (approximate, calibrate as needed)
  float v_in_raw = raw * (3.3f / 4095.0f);
  
  // 2. REMOVE DC OFFSET (Adaptive High Pass)
  // Simple LPF to find the DC component, then subtract it
  v_grid_offset = (v_grid_offset * (1.0f - dc_filter_alpha)) + (v_in_raw * dc_filter_alpha);
  float v_grid_clean = v_in_raw - v_grid_offset;

  // 3. SOGI FILTER (Bandpass + Quadrature Generator)
  // Discretized integration of SOGI dynamics
  // error = v_input - v_alpha
  float error_sogi = v_grid_clean - s_v_alpha;
  
  // Update Alpha (In-phase component)
  // v_alpha_dot = omega * v_beta + k * omega * error
  s_v_alpha = s_v_alpha + ((pll_omega * s_v_beta) + (SOGI_K * pll_omega * error_sogi)) * TS;
  
  // Update Beta (Quadrature component, -90 deg shifted)
  // v_beta_dot = -omega * v_alpha
  s_v_beta = s_v_beta + (-pll_omega * s_v_alpha) * TS;

  // 4. PARK TRANSFORM (Phase Detector)
  // We want to align the PLL such that v_q becomes 0
  float sin_val = sin(pll_theta);
  float cos_val = cos(pll_theta);
  
  // v_q = -v_alpha * sin(theta) + v_beta * cos(theta)
  float v_q = -s_v_alpha * sin_val + s_v_beta * cos_val;

  // 5. PI REGULATOR (Loop Filter)
  // Adjusts frequency based on phase error (v_q)
  static float integrator_sum = 314.159f; // Initialize at nominal 50Hz
  
  integrator_sum += (PLL_KI * TS * v_q);
  pll_omega = integrator_sum + (PLL_KP * v_q);

  // Clamp Frequency (45Hz to 55Hz safety limits)
  // 45Hz ~= 282 rad/s, 55Hz ~= 345 rad/s
  if (pll_omega > 345.0f) pll_omega = 345.0f;
  if (pll_omega < 282.0f) pll_omega = 282.0f;

  // 6. PHASE INTEGRATOR (VCO)
  pll_theta += pll_omega * TS;
  
  // Wrap Phase 0 -> 2PI
  if (pll_theta > PI_2) {
    pll_theta -= PI_2;
  }

  // 7. GENERATE OUTPUT
  // Generate a clean sine wave locked to the grid
  // We use the locked theta to generate a perfect sine, regardless of input distortion
  float v_out = 1.65f + (1.0f * sin_val); // 1.65V offset, 1V amplitude
  
  // Convert to 8-bit for DAC (0-255)
  int dac_val = (int)(v_out * (255.0f / 3.3f));
  if (dac_val > 255) dac_val = 255;
  if (dac_val < 0) dac_val = 0;
  
  dac_output_voltage(DAC_CHANNEL_1, dac_val);

  // Store for main loop debugging
  debug_v_grid = v_grid_clean;
  debug_v_out = v_out;
  pll_freq = pll_omega / PI_2;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  // ADC Setup
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

  // DAC Setup
  dac_output_enable(DAC_CHANNEL_1);

  // Timer Setup (ESP32 Specific)
  // Timer 0, Prescaler 80 (80MHz / 80 = 1MHz tick)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  // Alarm every 100 ticks (100us = 10kHz)
  timerAlarmWrite(timer, 100, true);
  timerAlarmEnable(timer);

  Serial.println("SOGI-PLL Initialized");
  Serial.println("Time(ms), Grid(V), Filtered(V), Output(V), Freq(Hz), PhaseErr");
}

// ================================================================
// LOOP (Low priority tasks only)
// ================================================================
void loop() {
  // We only print data here. All calculation is done in ISR.
  // This ensures the PLL never "stutters" due to Serial printing.
  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) { // Print every 50ms
    lastPrint = millis();
    
    // We can print the "filtered" internal signal (s_v_alpha) to see
    // how well the SOGI is rejecting distortion.
    Serial.print(millis());
    Serial.print(",");
    Serial.print(debug_v_grid); // Raw Input (DC removed)
    Serial.print(",");
    Serial.print(s_v_alpha);    // SOGI Cleaned Signal
    Serial.print(",");
    Serial.print(debug_v_out);  // Locked Output
    Serial.print(",");
    Serial.print(pll_freq);     // Detected Frequency
    Serial.println();
  }
}
