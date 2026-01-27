/**
 * @file sogi_pll_production.ino
 * @brief Production-Ready SOGI-PLL Grid Synchronization for ESP32
 * 
 * Implements Second Order Generalized Integrator Phase Locked Loop
 * for single-phase grid synchronization with robust error handling.
 * 
 * @author Refactored for Production Use
 * @date 2026-01-27
 * @version 2.0.0
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <esp_task_wdt.h>

// ================================================================
// HARDWARE CONFIGURATION
// ================================================================
#define ADC_CHANNEL         ADC1_CHANNEL_0
#define ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC_WIDTH           ADC_WIDTH_BIT_12
#define DAC_CHANNEL         DAC_CHANNEL_1

// ADC/DAC Calibration
#define ADC_MAX_VALUE       4095.0f
#define ADC_VREF            3.3f
#define ADC_TO_VOLTS        (ADC_VREF / ADC_MAX_VALUE)  // 0.00080586f
#define DAC_MAX_VALUE       255
#define DAC_VREF            3.3f
#define VOLTS_TO_DAC        (DAC_MAX_VALUE / DAC_VREF)  // 77.27f

// Signal conditioning
#define V_BIAS              1.65f  // Mid-point bias voltage
#define V_OUT_AMPLITUDE     1.0f   // Output sine wave amplitude

// ================================================================
// TIMING CONFIGURATION
// ================================================================
#define SAMPLING_FREQ_HZ    10000.0f
#define TIMER_FREQ_HZ       1000000UL  // 1 MHz timer base
#define TIMER_ALARM_US      (TIMER_FREQ_HZ / SAMPLING_FREQ_HZ)  // 100 µs
#define SAMPLE_TIME_S       (1.0f / SAMPLING_FREQ_HZ)  // 0.0001s

// ================================================================
// PLL CONFIGURATION
// ================================================================
// SOGI Parameters
#define SOGI_GAIN           1.414f  // Damping factor (√2 for optimal response)

// PLL Controller Gains (tuned for 50Hz nominal)
#define PLL_KP              2.0f    // Proportional gain
#define PLL_KI              15.0f   // Integral gain

// Frequency Limits
#define GRID_FREQ_NOMINAL   50.0f   // Hz
#define GRID_FREQ_MIN       45.0f   // Hz
#define GRID_FREQ_MAX       55.0f   // Hz
#define OMEGA_NOMINAL       (2.0f * PI * GRID_FREQ_NOMINAL)  // 314.159 rad/s
#define OMEGA_MIN           (2.0f * PI * GRID_FREQ_MIN)      // 282.743 rad/s
#define OMEGA_MAX           (2.0f * PI * GRID_FREQ_MAX)      // 345.575 rad/s

// DC Offset Filter
#define DC_FILTER_ALPHA     0.001f  // Low-pass filter coefficient (τ ≈ 1s)

// ================================================================
// DIAGNOSTICS CONFIGURATION
// ================================================================
#define SERIAL_BAUD         115200
#define PRINT_INTERVAL_MS   100
#define WDT_TIMEOUT_S       3
#define STACK_SIZE_WORDS    8192   // Increased for safety margin
#define TASK_PRIORITY       24     // High priority (max is 25)
#define TASK_CORE           1      // Pin to Core 1

// Signal quality thresholds
#define ADC_MIN_VALID       100    // Minimum valid ADC reading
#define ADC_MAX_VALID       3995   // Maximum valid ADC reading
#define MAX_OFFSET_DRIFT    0.5f   // Maximum allowed DC offset (V)

// ================================================================
// STATE STRUCTURE
// ================================================================
struct SOGIState {
  // SOGI filter states
  float v_alpha;
  float v_beta;
  
  // PLL states
  float omega;        // Angular frequency (rad/s)
  float theta;        // Phase angle (rad)
  float integrator;   // PI controller integrator state
  
  // DC offset tracking
  float dc_offset;
  
  // Diagnostics
  uint32_t error_count;
  uint32_t sample_count;
  bool signal_valid;
};

// ================================================================
// THREAD-SAFE DATA SHARING
// ================================================================
struct SharedData {
  float frequency;    // Hz
  float dc_offset;    // V
  float v_grid;       // V
  float v_out;        // V
  uint32_t errors;
  bool valid;
};

// Protected by mutex
static SemaphoreHandle_t dataMutex = NULL;
static SharedData sharedData = {GRID_FREQ_NOMINAL, V_BIAS, 0.0f, V_BIAS, 0, true};

// ================================================================
// TASK SYNCHRONIZATION
// ================================================================
static SemaphoreHandle_t timerSemaphore = NULL;
static hw_timer_t *timer = NULL;
static TaskHandle_t sogiTaskHandle = NULL;

// ================================================================
// UTILITY FUNCTIONS
// ================================================================

/**
 * @brief Normalize angle to [0, 2π)
 */
inline float normalizeAngle(float angle) {
  angle = fmodf(angle, 2.0f * PI);
  if (angle < 0.0f) angle += 2.0f * PI;
  return angle;
}

/**
 * @brief Clamp value between min and max
 */
inline float clamp(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

/**
 * @brief Validate ADC reading
 */
inline bool isADCValid(int raw) {
  return (raw >= ADC_MIN_VALID && raw <= ADC_MAX_VALID);
}

/**
 * @brief Update shared data (thread-safe)
 */
void updateSharedData(const SOGIState& state, float v_grid, float v_out) {
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    sharedData.frequency = state.omega / (2.0f * PI);
    sharedData.dc_offset = state.dc_offset;
    sharedData.v_grid = v_grid;
    sharedData.v_out = v_out;
    sharedData.errors = state.error_count;
    sharedData.valid = state.signal_valid;
    xSemaphoreGive(dataMutex);
  }
}

/**
 * @brief Get shared data copy (thread-safe)
 */
SharedData getSharedData() {
  SharedData data;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    data = sharedData;
    xSemaphoreGive(dataMutex);
  }
  return data;
}

// ================================================================
// TIMER ISR
// ================================================================
void IRAM_ATTR onTimer() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(timerSemaphore, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// ================================================================
// SOGI-PLL PROCESSING
// ================================================================

/**
 * @brief Initialize SOGI-PLL state
 */
void initSOGIState(SOGIState& state) {
  state.v_alpha = 0.0f;
  state.v_beta = 0.0f;
  state.omega = OMEGA_NOMINAL;
  state.theta = 0.0f;
  state.integrator = OMEGA_NOMINAL;
  state.dc_offset = V_BIAS;
  state.error_count = 0;
  state.sample_count = 0;
  state.signal_valid = true;
}

/**
 * @brief Process one SOGI-PLL iteration
 * @return Output voltage value (V)
 */
float processSOGIPLL(SOGIState& state, float v_grid_raw) {
  state.sample_count++;
  
  // ============================================================
  // 1. DC OFFSET REMOVAL (Exponential moving average)
  // ============================================================
  state.dc_offset = state.dc_offset * (1.0f - DC_FILTER_ALPHA) + 
                    v_grid_raw * DC_FILTER_ALPHA;
  
  // Check for excessive DC drift
  if (fabsf(state.dc_offset - V_BIAS) > MAX_OFFSET_DRIFT) {
    state.error_count++;
    state.signal_valid = false;
  } else {
    state.signal_valid = true;
  }
  
  float v_grid_clean = v_grid_raw - state.dc_offset;
  
  // ============================================================
  // 2. SOGI FILTER (Orthogonal signal generation)
  // ============================================================
  // CRITICAL: Update both states simultaneously to avoid coupling
  float omega_ts = state.omega * SAMPLE_TIME_S;
  float error = v_grid_clean - state.v_alpha;
  
  float alpha_derivative = omega_ts * state.v_beta + 
                          SOGI_GAIN * omega_ts * error;
  float beta_derivative = -omega_ts * state.v_alpha;
  
  state.v_alpha += alpha_derivative;
  state.v_beta += beta_derivative;
  
  // ============================================================
  // 3. PHASE DETECTOR (Park transformation to rotating frame)
  // ============================================================
  float sin_theta = sinf(state.theta);
  float cos_theta = cosf(state.theta);
  
  // Transform to dq frame (v_q should be zero when locked)
  float v_q = -state.v_alpha * sin_theta + state.v_beta * cos_theta;
  
  // ============================================================
  // 4. PI CONTROLLER (Loop filter)
  // ============================================================
  // Integrator with anti-windup
  float integrator_increment = PLL_KI * SAMPLE_TIME_S * v_q;
  state.integrator += integrator_increment;
  
  // Clamp integrator to frequency limits
  state.integrator = clamp(state.integrator, OMEGA_MIN, OMEGA_MAX);
  
  // PI output
  state.omega = state.integrator + PLL_KP * v_q;
  
  // Additional output clamping for safety
  state.omega = clamp(state.omega, OMEGA_MIN, OMEGA_MAX);
  
  // ============================================================
  // 5. PHASE INTEGRATOR (VCO)
  // ============================================================
  state.theta += state.omega * SAMPLE_TIME_S;
  state.theta = normalizeAngle(state.theta);
  
  // ============================================================
  // 6. OUTPUT GENERATION
  // ============================================================
  float v_out = V_BIAS + V_OUT_AMPLITUDE * sin_theta;
  
  return v_out;
}

/**
 * @brief Write voltage to DAC with safety checks
 */
void writeDAC(float voltage) {
  // Convert voltage to DAC value
  int dac_value = (int)(voltage * VOLTS_TO_DAC);
  
  // Clamp to valid range
  dac_value = constrain(dac_value, 0, DAC_MAX_VALUE);
  
  // Write to DAC
  dac_output_voltage(DAC_CHANNEL, (uint8_t)dac_value);
}

// ================================================================
// MAIN PROCESSING TASK
// ================================================================
void sogiTask(void *pvParameters) {
  SOGIState state;
  initSOGIState(state);
  
  // Add this task to watchdog
  esp_task_wdt_add(NULL);
  
  Serial.println("[SOGI] Task started on Core " + String(xPortGetCoreID()));
  
  while (true) {
    // Wait for timer interrupt
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
      
      // Reset watchdog
      esp_task_wdt_reset();
      
      // ============================================================
      // READ ADC
      // ============================================================
      int adc_raw = adc1_get_raw(ADC_CHANNEL);
      
      // Validate reading
      if (!isADCValid(adc_raw)) {
        state.error_count++;
        // Use last valid offset if ADC fails
        writeDAC(V_BIAS);
        continue;
      }
      
      // Convert to voltage
      float v_grid_raw = (float)adc_raw * ADC_TO_VOLTS;
      
      // ============================================================
      // PROCESS SOGI-PLL
      // ============================================================
      float v_out = processSOGIPLL(state, v_grid_raw);
      
      // ============================================================
      // WRITE DAC
      // ============================================================
      writeDAC(v_out);
      
      // ============================================================
      // UPDATE SHARED DATA (every ~10 samples to reduce overhead)
      // ============================================================
      if (state.sample_count % 10 == 0) {
        float v_grid_clean = v_grid_raw - state.dc_offset;
        updateSharedData(state, v_grid_clean, v_out);
      }
    }
  }
}

// ================================================================
// ARDUINO SETUP
// ================================================================
void setup() {
  // Initialize serial
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println("\n\n================================");
  Serial.println("SOGI-PLL v2.0 - Production Ready");
  Serial.println("================================\n");
  
  // ============================================================
  // HARDWARE INITIALIZATION
  // ============================================================
  Serial.println("[INIT] Configuring ADC...");
  adc1_config_width(ADC_WIDTH);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
  
  Serial.println("[INIT] Configuring DAC...");
  dac_output_enable(DAC_CHANNEL);
  dac_output_voltage(DAC_CHANNEL, (uint8_t)(V_BIAS * VOLTS_TO_DAC));
  
  // ============================================================
  // RTOS INITIALIZATION
  // ============================================================
  Serial.println("[INIT] Creating synchronization primitives...");
  
  timerSemaphore = xSemaphoreCreateBinary();
  if (timerSemaphore == NULL) {
    Serial.println("[ERROR] Failed to create timer semaphore!");
    while(1) delay(1000);
  }
  
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("[ERROR] Failed to create data mutex!");
    while(1) delay(1000);
  }
  
  // ============================================================
  // TASK CREATION
  // ============================================================
  Serial.println("[INIT] Creating SOGI processing task...");
  
  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    sogiTask,           // Task function
    "SOGI_PLL",         // Name
    STACK_SIZE_WORDS,   // Stack size
    NULL,               // Parameters
    TASK_PRIORITY,      // Priority
    &sogiTaskHandle,    // Task handle
    TASK_CORE           // Core
  );
  
  if (taskCreated != pdPASS) {
    Serial.println("[ERROR] Failed to create SOGI task!");
    while(1) delay(1000);
  }
  
  // ============================================================
  // WATCHDOG CONFIGURATION
  // ============================================================
  Serial.println("[INIT] Configuring watchdog...");
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  
  // ============================================================
  // TIMER INITIALIZATION
  // ============================================================
  Serial.println("[INIT] Starting hardware timer...");
  
  timer = timerBegin(TIMER_FREQ_HZ);
  if (timer == NULL) {
    Serial.println("[ERROR] Failed to initialize timer!");
    while(1) delay(1000);
  }
  
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, TIMER_ALARM_US, true, 0);
  
  // ============================================================
  // DIAGNOSTICS
  // ============================================================
  Serial.println("\n[INFO] Configuration:");
  Serial.printf("  Sampling Rate: %.0f Hz\n", SAMPLING_FREQ_HZ);
  Serial.printf("  Sample Time: %.1f µs\n", SAMPLE_TIME_S * 1e6f);
  Serial.printf("  Nominal Frequency: %.1f Hz\n", GRID_FREQ_NOMINAL);
  Serial.printf("  Frequency Range: %.1f - %.1f Hz\n", GRID_FREQ_MIN, GRID_FREQ_MAX);
  Serial.printf("  SOGI Gain: %.3f\n", SOGI_GAIN);
  Serial.printf("  PLL Kp: %.1f, Ki: %.1f\n", PLL_KP, PLL_KI);
  Serial.printf("  Task Stack: %d words (%.1f KB)\n", STACK_SIZE_WORDS, STACK_SIZE_WORDS * 4 / 1024.0f);
  
  // Print free heap
  Serial.printf("\n[INFO] Free Heap: %d bytes\n", ESP.getFreeHeap());
  
  Serial.println("\n[READY] System initialized successfully!");
  Serial.println("Starting data acquisition...\n");
}

// ================================================================
// ARDUINO LOOP (MONITORING)
// ================================================================
void loop() {
  static unsigned long lastPrint = 0;
  static uint32_t lastErrorCount = 0;
  
  unsigned long now = millis();
  
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;
    
    // Get thread-safe copy of data
    SharedData data = getSharedData();
    
    // Print status
    Serial.print("Freq: ");
    Serial.print(data.frequency, 3);
    Serial.print(" Hz | Theta: ");
    Serial.print(data.v_grid, 3);
    Serial.print(" V | Out: ");
    Serial.print(data.v_out, 3);
    Serial.print(" V | DC: ");
    Serial.print(data.dc_offset, 3);
    Serial.print(" V | Errors: ");
    Serial.print(data.errors);
    
    // Check for new errors
    if (data.errors > lastErrorCount) {
      Serial.print(" [NEW!]");
      lastErrorCount = data.errors;
    }
    
    // Signal validity indicator
    Serial.print(data.valid ? " [OK]" : " [FAULT]");
    
    Serial.println();
    
    // Stack monitoring (every 5 seconds)
    static unsigned long lastStackCheck = 0;
    if (now - lastStackCheck >= 5000) {
      lastStackCheck = now;
      if (sogiTaskHandle != NULL) {
        UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(sogiTaskHandle);
        Serial.printf("[DIAG] Stack remaining: %d words (%.1f%%)\n", 
                      stackRemaining, 
                      (stackRemaining * 100.0f) / STACK_SIZE_WORDS);
        
        if (stackRemaining < 512) {
          Serial.println("[WARN] Low stack space!");
        }
      }
      Serial.printf("[DIAG] Free heap: %d bytes\n", ESP.getFreeHeap());
    }
  }
}
