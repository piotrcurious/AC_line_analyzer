/**
 * @file sogi_pll_production3b_fixed_wdt.ino
 * @brief SOGI-PLL for ESP32 — patched for esp_task_wdt IDF v5 signature
 *
 * Notes:
 *  - This file assumes ESP-IDF v5-style WDT API: esp_task_wdt_init(const esp_task_wdt_config_t*).
 *  - The sampling task registers itself with the WDT; the Arduino loop() task is NOT registered.
 *  - Uses esp_timer for periodic sampling.
 *  - Uses esp_adc_cal for ADC calibration.
 *
 * Author: Patched for user's toolchain
 * Date: 2026-01-27
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <esp_task_wdt.h>
#include "esp_adc_cal.h"
#include "esp_timer.h"

// -------------------- Config --------------------
#define ADC_CHANNEL         ADC1_CHANNEL_0
#define ADC_ATTEN           ADC_ATTEN_DB_11
#define ADC_WIDTH           ADC_WIDTH_BIT_12
#define DAC_CHANNEL         DAC_CHANNEL_1

#define DAC_MAX_VALUE       255
#define DAC_VREF            3.3f
#define VOLTS_TO_DAC        (DAC_MAX_VALUE / DAC_VREF)

#define V_BIAS              0.55f
#define V_OUT_AMPLITUDE     3.0f

#define SAMPLING_FREQ_HZ    10000.0f
#define SAMPLE_TIME_S       (1.0f / SAMPLING_FREQ_HZ)

#define SOGI_GAIN           1.414f
#define PLL_KP              2.0f
#define PLL_KI              15.0f

#define GRID_FREQ_NOMINAL   50.0f
#define GRID_FREQ_MIN       40.0f
#define GRID_FREQ_MAX       70.0f
#define OMEGA_NOMINAL       (2.0f * PI * GRID_FREQ_NOMINAL)
#define OMEGA_MIN           (2.0f * PI * GRID_FREQ_MIN)
#define OMEGA_MAX           (2.0f * PI * GRID_FREQ_MAX)

#define DC_FILTER_ALPHA     0.001f

#define SERIAL_BAUD         115200
#define PRINT_INTERVAL_MS   100
#define WDT_TIMEOUT_S       3

#define STACK_SIZE_WORDS    8192
#define TASK_PRIORITY       24
#define TASK_CORE           1

#define ADC_MIN_VALID_MV    50
#define ADC_MAX_VALID_MV    3200

#define MAX_OFFSET_DRIFT    0.5f
#define INVALID_THRESHOLD   50   // number of consecutive samples before declaring fault

// -------------------- Types --------------------
struct SOGIState {
  float v_alpha;
  float v_beta;
  float omega;
  float theta;
  float integrator;
  float dc_offset;
  uint32_t error_count;
  uint32_t sample_count;
  bool signal_valid;
  uint16_t invalid_count;
};

struct SharedData {
  float frequency;
  float dc_offset;
  float v_grid;
  float v_out;
  uint32_t errors;
  bool valid;
};

// -------------------- Globals --------------------
static SemaphoreHandle_t dataMutex = NULL;
static SharedData sharedData = {GRID_FREQ_NOMINAL, V_BIAS, 0.0f, V_BIAS, 0, true};

static SemaphoreHandle_t timerSemaphore = NULL;
static TaskHandle_t sogiTaskHandle = NULL;
static esp_timer_handle_t periodic_timer = NULL;
static esp_adc_cal_characteristics_t adc_chars;

// -------------------- Utility --------------------
inline float normalizeAngle(float angle) {
  angle = fmodf(angle, 2.0f * PI);
  if (angle < 0.0f) angle += 2.0f * PI;
  return angle;
}
inline float clampf(float v, float a, float b) {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}
inline bool isADCValid_mv(uint32_t mv) {
  return (mv >= ADC_MIN_VALID_MV && mv <= ADC_MAX_VALID_MV);
}

void updateSharedData(const SOGIState& state, float v_grid, float v_out) {
  if (xSemaphoreTake(dataMutex, (TickType_t)0) == pdTRUE) {
    sharedData.frequency = state.omega / (2.0f * PI);
    sharedData.dc_offset = state.dc_offset;
    sharedData.v_grid = v_grid;
    sharedData.v_out = v_out;
    sharedData.errors = state.error_count;
    sharedData.valid = state.signal_valid;
    xSemaphoreGive(dataMutex);
  }
}

SharedData getSharedData() {
  SharedData d;
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    d = sharedData;
    xSemaphoreGive(dataMutex);
  } else {
    d = sharedData;
  }
  return d;
}

// -------------------- esp_timer callback --------------------
static void espTimerCallback(void* arg) {
  xSemaphoreGive(timerSemaphore);
}

// -------------------- ADC calibration --------------------
void initADCCalibration() {
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 1100, &adc_chars);
}

// -------------------- SOGI-PLL --------------------
void initSOGIState(SOGIState& s) {
  s.v_alpha = 0.0f; s.v_beta = 0.0f;
  s.omega = OMEGA_NOMINAL; s.theta = 0.0f;
  s.integrator = OMEGA_NOMINAL;
  s.dc_offset = V_BIAS;
  s.error_count = 0; s.sample_count = 0; s.signal_valid = true;
  s.invalid_count = 0;
}

float processSOGIPLL(SOGIState& state, float v_grid_raw) {
  state.sample_count++;

  // DC offset tracking
  state.dc_offset = state.dc_offset * (1.0f - DC_FILTER_ALPHA) + v_grid_raw * DC_FILTER_ALPHA;
  float delta_offset = fabsf(state.dc_offset - V_BIAS);

  // Hysteresis counting
  if (delta_offset > MAX_OFFSET_DRIFT) {
    if (state.invalid_count < 0xFFFF) state.invalid_count++;
  } else {
    if (state.invalid_count > 0) state.invalid_count--;
  }

  if (state.invalid_count >= INVALID_THRESHOLD) {
    // transition to persistent fault — increment error only once per transition
    if (state.signal_valid) {
      state.error_count++;
    }
    state.signal_valid = false;
  } else {
    state.signal_valid = true;
  }

  float v_grid_clean = v_grid_raw - state.dc_offset;

  if (!state.signal_valid) {
    // Safe re-lock: bias integrator gently towards nominal
    state.integrator = state.integrator * 0.995f + OMEGA_NOMINAL * 0.005f;
    state.omega = clampf(state.integrator, OMEGA_MIN, OMEGA_MAX);
    state.theta += state.omega * SAMPLE_TIME_S;
    state.theta = normalizeAngle(state.theta);
    return V_BIAS + V_OUT_AMPLITUDE * sinf(state.theta);
  }

  // SOGI updates (Euler)
  float omega_ts = state.omega * SAMPLE_TIME_S;
  float error = v_grid_clean - state.v_alpha;
  float alpha_derivative = omega_ts * state.v_beta + SOGI_GAIN * omega_ts * error;
  float beta_derivative  = -omega_ts * state.v_alpha;
  state.v_alpha += alpha_derivative;
  state.v_beta  += beta_derivative;

  // Park transform
  float sin_theta = sinf(state.theta);
  float cos_theta = cosf(state.theta);
  float v_q = -state.v_alpha * sin_theta + state.v_beta * cos_theta;

  // PI with anti-windup
  float prop = PLL_KP * v_q;
  float integ_inc = PLL_KI * SAMPLE_TIME_S * v_q;
  bool will_high = (state.integrator >= OMEGA_MAX) && (integ_inc > 0.0f);
  bool will_low  = (state.integrator <= OMEGA_MIN) && (integ_inc < 0.0f);
  if (!(will_high || will_low)) state.integrator += integ_inc;
  else state.integrator += integ_inc * 0.1f;
  state.integrator = clampf(state.integrator, OMEGA_MIN, OMEGA_MAX);
  state.omega = clampf(state.integrator + prop, OMEGA_MIN, OMEGA_MAX);

  // VCO
  state.theta += state.omega * SAMPLE_TIME_S;
  state.theta = normalizeAngle(state.theta);

  return V_BIAS + V_OUT_AMPLITUDE * sinf(state.theta);
}

// -------------------- DAC write --------------------
void writeDAC(float voltage) {
  int dac_value = (int)(voltage * VOLTS_TO_DAC);
  if (dac_value < 0) dac_value = 0;
  if (dac_value > DAC_MAX_VALUE) dac_value = DAC_MAX_VALUE;
  dac_output_voltage(DAC_CHANNEL, (uint8_t)dac_value);
}

// -------------------- Sampling task --------------------
void sogiTask(void *pvParameters) {
  SOGIState state;
  initSOGIState(state);

  // Register sampling task with WDT
  esp_task_wdt_add(NULL);

  Serial.println("[SOGI] Task started on Core " + String((int)xPortGetCoreID()));

  while (true) {
    if (xSemaphoreTake(timerSemaphore, portMAX_DELAY) == pdTRUE) {
      // reset WDT
      esp_task_wdt_reset();

      // Read & calibrate ADC
      int raw = adc1_get_raw(ADC_CHANNEL);
      uint32_t mv = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
      if (!isADCValid_mv(mv)) {
        // transient ADC invalid — increase error_count modestly, output bias
        state.error_count++;
        writeDAC(V_BIAS);
        continue;
      }

      float v_grid_raw = ((float)mv) / 1000.0f;
      float v_out = processSOGIPLL(state, v_grid_raw);
      writeDAC(v_out);

      if (state.sample_count % 10 == 0) {
        float v_grid_clean = v_grid_raw - state.dc_offset;
        updateSharedData(state, v_grid_clean, v_out);
      }
    }
  }
}

// -------------------- Timer init using esp_timer --------------------
void initHardwareTimer() {
  timerSemaphore = xSemaphoreCreateBinary();
  if (!timerSemaphore) {
    Serial.println("[ERROR] timer sem fail");
    while (1) delay(1000);
  }

  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &espTimerCallback,
    .arg = NULL,
    .name = "sogi_periodic"
  };

  if (esp_timer_create(&periodic_timer_args, &periodic_timer) != ESP_OK) {
    Serial.println("[ERROR] esp_timer_create failed");
    while (1) delay(1000);
  }

  uint64_t period_us = (uint64_t)(SAMPLE_TIME_S * 1e6);
  if (esp_timer_start_periodic(periodic_timer, period_us) != ESP_OK) {
    Serial.println("[ERROR] esp_timer_start_periodic failed");
    while (1) delay(1000);
  }
}

// -------------------- Setup & Loop --------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println("\n[SOGI] starting...");

  adc1_config_width(ADC_WIDTH);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
  initADCCalibration();

  dac_output_enable(DAC_CHANNEL);
  dac_output_voltage(DAC_CHANNEL, (uint8_t)(V_BIAS * VOLTS_TO_DAC));

  // Initialize WDT using IDF v5 signature (pointer to config)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = (uint32_t)(WDT_TIMEOUT_S * 1000),
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);

  dataMutex = xSemaphoreCreateMutex();
  if (!dataMutex) {
    Serial.println("[ERROR] dataMutex fail");
    while (1) delay(1000);
  }

  initHardwareTimer();

  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    sogiTask,
    "SOGI_PLL",
    STACK_SIZE_WORDS,
    NULL,
    TASK_PRIORITY,
    &sogiTaskHandle,
    TASK_CORE
  );
  if (taskCreated != pdPASS) {
    Serial.println("[ERROR] task create fail");
    while (1) delay(1000);
  }

  // Do NOT register the loop() task with the WDT here.
  // The sampling task (sogiTask) registers itself with esp_task_wdt_add(NULL).

  Serial.printf("[INFO] Free Heap: %d\n", ESP.getFreeHeap());
  Serial.println("[READY]");
}

void loop() {
  static unsigned long lastPrint = 0;
  static uint32_t lastErrorCount = 0;
  unsigned long now = millis();

  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    SharedData d = getSharedData();

    Serial.print("Freq: ");
    Serial.print(d.frequency, 3);
    Serial.print(" Hz | Vgrid: ");
    Serial.print(d.v_grid, 3);
    Serial.print(" V | Out: ");
    Serial.print(d.v_out, 3);
    Serial.print(" V | DC: ");
    Serial.print(d.dc_offset, 3);
    Serial.print(" V | Errors: ");
    Serial.print(d.errors);

    if (d.errors > lastErrorCount) {
      Serial.print(" [NEW!]");
      lastErrorCount = d.errors;
    }

    Serial.print(d.valid ? " [OK]" : " [FAULT]");
    Serial.println();

    static unsigned long lastDiag = 0;
    if (now - lastDiag >= 5000) {
      lastDiag = now;
      if (sogiTaskHandle) {
        UBaseType_t stackRemaining = uxTaskGetStackHighWaterMark(sogiTaskHandle);
        Serial.printf("[DIAG] Stack remaining: %u words\n", (unsigned)stackRemaining);
      }
      Serial.printf("[DIAG] Free heap: %d bytes\n", ESP.getFreeHeap());
    }
  }
}
