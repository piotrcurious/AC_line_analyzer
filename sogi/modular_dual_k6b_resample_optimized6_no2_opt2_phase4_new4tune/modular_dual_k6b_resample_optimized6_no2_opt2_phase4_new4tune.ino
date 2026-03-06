#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "SOGI.h"
#include "SOGIvisualizer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"

/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK — Phase 4 Refined: Active Distortion Correction
 * =============================================================================
 */

#define ADC_PIN_V   36
#define ADC_PIN_I   39
#define V_CHANNEL   0
#define I_CHANNEL   3
#define ADC_OVERSAMPLE_RATE   200000
#define CONV_FRAME_SIZE       64
#define FRAME_BUFFER_SIZE     1024

#define NOMINAL_FREQ       50.0f
#define SOGI_K             0.7071f
#define FLL_GAMMA          8500.0f
#define FLL_LEARN_RATE     0.1f

#define TARGET_VIRTUAL_RATE    6400.0f
#define MIN_SAMPLES_PER_CYCLE  100
#define MAX_SAMPLES_PER_CYCLE  256

uint32_t samples_per_cycle = 128;

// ── Global operators ─────────────────────────────────────────────────────────
SOGIVisualizer vis;
SOGI           sogi_v1(SOGI_K);         // Fundamental
SOGI           sogi_v3(SOGI_K);         // 3rd Harmonic
SOGI           sogi_v5(SOGI_K);         // 5th Harmonic
SOGI           sogi_v7(SOGI_K);         // 7th Harmonic
SOGI           sogi_v9(SOGI_K);         // 9th Harmonic
SOGI           sogi_v11(SOGI_K);        // 11th Harmonic
AdaptiveFLL    fll(NOMINAL_FREQ, FLL_GAMMA, FLL_LEARN_RATE);

// ── DMA frame ring ──────────────────────────────────────────────────────────
struct TimestampedFrame {
    uint32_t end_timestamp;
    uint8_t  raw_data[CONV_FRAME_SIZE];
    uint16_t data_size;
};

volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint32_t frame_write_idx    = 0;
volatile uint32_t frame_read_idx     = 0;
volatile uint32_t isr_callback_count = 0;
volatile uint32_t frames_dropped     = 0;

adc_continuous_handle_t adc_handle = NULL;
uint32_t cpu_freq_hz = 0; float inv_cpu_freq = 0.0f; uint32_t cycles_per_adc_sample = 0;
uint32_t ticks_per_sample_int = 0; double ticks_per_sample_frac = 0;
uint32_t next_sample_int = 0; double next_sample_frac = 0;

float v_buf[MAX_SAMPLES_PER_CYCLE]; float i_buf[MAX_SAMPLES_PER_CYCLE]; uint32_t buf_wr = 0;

struct ResamplerState {
    uint32_t last_frame_end_ts = 0;
    double   acc_v = 0.0; double acc_i = 0.0; double acc_weight = 0.0;
    uint32_t prev_hw_ts = 0; float prev_hw_frac = 0.0f;
    bool     initialized = false;
} resamp;

float v_dc_offset = 1650.0f; float i_dc_offset = 1650.0f; bool dc_bootstrap_done = false;

struct PhaseTrack { float prev_phase = 0.0f; int32_t winding = 0; bool initialized = false; } phase_track;
#define HARMONIC_SMOOTH_ALPHA 0.2f
float harmonic_mag1_smooth = 1e-6f; float harmonic_mag3_smooth = 1e-6f; float harmonic_mag5_smooth = 1e-6f; float harmonic_mag7_smooth = 1e-6f;
float harmonic_mag9_smooth = 1e-6f; float harmonic_mag11_smooth = 1e-6f;

TaskHandle_t visTaskHandle = NULL; TaskHandle_t logTaskHandle = NULL;
QueueHandle_t visQueue = NULL; QueueHandle_t logQueue = NULL;

struct VisSnapshot { float v_copy[MAX_SAMPLES_PER_CYCLE]; float i_copy[MAX_SAMPLES_PER_CYCLE]; int aligned_start; int count; float pll_freq; float vdc, idc; bool in_use; };
#define VIS_SNAPSHOT_COUNT 4
VisSnapshot vis_snapshots[VIS_SNAPSHOT_COUNT];

struct SerialMsg { float pll_freq; float core_us; uint32_t isr_count; uint32_t dropped; float h3_ratio; float h5_ratio; float h7_ratio; float h9_ratio; float h11_ratio; };

static inline uint32_t IRAM_ATTR get_cycle_count() { uint32_t ccount; asm volatile("rsr %0, ccount" : "=a"(ccount)); return ccount; }
static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw) { return (float)raw * (3300.0f / 4095.0f); }
static inline int32_t IRAM_ATTR signed_time_diff(uint32_t a, uint32_t b) { return (int32_t)(a - b); }

static void IRAM_ATTR updateTimingParameters(float frequency) {
    uint32_t spc = __atomic_load_n(&samples_per_cycle, __ATOMIC_ACQUIRE);
    double tps_d = (double)cpu_freq_hz / (double)frequency / (double)spc;
    ticks_per_sample_int = (uint32_t)tps_d; ticks_per_sample_frac = tps_d - (double)ticks_per_sample_int;
}

static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    uint32_t ts = get_cycle_count(); uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE); uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_RELAXED);
    uint32_t next_wr = (wr + 1) % FRAME_BUFFER_SIZE;
    if (next_wr == rd) { rd = (rd + 1) % FRAME_BUFFER_SIZE; __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE); __atomic_fetch_add(&frames_dropped, 1u, __ATOMIC_RELAXED); }
    uint32_t sz = edata->size; if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;
    memcpy((void *)frame_buffer[wr].raw_data, edata->conv_frame_buffer, sz);
    frame_buffer[wr].end_timestamp = ts; frame_buffer[wr].data_size = (uint16_t)sz;
    __atomic_store_n(&frame_write_idx, next_wr, __ATOMIC_RELEASE); __atomic_fetch_add(&isr_callback_count, 1u, __ATOMIC_RELAXED);
    return false;
}

void IRAM_ATTR processIncomingDMA() {
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE); uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);
    if (rd == wr) return; uint32_t spc = __atomic_load_n(&samples_per_cycle, __ATOMIC_ACQUIRE);
    while (rd != wr) {
        volatile TimestampedFrame *f_ptr = &frame_buffer[rd]; uint32_t f_end = f_ptr->end_timestamp;
        int n = f_ptr->data_size / sizeof(adc_digi_output_data_t); if (n <= 0) { rd = (rd + 1) % FRAME_BUFFER_SIZE; continue; }
        int total_pairs = n / 2; uint32_t elapsed = (resamp.initialized) ? (uint32_t)(f_end - resamp.last_frame_end_ts) : (uint32_t)cycles_per_adc_sample * total_pairs;
        uint32_t f_start = resamp.last_frame_end_ts; const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f_ptr->raw_data;
        for (int pair = 0; pair < total_pairs; ++pair) {
            uint16_t rv = 0, ri = 0; int base = pair * 2;
            if (p[base].type1.channel == V_CHANNEL) rv = p[base].type1.data; else ri = p[base].type1.data;
            if (p[base+1].type1.channel == V_CHANNEL) rv = p[base+1].type1.data; else ri = p[base+1].type1.data;
            float fv = adcRawToMillivolts(rv); float fi = adcRawToMillivolts(ri);
            uint32_t ts = f_start + (uint32_t)(((uint64_t)(pair + 1) * (uint64_t)elapsed) / total_pairs);
            if (!resamp.initialized) { resamp.last_frame_end_ts = f_end; resamp.prev_hw_ts = ts; resamp.prev_hw_frac = 0.0f; resamp.initialized = true; next_sample_int = ts; next_sample_frac = 0; continue; }
            while (signed_time_diff(ts, next_sample_int) >= 0) {
                double d_win = (double)signed_time_diff(next_sample_int, resamp.prev_hw_ts) + next_sample_frac - (double)resamp.prev_hw_frac;
                if (d_win < 0.0) d_win = 0.0; resamp.acc_v += (double)fv * d_win; resamp.acc_i += (double)fi * d_win; resamp.acc_weight += d_win;
                if (resamp.acc_weight > 0.0) {
                    float v_val = (float)(resamp.acc_v / resamp.acc_weight); float i_val = (float)(resamp.acc_i / resamp.acc_weight);
                    v_buf[buf_wr % spc] = v_val; i_buf[buf_wr % spc] = i_val;
                    float u = v_val - v_dc_offset;
                    // Full Harmonic Decoupling: Fundamental - 3rd - 5th - 7th
                    float v_sum_harmonics = sogi_v3.v_alpha + sogi_v5.v_alpha + sogi_v7.v_alpha + sogi_v9.v_alpha + sogi_v11.v_alpha;
                    float u1 = u - v_sum_harmonics;
                    float u3 = u - (v_sum_harmonics - sogi_v3.v_alpha + sogi_v1.v_alpha);
                    float u5 = u - (v_sum_harmonics - sogi_v5.v_alpha + sogi_v1.v_alpha);
                    float u7 = u - (v_sum_harmonics - sogi_v7.v_alpha + sogi_v1.v_alpha);
                    float u9 = u - (v_sum_harmonics - sogi_v9.v_alpha + sogi_v1.v_alpha);
                    float u11 = u - (v_sum_harmonics - sogi_v11.v_alpha + sogi_v1.v_alpha);

                    float ts_v = (float)((double)ticks_per_sample_int + ticks_per_sample_frac) * inv_cpu_freq;
                    sogi_v1.step(u1, fll.omega, ts_v);
                    sogi_v3.step(u3, 3.0f * fll.omega, ts_v);
                    sogi_v5.step(u5, 5.0f * fll.omega, ts_v);
                    sogi_v7.step(u7, 7.0f * fll.omega, ts_v);
                    sogi_v9.step(u9, 9.0f * fll.omega, ts_v);
                    sogi_v11.step(u11, 11.0f * fll.omega, ts_v);

                    float mag1 = sqrtf(sogi_v1.v_alpha*sogi_v1.v_alpha + sogi_v1.v_beta*sogi_v1.v_beta + 1e-3f);
                    float mag3 = sqrtf(sogi_v3.v_alpha*sogi_v3.v_alpha + sogi_v3.v_beta*sogi_v3.v_beta + 1e-3f);
                    float mag5 = sqrtf(sogi_v5.v_alpha*sogi_v5.v_alpha + sogi_v5.v_beta*sogi_v5.v_beta + 1e-3f);
                    float mag7 = sqrtf(sogi_v7.v_alpha*sogi_v7.v_alpha + sogi_v7.v_beta*sogi_v7.v_beta + 1e-3f);
                    float mag9 = sqrtf(sogi_v9.v_alpha*sogi_v9.v_alpha + sogi_v9.v_beta*sogi_v9.v_beta + 1e-3f);
                    float mag11 = sqrtf(sogi_v11.v_alpha*sogi_v11.v_alpha + sogi_v11.v_beta*sogi_v11.v_beta + 1e-3f);

                    harmonic_mag1_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag1_smooth + HARMONIC_SMOOTH_ALPHA * mag1;
                    harmonic_mag3_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag3_smooth + HARMONIC_SMOOTH_ALPHA * mag3;
                    harmonic_mag5_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag5_smooth + HARMONIC_SMOOTH_ALPHA * mag5;
                    harmonic_mag7_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag7_smooth + HARMONIC_SMOOTH_ALPHA * mag7;
                    harmonic_mag9_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag9_smooth + HARMONIC_SMOOTH_ALPHA * mag9;
                    harmonic_mag11_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag11_smooth + HARMONIC_SMOOTH_ALPHA * mag11;
                    float fll_err = sogi_v1.getFllError(u1);
                    float rot_err = (sogi_v1.getRotationRate() - fll.omega) / (fll.omega + 1.0f);
                    fll.update(fll_err, rot_err, ts_v);
                    buf_wr++;
                }
                resamp.acc_v = 0.0; resamp.acc_i = 0.0; resamp.acc_weight = 0.0;
                resamp.prev_hw_ts = next_sample_int; resamp.prev_hw_frac = (float)next_sample_frac;
                next_sample_frac += ticks_per_sample_frac; next_sample_int += ticks_per_sample_int + (uint32_t)next_sample_frac; next_sample_frac -= (uint32_t)next_sample_frac;
            }
            double d_rem = (double)signed_time_diff(ts, resamp.prev_hw_ts) - (double)resamp.prev_hw_frac;
            if (d_rem < 0.0) d_rem = 0.0; resamp.acc_v += (double)fv * d_rem; resamp.acc_i += (double)fi * d_rem; resamp.acc_weight += d_rem;
            resamp.prev_hw_ts = ts; resamp.prev_hw_frac = 0.0f;
        }
        resamp.last_frame_end_ts = f_end; rd = (rd + 1) % FRAME_BUFFER_SIZE; __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }
}

void visTask(void *pv) {
    int idx; for (;;) { if (xQueueReceive(visQueue, &idx, portMAX_DELAY) == pdTRUE) {
        VisSnapshot &s = vis_snapshots[idx]; vis.update(s.v_copy, s.i_copy, s.count, s.aligned_start, s.count, s.pll_freq, 0.0f, s.vdc, s.idc);
        __atomic_store_n(&s.in_use, false, __ATOMIC_RELEASE);
    } }
}

void logTask(void *pv) {
    SerialMsg msg; for (;;) { if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
//        Serial.printf("F:%.4fHz  H3:%.3f  H5:%.3f  H7:%.3f  H9:%.3f  H11:%.3f  Core:%.1fus  ISR:%lu\n",
//            msg.pll_freq, msg.h3_ratio, msg.h5_ratio, msg.h7_ratio, msg.h9_ratio, msg.h11_ratio, msg.core_us, (unsigned long)msg.isr_count);
        Serial.printf("F:%.4fHz,H3:%.3f,H5:%.3f,H7:%.3f,H9:%.3f,H11:%.3f,Core:%.1fus \n",
            msg.pll_freq, msg.h3_ratio, msg.h5_ratio, msg.h7_ratio, msg.h9_ratio, msg.h11_ratio, msg.core_us);
            
    } }
}

void setup() {
    Serial.begin(115200); delay(100); cpu_freq_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U; inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2); updateTimingParameters(NOMINAL_FREQ); vis.begin();
    adc_continuous_handle_cfg_t cfg = { .max_store_buf_size = 4096, .conv_frame_size = CONV_FRAME_SIZE }; adc_continuous_new_handle(&cfg, &adc_handle);
    adc_digi_pattern_config_t pat[2]; pat[0] = { .atten = ADC_ATTEN_DB_12, .channel = V_CHANNEL, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 };
    pat[1] = { .atten = ADC_ATTEN_DB_12, .channel = I_CHANNEL, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 };
    adc_continuous_config_t dig = { .pattern_num = 2, .adc_pattern = pat, .sample_freq_hz = ADC_OVERSAMPLE_RATE, .conv_mode = ADC_CONV_SINGLE_UNIT_1, .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1 };
    adc_continuous_config(adc_handle, &dig);
    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_callback }; adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL); adc_continuous_start(adc_handle);
    visQueue = xQueueCreate(VIS_SNAPSHOT_COUNT, sizeof(int)); logQueue = xQueueCreate(8, sizeof(SerialMsg));
    for (int i=0; i<VIS_SNAPSHOT_COUNT; i++) vis_snapshots[i].in_use = false;
    xTaskCreatePinnedToCore(visTask, "vis", 8192, NULL, 2, &visTaskHandle, 0); xTaskCreatePinnedToCore(logTask, "log", 4096, NULL, 1, &logTaskHandle, 0);
}

void loop() {
    static uint32_t last_boundary = 0; processIncomingDMA();
    uint32_t spc = __atomic_load_n(&samples_per_cycle, __ATOMIC_ACQUIRE); if (buf_wr - last_boundary < spc) return;
    uint32_t t_start = get_cycle_count(); float v_sum = 0, i_sum = 0; for (uint32_t i=0; i<spc; i++) { v_sum += v_buf[i]; i_sum += i_buf[i]; }
    float v_avg = v_sum / spc; float i_avg = i_sum / spc;
    if (!dc_bootstrap_done) { v_dc_offset = v_avg; i_dc_offset = i_avg; dc_bootstrap_done = true; }
    else { v_dc_offset = 0.95f * v_dc_offset + 0.05f * v_avg; i_dc_offset = 0.95f * i_dc_offset + 0.05f * i_avg; }
    float phase = atan2f(sogi_v1.v_alpha, -sogi_v1.v_beta); if (phase < 0) phase += 2.0f * M_PI;
    if (phase_track.initialized) { float d = phase - phase_track.prev_phase; if (d < -M_PI) phase_track.winding++; else if (d > M_PI) phase_track.winding--; } else phase_track.initialized = true;
    phase_track.prev_phase = phase; float unwrapped = phase + phase_track.winding * 2.0f * M_PI; if (fabsf(unwrapped) > 1000.0f) phase_track.winding = 0;
    float phase_norm = fmodf(unwrapped, 2.0f * M_PI); if (phase_norm < 0) phase_norm += 2.0f * M_PI;
    int start = (int)((buf_wr % spc + spc - (int)( (phase_norm/(2.0f*M_PI)) * spc + 0.5f)) % spc);
    int f_idx = -1; for (int i=0; i<VIS_SNAPSHOT_COUNT; i++) if (!__atomic_load_n(&vis_snapshots[i].in_use, __ATOMIC_ACQUIRE)) { f_idx = i; break; }
    if (f_idx != -1) {
        VisSnapshot &s = vis_snapshots[f_idx]; __atomic_store_n(&s.in_use, true, __ATOMIC_RELEASE);
        memcpy(s.v_copy, v_buf, sizeof(float) * spc); memcpy(s.i_copy, i_buf, sizeof(float) * spc);
        s.aligned_start = start; s.count = spc; s.pll_freq = fll.freq; s.vdc = v_dc_offset; s.idc = i_dc_offset;
        xQueueSend(visQueue, &f_idx, 0);
    }
    SerialMsg sm; sm.pll_freq = fll.freq;
    sm.h3_ratio = harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f);
    sm.h5_ratio = harmonic_mag5_smooth / (harmonic_mag1_smooth + 1e-9f);
    sm.h7_ratio = harmonic_mag7_smooth / (harmonic_mag1_smooth + 1e-9f);
    sm.h9_ratio = harmonic_mag9_smooth / (harmonic_mag1_smooth + 1e-9f);
    sm.h11_ratio = harmonic_mag11_smooth / (harmonic_mag1_smooth + 1e-9f);
    sm.isr_count = __atomic_load_n(&isr_callback_count, __ATOMIC_RELAXED); sm.dropped = __atomic_load_n(&frames_dropped, __ATOMIC_RELAXED);
    sm.core_us = (float)(get_cycle_count() - t_start) * inv_cpu_freq * 1e6f;
    xQueueSend(logQueue, &sm, 0);
    uint32_t next_spc = (uint32_t)lrintf(TARGET_VIRTUAL_RATE / fll.freq); if (next_spc < MIN_SAMPLES_PER_CYCLE) next_spc = MIN_SAMPLES_PER_CYCLE; if (next_spc > MAX_SAMPLES_PER_CYCLE) next_spc = MAX_SAMPLES_PER_CYCLE;
    __atomic_store_n(&samples_per_cycle, next_spc, __ATOMIC_RELEASE); updateTimingParameters(fll.freq);
    last_boundary = buf_wr; yield();
}
