/* ESP32 SOGI-PLL — Phase 7: Unified Transform Framework (UTF)
 *
 * Features:
 *  - Adaptive virtual grid: N in [100, 256] to maintain ~6400Hz rate.
 *  - High-precision recursive states (double).
 *  - Kahan summation for frequency integration.
 *  - Unified Transform Framework (UTF) framing.
 */

#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "SOGI.h"
#include "SOGIvisualizer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"

// ── Pin / channel ────────────────────────────────────────────────────────────
#define ADC_PIN_V   36   // ADC1_CH0
#define ADC_PIN_I   39   // ADC1_CH3
#define V_CHANNEL   0
#define I_CHANNEL   3

// ── Algorithm ───────────────────────────────────────────────────────────────
#define NOMINAL_FREQ       50.0f
#define SOGI_K             0.7071f
#define PLL_KP             2.55f
#define PLL_KI             10.0f          // Enabled integrator for Phase 7
#define TARGET_VIRTUAL_RATE 6400.0f

// ── Adaptive Sampling ───────────────────────────────────────────────────────
#define MIN_N 100
#define MAX_N 256
uint32_t samples_per_cycle = 128; // Current N

// ── Harmonic detection knobs ─────────────────────────────────────────────────
#define SOGI3_K                         SOGI_K
#define SOGI3_HARMONIC_THRESHOLD        0.1f
#define SOGI3_DAMP_MIN                  0.01f
#define HARMONIC_SMOOTH_ALPHA           0.05f   // BUG FIX: changed from 0.99 to 0.05

// ── ADC / DMA ───────────────────────────────────────────────────────────────
#define ADC_OVERSAMPLE_RATE   250000
#define CONV_FRAME_SIZE       16

// ── Frame ring ───────────────────────────────────────────────────────────────
#define FRAME_BUFFER_SIZE     1024

// ── Diagnostics throttle ─────────────────────────────────────────────────────
#define SERIAL_EVERY_N_CYCLES   2

// ── Global objects ──────────────────────────────────────────────────────────
SOGIVisualizer vis;
SOGI           sogi_v(SOGI_K);
SOGI           sogi_v3(SOGI3_K);
AdaptivePLL    pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

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

// ── CPU timing ───────────────────────────────────────────────────────────────
uint32_t cpu_freq_hz           = 0;
double   inv_cpu_freq          = 0.0;
uint32_t cycles_per_adc_sample = 0;
double   ticks_per_sample_d    = 0.0; // ticks per virtual sample

// ── Precomputed constant ─────────────────────────────────────────────────────
static const float RAW_TO_MV = 3300.0f / 4095.0f;

// ── PATH A state – Decimating Resampler ─────────────────────────────────────
float    v_buf[MAX_N];
float    i_buf[MAX_N];
uint32_t buf_wr = 0;
double   next_sample_offset_d = 0.0;

struct ResamplerState {
    uint32_t last_frame_end_ts = 0;
    double    acc_v = 0.0;
    double    acc_i = 0.0;
    bool     initialized = false;
} resamp;

// ── PATH B state – cycle boundary ─────────────────────────────────────────────
uint32_t last_cycle_boundary_samples = 0;

// ── DC offsets ───────────────────────────────────────────────────────────────
double v_dc_offset = 1650.0;
double i_dc_offset = 1650.0;

// ── Phase tracking ───────────────────────────────────────────────────────────
struct PhaseTrack {
    float prev_phase   = 0.0f;
    bool  initialized  = false;
} phase_track;

// ── Diagnostics ──────────────────────────────────────────────────────────────
uint32_t interp_ok_count   = 0;
uint32_t interp_fail_count = 0;

// ── Harmonic detection smoothing state ────────────────────────────────────────
double harmonic_mag1_smooth = 1e-6;
double harmonic_mag3_smooth = 1e-6;

// ── FreeRTOS objects ────────────────────────────────────────────────────────
TaskHandle_t visTaskHandle = NULL;
TaskHandle_t logTaskHandle = NULL;
QueueHandle_t visQueue = NULL;
QueueHandle_t logQueue = NULL;

struct VisSnapshot {
    float v_copy[MAX_N];
    float i_copy[MAX_N];
    uint32_t spc;
    int    aligned_start;
    float  pll_freq;
    float  pll_mag;
    float  vdc, idc;
};

struct SerialMsg {
    float pll_freq;
    float core_us;
    uint32_t isr_callback_count;
    uint32_t frames_dropped;
    uint32_t interp_ok;
    uint32_t interp_total;
    float vdc, idc;
    uint32_t spc;
    float harmonic_ratio;
};

static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a"(ccount));
    return ccount;
}

static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw) {
    return (float)raw * RAW_TO_MV;
}

static inline void IRAM_ATTR updateTimingParameters(float frequency) {
    ticks_per_sample_d = (double)cpu_freq_hz / ((double)frequency * (double)samples_per_cycle);
}

static inline int32_t IRAM_ATTR signed_time_diff(uint32_t a, uint32_t b) {
    return (int32_t)(a - b);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ISR
// ─────────────────────────────────────────────────────────────────────────────
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data) {
    uint32_t ts = get_cycle_count();
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_RELAXED);
    uint32_t next_wr = (wr + 1) % FRAME_BUFFER_SIZE;

    if (next_wr == rd) {
        rd = (rd + 1) % FRAME_BUFFER_SIZE;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
        ++frames_dropped;
    }

    uint32_t sz = edata->size;
    if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;
    memcpy((void *)frame_buffer[wr].raw_data, edata->conv_frame_buffer, sz);

    frame_buffer[wr].end_timestamp = ts;
    frame_buffer[wr].data_size     = (uint16_t)sz;

    __atomic_store_n(&frame_write_idx, next_wr, __ATOMIC_RELEASE);
    ++isr_callback_count;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ADC init
// ─────────────────────────────────────────────────────────────────────────────
bool initADCContinuous() {
    adc_continuous_handle_cfg_t cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size    = CONV_FRAME_SIZE,
    };
    if (adc_continuous_new_handle(&cfg, &adc_handle) != ESP_OK) return false;

    adc_digi_pattern_config_t pat[2];
    pat[0] = { .atten = ADC_ATTEN_DB_12, .channel = V_CHANNEL, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 };
    pat[1] = { .atten = ADC_ATTEN_DB_12, .channel = I_CHANNEL, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 };

    adc_continuous_config_t dig = {
        .pattern_num    = 2,
        .adc_pattern    = pat,
        .sample_freq_hz = ADC_OVERSAMPLE_RATE,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    if (adc_continuous_config(adc_handle, &dig) != ESP_OK) return false;

    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_callback };
    adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
    adc_continuous_start(adc_handle);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tasks
// ─────────────────────────────────────────────────────────────────────────────

void visTask(void *pv) {
    VisSnapshot snap;
    for (;;) {
        if (xQueueReceive(visQueue, &snap, portMAX_DELAY) == pdTRUE) {
            vis.update(snap.v_copy, snap.i_copy, snap.spc,
                       snap.aligned_start, MAX_N,
                       snap.pll_freq, snap.pll_mag,
                       snap.vdc, snap.idc);
        }
    }
}

void logTask(void *pv) {
    SerialMsg msg;
    for (;;) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            Serial.printf(
                "F:%.4fHz  N:%lu  Core:%.1fus  ISR:%lu  Drop:%lu  Interp:%lu/%lu  Vdc:%.1f Idc:%.1f  H3ratio:%.3f\n",
                msg.pll_freq,
                (unsigned long)msg.spc,
                msg.core_us,
                (unsigned long)msg.isr_callback_count,
                (unsigned long)msg.frames_dropped,
                (unsigned long)msg.interp_ok,
                (unsigned long)msg.interp_total,
                msg.vdc, msg.idc,
                (double)msg.harmonic_ratio
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  T_resamp: Sub-Sample Accurate Resampler
// ─────────────────────────────────────────────────────────────────────────────

void IRAM_ATTR processIncomingDMA() {
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);

    while (rd != wr) {
        volatile TimestampedFrame *f_ptr = &frame_buffer[rd];
        uint32_t frame_end_ts = f_ptr->end_timestamp;
        uint16_t sz = f_ptr->data_size;

        const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f_ptr->raw_data;
        int n_samples = sz / sizeof(adc_digi_output_data_t);
        int total_pairs = n_samples / 2;
        if (total_pairs <= 0) { rd = (rd + 1) % FRAME_BUFFER_SIZE; continue; }

        uint32_t elapsed = (resamp.initialized) ? (uint32_t)(frame_end_ts - resamp.last_frame_end_ts) : (uint32_t)cycles_per_adc_sample * total_pairs;

        if (!resamp.initialized) {
            resamp.last_frame_end_ts = frame_end_ts;
            next_sample_offset_d = ticks_per_sample_d;
            resamp.initialized = true;
            rd = (rd + 1) % FRAME_BUFFER_SIZE;
            continue;
        }

        double frame_duration = (double)elapsed;
        double current_pair_offset = 0.0;

        for (int pair = 0; pair < total_pairs; ++pair) {
            int base = pair * 2;
            uint16_t rv = 0, ri = 0;
            if (p[base].type1.channel == V_CHANNEL) rv = p[base].type1.data;
            else ri = p[base].type1.data;
            if (base+1 < n_samples) {
                if (p[base+1].type1.channel == V_CHANNEL) rv = p[base+1].type1.data;
                else ri = p[base+1].type1.data;
            }

            float fv = adcRawToMillivolts(rv);
            float fi = adcRawToMillivolts(ri);

            double next_pair_offset = ((double)(pair + 1) * frame_duration) / (double)total_pairs;

            while (next_sample_offset_d <= next_pair_offset) {
                double win_start = (current_pair_offset > (next_sample_offset_d - ticks_per_sample_d)) ?
                                    current_pair_offset : (next_sample_offset_d - ticks_per_sample_d);
                double d_win = next_sample_offset_d - win_start;
                if (d_win > 0.0) {
                    resamp.acc_v += (double)fv * d_win;
                    resamp.acc_i += (double)fi * d_win;
                } else if (resamp.acc_v == 0.0 && resamp.acc_i == 0.0) {
                    // Zero-order hold path (window empty). Fill with neutral DC.
                    resamp.acc_v = v_dc_offset * ticks_per_sample_d;
                    resamp.acc_i = i_dc_offset * ticks_per_sample_d;
                }

                float v_val = (float)(resamp.acc_v / ticks_per_sample_d);
                float i_val = (float)(resamp.acc_i / ticks_per_sample_d);
                float ts_v = (float)(ticks_per_sample_d * inv_cpu_freq);

                sogi_v.step(v_val - (float)v_dc_offset, pll.omega, ts_v);
                sogi_v3.step(v_val - (float)v_dc_offset, 3.0f * pll.omega, ts_v);

                double mag1 = sqrt((double)sogi_v.v_alpha * sogi_v.v_alpha + (double)sogi_v.v_beta * sogi_v.v_beta);
                double mag3 = sqrt((double)sogi_v3.v_alpha * sogi_v3.v_alpha + (double)sogi_v3.v_beta * sogi_v3.v_beta);
                harmonic_mag1_smooth = (1.0 - (double)HARMONIC_SMOOTH_ALPHA) * harmonic_mag1_smooth + (double)HARMONIC_SMOOTH_ALPHA * mag1;
                harmonic_mag3_smooth = (1.0 - (double)HARMONIC_SMOOTH_ALPHA) * harmonic_mag3_smooth + (double)HARMONIC_SMOOTH_ALPHA * mag3;

                float ratio = (float)(harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9));
                float damp_factor = 1.0f;
                float learn_att   = 1.0f;

                if (ratio > SOGI3_HARMONIC_THRESHOLD) {
                    float overshoot = (ratio - SOGI3_HARMONIC_THRESHOLD) / (3.0f * SOGI3_HARMONIC_THRESHOLD);
                    if (overshoot > 1.0f) overshoot = 1.0f;
                    damp_factor = 1.0f - overshoot * (1.0f - SOGI3_DAMP_MIN);
                    learn_att   = 1.0f - overshoot; // Linear ramp for learn_att
                    if (learn_att < 0.0f) learn_att = 0.0f;
                }
                pll.setDistortionDamping(damp_factor, learn_att);

                pll.update(sogi_v.v_alpha * damp_factor, sogi_v.v_beta * damp_factor, ts_v);

                v_buf[buf_wr % MAX_N] = v_val;
                i_buf[buf_wr % MAX_N] = i_val;
                buf_wr++;
                interp_ok_count++;

                resamp.acc_v = 0.0; resamp.acc_i = 0.0;
                next_sample_offset_d += ticks_per_sample_d;
            }

            double d_rem = next_pair_offset - ( (current_pair_offset > (next_sample_offset_d - ticks_per_sample_d)) ?
                                                 current_pair_offset : (next_sample_offset_d - ticks_per_sample_d) );
            if (d_rem > 0.0) {
                resamp.acc_v += (double)fv * d_rem;
                resamp.acc_i += (double)fi * d_rem;
            }
            current_pair_offset = next_pair_offset;
        }

        resamp.last_frame_end_ts = frame_end_ts;
        next_sample_offset_d -= frame_duration;

        rd = (rd + 1) % FRAME_BUFFER_SIZE;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(100);
    cpu_freq_hz  = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0 / (double)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);

    updateTimingParameters(NOMINAL_FREQ);

    for (int i = 0; i < MAX_N; ++i) {
        v_buf[i] = (float)v_dc_offset;
        i_buf[i] = (float)i_dc_offset;
    }

    vis.begin();
    initADCContinuous();

    visQueue = xQueueCreate(4, sizeof(VisSnapshot));
    logQueue = xQueueCreate(8, sizeof(SerialMsg));
    xTaskCreatePinnedToCore(visTask, "visTask", 8192, NULL, tskIDLE_PRIORITY + 2, &visTaskHandle, 0);
    xTaskCreatePinnedToCore(logTask, "logTask", 4096, NULL, tskIDLE_PRIORITY + 1, &logTaskHandle, 1);

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR loop() {
    static uint32_t serial_ctr  = 0;

    processIncomingDMA();

    uint32_t current_N = samples_per_cycle;
    if (buf_wr - last_cycle_boundary_samples < current_N) return;

    uint32_t proc_start = get_cycle_count();

    // DC estimation using double precision
    double v_sum = 0.0, i_sum = 0.0;
    for (int k = 0; k < current_N; ++k) {
        v_sum += (double)v_buf[(buf_wr - 1 - k) % MAX_N];
        i_sum += (double)i_buf[(buf_wr - 1 - k) % MAX_N];
    }
    double inv_n = 1.0 / (double)current_N;
    v_dc_offset = 0.9995 * v_dc_offset + 0.0005 * (v_sum * inv_n);
    i_dc_offset = 0.9995 * i_dc_offset + 0.0005 * (i_sum * inv_n);

    // Phase projection for visualization. Using wrapped phase directly.
    float phase = atan2f(sogi_v.v_alpha, -sogi_v.v_beta);
    if (phase < 0.0f) phase += 2.0f * (float)M_PI;

    float samples_back = (phase / (2.0f * (float)M_PI)) * (float)current_N;
    int curr_head_idx = (int)(buf_wr % MAX_N);
    int aligned_start = (int)((curr_head_idx + MAX_N - (int)(samples_back + 0.5f)) % MAX_N);

    // Adaptive N Selection with Hysteresis
    uint32_t target_N = (uint32_t)lrintf(TARGET_VIRTUAL_RATE / pll.freq);
    if (target_N < MIN_N) target_N = MIN_N;
    if (target_N > MAX_N) target_N = MAX_N;

    // 2-sample hysteresis margin to prevent rapid toggling
    if (abs((int)target_N - (int)samples_per_cycle) > 2) {
        samples_per_cycle = target_N;
    }

    last_cycle_boundary_samples = buf_wr;
    updateTimingParameters(pll.freq);

    uint32_t proc_end = get_cycle_count();
    float core_us = (float)(proc_end - proc_start) * (float)inv_cpu_freq * 1e6f;

    VisSnapshot snap;
    memcpy(snap.v_copy, v_buf, sizeof(v_buf));
    memcpy(snap.i_copy, i_buf, sizeof(i_buf));
    snap.aligned_start = aligned_start;
    snap.spc = current_N;
    snap.pll_freq = pll.freq;
    snap.pll_mag  = 0.0f; // Placeholder
    snap.vdc = (float)v_dc_offset;
    snap.idc = (float)i_dc_offset;
    xQueueSend(visQueue, &snap, 0);

    if (++serial_ctr >= SERIAL_EVERY_N_CYCLES) {
        serial_ctr = 0;
        SerialMsg sm;
        sm.pll_freq = pll.freq;
        sm.core_us  = core_us;
        sm.isr_callback_count = isr_callback_count;
        sm.frames_dropped     = frames_dropped;
        sm.interp_ok          = interp_ok_count;
        sm.interp_total       = interp_ok_count + interp_fail_count;
        sm.vdc = (float)v_dc_offset;
        sm.idc = (float)i_dc_offset;
        sm.spc = current_N;
        sm.harmonic_ratio = (float)(harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9));
        interp_ok_count = 0; interp_fail_count = 0;
        xQueueSend(logQueue, &sm, 0);
    }

    yield();
}
