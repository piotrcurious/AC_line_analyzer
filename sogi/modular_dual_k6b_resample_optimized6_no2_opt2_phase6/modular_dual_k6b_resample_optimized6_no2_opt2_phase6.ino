/* ESP32 SOGI-PLL — Phase 6: Unified Transform Framework (UTF)
 *
 * Composition: T_resamp -> T_dc -> P_dec(P_sogi) -> Φ_fll
 *
 * Features:
 *  - Adaptive virtual grid: N in [100, 256] to maintain ~6400Hz rate.
 *  - High-precision recursive states (double).
 *  - Kahan summation for frequency integration.
 *  - Cycle-averaging for ripple rejection.
 *  - Reframed perspective: All blocks treated as mathematical operators.
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

// ── Algorithm Constants ─────────────────────────────────────────────────────
#define NOMINAL_FREQ       50.0f
#define SOGI_K             0.7071f
#define FLL_GAMMA          2500.0f

// ── T_resamp: Adaptive Sampling Parameters ──────────────────────────────────
#define MIN_N 100
#define MAX_N 256
#define TARGET_VIRTUAL_RATE 6400.0f

uint32_t samples_per_cycle = 128; // Current N

// ── ADC / DMA ───────────────────────────────────────────────────────────────
#define ADC_OVERSAMPLE_RATE   250000
#define CONV_FRAME_SIZE       16

// ── Frame ring ───────────────────────────────────────────────────────────────
#define FRAME_BUFFER_SIZE     1024

// ── Diagnostics ──────────────────────────────────────────────────────────────
#define SERIAL_EVERY_N_CYCLES   2

// ── UTF Global Operator ─────────────────────────────────────────────────────
UnifiedSOGIAnalyzer analyzer(NOMINAL_FREQ, SOGI_K, FLL_GAMMA);
SOGIVisualizer      vis;

// ── DMA frame ring ──────────────────────────────────────────────────────────
struct TimestampedFrame {
    uint32_t end_timestamp;
    uint8_t  raw_data[CONV_FRAME_SIZE];
    uint16_t data_size;
};

volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint32_t frame_write_idx    = 0;
volatile uint32_t frame_read_idx     = 0;
volatile uint32_t isr_callback_count = 0; // Atomic
volatile uint32_t frames_dropped     = 0; // Atomic

adc_continuous_handle_t adc_handle = NULL;

// ── CPU timing ───────────────────────────────────────────────────────────────
uint32_t cpu_freq_hz           = 0;
double   inv_cpu_freq          = 0.0;
uint32_t cycles_per_adc_sample = 0;
double   ticks_per_sample_d    = 0.0; // T_s in CCOUNT ticks

// ── Precomputed constant ─────────────────────────────────────────────────────
static const float RAW_TO_MV = 3300.0f / 4095.0f;

// ── T_resamp: Decimating Resampler State ────────────────────────────────────
float    v_buf[MAX_N];
float    i_buf[MAX_N];
uint32_t buf_wr = 0;
double   next_sample_offset_d = 0.0; // Relative timeline (double)

struct ResamplerState {
    uint32_t last_frame_end_ts = 0;
    double   acc_v = 0.0;
    double   acc_i = 0.0;
    bool     initialized = false;
} resamp;

// ── Path B state – cycle boundary ─────────────────────────────────────────────
uint32_t last_cycle_boundary_samples = 0;

// ── Phase unwrap / Visualization ────────────────────────────────────────────
struct PhaseTrack {
    float prev_phase   = 0.0f;
    float phase_offset = 0.0f;
    bool  initialized  = false;
} phase_track;

uint32_t interp_ok_count   = 0;
uint32_t interp_fail_count = 0;

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
    float  vdc;
};

struct SerialMsg {
    float pll_freq;
    float core_us;
    uint32_t isr_callback_count;
    uint32_t frames_dropped;
    uint32_t interp_ok;
    uint32_t interp_total;
    float vdc;
    uint32_t spc;
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
    // T_s = 1 / (f * N)
    ticks_per_sample_d = (double)cpu_freq_hz / ((double)frequency * (double)samples_per_cycle);
}

static inline int32_t IRAM_ATTR signed_time_diff(uint32_t a, uint32_t b) {
    return (int32_t)(a - b);
}

// ─────────────────────────────────────────────────────────────────────────────
//  ISR: Hardware Input Capture (T_adc)
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
        __atomic_add_fetch(&frames_dropped, 1, __ATOMIC_RELAXED);
    }

    uint32_t sz = edata->size;
    if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;
    memcpy((void *)frame_buffer[wr].raw_data, edata->conv_frame_buffer, sz);

    frame_buffer[wr].end_timestamp = ts;
    frame_buffer[wr].data_size     = (uint16_t)sz;

    __atomic_store_n(&frame_write_idx, next_wr, __ATOMIC_RELEASE);
    __atomic_add_fetch(&isr_callback_count, 1, __ATOMIC_RELAXED);
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ADC Init
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
            // Fix: pass physical buffer size MAX_N to visualizer
            vis.update(snap.v_copy, snap.i_copy, snap.spc,
                       snap.aligned_start, MAX_N,
                       snap.pll_freq, 0.0f,
                       snap.vdc, 1650.0f);
        }
    }
}

void logTask(void *pv) {
    SerialMsg msg;
    for (;;) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            Serial.printf(
                "F:%.4fHz  N:%lu  Vdc:%.1f  CPU:%.1fus  ISR:%lu  Drop:%lu  Int:%lu/%lu\n",
                msg.pll_freq, (unsigned long)msg.spc, msg.vdc, msg.core_us,
                (unsigned long)msg.isr_callback_count, (unsigned long)msg.frames_dropped,
                (unsigned long)msg.interp_ok, (unsigned long)msg.interp_total
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
            next_sample_offset_d = ticks_per_sample_d; // Start at first virtual boundary
            resamp.initialized = true;
            rd = (rd + 1) % FRAME_BUFFER_SIZE;
            continue;
        }

        double frame_duration = (double)elapsed;
        double current_pair_offset = 0.0; // Offset from last_frame_end_ts

        for (int pair = 0; pair < total_pairs; ++pair) {
            int base = pair * 2;
            uint16_t rv = p[base].type1.data, ri = p[base].type1.data;
            // Correcting channel mapping (Simplified for brevity in IRQ,
            // in practice we check p[base].type1.channel)
            if (p[base].type1.channel == V_CHANNEL) rv = p[base].type1.data;
            else ri = p[base].type1.data;
            if (base+1 < n_samples) {
                if (p[base+1].type1.channel == V_CHANNEL) rv = p[base+1].type1.data;
                else ri = p[base+1].type1.data;
            }

            float fv = adcRawToMillivolts(rv);
            float fi = adcRawToMillivolts(ri);

            double next_pair_offset = ((double)(pair + 1) * frame_duration) / (double)total_pairs;

            // Box-filter integration with sub-sample windowing (Using relative offsets)
            while (next_sample_offset_d <= next_pair_offset) {
                double win_start = (current_pair_offset > (next_sample_offset_d - ticks_per_sample_d)) ?
                                    current_pair_offset : (next_sample_offset_d - ticks_per_sample_d);
                double d_win = next_sample_offset_d - win_start;
                if (d_win > 0.0) {
                    resamp.acc_v += (double)fv * d_win;
                    resamp.acc_i += (double)fi * d_win;
                }

                // Virtual sample complete
                float v_val = (float)(resamp.acc_v / ticks_per_sample_d);
                float i_val = (float)(resamp.acc_i / ticks_per_sample_d);
                float ts_v = (float)(ticks_per_sample_d * inv_cpu_freq);

                analyzer.process(v_val, ts_v, samples_per_cycle);
                v_buf[buf_wr % MAX_N] = v_val;
                i_buf[buf_wr % MAX_N] = i_val;
                buf_wr++;
                interp_ok_count++;

                resamp.acc_v = 0.0; resamp.acc_i = 0.0;
                next_sample_offset_d += ticks_per_sample_d;
            }

            // Remainder of current pair for next virtual sample
            double d_rem = next_pair_offset - ( (current_pair_offset > (next_sample_offset_d - ticks_per_sample_d)) ?
                                                 current_pair_offset : (next_sample_offset_d - ticks_per_sample_d) );
            if (d_rem > 0.0) {
                resamp.acc_v += (double)fv * d_rem;
                resamp.acc_i += (double)fi * d_rem;
            }
            current_pair_offset = next_pair_offset;
        }

        // Advance to next frame and normalize offset to prevent accumulation
        resamp.last_frame_end_ts = frame_end_ts;
        next_sample_offset_d -= frame_duration;

        rd = (rd + 1) % FRAME_BUFFER_SIZE;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(100);
    cpu_freq_hz  = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0 / (double)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);

    updateTimingParameters(NOMINAL_FREQ);

    for (int i = 0; i < MAX_N; ++i) { v_buf[i] = 1650.0f; i_buf[i] = 1650.0f; }

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
//  Loop: Path B Closure & Operator Supervision
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR loop() {
    static uint32_t serial_ctr  = 0;

    processIncomingDMA();

    // Trigger on cycle boundary
    uint32_t current_N = samples_per_cycle;
    if (buf_wr - last_cycle_boundary_samples < current_N) return;

    uint32_t proc_start = get_cycle_count();

    // 1. Path B Synchronization
    // Align visualization window to fundamental phase
    float phase = atan2f(analyzer.getVAlpha(), -analyzer.getVBeta());
    if (phase < 0.0f) phase += 2.0f * (float)M_PI;

    if (phase_track.initialized) {
        float delta = phase - phase_track.prev_phase;
        if      (delta < -(float)M_PI) phase_track.phase_offset += 2.0f * (float)M_PI;
        else if (delta >  (float)M_PI) phase_track.phase_offset -= 2.0f * (float)M_PI;
    } else { phase_track.initialized = true; }
    phase_track.prev_phase = phase;

    float unwrapped = phase + phase_track.phase_offset;
    float phase_norm = fmodf(unwrapped, 2.0f * (float)M_PI);
    if (phase_norm < 0.0f) phase_norm += 2.0f * (float)M_PI;

    float samples_back = (phase_norm / (2.0f * (float)M_PI)) * (float)current_N;
    int curr_head_idx = (int)(buf_wr % MAX_N);
    int aligned_start = (int)((curr_head_idx + MAX_N - (int)(samples_back + 0.5f)) % MAX_N);

    // 2. Adaptive N Selection (T_resamp refinement)
    // Goal: Maintain virtual rate ~6400Hz
    uint32_t next_N = (uint32_t)lrintf(TARGET_VIRTUAL_RATE / analyzer.getFreq());
    if (next_N < MIN_N) next_N = MIN_N;
    if (next_N > MAX_N) next_N = MAX_N;
    samples_per_cycle = next_N;

    last_cycle_boundary_samples = buf_wr;
    updateTimingParameters(analyzer.getFreq());

    uint32_t proc_end = get_cycle_count();
    float core_us = (float)(proc_end - proc_start) * (float)inv_cpu_freq * 1e6f;

    // 3. Dispatch Visualizer Snapshot (Copied into queue to avoid fragmentation and races)
    VisSnapshot snap;
    memcpy(snap.v_copy, v_buf, sizeof(v_buf));
    memcpy(snap.i_copy, i_buf, sizeof(i_buf));
    snap.aligned_start = aligned_start;
    snap.spc = current_N;
    snap.pll_freq = analyzer.getFreq();
    snap.vdc = analyzer.getDC();
    xQueueSend(visQueue, &snap, 0);

    if (++serial_ctr >= SERIAL_EVERY_N_CYCLES) {
        serial_ctr = 0;
        SerialMsg sm;
        sm.pll_freq = analyzer.getFreq();
        sm.core_us  = core_us;
        sm.isr_callback_count = isr_callback_count;
        sm.frames_dropped     = frames_dropped;
        sm.interp_ok          = interp_ok_count;
        sm.interp_total       = interp_ok_count + interp_fail_count;
        sm.vdc = analyzer.getDC();
        sm.spc = current_N;
        interp_ok_count = 0; interp_fail_count = 0;
        xQueueSend(logQueue, &sm, 0);
    }

    yield();
}
