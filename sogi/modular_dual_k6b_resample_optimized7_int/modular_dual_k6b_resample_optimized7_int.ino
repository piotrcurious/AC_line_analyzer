/* ESP32 SOGI-PLL – Optimized streaming resampler with integer math
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
#define PLL_KI             0.000000f
#define SAMPLES_PER_CYCLE  128        // virtual samples per AC cycle

// ── 3rd-harmonic detection knobs ─────────────────────────────────────────────
#define SOGI3_K                         SOGI_K
#define SOGI3_HARMONIC_THRESHOLD        0.1f
#define SOGI3_DAMP_MIN                  0.01f
#define HARMONIC_SMOOTH_ALPHA           0.99f

// ── ADC / DMA ───────────────────────────────────────────────────────────────
#define ADC_OVERSAMPLE_RATE   250000  // total samples/s across both channels
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
float    inv_cpu_freq          = 0.0f;
uint32_t cycles_per_adc_sample = 0;
uint32_t single_cycle_cycles   = 0;
uint32_t ticks_per_sample      = 0;

// ── Precomputed constant ─────────────────────────────────────────────────────
// RAW_TO_Q16 = (3300.0 / 4095.0) * 65536
static const q16_t RAW_TO_Q16 = (q16_t)( (3300.0f / 4095.0f) * 65536.0f );

// ── PATH A state – Decimating Resampler ─────────────────────────────────────
q16_t    v_buf[SAMPLES_PER_CYCLE];
q16_t    i_buf[SAMPLES_PER_CYCLE];
uint32_t buf_wr = 0;
uint32_t next_sample_time = 0;

struct ResamplerState {
    uint32_t last_frame_end_ts = 0;
    int64_t  acc_v = 0;
    int64_t  acc_i = 0;
    int      acc_count = 0;
    bool     initialized = false;
} resamp;

// ── PATH B state – cycle boundary ─────────────────────────────────────────────
uint32_t last_cycle_boundary_samples = 0;

// ── DC offsets (Q16.16) ──────────────────────────────────────────────────────
q16_t v_dc_offset = FLOAT_TO_Q16(1650.0f);
q16_t i_dc_offset = FLOAT_TO_Q16(1650.0f);

// ── Phase unwrap ─────────────────────────────────────────────────────────────
struct PhaseTrack {
    float prev_phase   = 0.0f;
    float phase_offset = 0.0f;
    bool  initialized  = false;
} phase_track;

// ── Diagnostics ──────────────────────────────────────────────────────────────
uint32_t interp_ok_count   = 0;
uint32_t interp_fail_count = 0;

// ── Harmonic detection smoothing state ────────────────────────────────────────
float harmonic_mag1_smooth = 1e-6f;
float harmonic_mag3_smooth = 1e-6f;

// ── FreeRTOS objects ────────────────────────────────────────────────────────
TaskHandle_t visTaskHandle = NULL;
TaskHandle_t logTaskHandle = NULL;
QueueHandle_t visQueue = NULL;
QueueHandle_t logQueue = NULL;

struct VisSnapshot {
    float *v_copy;
    float *i_copy;
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
};

static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a"(ccount));
    return ccount;
}

static inline q16_t IRAM_ATTR adcRawToQ16(uint16_t raw) {
    // raw is 12-bit (0-4095).
    // result is millivolts in Q16.16
    return (q16_t)(((int64_t)raw * RAW_TO_Q16) >> 0); // No additional shift needed as RAW_TO_Q16 already incorporates Q16
}

static inline void IRAM_ATTR updateTimingParameters(float frequency) {
    float fc            = (frequency < 40.0f) ? 40.0f : (frequency > 90.0f ? 90.0f : frequency);
    single_cycle_cycles = (uint32_t)lrintf((float)cpu_freq_hz / fc);
    uint32_t ticks = (single_cycle_cycles / SAMPLES_PER_CYCLE);
    if (ticks == 0) ticks = 1;
    ticks_per_sample = ticks;
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

void visTask(void *pv) {
    VisSnapshot snap;
    for (;;) {
        if (xQueueReceive(visQueue, &snap, portMAX_DELAY) == pdTRUE) {
            vis.update(snap.v_copy, snap.i_copy, SAMPLES_PER_CYCLE,
                       snap.aligned_start, SAMPLES_PER_CYCLE,
                       snap.pll_freq, snap.pll_mag,
                       snap.vdc, snap.idc);
            heap_caps_free(snap.v_copy);
            heap_caps_free(snap.i_copy);
        }
    }
}

void logTask(void *pv) {
    SerialMsg msg;
    for (;;) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            Serial.printf(
                "F:%.4fHz  Core:%.1fus  ISR:%lu  Drop:%lu  Interp:%lu/%lu  Vdc:%.1f Idc:%.1f  H3ratio:%.3f\n",
                msg.pll_freq, msg.core_us,
                (unsigned long)msg.isr_callback_count,
                (unsigned long)msg.frames_dropped,
                (unsigned long)msg.interp_ok,
                (unsigned long)msg.interp_total,
                msg.vdc, msg.idc,
                (double)(harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f))
            );
        }
    }
}

void IRAM_ATTR processIncomingDMA() {
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);
    if (rd == wr) return;

    while (rd != wr) {
        volatile TimestampedFrame *f_ptr = &frame_buffer[rd];
        uint32_t frame_end_ts = f_ptr->end_timestamp;
        uint16_t sz = f_ptr->data_size;

        const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f_ptr->raw_data;
        int n = sz / sizeof(adc_digi_output_data_t);
        if (n <= 0) { rd = (rd + 1) % FRAME_BUFFER_SIZE; continue; }

        int total_pairs = n / 2;
        uint32_t elapsed = 0;
        if (resamp.initialized) {
            elapsed = (uint32_t)(frame_end_ts - resamp.last_frame_end_ts);
        } else {
            elapsed = (uint32_t)cycles_per_adc_sample * (uint32_t)total_pairs;
        }
        uint32_t frame_start_ts = resamp.last_frame_end_ts;

        const int max_emit_per_frame = SAMPLES_PER_CYCLE * 2;
        int emitted_in_frame = 0;

        for (int pair = 0; pair < total_pairs; ++pair) {
            int i = pair * 2;
            uint16_t rv = 0, ri = 0;
            if (p[i].type1.channel == V_CHANNEL) rv = p[i].type1.data;
            else if (p[i].type1.channel == I_CHANNEL) ri = p[i].type1.data;
            if (i+1 < n) {
                if (p[i+1].type1.channel == V_CHANNEL) rv = p[i+1].type1.data;
                else if (p[i+1].type1.channel == I_CHANNEL) ri = p[i+1].type1.data;
            }

            q16_t qv = adcRawToQ16(rv);
            q16_t qi = adcRawToQ16(ri);

            uint32_t ts = frame_start_ts + (uint32_t)(((uint64_t)(pair + 1) * (uint64_t)elapsed) / (uint64_t)total_pairs);

            if (!resamp.initialized) {
                resamp.last_frame_end_ts = frame_end_ts;
                resamp.initialized = true;
                if (next_sample_time == 0) next_sample_time = frame_end_ts;
                resamp.acc_v += qv;
                resamp.acc_i += qi;
                resamp.acc_count++;
                continue;
            }

            resamp.acc_v += qv;
            resamp.acc_i += qi;
            resamp.acc_count++;

            while ( signed_time_diff(ts, next_sample_time) >= 0 ) {
                if (resamp.acc_count > 0) {
                    q16_t v_val = (q16_t)(resamp.acc_v / resamp.acc_count);
                    q16_t i_val = (q16_t)(resamp.acc_i / resamp.acc_count);

                    v_buf[buf_wr % SAMPLES_PER_CYCLE] = v_val;
                    i_buf[buf_wr % SAMPLES_PER_CYCLE] = i_val;

                    float ts_virtual = (float)ticks_per_sample * inv_cpu_freq;
                    sogi_v.step(v_val - v_dc_offset, pll.omega, ts_virtual);
                    sogi_v3.step(v_val - v_dc_offset, 3.0f * pll.omega, ts_virtual);

                    float mag1 = sqrtf(Q16_TO_FLOAT(sogi_v.v_alpha) * Q16_TO_FLOAT(sogi_v.v_alpha) +
                                       Q16_TO_FLOAT(sogi_v.v_beta) * Q16_TO_FLOAT(sogi_v.v_beta));
                    float mag3 = sqrtf(Q16_TO_FLOAT(sogi_v3.v_alpha) * Q16_TO_FLOAT(sogi_v3.v_alpha) +
                                       Q16_TO_FLOAT(sogi_v3.v_beta) * Q16_TO_FLOAT(sogi_v3.v_beta));

                    harmonic_mag1_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag1_smooth + HARMONIC_SMOOTH_ALPHA * mag1;
                    harmonic_mag3_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag3_smooth + HARMONIC_SMOOTH_ALPHA * mag3;

                    float ratio = harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f);
                    float damp_factor = 1.0f;
                    if (ratio > SOGI3_HARMONIC_THRESHOLD) {
                        float overshoot = (ratio - SOGI3_HARMONIC_THRESHOLD) / (3.0f * SOGI3_HARMONIC_THRESHOLD);
                        if (overshoot < 0.0f) overshoot = 0.0f;
                        if (overshoot > 1.0f) overshoot = 1.0f;
                        damp_factor = 1.0f - overshoot * (1.0f - SOGI3_DAMP_MIN);
                    }
                    float learn_att = (ratio > SOGI3_HARMONIC_THRESHOLD) ? 0.0f : 1.0f;
                    pll.setDistortionDamping(damp_factor, learn_att);

                    // SOGI outputs already damping-ready if we scale here
                    q16_t alpha_in = (q16_t)(sogi_v.v_alpha * damp_factor);
                    q16_t beta_in  = (q16_t)(sogi_v.v_beta  * damp_factor);
                    pll.update(alpha_in, beta_in, ts_virtual);

                    buf_wr++;
                    next_sample_time += ticks_per_sample;
                    interp_ok_count++;
                    resamp.acc_v = 0; resamp.acc_i = 0; resamp.acc_count = 0;
                    emitted_in_frame++;
                    if (emitted_in_frame > max_emit_per_frame) break;
                } else {
                    next_sample_time += ticks_per_sample;
                    emitted_in_frame++;
                    if (emitted_in_frame > max_emit_per_frame) break;
                }
            }
            if (emitted_in_frame > max_emit_per_frame) break;
        }
        resamp.last_frame_end_ts = frame_end_ts;
        rd = (rd + 1) % FRAME_BUFFER_SIZE;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    cpu_freq_hz  = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);
    updateTimingParameters(NOMINAL_FREQ);

    for (int i = 0; i < SAMPLES_PER_CYCLE; ++i) {
        v_buf[i] = v_dc_offset;
        i_buf[i] = i_dc_offset;
    }

    harmonic_mag1_smooth = 1e-6f;
    harmonic_mag3_smooth = 1e-6f;

    vis.begin();
    initADCContinuous();

    visQueue = xQueueCreate(4, sizeof(VisSnapshot));
    logQueue = xQueueCreate(8, sizeof(SerialMsg));
    xTaskCreatePinnedToCore(visTask, "visTask", 8192, NULL, tskIDLE_PRIORITY + 2, &visTaskHandle, 0);
    xTaskCreatePinnedToCore(logTask, "logTask", 4096, NULL, tskIDLE_PRIORITY + 1, &logTaskHandle, 0);
}

void IRAM_ATTR loop() {
    static uint32_t serial_ctr  = 0;
    processIncomingDMA();

    if (buf_wr - last_cycle_boundary_samples < SAMPLES_PER_CYCLE) return;

    uint32_t now = get_cycle_count();
    uint32_t last_virtual_ts = next_sample_time - ticks_per_sample;
    int32_t  lag_ticks = signed_time_diff(now, last_virtual_ts);
    float    lag_sec   = (float)lag_ticks * inv_cpu_freq;

    // ROBUST INTEGER EMA for DC OFFSET
    int64_t v_sum = 0, i_sum = 0;
    for (int k = 0; k < SAMPLES_PER_CYCLE; ++k) {
        v_sum += v_buf[k];
        i_sum += i_buf[k];
    }
    q16_t v_avg = (q16_t)(v_sum / SAMPLES_PER_CYCLE);
    q16_t i_avg = (q16_t)(i_sum / SAMPLES_PER_CYCLE);

    // Smooth estimate: y[n] = 0.98 * y[n-1] + 0.02 * x[n]
    // approximately: y[n] = y[n-1] + (x[n] - y[n-1]) / 50
    v_dc_offset += (v_avg - v_dc_offset) / 50;
    i_dc_offset += (i_avg - i_dc_offset) / 50;

    float ts_virtual = (float)ticks_per_sample * inv_cpu_freq;
    uint32_t proc_start = get_cycle_count();

    float phase_corr = pll.omega * lag_sec;
    float phase = atan2f(Q16_TO_FLOAT(sogi_v.v_alpha), -Q16_TO_FLOAT(sogi_v.v_beta)) + phase_corr;
    if (phase < 0.0f) phase += 2.0f * (float)PI;
    else if (phase > 2.0f * (float)PI) phase -= 2.0f * (float)PI;

    if (phase_track.initialized) {
        float delta = phase - phase_track.prev_phase;
        if      (delta < -(float)PI) phase_track.phase_offset += 2.0f * (float)PI;
        else if (delta >  (float)PI) phase_track.phase_offset -= 2.0f * (float)PI;
    } else { phase_track.initialized = true; }
    phase_track.prev_phase = phase;

    float unwrapped  = phase + phase_track.phase_offset;
    float phase_norm = fmodf(unwrapped, 2.0f * (float)PI);
    if (phase_norm < 0.0f) phase_norm += 2.0f * (float)PI;

    float samples_per_cycle_f = 1.0f / (pll.freq * ts_virtual);
    float samples_back        = (phase_norm / (2.0f * (float)PI)) * samples_per_cycle_f;
    int   curr_head_idx       = (int)(buf_wr % SAMPLES_PER_CYCLE);
    int   aligned_start       = (int)((curr_head_idx + SAMPLES_PER_CYCLE - (int)(samples_back + 0.5f)) % SAMPLES_PER_CYCLE);

    last_cycle_boundary_samples = buf_wr;
    updateTimingParameters(pll.freq);

    uint32_t proc_end = get_cycle_count();
    float    core_us  = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;

    VisSnapshot snap;
    snap.v_copy = (float*)heap_caps_malloc(sizeof(float) * SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    snap.i_copy = (float*)heap_caps_malloc(sizeof(float) * SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    if (snap.v_copy && snap.i_copy) {
        for(int k=0; k<SAMPLES_PER_CYCLE; k++) {
            snap.v_copy[k] = Q16_TO_FLOAT(v_buf[k]);
            snap.i_copy[k] = Q16_TO_FLOAT(i_buf[k]);
        }
        snap.aligned_start = aligned_start;
        snap.pll_freq = pll.freq;
        snap.pll_mag  = pll.mag_smooth;
        snap.vdc = Q16_TO_FLOAT(v_dc_offset);
        snap.idc = Q16_TO_FLOAT(i_dc_offset);
        if (xQueueSend(visQueue, &snap, 0) != pdTRUE) { heap_caps_free(snap.v_copy); heap_caps_free(snap.i_copy); }
    } else { if (snap.v_copy) heap_caps_free(snap.v_copy); if (snap.i_copy) heap_caps_free(snap.i_copy); }

    if (++serial_ctr >= SERIAL_EVERY_N_CYCLES) {
        serial_ctr = 0;
        SerialMsg sm;
        sm.pll_freq = pll.freq;
        sm.core_us  = core_us;
        sm.isr_callback_count = isr_callback_count;
        sm.frames_dropped     = frames_dropped;
        sm.interp_ok          = interp_ok_count;
        sm.interp_total       = interp_ok_count + interp_fail_count;
        sm.vdc = Q16_TO_FLOAT(v_dc_offset);
        sm.idc = Q16_TO_FLOAT(i_dc_offset);
        interp_ok_count = 0; interp_fail_count = 0;
        xQueueSend(logQueue, &sm, 0);
    }
    yield();
}
