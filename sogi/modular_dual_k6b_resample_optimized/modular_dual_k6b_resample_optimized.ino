/* ESP32 SOGI-PLL – Hardware-Anchored Streaming Resampler with Batch SOGI-PLL
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

// ── ADC / DMA ───────────────────────────────────────────────────────────────
#define ADC_OVERSAMPLE_RATE   200000  // total samples/s across both channels
#define CONV_FRAME_SIZE       128     // Large DMA frame size

// ── Frame ring ────────────────────────────────────────────────────────────
#define FRAME_BUFFER_SIZE     256

// ── Diagnostics throttle ─────────────────────────────────────────────────────
#define SERIAL_EVERY_N_CYCLES   50

// ── Global objects ──────────────────────────────────────────────────────────
SOGIVisualizer vis;
SOGI           sogi_v(SOGI_K);
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
static const float RAW_TO_MV = 3300.0f / 4095.0f;

// ── PATH A state – Decimating Resampler ─────────────────────────────────────
float    v_buf[SAMPLES_PER_CYCLE];
float    i_buf[SAMPLES_PER_CYCLE];
uint32_t buf_wr = 0;
uint32_t next_sample_time = 0;

struct ResamplerState {
    uint32_t last_hw_ts = 0;
    float    acc_v = 0.0f;
    float    acc_i = 0.0f;
    int      acc_count = 0;
    bool     initialized = false;
} resamp;

// ── PATH B state ─────────────────────────────────────────────────────────────
uint32_t last_cycle_boundary_samples = 0;
float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;

struct PhaseTrack {
    float prev_phase   = 0.0f;
    float phase_offset = 0.0f;
    bool  initialized  = false;
} phase_track;

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

static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw) {
    return (float)raw * RAW_TO_MV;
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
                "F:%.4fHz  Core:%.1fus  ISR:%lu  Drop:%lu  Vdc:%.1f Idc:%.1f\n",
                msg.pll_freq, msg.core_us, (unsigned long)msg.isr_callback_count,
                (unsigned long)msg.frames_dropped, msg.vdc, msg.idc
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Continuous Hardware-Anchored Resampler (PATH A)
// ─────────────────────────────────────────────────────────────────────────────

void IRAM_ATTR processIncomingDMA() {
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);

    if (rd == wr) return;

    const uint32_t cycles_per_pair = cycles_per_adc_sample;

    while (rd != wr) {
        volatile TimestampedFrame *f_ptr = &frame_buffer[rd];
        uint32_t isr_end_ts = f_ptr->end_timestamp;
        uint16_t sz = f_ptr->data_size;

        const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f_ptr->raw_data;
        int n = sz / sizeof(adc_digi_output_data_t);
        if (n <= 0) { rd = (rd + 1) % FRAME_BUFFER_SIZE; continue; }

        int total_pairs = n / 2;

        if (!resamp.initialized) {
            // Anchor timeline to hardware
            resamp.last_hw_ts = isr_end_ts - (total_pairs * cycles_per_pair);
            resamp.initialized = true;
            if (next_sample_time == 0) next_sample_time = isr_end_ts;
        }

        // Timeline drift correction (very slow low-pass)
        uint32_t projected_end = resamp.last_hw_ts + (total_pairs * cycles_per_pair);
        int32_t drift = signed_time_diff(isr_end_ts, projected_end);
        if (abs(drift) > (int)(cycles_per_pair * total_pairs * 4)) {
            resamp.last_hw_ts = isr_end_ts - (total_pairs * cycles_per_pair);
        } else {
            resamp.last_hw_ts += (drift >> 8); // Nudge
        }

        for (int pair = 0; pair < total_pairs; ++pair) {
            int i = pair * 2;
            uint16_t rv = 0, ri = 0;
            if (p[i].type1.channel == V_CHANNEL) rv = p[i].type1.data;
            else if (p[i].type1.channel == I_CHANNEL) ri = p[i].type1.data;
            if (i+1 < n) {
                if (p[i+1].type1.channel == V_CHANNEL) rv = p[i+1].type1.data;
                else if (p[i+1].type1.channel == I_CHANNEL) ri = p[i+1].type1.data;
            }

            resamp.acc_v += adcRawToMillivolts(rv);
            resamp.acc_i += adcRawToMillivolts(ri);
            resamp.acc_count++;

            uint32_t ts = resamp.last_hw_ts + cycles_per_pair;

            while ( signed_time_diff(ts, next_sample_time) >= 0 ) {
                if (resamp.acc_count > 0) {
                    float inv = 1.0f / (float)resamp.acc_count;
                    v_buf[buf_wr % SAMPLES_PER_CYCLE] = resamp.acc_v * inv;
                    i_buf[buf_wr % SAMPLES_PER_CYCLE] = resamp.acc_i * inv;

                    buf_wr++;
                    next_sample_time += ticks_per_sample;
                    resamp.acc_v = 0; resamp.acc_i = 0; resamp.acc_count = 0;
                } else {
                    next_sample_time += ticks_per_sample;
                }
            }
            resamp.last_hw_ts = ts;
        }

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
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);

    updateTimingParameters(NOMINAL_FREQ);

    for (int i = 0; i < SAMPLES_PER_CYCLE; ++i) {
        v_buf[i] = v_dc_offset;
        i_buf[i] = i_dc_offset;
    }

    vis.begin();
    initADCContinuous();

    visQueue = xQueueCreate(4, sizeof(VisSnapshot));
    logQueue = xQueueCreate(8, sizeof(SerialMsg));
    xTaskCreatePinnedToCore(visTask, "visTask", 8192, NULL, tskIDLE_PRIORITY + 2, &visTaskHandle, 0);
    xTaskCreatePinnedToCore(logTask, "logTask", 4096, NULL, tskIDLE_PRIORITY + 1, &logTaskHandle, 0);

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR loop() {
    static uint32_t serial_ctr  = 0;

    processIncomingDMA();

    // Trigger Path B every cycle
    if (buf_wr - last_cycle_boundary_samples < SAMPLES_PER_CYCLE) return;

    uint32_t now = get_cycle_count();
    uint32_t last_virtual_ts = next_sample_time - ticks_per_sample;
    float lag_sec = (float)signed_time_diff(now, last_virtual_ts) * inv_cpu_freq;

    // Batch SOGI processing for stability
    float ts_virtual = (float)ticks_per_sample * inv_cpu_freq;
    int start_idx = (int)(buf_wr % SAMPLES_PER_CYCLE); // current write head

    uint32_t proc_start = get_cycle_count();

    // Estimate DC offset from this cycle
    float v_sum = 0, i_sum = 0;
    for(int k=0; k<SAMPLES_PER_CYCLE; k++) { v_sum += v_buf[k]; i_sum += i_buf[k]; }
    v_dc_offset = 0.95f * v_dc_offset + 0.05f * (v_sum / SAMPLES_PER_CYCLE);
    i_dc_offset = 0.95f * i_dc_offset + 0.05f * (i_sum / SAMPLES_PER_CYCLE);

    // Process exactly one cycle ending at data head
    sogi_v.processWindow(v_buf, SAMPLES_PER_CYCLE, start_idx, SAMPLES_PER_CYCLE,
                         pll.omega, ts_virtual, v_dc_offset);

    // Update PLL
    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts_virtual);

    // Lag compensation for visualization ONLY
    float phase_corr = pll.omega * lag_sec;
    float phase = atan2f(sogi_v.v_alpha, -sogi_v.v_beta) + phase_corr;
    if (phase < 0) phase += 2*PI; else if (phase > 2*PI) phase -= 2*PI;

    if (phase_track.initialized) {
        float delta = phase - phase_track.prev_phase;
        if (delta < -PI) phase_track.phase_offset += 2*PI;
        else if (delta > PI) phase_track.phase_offset -= 2*PI;
    } else { phase_track.initialized = true; }
    phase_track.prev_phase = phase;

    float unwrapped = phase + phase_track.phase_offset;
    float phase_norm = fmodf(unwrapped, 2*PI);
    if (phase_norm < 0) phase_norm += 2*PI;

    float samples_per_cycle_f = 1.0f / (pll.freq * ts_virtual);
    float samples_back = (phase_norm / (2*PI)) * samples_per_cycle_f;
    int aligned_start = (int)((start_idx + SAMPLES_PER_CYCLE - (int)(samples_back + 0.5f)) % SAMPLES_PER_CYCLE);

    last_cycle_boundary_samples = buf_wr;
    updateTimingParameters(pll.freq);

    uint32_t proc_end = get_cycle_count();
    float core_us = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;

    VisSnapshot snap;
    snap.v_copy = (float*)heap_caps_malloc(sizeof(float)*SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    snap.i_copy = (float*)heap_caps_malloc(sizeof(float)*SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    if (snap.v_copy && snap.i_copy) {
        memcpy(snap.v_copy, v_buf, sizeof(v_buf));
        memcpy(snap.i_copy, i_buf, sizeof(i_buf));
        snap.aligned_start = aligned_start; snap.pll_freq = pll.freq;
        snap.pll_mag = pll.mag_smooth; snap.vdc = v_dc_offset; snap.idc = i_dc_offset;
        if (xQueueSend(visQueue, &snap, 0) != pdTRUE) { heap_caps_free(snap.v_copy); heap_caps_free(snap.i_copy); }
    } else { if(snap.v_copy) heap_caps_free(snap.v_copy); if(snap.i_copy) heap_caps_free(snap.i_copy); }

    if (++serial_ctr >= SERIAL_EVERY_N_CYCLES) {
        serial_ctr = 0;
        SerialMsg sm; sm.pll_freq = pll.freq; sm.core_us = core_us;
        sm.isr_callback_count = isr_callback_count; sm.frames_dropped = frames_dropped;
        sm.vdc = v_dc_offset; sm.idc = i_dc_offset;
        xQueueSend(logQueue, &sm, 0);
    }
    yield();
}
