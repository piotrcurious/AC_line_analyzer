/* ESP32 SOGI-PLL – Continuous DMA resampling with isolated visualizer + logger tasks
 *
 * Changes:
 *  - vis.update() moved to a separate FreeRTOS task (visTask).
 *  - Serial output moved to a low-priority logging task (logTask).
 *  - visTask receives heap-allocated snapshots of v_buf/i_buf so Path A can continue writing.
 *  - interpolateSampleAtTime() snapshots frame indices under noInterrupts() to avoid races.
 *  - Main loop only queues snapshots and returns quickly.
 *
 * Notes:
 *  - Keeps original sampling rates and SERIAL_EVERY_N_CYCLES behavior (no change).
 *  - If queue is full, snapshot is freed (skips one vis update) to avoid blocking loop().
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

// ── Algorithm ────────────────────────────────────────────────────────────────
#define NOMINAL_FREQ       50.0f
#define SOGI_K             0.7071f
#define PLL_KP             2.55f
#define PLL_KI             0.000000f
#define SAMPLES_PER_CYCLE  128        // virtual samples per AC cycle

// ── ADC / DMA ───────────────────────────────────────────────────────────────
#define ADC_OVERSAMPLE_RATE   200000  // total samples/s across both channels
#define CONV_FRAME_SIZE       16      // bytes per DMA interrupt frame

// ── Frame ring ───────────────────────────────────────────────────────────
#define FRAME_BUFFER_SIZE     4000
#define MAX_SEARCH            4000

#define CLEANUP_FRAMES_DIVIDER  25

// ── Diagnostics throttle ─────────────────────────────────────────────────────
#define SERIAL_EVERY_N_CYCLES   1
//#define FULL_DEBUG_SERIAL  // leave disabled; serialing is offloaded anyway

// ── Global objects ──────────────────────────────────────────────────────────
SOGIVisualizer vis;
SOGI           sogi_v(SOGI_K);
AdaptivePLL    pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

// ── DMA frame ring ───────────────────────────────────────────────────────────
struct TimestampedFrame {
    uint32_t end_timestamp;
    uint8_t  raw_data[CONV_FRAME_SIZE];
    uint16_t data_size;
    uint8_t  sample_count;   // lazily cached per frame
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
uint32_t cycles_per_adc_sample = 0;   // CPU ticks between successive DMA samples (per channel)
uint32_t single_cycle_cycles   = 0;   // CPU ticks per AC cycle at pll.freq
uint32_t ticks_per_sample      = 0;   // single_cycle_cycles / SAMPLES_PER_CYCLE

// ── Precomputed constant ─────────────────────────────────────────────────────
static const float RAW_TO_MV = 3300.0f / 4095.0f;

// ── PATH A state – continuous resampler ──────────────────────────────────────
//   Circular sample buffer (single-cycle sized, continuously overwritten)
float    v_buf[SAMPLES_PER_CYCLE];
float    i_buf[SAMPLES_PER_CYCLE];
uint32_t buf_wr = 0;              // ever-increasing write counter
                                  // slot = buf_wr % SAMPLES_PER_CYCLE
uint32_t next_sample_time = 0;    // CPU-tick timestamp of next virtual sample

// ── PATH B state – cycle boundary ────────────────────────────────────────────
uint32_t last_cycle_boundary = 0;

// ── DC offsets ───────────────────────────────────────────────────────────────
float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;

// ── Phase unwrap ─────────────────────────────────────────────────────────────
struct PhaseTrack {
    float prev_phase   = 0.0f;
    float phase_offset = 0.0f;
    bool  initialized  = false;
} phase_track;

// ── Diagnostics ──────────────────────────────────────────────────────────────
uint32_t interp_ok_count   = 0;
uint32_t interp_fail_count = 0;

// ── FreeRTOS objects for vis + logging ───────────────────────────────────────
TaskHandle_t visTaskHandle = NULL;
TaskHandle_t logTaskHandle = NULL;
QueueHandle_t visQueue = NULL;     // holds VisSnapshot
QueueHandle_t logQueue = NULL;     // holds SerialMsg

// Snapshot structures
struct VisSnapshot {
    float *v_copy; // heap allocated arrays
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

//
// ----- low-level utilities (unchanged, small tweaks for atomic snapshots) -----
//

static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a"(ccount));
    return ccount;
}

static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw) {
    return (float)raw * RAW_TO_MV;
}

// Count V+I sample pairs in frame; cache result.
static inline uint8_t IRAM_ATTR countFrameSamples(TimestampedFrame *f) {
    if (f->sample_count) return f->sample_count;
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f->raw_data;
    size_t n = f->data_size / sizeof(adc_digi_output_data_t);
    uint8_t vc = 0, ic = 0;
    for (size_t i = 0; i < n; ++i) {
        uint32_t ch = p[i].type1.channel;
        if      (ch == V_CHANNEL) ++vc;
        else if (ch == I_CHANNEL) ++ic;
    }
    f->sample_count = (vc < ic) ? vc : ic;
    return f->sample_count;
}

// Parse the sample_idx-th V+I pair; single pass, early exit.
static inline bool IRAM_ATTR parseFrameSample(const TimestampedFrame *f,
                                               uint8_t idx,
                                               uint16_t &vr, uint16_t &ir) {
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f->raw_data;
    size_t  n  = f->data_size / sizeof(adc_digi_output_data_t);
    uint8_t vc = 0, ic = 0;
    bool    vf = false, xf = false;
    for (size_t i = 0; i < n; ++i) {
        uint32_t ch = p[i].type1.channel;
        if (ch == V_CHANNEL) {
            if (vc == idx) { vr = p[i].type1.data; vf = true; if (xf) return true; }
            ++vc;
        } else if (ch == I_CHANNEL) {
            if (ic == idx) { ir = p[i].type1.data; xf = true; if (vf) return true; }
            ++ic;
        }
    }
    return (vf && xf);
}

// CPU-tick timestamp of the k-th sample inside a frame.
static inline uint32_t IRAM_ATTR frameSampleTimestamp(uint32_t frame_end,
                                                        uint8_t  k,
                                                        uint8_t  sc) {
    return frame_end - (uint32_t)(sc - 1u - k) * cycles_per_adc_sample;
}

static inline uint32_t IRAM_ATTR decIndex(uint32_t idx) {
    // faster than modulo for common circular-buffer decrement
    return (idx == 0) ? (FRAME_BUFFER_SIZE - 1) : (idx - 1);
}

// Atomic snapshot helper: grab rd/wr in a tiny critical window.
static inline void IRAM_ATTR snapshotFrameIndices(uint32_t &rd_out, uint32_t &wr_out) {
    noInterrupts();
    rd_out = frame_read_idx;
    wr_out = frame_write_idx;
    interrupts();
}

// Core interpolation — modified to use atomic snapshot of indices
static inline bool IRAM_ATTR interpolateSampleAtTime(uint32_t target, float &v_out, float &i_out) {
    // atomic snapshot of indices to avoid races with ISR
    uint32_t rd, wr;
    snapshotFrameIndices(rd, wr);

    if (rd == wr) return false;

    uint32_t avail = (wr >= rd) ? (wr - rd) : (FRAME_BUFFER_SIZE - rd + wr);
    uint32_t lim   = (avail < MAX_SEARCH) ? avail : MAX_SEARCH;

    // single cast for faster access
    TimestampedFrame *frames = (TimestampedFrame *)frame_buffer;

    // start from newest frame (wr - 1) without modulo
    uint32_t fi = decIndex(wr);
    uint32_t searched = 0;

    // precompute reciprocal to avoid repeated divisions
    const float inv_cps = 1.0f / (float)cycles_per_adc_sample;

    while (searched < lim) {
        TimestampedFrame *f = &frames[fi];

        if (f->data_size) {
            uint8_t sc = countFrameSamples(f);
            if (sc) {
                uint32_t fe = f->end_timestamp;
                uint32_t fs = frameSampleTimestamp(fe, 0, sc);

                // if target is far newer than the frame, break (same heuristic as original)
                if ((int32_t)(target - fe) > (int32_t)(cpu_freq_hz / 10)) break;

                int32_t ds = (int32_t)(target - fs);
                int32_t de = (int32_t)(target - fe);

                // target inside this frame (between fs and fe)
                if (ds >= 0 && de <= 0) {
                    // pos = (target - fs) / cycles_per_adc_sample
                    float pos = (float)(uint32_t)(target - fs) * inv_cps;
                    uint32_t ib = (uint32_t)pos;                // integer index
                    if (ib >= (uint32_t)(sc - 1)) {
                        // clamp to last sample
                        uint16_t vr, ir;
                        if (parseFrameSample(f, sc - 1, vr, ir)) {
                            v_out = adcRawToMillivolts(vr);
                            i_out = adcRawToMillivolts(ir);
                            return true;
                        }
                    } else {
                        uint32_t ia = ib + 1;
                        float alpha = pos - (float)ib;
                        uint16_t vb_raw, ib_raw, va_raw, ia_raw;
                        // try to parse both samples once each
                        if (parseFrameSample(f, (uint8_t)ib, vb_raw, ib_raw) &&
                            parseFrameSample(f, (uint8_t)ia, va_raw, ia_raw)) {
                            float fvb = adcRawToMillivolts(vb_raw);
                            float fva = adcRawToMillivolts(va_raw);
                            float fib = adcRawToMillivolts(ib_raw);
                            float fia = adcRawToMillivolts(ia_raw);
                            v_out = fvb + alpha * (fva - fvb);
                            i_out = fib + alpha * (fia - fib);
                            return true;
                        }
                    }
                }

                // if frame ends after target (de <= 0), return frame's first sample as fallback
                if (de <= 0) {
                    uint16_t vr, ir;
                    if (parseFrameSample(f, 0, vr, ir)) {
                        v_out = adcRawToMillivolts(vr);
                        i_out = adcRawToMillivolts(ir);
                        return true;
                    }
                }
            }
        }

        // step to previous frame (fast decrement)
        fi = decIndex(fi);
        ++searched;
        if (fi == rd) break;
    }

    // Last-resort: newest frame's last sample
    uint32_t lfi = decIndex(wr);
    TimestampedFrame *lf = &frames[lfi];
    if (lf->data_size) {
        uint8_t sc = countFrameSamples(lf);
        if (sc) {
            uint16_t vr, ir;
            if (parseFrameSample(lf, sc - 1, vr, ir)) {
                v_out = adcRawToMillivolts(vr);
                i_out = adcRawToMillivolts(ir);
                return true;
            }
        }
    }
    return false;
}

// Discard frames older than ~40 ms
static inline void IRAM_ATTR cleanupOldFrames(uint32_t now) {
    uint32_t keep = cpu_freq_hz / CLEANUP_FRAMES_DIVIDER;
    while (frame_read_idx != frame_write_idx) {
        int32_t age = (int32_t)(now - frame_buffer[frame_read_idx].end_timestamp);
        if (age < (int32_t)keep) break;
        frame_read_idx = (frame_read_idx + 1) % FRAME_BUFFER_SIZE;
    }
}

// Recompute single_cycle_cycles and ticks_per_sample from current PLL frequency
static inline void IRAM_ATTR updateTimingParameters(float frequency) {
    float fc            = constrain(frequency, 40.0f, 90.0f);
    single_cycle_cycles = (uint32_t)lrintf((float)cpu_freq_hz / fc);
    ticks_per_sample    = single_cycle_cycles / SAMPLES_PER_CYCLE;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ISR
// ─────────────────────────────────────────────────────────────────────────────
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data) {
    uint32_t ts      = get_cycle_count();
    uint32_t next_wr = (frame_write_idx + 1) % FRAME_BUFFER_SIZE;

    if (next_wr == frame_read_idx) {
        frame_read_idx = (frame_read_idx + 1) % FRAME_BUFFER_SIZE;
        ++frames_dropped;
    }

    uint32_t sz = edata->size;
    if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;
    memcpy((void *)frame_buffer[frame_write_idx].raw_data, edata->conv_frame_buffer, sz);

    frame_buffer[frame_write_idx].end_timestamp = ts;
    frame_buffer[frame_write_idx].data_size     = (uint16_t)sz;
    frame_buffer[frame_write_idx].sample_count  = 0;

    frame_write_idx = next_wr;
    ++isr_callback_count;
    return false;
}

// ─────────────────────────────────────────────────────────────────────────────
//  ADC init (unchanged)
// ─────────────────────────────────────────────────────────────────────────────
bool initADCContinuous() {
    adc_continuous_handle_cfg_t cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size    = CONV_FRAME_SIZE,
    };
    if (adc_continuous_new_handle(&cfg, &adc_handle) != ESP_OK) {
        Serial.println("ADC handle failed"); return false;
    }

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
    if (adc_continuous_config(adc_handle, &dig) != ESP_OK) {
        Serial.println("ADC config failed"); return false;
    }

    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_callback };
    if (adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL) != ESP_OK) {
        Serial.println("ADC cb reg failed"); return false;
    }
    if (adc_continuous_start(adc_handle) != ESP_OK) {
        Serial.println("ADC start failed"); return false;
    }

    Serial.printf("ADC OK  rate=%d Hz  frame=%d bytes\n", ADC_OVERSAMPLE_RATE, CONV_FRAME_SIZE);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Tasks
// ─────────────────────────────────────────────────────────────────────────────

void visTask(void *pv) {
    VisSnapshot snap;
    for (;;) {
        if (xQueueReceive(visQueue, &snap, portMAX_DELAY) == pdTRUE) {
            // call visualizer with the snapshot copies
            vis.update(snap.v_copy, snap.i_copy, SAMPLES_PER_CYCLE,
                       snap.aligned_start, SAMPLES_PER_CYCLE,
                       snap.pll_freq, snap.pll_mag,
                       snap.vdc, snap.idc);
            // free the copies (they were allocated with heap_caps_malloc)
            heap_caps_free(snap.v_copy);
            heap_caps_free(snap.i_copy);
        }
    }
}

void logTask(void *pv) {
    SerialMsg msg;
    for (;;) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            // perform Serial.printf() here at low priority
            Serial.printf(
                "F:%.4fHz  Core:%.1fus  ISR:%lu  Drop:%lu  Interp:%lu/%lu  Vdc:%.1f Idc:%.1f\n",
                msg.pll_freq,
                msg.core_us,
                (unsigned long)msg.isr_callback_count,
                (unsigned long)msg.frames_dropped,
                (unsigned long)msg.interp_ok,
                (unsigned long)msg.interp_total,
                msg.vdc, msg.idc
            );
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  setup()
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nSOGI-PLL  continuous resampling + per-cycle processing (tasks)");

    cpu_freq_hz  = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;

    // Each channel is interleaved, so effective per-channel rate = total / 2
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);
    Serial.printf("cycles_per_adc_sample=%lu (%.2f us)\n",
                  cycles_per_adc_sample,
                  (float)cycles_per_adc_sample * inv_cpu_freq * 1e6f);

    updateTimingParameters(NOMINAL_FREQ);

    // Seed both paths from the same "now"
    uint32_t now      = get_cycle_count();
    next_sample_time  = now;
    last_cycle_boundary = now;

    for (int i = 0; i < SAMPLES_PER_CYCLE; ++i) {
        v_buf[i] = v_dc_offset;
        i_buf[i] = i_dc_offset;
    }

    vis.begin();

    if (!initADCContinuous()) {
        Serial.println("ADC init failed!");
        while (1) delay(100);
    }

    // Create queues & tasks
    visQueue = xQueueCreate(4, sizeof(VisSnapshot));
    logQueue = xQueueCreate(8, sizeof(SerialMsg));
    if (!visQueue || !logQueue) {
        Serial.println("Queue alloc failed"); while(1) delay(1000);
    }

    // visTask: slightly higher priority than logger, but both lower than system-critical ISRs
    xTaskCreatePinnedToCore(visTask, "visTask", 8192, NULL, tskIDLE_PRIORITY + 2, &visTaskHandle, 0);
    // logTask: low priority
    xTaskCreatePinnedToCore(logTask, "logTask", 4096, NULL, tskIDLE_PRIORITY + 1, &logTaskHandle, 0);

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);

    Serial.printf("Ready.  CPU=%d MHz  single_cycle_cycles=%lu  ticks_per_sample=%lu\n",
                  ESP.getCpuFreqMHz(), single_cycle_cycles, ticks_per_sample);
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop()
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR loop() {
    static uint32_t cleanup_ctr = 0;
    static uint32_t serial_ctr  = 0;

    uint32_t now = get_cycle_count();

    // PATH A – Continuous resampling
    while ((int32_t)(now - next_sample_time) >= 0) {

// --- NEW: Lookahead Guard ---
        // Ensure the DMA has actually delivered a frame that is newer than our target time.
        uint32_t rd, wr;
        snapshotFrameIndices(rd, wr);
        
        if (rd == wr) break; // No data available at all yet
        
        uint32_t newest_idx = decIndex(wr);
        uint32_t newest_ts = frame_buffer[newest_idx].end_timestamp;
        
        // If the sample we want is in the future relative to the newest frame, 
        // it is trapped in the active DMA buffer. Break and wait for the next loop.
        if ((int32_t)(next_sample_time - newest_ts) > 0) {
            break; 
        }
        // -----------------------------
      
        float v, i;
        if (interpolateSampleAtTime(next_sample_time, v, i)) {
            ++interp_ok_count;
        } else {
            v = v_dc_offset;
            i = i_dc_offset;
            ++interp_fail_count;
        }
        v_buf[buf_wr % SAMPLES_PER_CYCLE] = v;
        i_buf[buf_wr % SAMPLES_PER_CYCLE] = i;
        ++buf_wr;
        next_sample_time += ticks_per_sample;
    }

    if (++cleanup_ctr >= 50) {
        cleanupOldFrames(now);
        cleanup_ctr = 0;
    }

//    // PATH B – Cycle boundary processing
//    if ((int32_t)(now - (last_cycle_boundary)) < (int32_t)single_cycle_cycles) return;

    // PATH B – Cycle boundary processing
    // Trigger based on the virtual timeline of the data, not the CPU clock!
    if ((int32_t)(next_sample_time - last_cycle_boundary) < (int32_t)single_cycle_cycles) return;

    last_cycle_boundary += single_cycle_cycles;

    // Need at least one full buffer of data before processing
    if (buf_wr < SAMPLES_PER_CYCLE) return;

    // DC estimation
    float v_sum = 0.0f, i_sum = 0.0f;
    for (int k = 0; k < SAMPLES_PER_CYCLE; ++k) {
        v_sum += v_buf[k];
        i_sum += i_buf[k];
    }
    v_dc_offset = 0.8f * v_dc_offset + 0.2f * (v_sum * (1.0f / SAMPLES_PER_CYCLE));
    i_dc_offset = 0.8f * i_dc_offset + 0.2f * (i_sum * (1.0f / SAMPLES_PER_CYCLE));

    // SOGI
    float ts        = (float)ticks_per_sample * inv_cpu_freq;
    int   start_idx = (int)(buf_wr % SAMPLES_PER_CYCLE);

    uint32_t proc_start = get_cycle_count();

    sogi_v.processWindow(v_buf, SAMPLES_PER_CYCLE, start_idx, SAMPLES_PER_CYCLE,
                         pll.omega, ts, v_dc_offset);

    // PLL
    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts);

    // Phase alignment
    SOGI phase_sogi(SOGI_K);
    phase_sogi.processWindow(v_buf, SAMPLES_PER_CYCLE, start_idx, SAMPLES_PER_CYCLE,
                             pll.omega, ts, v_dc_offset);

    float phase = atan2f(phase_sogi.v_alpha, -phase_sogi.v_beta);
    if (phase < 0.0f) phase += 2.0f * (float)PI;

    if (phase_track.initialized) {
        float delta = phase - phase_track.prev_phase;
        if      (delta < -(float)PI) phase_track.phase_offset += 2.0f * (float)PI;
        else if (delta >  (float)PI) phase_track.phase_offset -= 2.0f * (float)PI;
    } else {
        phase_track.initialized = true;
    }
    phase_track.prev_phase = phase;

    float unwrapped  = phase + phase_track.phase_offset;
    float phase_norm = fmodf(unwrapped, 2.0f * (float)PI);
    if (phase_norm < 0.0f) phase_norm += 2.0f * (float)PI;

    float samples_per_cycle_f = 1.0f / (pll.freq * ts);
    float samples_back        = (phase_norm / (2.0f * (float)PI)) * samples_per_cycle_f;
    int   aligned_start       = (int)((start_idx
                                       + SAMPLES_PER_CYCLE
                                       - (int)(samples_back + 0.5f))
                                      % SAMPLES_PER_CYCLE);

    // Update timing for next cycle
    updateTimingParameters(pll.freq);

    uint32_t proc_end = get_cycle_count();
    float    core_us  = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;

    // ---------- Queue a vis snapshot (copy) ----------
    // Allocate heap memory for two float arrays. FreeRTOS queue stores the pointer+meta.
    // Copy performed inside a tiny critical section to avoid concurrent writes from Path A.
    VisSnapshot snap;
    snap.v_copy = (float*)heap_caps_malloc(sizeof(float) * SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    snap.i_copy = (float*)heap_caps_malloc(sizeof(float) * SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    if (snap.v_copy && snap.i_copy) {
        noInterrupts();
        for (int k = 0; k < SAMPLES_PER_CYCLE; ++k) {
            snap.v_copy[k] = v_buf[k];
            snap.i_copy[k] = i_buf[k];
        }
        interrupts();

        snap.aligned_start = aligned_start;
        snap.pll_freq = pll.freq;
        snap.pll_mag  = pll.mag_smooth;
        snap.vdc = v_dc_offset;
        snap.idc = i_dc_offset;

        // Non-blocking send: if queue full, drop this snapshot and free memory.
        if (xQueueSend(visQueue, &snap, 0) != pdTRUE) {
            heap_caps_free(snap.v_copy);
            heap_caps_free(snap.i_copy);
        }
    } else {
        // allocation failed: free any that succeeded
        if (snap.v_copy) heap_caps_free(snap.v_copy);
        if (snap.i_copy) heap_caps_free(snap.i_copy);
    }

    // ---------- Queue a serial/log message (do not change update rate) ----------
    if (++serial_ctr >= SERIAL_EVERY_N_CYCLES) {
        serial_ctr = 0;
        SerialMsg sm;
        sm.pll_freq = pll.freq;
        sm.core_us  = core_us;
        // snapshot counters atomically
        noInterrupts();
        sm.isr_callback_count = isr_callback_count;
        sm.frames_dropped     = frames_dropped;
        sm.interp_ok          = interp_ok_count;
        sm.interp_total       = interp_ok_count + interp_fail_count;
        sm.vdc = v_dc_offset;
        sm.idc = i_dc_offset;
        // reset counters for next period
        interp_ok_count   = 0;
        interp_fail_count = 0;
        
        interrupts();

        // non-blocking send; if queue full, we drop the message (but must not block loop)
        if (xQueueSend(logQueue, &sm, 0) != pdTRUE) {
            // dropped; nothing to free here
        }
    }

    yield();
}
