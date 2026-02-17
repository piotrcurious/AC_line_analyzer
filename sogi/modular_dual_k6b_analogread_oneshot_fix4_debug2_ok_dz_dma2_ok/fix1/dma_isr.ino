/*
 * ESP32 SOGI-PLL - ADC Continuous Mode (PLL-Paced Polling)
 * Improved: DMA/frame timestamps in ISR + per-conversion timestamps +
 * interpolation at PLL sample time.
 *
 * Notes:
 * - Uses adc_continuous event callback to get frame pointer and size.
 * - Timestamps frame end with CPU cycle counter; computes sample timestamps
 *   assuming stable sample_freq_hz.
 * - Stores per-channel samples (value + ccount) into circular buffers.
 * - At PLL sample time we interpolate between adjacent per-channel samples.
 *
 * Requires: ESP32 Arduino core with esp_adc/adc_continuous.h available.
 */

#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "SOGI.h"
#include "SOGIvisualizer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h" // IRAM_ATTR
#include "soc/rtc.h"
#include "esp32/rom/ets_sys.h"

// ---------- user parameters (same as original) ----------
#define ADC_PIN_V 36  // ADC1_CH0
#define ADC_PIN_I 39  // ADC1_CH3
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f
#define PLL_KP 1.25f
#define PLL_KI 0.000000f
#define SAMPLES_PER_CYCLE 220
#define V_REF 3.3f

#define V_CHANNEL 0  // GPIO36 = ADC1_CH0
#define I_CHANNEL 3  // GPIO39 = ADC1_CH3

// ADC runs faster than PLL sample rate to ensure fresh data
#define ADC_OVERSAMPLE_RATE 200000  // total conversions per second (both channels combined)
#define CONV_FRAME_BYTES 512        // bytes per conversion frame from driver (tune as needed)

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;
    bool initialized = false;
} phase_track;

adc_continuous_handle_t adc_handle = NULL;

// cycle counter helper
static inline uint32_t get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

// convert raw ADC to mV
float adcRawToMillivolts(uint16_t raw_value) {
    return (float)raw_value * 3300.0f / 4095.0f;
}

// ---------- timing variables ----------
uint32_t cpu_hz = 0;
float inv_cpu_freq = 0.0f;
uint32_t cycles_per_conv = 0; // CPU cycles between adjacent conversions (all conversions, not per-channel)

// Bresenham scheduling state (same as original)
uint32_t base_ticks_per_sample = 0;
uint32_t ticks_remainder = 0;
uint32_t bresenham_acc = 0;
uint32_t sample_slot_count = 0;

float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_cycle_boundary = 0;
uint32_t single_cycle_cycles = 0;
uint32_t ticks_per_sample = 0;

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};

const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float v_samp_buf[BUF_N];
float i_samp_buf[BUF_N];
int buf_idx = 0;

// ---------- per-channel timestamped circular buffers ----------
uint32_t v_ts_buf[BUF_N]; // ccount timestamp for each v sample
uint32_t i_ts_buf[BUF_N]; // ccount timestamp for each i sample
int v_head = 0; // next write index
int i_head = 0;
int v_count = 0;
int i_count = 0;

// small spinlock for short critical sections between ISR and main loop
portMUX_TYPE samples_mux = portMUX_INITIALIZER_UNLOCKED;

// frame copying ring (filled in ISR callback)
#define FRAME_RING_SIZE 8
#define FRAME_BUF_SIZE CONV_FRAME_BYTES
struct FrameRingItem {
    uint32_t frame_start_ccount;   // computed start (ccount) of the first conversion in this frame
    int      bytes;                // number of bytes stored
    uint8_t  buf[FRAME_BUF_SIZE];  // copied frame bytes
};
volatile FrameRingItem frame_ring[FRAME_RING_SIZE];
volatile int frame_head = 0; // written by ISR (next index to write)
volatile int frame_tail = 0; // read by loop (next index to read)
volatile uint32_t frame_overruns = 0;

// ADC read scratch (keeps compatibility with earlier code when not using callback)
uint8_t adc_result[CONV_FRAME_BYTES] = {0};
uint32_t max_jitter_ticks = 0;
uint32_t overrun_count = 0;

// convert driver event bytes per conv: we will treat as 2 bytes per conv (type1)
const int BYTES_PER_CONV = 2; // use 2 to match the type1->data layout we used previously

// ---------- helper: push per-channel sample (value + ccount) ----------
static inline void push_v_sample(float mv, uint32_t ccount) {
    portENTER_CRITICAL(&samples_mux);
    v_samp_buf[v_head] = mv;
    v_ts_buf[v_head] = ccount;
    v_head = (v_head + 1) % BUF_N;
    if (v_count < BUF_N) ++v_count;
    portEXIT_CRITICAL(&samples_mux);
}
static inline void push_i_sample(float mv, uint32_t ccount) {
    portENTER_CRITICAL(&samples_mux);
    i_samp_buf[i_head] = mv;
    i_ts_buf[i_head] = ccount;
    i_head = (i_head + 1) % BUF_N;
    if (i_count < BUF_N) ++i_count;
    portEXIT_CRITICAL(&samples_mux);
}

// ---------- helper: linear interpolation by timestamp (ccount) ----------
bool interpolate_channel_at(uint32_t target_ccount, const float *vals, const uint32_t *ts, int count, int head_index, float &out) {
    // If not enough samples, return false
    if (count < 2) return false;

    // We'll search backwards from head_index-1 (most recent)
    int idx_recent = (head_index - 1 + BUF_N) % BUF_N;

    // Acquire short critical to prevent concurrent writer
    portENTER_CRITICAL(&samples_mux);

    // Quick checks: newest and oldest timestamps
    uint32_t newest_ts = ts[idx_recent];
    int oldest_index = (head_index - count + BUF_N) % BUF_N;
    uint32_t oldest_ts = ts[oldest_index];

    // If target is newer than newest_ts or older than oldest_ts, we can still extrapolate or clamp:
    if (target_ccount >= newest_ts) {
        // clamp to newest
        out = vals[idx_recent];
        portEXIT_CRITICAL(&samples_mux);
        return true;
    }
    if (target_ccount <= oldest_ts) {
        // clamp to oldest
        out = vals[oldest_index];
        portEXIT_CRITICAL(&samples_mux);
        return true;
    }

    // Walk backward to find bounding samples where ts[idx_lo] <= target <= ts[idx_hi]
    int idx_hi = idx_recent;
    int idx_lo;
    bool found = false;
    for (int i = 0; i < count - 1; ++i) {
        idx_lo = (idx_hi - 1 + BUF_N) % BUF_N;
        uint32_t t_lo = ts[idx_lo];
        uint32_t t_hi = ts[idx_hi];

        if (t_lo <= target_ccount && target_ccount <= t_hi) {
            // weight = (target - t_lo) / (t_hi - t_lo)
            float denom = (float)( (int32_t)(t_hi - t_lo) );
            if (denom == 0.0f) {
                out = vals[idx_lo];
            } else {
                float w = ((float)( (int32_t)(target_ccount - t_lo) )) / denom;
                out = vals[idx_lo] + w * (vals[idx_hi] - vals[idx_lo]);
            }
            found = true;
            break;
        }
        idx_hi = idx_lo;
    }

    portEXIT_CRITICAL(&samples_mux);
    return found;
}

// ---------- ADC continuous callback (ISR context) ----------
// Prototype: typedef bool (*adc_continuous_callback_t)(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
IRAM_ATTR bool adc_frame_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    if (!edata || edata->conv_frame_buffer == NULL || edata->size <= 0) return false;

    // Timestamp at callback invocation: this typically corresponds to "frame end" (last conversion just done)
    uint32_t frame_end_ccount = get_cycle_count();

    // number of conversions in this frame
    int n_conv = edata->size / BYTES_PER_CONV;
    if (n_conv <= 0) return false;

    // compute approximate frame start timestamp (first conversion) as:
    // frame_start = frame_end - (n_conv - 1) * cycles_per_conv
    uint32_t frame_start_ccount = frame_end_ccount;
    uint32_t total_offset = 0;
    if (cycles_per_conv > 0 && n_conv > 1) {
        uint64_t offs = (uint64_t)(n_conv - 1) * (uint64_t)cycles_per_conv;
        // avoid 32-bit overflow: do 64-bit then cast
        frame_start_ccount = (uint32_t)( (uint64_t)frame_end_ccount - offs );
    }

    // Copy frame quickly into our ring buffer
    int next_head = (frame_head + 1) % FRAME_RING_SIZE;
    if (next_head == frame_tail) {
        // ring full -- drop oldest
        frame_overruns++;
        frame_tail = (frame_tail + 1) % FRAME_RING_SIZE;
    }
    // Copy content
    FrameRingItem &slot = (FrameRingItem&)frame_ring[frame_head];
    slot.frame_start_ccount = frame_start_ccount;
    slot.bytes = min((int)edata->size, FRAME_BUF_SIZE);
    // copy bytes (driver buffer is owned by driver; safe to memcpy)
    memcpy((void*)slot.buf, edata->conv_frame_buffer, slot.bytes);

    // advance head (note: single-word write is atomic on ESP32)
    frame_head = next_head;

    // Return true -> indicates handled
    return true;
}

// ---------- initADCContinuous: register callback and start ----------
bool initADCContinuous() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 8192,
        .conv_frame_size = CONV_FRAME_BYTES,
    };

    if (adc_continuous_new_handle(&adc_config, &adc_handle) != ESP_OK) {
        Serial.println("Failed to create ADC handle");
        return false;
    }

    adc_digi_pattern_config_t adc_pattern[2];

    // Voltage channel (GPIO36 = CH0)
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = V_CHANNEL;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;

    // Current channel (GPIO39 = CH3)
    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = I_CHANNEL;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = ADC_BITWIDTH_12;

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 2,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = ADC_OVERSAMPLE_RATE,  // total conversions per second
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    if (adc_continuous_config(adc_handle, &dig_cfg) != ESP_OK) {
        Serial.println("Failed to configure ADC");
        return false;
    }

    // Register the ISR callback - must be done BEFORE adc_continuous_start()
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_frame_callback,
        .on_pool_ovf = NULL,
    };
    if (adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL) != ESP_OK) {
        Serial.println("Failed to register ADC event callbacks");
        return false;
    }

    if (adc_continuous_start(adc_handle) != ESP_OK) {
        Serial.println("Failed to start ADC");
        return false;
    }

    Serial.printf("ADC Continuous mode OK (%d Hz total)\n", ADC_OVERSAMPLE_RATE);

    // compute cycles per conversion for timestamp calculations
    cpu_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_hz;
    if (ADC_OVERSAMPLE_RATE > 0) {
        cycles_per_conv = (uint32_t) lrintf((float)cpu_hz / (float)ADC_OVERSAMPLE_RATE);
        if (cycles_per_conv == 0) cycles_per_conv = 1;
    }

    Serial.printf("cpu_hz=%u cycles_per_conv=%u\n", cpu_hz, cycles_per_conv);

    return true;
}

// ---------- transfer frames from ring into per-channel timestamped buffers ----------
void process_pending_frames() {
    // Pop frames while present
    while (frame_tail != frame_head) {
        int idx = frame_tail;
        // read slot (single word reads are atomic enough for these few fields)
        FrameRingItem slot = frame_ring[idx];
        frame_tail = (frame_tail + 1) % FRAME_RING_SIZE;

        // parse slot.buf which contains conv results in type1 format (2 bytes each)
        int bytes = slot.bytes;
        int n_conv = bytes / BYTES_PER_CONV;
        uint32_t base_ccount = slot.frame_start_ccount;

        for (int k = 0; k < n_conv; ++k) {
            int offset = k * BYTES_PER_CONV;
            // interpret two bytes as adc_digi_output_data_t->type1.* layout
            // When reading raw bytes assembled in driver, cast accordingly:
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&slot.buf[offset];
            uint32_t chan = p->type1.channel;
            uint16_t raw = p->type1.data;
            float mv = adcRawToMillivolts(raw);

            // compute this conversion's ccount timestamp:
            // conversion k has timestamp = base_ccount + k * cycles_per_conv
            uint32_t sample_ccount = base_ccount + (uint32_t)k * cycles_per_conv;

            if (chan == V_CHANNEL) {
                push_v_sample(mv, sample_ccount);
            } else if (chan == I_CHANNEL) {
                push_i_sample(mv, sample_ccount);
            } else {
                // ignore other channels
            }
        }
    }
}

// ---------- main setup/loop, adapted sampling to use timestamped interpolation ----------
void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    cpu_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_hz;

    single_cycle_cycles = (uint32_t) lrintf((float)cpu_hz / f_clamped);
    base_ticks_per_sample = single_cycle_cycles / (uint32_t)SAMPLES_PER_CYCLE;
    ticks_remainder = single_cycle_cycles % (uint32_t)SAMPLES_PER_CYCLE;
    bresenham_acc = 0;
    ticks_per_sample = base_ticks_per_sample;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nSOGI-PLL ADC Continuous (PLL-Paced) with DMA timestamps");

    updateTimingParameters(NOMINAL_FREQ);

    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_cycle_boundary = start_c;

    for (int i=0; i<4; i++) cycle_start_idx[i] = 0;
    current_cycle = 0;
    buf_idx = 0;

    vis.begin();

    if (!initADCContinuous()) {
        Serial.println("ADC init failed!");
        while(1) delay(100);
    }

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);

    Serial.printf("Ready. CPU: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.println("PLL controls timing, DMA provides timestamped data frames");
}

// Main loop: process frames into per-channel buffers and then do PLL-paced sampling
void loop() {
    // First, transfer any frames copied by ISR into per-channel buffers
    process_pending_frames();

    // --- PLL-PACED SAMPLING (timing logic) ---
    uint32_t now = get_cycle_count();

    // Check if we have reached or passed the scheduled time for the next sample
    if ((int32_t)(now - last_sample_cycles) >= 0) {

        // --- DEBUG ACCOUNTING ---
        uint32_t latency = now - last_sample_cycles;
        if (current_cycle < 3 && latency > max_jitter_ticks) {
            max_jitter_ticks = latency;
        }

        if (current_cycle < 3 && latency > (ticks_per_sample + (ticks_per_sample >> 1))) {
            overrun_count++;
            last_sample_cycles = now;
        }
        // -------------------------

        // Sample at PLL-determined time (compute interpolated values at last_sample_cycles)
        bool do_sample = (current_cycle < 3);

        if (do_sample) {
            // requested timestamp at which we want V and I
            uint32_t target_ccount = last_sample_cycles;

            // Interpolate for V and I separately using per-channel buffers
            float v_interp = 0.0f, i_interp = 0.0f;
            bool have_v = false, have_i = false;

            // attempt interpolation (short critical inside function)
            have_v = interpolate_channel_at(target_ccount, v_samp_buf, v_ts_buf, v_count, v_head, v_interp);
            have_i = interpolate_channel_at(target_ccount, i_samp_buf, i_ts_buf, i_count, i_head, i_interp);

            // Fallback: if we couldn't interpolate (not enough samples), use latest known value
            if (!have_v) {
                // pick most recent V sample
                portENTER_CRITICAL(&samples_mux);
                if (v_count > 0) {
                    int idx = (v_head - 1 + BUF_N) % BUF_N;
                    v_interp = v_samp_buf[idx];
                    have_v = true;
                }
                portEXIT_CRITICAL(&samples_mux);
            }
            if (!have_i) {
                portENTER_CRITICAL(&samples_mux);
                if (i_count > 0) {
                    int idx = (i_head - 1 + BUF_N) % BUF_N;
                    i_interp = i_samp_buf[idx];
                    have_i = true;
                }
                portEXIT_CRITICAL(&samples_mux);
            }

            // store into your circular analysis buffers for the SOGI/PLL processing
            if (have_v && have_i) {
                v_samp_buf[buf_idx] = v_interp;
                i_samp_buf[buf_idx] = i_interp;
                buf_idx = (buf_idx + 1) % BUF_N;
            }
        }

        // Increment scheduled time by the CURRENT ticks_per_sample
        last_sample_cycles += ticks_per_sample;
        sample_slot_count++;

        // Bresenham: compute integer step for the NEXT scheduled slot
        uint32_t step = base_ticks_per_sample;
        bresenham_acc += ticks_remainder;
        if (bresenham_acc >= (uint32_t)SAMPLES_PER_CYCLE) {
            bresenham_acc -= (uint32_t)SAMPLES_PER_CYCLE;
            step += 1;
        }
        ticks_per_sample = step;

        // --- TIME-BASED cycle boundary detection ---
        uint32_t elapsed_cycle = now - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            last_cycle_boundary += single_cycle_cycles;

            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;

            if (prev_cycle == 2) {
                // --- Deferred heavy processing ---
                uint32_t proc_start = get_cycle_count();

                sample_slot_count = 0;
                bresenham_acc = 0;

                int s_idx = cycle_start_idx[1];
                int e_idx = cycle_start_idx[2];
                int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;

                if (actual_count > 0) {
                    // DC Estimation
                    float v_sum = 0, i_sum = 0;
                    for (int i = 0; i < actual_count; ++i) {
                        int idx = (s_idx + i) % BUF_N;
                        v_sum += v_samp_buf[idx];
                        i_sum += i_samp_buf[idx];
                    }
                    float v_win_dc = v_sum / (float)actual_count;
                    float i_win_dc = i_sum / (float)actual_count;
                    v_dc_offset = (0.2f * v_win_dc) + (0.8f * v_dc_offset);
                    i_dc_offset = (0.2f * i_win_dc) + (0.8f * i_dc_offset);

                    float ts = (float)ticks_per_sample * inv_cpu_freq;

                    // Process Window with Adaptive PLL
                    sogi_v.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);
                    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts);

                    // Phase Alignment for Visualization
                    SOGI phase_sogi(SOGI_K);
                    phase_sogi.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);

                    float phase = atan2f(phase_sogi.v_alpha, -phase_sogi.v_beta);
                    if (phase < 0) phase += 2.0f * PI;

                    if (phase_track.initialized) {
                        float phase_delta = phase - phase_track.prev_phase;
                        if (phase_delta < -PI) phase_track.phase_offset += 2.0f * PI;
                        else if (phase_delta > PI) phase_track.phase_offset -= 2.0f * PI;
                    } else {
                        phase_track.initialized = true;
                    }
                    phase_track.prev_phase = phase;
                    float unwrapped_phase = phase + phase_track.phase_offset;
                    float phase_for_alignment = fmodf(unwrapped_phase, 2.0f * PI);
                    if (phase_for_alignment < 0) phase_for_alignment += 2.0f * PI;

                    float samples_per_cycle_f = 1.0f / (pll.freq * ts);
                    float samples_back = (phase_for_alignment / (2.0f * PI)) * samples_per_cycle_f;
                    int aligned_start_idx = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
                    int vis_count = (int)(samples_per_cycle_f + 0.5f);

                    updateTimingParameters(pll.freq);

                    if (vis_count > actual_count) vis_count = actual_count;
                    if (vis_count > 0) {
                        vis.update(v_samp_buf, i_samp_buf, BUF_N, aligned_start_idx, vis_count, pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset);
                    }

                    uint32_t proc_end = get_cycle_count();
                    float core_us = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;
                    float max_jitter_us = (float)max_jitter_ticks * inv_cpu_freq * 1e6f;

                    Serial.printf("F:%.4fHz, Mag:%.3f, Vdc:%.1f, Idc:%.1f, Core:%.1fus, Jit:%.1fus, GainEst:%.4f\n",
                                  pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset, core_us,
                                  max_jitter_us, pll.gain_est);

                    yield();
                    yield();
                    yield();
                    yield();
                    max_jitter_ticks = 0;
                }
            }
        }
    }

    // small sleep/yield to let other tasks run; loop is not blocking on ADC
    // do not block too long else ADC pool may overflow — rely on ISR copying frames
    delayMicroseconds(50);
}
