/*
 * Optimized ESP32 SOGI-PLL - ADC Continuous Mode with Wavelet-EKF Fusion
 * Per-cycle processing for better tracking responsiveness.
 */

#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "SOGI.h"
#include "SOGIvisualizer.h"
#include "phase_estimator.h"

#define ADC_PIN_V 36
#define ADC_PIN_I 39
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f
#define PLL_KP 1.25f
#define PLL_KI 0.000000f
#define SAMPLES_PER_CYCLE 128
#define V_REF 3.3f

#define V_CHANNEL 0
#define I_CHANNEL 3

#define ADC_OVERSAMPLE_RATE 200000
#define CONV_FRAME_SIZE 32

#define FRAME_BUFFER_SIZE 400
#define MAX_SEARCH 8
#define CLEANUP_FRAMES_DIVIDER 200

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);
PhaseEstimator phase_est;

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;
    bool initialized = false;
} phase_track;

struct TimestampedFrame {
    uint32_t end_timestamp;
    uint8_t raw_data[CONV_FRAME_SIZE];
    uint16_t data_size;
    uint8_t sample_count;
};

volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint32_t frame_write_idx = 0;
volatile uint32_t frame_read_idx = 0;
volatile uint32_t isr_callback_count = 0;
volatile uint32_t frames_dropped = 0;

adc_continuous_handle_t adc_handle = NULL;

uint32_t base_ticks_per_sample = 0;
uint32_t ticks_remainder = 0;
uint32_t bresenham_acc = 0;
uint32_t sample_slot_count = 0;

static const float RAW_TO_MV = 3300.0f / 4095.0f;

static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw_value) {
    return (float)raw_value * RAW_TO_MV;
}

static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                             const adc_continuous_evt_data_t *edata,
                                             void *user_data) {
    uint32_t end_timestamp = get_cycle_count();
    uint32_t next_write = (frame_write_idx + 1) % FRAME_BUFFER_SIZE;
    if (next_write == frame_read_idx) {
        frames_dropped++;
        return false;
    }
    uint32_t size = edata->size;
    if (size > CONV_FRAME_SIZE) size = CONV_FRAME_SIZE;
    memcpy((void*)frame_buffer[frame_write_idx].raw_data, edata->conv_frame_buffer, size);
    frame_buffer[frame_write_idx].end_timestamp = end_timestamp;
    frame_buffer[frame_write_idx].data_size = (uint16_t)size;
    frame_buffer[frame_write_idx].sample_count = 0;
    frame_write_idx = next_write;
    isr_callback_count++;
    return false;
}

static inline uint8_t IRAM_ATTR countFrameSamples(TimestampedFrame* frame) {
    if (frame->sample_count) return frame->sample_count;
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)frame->raw_data;
    size_t entries = frame->data_size / sizeof(adc_digi_output_data_t);
    uint8_t v_count = 0, i_count = 0;
    for (size_t n = 0; n < entries; ++n) {
        uint32_t chan = p[n].type1.channel;
        if (chan == V_CHANNEL) ++v_count;
        else if (chan == I_CHANNEL) ++i_count;
    }
    frame->sample_count = (v_count < i_count) ? v_count : i_count;
    return frame->sample_count;
}

static inline bool IRAM_ATTR parseFrameSample(const TimestampedFrame* frame, uint8_t sample_idx, uint16_t &v_raw, uint16_t &i_raw) {
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)frame->raw_data;
    size_t entries = frame->data_size / sizeof(adc_digi_output_data_t);
    uint8_t v_count = 0, i_count = 0;
    bool v_found = false, i_found = false;
    for (size_t n = 0; n < entries; ++n) {
        uint32_t chan = p[n].type1.channel;
        if (chan == V_CHANNEL) {
            if (v_count == sample_idx) { v_raw = p[n].type1.data; v_found = true; if (i_found) return true; }
            ++v_count;
        } else if (chan == I_CHANNEL) {
            if (i_count == sample_idx) { i_raw = p[n].type1.data; i_found = true; if (v_found) return true; }
            ++i_count;
        }
    }
    return (v_found && i_found);
}

uint32_t cpu_freq_hz = 0;
float inv_cpu_freq = 0.0f;
uint32_t cycles_per_adc_sample = 0;
uint32_t single_cycle_cycles = 0;
uint32_t ticks_per_sample = 0;

static inline uint32_t IRAM_ATTR getFrameSampleTimestamp(uint32_t frame_end_time, uint8_t sample_idx, uint8_t sample_count) {
    uint32_t samples_from_end = sample_count - 1 - sample_idx;
    return frame_end_time - (samples_from_end * cycles_per_adc_sample);
}

bool interpolateSampleAtTime(uint32_t target_time, float &v_out, float &i_out) {
    uint32_t read_idx = frame_read_idx;
    uint32_t write_idx = frame_write_idx;
    if (read_idx == write_idx) return false;
    uint32_t available = (write_idx >= read_idx) ? (write_idx - read_idx) : (FRAME_BUFFER_SIZE - read_idx + write_idx);
    uint32_t max_search = (available > MAX_SEARCH) ? MAX_SEARCH : available;
    uint32_t frame_idx = (write_idx + FRAME_BUFFER_SIZE - 1) % FRAME_BUFFER_SIZE;
    uint32_t searched = 0;

    while (searched < max_search) {
        TimestampedFrame* frame = (TimestampedFrame*)&frame_buffer[frame_idx];
        if (frame->data_size) {
            uint8_t sample_count = countFrameSamples(frame);
            if (sample_count) {
                uint32_t frame_end = frame->end_timestamp;
                uint32_t frame_start = getFrameSampleTimestamp(frame_end, 0, sample_count);
                int32_t delta_start = (int32_t)(target_time - frame_start);
                int32_t delta_end   = (int32_t)(target_time - frame_end);

                if (delta_start >= 0 && delta_end <= 0) {
                    uint32_t time_into_frame = target_time - frame_start;
                    float sample_idx_f = (float)time_into_frame / (float)cycles_per_adc_sample;
                    uint8_t idx_before = (uint8_t)sample_idx_f;
                    if (idx_before >= (uint8_t)(sample_count - 1)) {
                        idx_before = sample_count - 1;
                        uint16_t v_raw, i_raw;
                        if (parseFrameSample(frame, idx_before, v_raw, i_raw)) {
                            v_out = adcRawToMillivolts(v_raw); i_out = adcRawToMillivolts(i_raw); return true;
                        }
                    } else {
                        uint8_t idx_after = idx_before + 1;
                        float alpha = sample_idx_f - (float)idx_before;
                        uint16_t v_b, i_b, v_a, i_a;
                        if (parseFrameSample(frame, idx_before, v_b, i_b) && parseFrameSample(frame, idx_after, v_a, i_a)) {
                            float v_before = adcRawToMillivolts(v_b), v_after = adcRawToMillivolts(v_a);
                            float i_before = adcRawToMillivolts(i_b), i_after = adcRawToMillivolts(i_a);
                            v_out = v_before + alpha * (v_after - v_before); i_out = i_before + alpha * (i_after - i_before); return true;
                        }
                    }
                }
                if (delta_end > 0) {
                    uint16_t v_raw, i_raw;
                    if (parseFrameSample(frame, sample_count - 1, v_raw, i_raw)) {
                        v_out = adcRawToMillivolts(v_raw); i_out = adcRawToMillivolts(i_raw); return true;
                    }
                }
            }
        }
        frame_idx = (frame_idx + FRAME_BUFFER_SIZE - 1) % FRAME_BUFFER_SIZE;
        ++searched;
    }
    return false;
}

void cleanupOldFrames(uint32_t current_time) {
    uint32_t keep_duration_cycles = cpu_freq_hz / CLEANUP_FRAMES_DIVIDER;
    while (frame_read_idx != frame_write_idx) {
        uint32_t frame_time = frame_buffer[frame_read_idx].end_timestamp;
        int32_t age = (int32_t)(current_time - frame_time);
        if (age < (int32_t)keep_duration_cycles) break;
        frame_read_idx = (frame_read_idx + 1) % FRAME_BUFFER_SIZE;
    }
}

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    single_cycle_cycles = (uint32_t) lrintf((float)cpu_freq_hz / f_clamped);
    base_ticks_per_sample = single_cycle_cycles / (uint32_t)SAMPLES_PER_CYCLE;
    ticks_remainder = single_cycle_cycles % (uint32_t)SAMPLES_PER_CYCLE;
    bresenham_acc = 0;
    ticks_per_sample = base_ticks_per_sample;
}

bool initADCContinuous() {
    adc_continuous_handle_cfg_t adc_config = { .max_store_buf_size = 4096, .conv_frame_size = CONV_FRAME_SIZE };
    if (adc_continuous_new_handle(&adc_config, &adc_handle) != ESP_OK) return false;
    adc_digi_pattern_config_t adc_pattern[2];
    adc_pattern[0].atten = ADC_ATTEN_DB_12; adc_pattern[0].channel = V_CHANNEL; adc_pattern[0].unit = ADC_UNIT_1; adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    adc_pattern[1].atten = ADC_ATTEN_DB_12; adc_pattern[1].channel = I_CHANNEL; adc_pattern[1].unit = ADC_UNIT_1; adc_pattern[1].bit_width = ADC_BITWIDTH_12;
    adc_continuous_config_t dig_cfg = { .pattern_num = 2, .adc_pattern = adc_pattern, .sample_freq_hz = ADC_OVERSAMPLE_RATE, .conv_mode = ADC_CONV_SINGLE_UNIT_1, .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1 };
    if (adc_continuous_config(adc_handle, &dig_cfg) != ESP_OK) return false;
    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_callback };
    if (adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL) != ESP_OK) return false;
    if (adc_continuous_start(adc_handle) != ESP_OK) return false;
    return true;
}

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};
const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float v_samp_buf[BUF_N], i_samp_buf[BUF_N];
int buf_idx = 0;

float v_dc_offset = 1650.0f, i_dc_offset = 1650.0f;
uint32_t last_sample_cycles = 0, last_cycle_boundary = 0;
uint32_t interpolation_count = 0, nearest_sample_count = 0, max_jitter_ticks = 0, overrun_count = 0;

float pll_phi_dev = 0.0f;
float phi_dev_history[4] = {0,0,0,0};

void setup() {
    Serial.begin(115200);
    cpu_freq_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);
    updateTimingParameters(NOMINAL_FREQ);
    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c; last_cycle_boundary = start_c;
    vis.begin();

    PhaseEstConfig pe_config;
    pe_config.history_depth = PE_HISTORY_DEPTH;
    pe_config.correction_threshold_rad = 0.3f;
    pe_config.nonlinear_threshold_rad = 0.1f;
    pe_config.stable_tolerance_rad = 0.02f;
    phase_est.begin(&pe_config);
    // strobe_cycles = 1.0f as we update every cycle
    phase_est.set_frequency_params(NOMINAL_FREQ, 1.0f/NOMINAL_FREQ, SAMPLES_PER_CYCLE, 1.0f, cpu_freq_hz);

    initADCContinuous();
    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

void loop() {
    static uint32_t cleanup_counter = 0;
    uint32_t now = get_cycle_count();
    if (++cleanup_counter >= 50) { cleanupOldFrames(now); cleanup_counter = 0; }

    while ((int32_t)(now - last_sample_cycles) >= 0) {
        float ts = (float)ticks_per_sample * inv_cpu_freq;

        pll_phi_dev += 2.0f * PI * (pll.freq - NOMINAL_FREQ) * ts;
        while (pll_phi_dev > PI) pll_phi_dev -= 2.0f * PI;
        while (pll_phi_dev < -PI) pll_phi_dev += 2.0f * PI;

        uint32_t latency = now - last_sample_cycles;
        if (latency > max_jitter_ticks) max_jitter_ticks = latency;
        if (latency > (ticks_per_sample + (ticks_per_sample >> 1))) overrun_count++;

        float v_interp = 0.0f, i_interp = 0.0f;
        if (interpolateSampleAtTime(last_sample_cycles, v_interp, i_interp)) {
            v_samp_buf[buf_idx] = v_interp; i_samp_buf[buf_idx] = i_interp; interpolation_count++;
        } else {
            v_samp_buf[buf_idx] = 0.0f; i_samp_buf[buf_idx] = 0.0f; nearest_sample_count++;
        }
        buf_idx = (buf_idx + 1) % BUF_N;

        last_sample_cycles += ticks_per_sample;
        sample_slot_count++;

        uint32_t step = base_ticks_per_sample;
        bresenham_acc += ticks_remainder;
        if (bresenham_acc >= (uint32_t)SAMPLES_PER_CYCLE) { bresenham_acc -= (uint32_t)SAMPLES_PER_CYCLE; step += 1; }
        ticks_per_sample = step;

        uint32_t elapsed_cycle = now - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            last_cycle_boundary += single_cycle_cycles;
            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;
            phi_dev_history[current_cycle] = pll_phi_dev;

            // Process EVERY cycle for better tracking
            uint32_t proc_start = get_cycle_count();

            // The cycle that just finished is Cycle 'prev_cycle'
            int s_idx = cycle_start_idx[prev_cycle];
            int e_idx = buf_idx;
            int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;

            if (actual_count > 0) {
                float v_sum = 0, i_sum = 0;
                for (int i = 0; i < actual_count; ++i) {
                    int id = (s_idx + i) % BUF_N;
                    v_sum += v_samp_buf[id]; i_sum += i_samp_buf[id];
                }
                v_dc_offset = (0.2f * (v_sum / (float)actual_count)) + (0.8f * v_dc_offset);
                i_dc_offset = (0.2f * (i_sum / (float)actual_count)) + (0.8f * i_dc_offset);

                float ts_p = (float)ticks_per_sample * inv_cpu_freq;
                sogi_v.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts_p, v_dc_offset);

                // Wavelet Buffer: Use last 3 cycles
                static float pe_buf[PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER];
                int pe_start = cycle_start_idx[(current_cycle + 1) % 4];
                for (int i = 0; i < PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER; i++) {
                    pe_buf[i] = v_samp_buf[(pe_start + i) % BUF_N] - v_dc_offset;
                }
                phase_est.add_frame(pe_buf, PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER, (float)latency * inv_cpu_freq * 2*PI*50, pll.freq, last_cycle_boundary);

                PhaseEstResult pe_res;
                float synced_phi_dev = phi_dev_history[current_cycle];

                if (phase_est.estimate_phase(pe_res)) {
                    float confidence = (pe_res.state == PE_STABLE) ? 1.0f : 0.5f;
                    pll.updateFused(sogi_v.v_alpha, sogi_v.v_beta, pe_res.absolute_phase, pe_res.linear_drift_rate, confidence, synced_phi_dev, ts_p * actual_count);
                } else {
                    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts_p * actual_count);
                }

                updateTimingParameters(pll.freq);

                // Update visualizer and Serial less frequently (every 4 cycles)
                if (current_cycle == 0) {
                    SOGI phase_sogi(SOGI_K);
                    phase_sogi.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts_p, v_dc_offset);
                    float phase = atan2f(phase_sogi.v_alpha, -phase_sogi.v_beta);
                    if (phase < 0) phase += 2.0f * PI;
                    if (phase_track.initialized) {
                        float phase_delta = phase - phase_track.prev_phase;
                        if (phase_delta < -PI) phase_track.phase_offset += 2.0f * PI;
                        else if (phase_delta > PI) phase_track.phase_offset -= 2.0f * PI;
                    } else { phase_track.initialized = true; }
                    phase_track.prev_phase = phase;
                    float unwrapped_phase = phase + phase_track.phase_offset;
                    float phase_for_alignment = fmodf(unwrapped_phase, 2.0f * PI);
                    if (phase_for_alignment < 0) phase_for_alignment += 2.0f * PI;
                    float samples_per_cycle_f = 1.0f / (pll.freq * ts_p);
                    float samples_back = (phase_for_alignment / (2.0f * PI)) * samples_per_cycle_f;
                    int aligned_start_idx = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
                    int vis_count = (int)(samples_per_cycle_f + 0.5f);
                    if (vis_count > 0) {
                        vis.update(v_samp_buf, i_samp_buf, BUF_N, aligned_start_idx, vis_count, pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset);
                    }
                    float core_us = (float)(get_cycle_count() - proc_start) * inv_cpu_freq * 1e6f;
                    Serial.printf("F:%.3fHz, Mag:%.2f, Vdc:%.1f, PE:%d, Core:%.1fus, Jit:%.1fus, Over:%lu, Interp:%lu/%lu\n",
                        pll.freq, pll.mag_smooth, v_dc_offset, phase_est.get_state(), core_us,
                        (float)max_jitter_ticks*inv_cpu_freq*1e6f, overrun_count, interpolation_count, interpolation_count+nearest_sample_count);
                    max_jitter_ticks = 0; overrun_count = 0; interpolation_count = 0; nearest_sample_count = 0;
                }
            }
        }
        now = get_cycle_count();
    }
}
