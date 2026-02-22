/*
 * Optimized ESP32 SOGI-PLL - ADC Continuous Mode with Frame Timestamping
 * Integrated Wavelet-EKF Fusion for robust tracking
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

// Timing parameters
uint32_t base_ticks_per_sample = 0;
uint32_t ticks_remainder = 0;
uint32_t bresenham_acc = 0;

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
                if (delta_end <= 0) {
                    uint16_t v_raw, i_raw;
                    if (parseFrameSample(frame, 0, v_raw, i_raw)) {
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
uint32_t nominal_sample_cycles = 0;
uint32_t interpolation_count = 0, nearest_sample_count = 0, max_jitter_ticks = 0;

uint32_t nominal_ticks_per_sample = 0;
float wavelet_v_circ_buf[PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER * 2];
int wavelet_circ_idx = 0;

void setup() {
    Serial.begin(115200);
    cpu_freq_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);
    updateTimingParameters(NOMINAL_FREQ);
    nominal_ticks_per_sample = cpu_freq_hz / (NOMINAL_FREQ * SAMPLES_PER_CYCLE);

    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c; last_cycle_boundary = start_c;
    nominal_sample_cycles = start_c;
    vis.begin();
    PhaseEstConfig pe_config; pe_config.history_depth = PE_HISTORY_DEPTH; pe_config.correction_threshold_rad = 0.3f; pe_config.nonlinear_threshold_rad = 0.1f; pe_config.stable_tolerance_rad = 0.02f;
    phase_est.begin(&pe_config);
    phase_est.set_frequency_params(NOMINAL_FREQ, 1.0f/NOMINAL_FREQ, SAMPLES_PER_CYCLE, 1.0f, cpu_freq_hz);
    initADCContinuous();
    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

void loop() {
    static uint32_t cleanup_counter = 0;
    uint32_t now = get_cycle_count();
    if (++cleanup_counter >= 50) { cleanupOldFrames(now); cleanup_counter = 0; }

    if ((int32_t)(now - last_sample_cycles) >= 0) {
        float ts = (float)ticks_per_sample * inv_cpu_freq;
        float v_interp = 0.0f, i_interp = 0.0f;

        if (interpolateSampleAtTime(last_sample_cycles, v_interp, i_interp)) {
            v_samp_buf[buf_idx] = v_interp; i_samp_buf[buf_idx] = i_interp; interpolation_count++;
        } else {
            v_samp_buf[buf_idx] = v_dc_offset; i_samp_buf[buf_idx] = i_dc_offset; nearest_sample_count++;
        }

        // --- Continuous EKF Path ---
        sogi_v.step(v_samp_buf[buf_idx] - v_dc_offset, pll.omega, ts);
        pll.predict(ts);
        pll.updateSOGI(sogi_v.v_alpha, sogi_v.v_beta, ts);

        buf_idx = (buf_idx + 1) % BUF_N;
        last_sample_cycles += ticks_per_sample;

        // --- Uniform Wavelet Capture ---
        while ((int32_t)(now - nominal_sample_cycles) >= 0) {
            float vw = 0.0f, iw = 0.0f;
            if (interpolateSampleAtTime(nominal_sample_cycles, vw, iw)) {
                wavelet_v_circ_buf[wavelet_circ_idx] = vw - v_dc_offset;
                wavelet_circ_idx = (wavelet_circ_idx + 1) % (PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER * 2);
            }
            nominal_sample_cycles += nominal_ticks_per_sample;
        }

        uint32_t step = base_ticks_per_sample;
        bresenham_acc += ticks_remainder;
        if (bresenham_acc >= (uint32_t)SAMPLES_PER_CYCLE) { bresenham_acc -= (uint32_t)SAMPLES_PER_CYCLE; step += 1; }
        ticks_per_sample = step;

        uint32_t elapsed_cycle = now - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            int32_t jitter_cycles = (int32_t)(elapsed_cycle - single_cycle_cycles);
            last_cycle_boundary += single_cycle_cycles;
            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;

            // Feed Wavelet with UNIFORM samples
            static float pe_buf[PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER];
            int pe_len = PE_SAMPLES_PER_CYCLE * PE_CYCLES_PER_BUFFER;
            for (int i = 0; i < pe_len; i++) {
                int id = (wavelet_circ_idx - pe_len + i + pe_len*2) % (pe_len*2);
                pe_buf[i] = wavelet_v_circ_buf[id];
            }
            // Since samples are at nominal rate, we can pass NOMINAL_FREQ
            // Capture jitter is now relative to nominal_sample_cycles grid.
            int32_t jit_cycles = (int32_t)(now - (nominal_sample_cycles - nominal_ticks_per_sample));
            float jitter_rad = (2.0f * PI * NOMINAL_FREQ * (float)jit_cycles) * inv_cpu_freq;
            phase_est.add_frame(pe_buf, pe_len, jitter_rad, NOMINAL_FREQ, nominal_sample_cycles);

            if (prev_cycle == 2) {
                uint32_t proc_start = get_cycle_count();
                int s_idx = cycle_start_idx[1], e_idx = cycle_start_idx[2];
                int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;
                if (actual_count > 0) {
                    float v_sum = 0; for (int i = 0; i < actual_count; ++i) v_sum += v_samp_buf[(s_idx + i) % BUF_N];
                    v_dc_offset = (0.1f * (v_sum / (float)actual_count)) + (0.9f * v_dc_offset);

                    PhaseEstResult pe_res;
                    if (phase_est.estimate_phase(pe_res) && (pe_res.state == PE_STABLE || pe_res.state == PE_READY)) {
                        float confidence = (pe_res.state == PE_STABLE) ? 0.9f : 0.6f;
                        // Samples are now at nominal rate, so linear_drift_rate is already deviation from nominal.
                        // x_omega = omega_sig - omega_nom = (2pi*50 + drift_rad_cyc*50) - 2pi*50 = drift_rad_cyc*50
                        float drift_rad_s = pe_res.linear_drift_rate * NOMINAL_FREQ;
                        pll.updateWavelet(pe_res.absolute_phase, drift_rad_s, confidence, (float)actual_count * ts);

                        float err_phase = pll.getPhaseError();
                        if (fabs(err_phase) > 0.01f && confidence > 0.8f) {
                            float shift_rad = err_phase * 0.5f;
                            int32_t shift_ticks = (int32_t)lrintf((shift_rad / (2.0f * PI)) * ((float)cpu_freq_hz / pll.freq));
                            last_sample_cycles += shift_ticks;
                            last_cycle_boundary += shift_ticks;
                            // Update EKF's internal model of sampling phase to reflect the hardware jump
                            pll.sampling_phi += shift_rad;
                            while (pll.sampling_phi > PI) pll.sampling_phi -= TWO_PI;
                            while (pll.sampling_phi < -PI) pll.sampling_phi += TWO_PI;

                            phase_est.notify_correction_applied(shift_rad);
                        }
                    }
                    updateTimingParameters(pll.freq);

                    float err_phi = pll.getPhaseError();
                    float samples_per_cycle_f = 1.0f / (pll.freq * ts);
                    float samples_back = (err_phi / (2.0f * PI)) * samples_per_cycle_f;
                    int vis_start = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
                    vis.update(v_samp_buf, i_samp_buf, BUF_N, vis_start, (int)samples_per_cycle_f, pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset);

                    Serial.printf("F:%.3fHz, Mag:%.2f, Vdc:%.1f, Core:%.0fus, PE:%d, Interp:%d/%d\n", pll.freq, pll.mag_smooth, v_dc_offset, (float)(get_cycle_count()-proc_start)*inv_cpu_freq*1e6f, phase_est.get_state(), interpolation_count, interpolation_count+nearest_sample_count);
                    interpolation_count = 0; nearest_sample_count = 0;
                }
            }
        }
    }
}
