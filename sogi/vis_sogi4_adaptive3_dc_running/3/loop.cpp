void loop() {
    const uint32_t now = get_cycle_count();

    // --- 1. Robust Sampling Hook ---
    // We use a 'while' to catch up if a heavy calculation (like SOGI) 
    // caused us to miss a sample window.
    while ((now - last_sample_cycles) >= sogi.ticks_per_sample) {
        // Increment by fixed ticks to maintain long-term phase-lock
        last_sample_cycles += sogi.ticks_per_sample;

        if (current_cycle < 3) {
            int raw = analogRead(ADC_PIN);
            float raw_f = (float)raw;

            // Update DC tracker (EMA Filter)
            dc_current = (DC_ALPHA * raw_f) + ((1.0f - DC_ALPHA) * dc_current);

            // Subtract the offset captured at the start of this cycle-set
            float u = (raw_f - dc_offset_sampling) * (V_REF / 4095.0f);

            // Store in circular buffer
            samp_buf[buf_idx] = u;
            ts_buf[buf_idx] = now;
            buf_idx = (buf_idx + 1) % BUF_N;
        }
        
        // Safety: prevent infinite loop if ticks_per_sample is somehow 0
        if (sogi.ticks_per_sample == 0) break; 
    }

    // --- 2. Cycle Boundary & Processing ---
    uint32_t elapsed_cycle = now - last_cycle_boundary;
    if (elapsed_cycle >= single_cycle_cycles) {
        // Maintain timing alignment
        last_cycle_boundary += single_cycle_cycles;

        int prev_cycle = current_cycle;
        current_cycle = (current_cycle + 1) % 4;
        cycle_start_idx[current_cycle] = buf_idx;

        // Logic executes at the end of the 3rd cycle (index 2)
        if (prev_cycle == 2) {
            process_cycle_data();
        }
    }
}

/**
 * Encapsulated processing logic to keep loop() clean
 */
void process_cycle_data() {
    int s_idx = cycle_start_idx[1];
    int e_idx = cycle_start_idx[2];
    int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;

    if (actual_count <= 0) return;

    // Snapshot the DC current for the NEXT sampling window 
    // to ensure the integrator sees a stable baseline.
    dc_offset_sampling = dc_current;

    // Run SOGI processing
    process_sogi_window(s_idx, actual_count);
    do_strobe_computation();

    // --- Phase Alignment for Visualizer ---
    // atan2f returns [-PI, PI]. We want to align to the negative zero-crossing.
    float end_phase = atan2f(sogi.v_beta, sogi.v_alpha);
    const float target_phase = -1.570796f; // -PI / 2
    
    float phase_diff = end_phase - target_phase;
    
    // Normalized wrap-around [0, 2PI]
    if (phase_diff < 0) phase_diff += 6.283185f;
    if (phase_diff >= 6.283185f) phase_diff -= 6.283185f;

    // Calculate how many samples back the trigger point occurred
    // We use actual_count/SAMPLES_PER_CYCLE ratio to adjust for freq drift
    float samples_per_rad = (float)actual_count / 6.283185f;
    int samples_back = (int)(phase_diff * samples_per_rad);

    int processed_end_idx = (e_idx - 1 + BUF_N) % BUF_N;
    int aligned_start_idx = (processed_end_idx - samples_back + BUF_N) % BUF_N;

    // Update Visualizer
    vis.update(samp_buf, BUF_N, aligned_start_idx, SAMPLES_PER_CYCLE, 
               sogi.freq, sogi.mag_smooth, sogi.filtered_err);

    // Production Telemetry
    Serial.printf("F:%.4fHz | Mag:%.3f | Offset:%.1f\n", 
                  sogi.freq, sogi.mag_smooth, dc_offset_sampling);
}
