// Helper struct to return interpolated results
struct InterpResult {
    float value;
    float confidence; // 0.0 to 1.0 (1.0 = perfect timing, 0.0 = huge data gap)
};

/**
 * Finds the two real samples surrounding 'target_time' and interpolates.
 * Handles circular buffer wrapping automatically.
 */
InterpResult get_interpolated_sample(uint32_t target_time, int start_idx, int count) {
    // 1. Find the interval [idx_prev, idx_next] that contains target_time
    // We scan forward from the last known position to keep it efficient (O(N) total)
    static int local_scan_idx = 0; 
    
    // Reset scan if we are starting a new batch
    // (In a real class, 'local_scan_idx' would be a member variable reset at start of process_cycle)
    if (target_time == ts_buf[start_idx]) local_scan_idx = 0;

    int idx_prev = (start_idx + local_scan_idx) % BUF_N;
    int idx_next = (idx_prev + 1) % BUF_N;
    
    // Safety limit to prevent infinite loops if target_time is way out of bounds
    int scan_limit = count; 
    
    // Walk forward until we bracket the target_time
    // We use (int32_t) casting to handle timer rollover correctly
    while (scan_limit > 0) {
        uint32_t t_prev = ts_buf[idx_prev];
        uint32_t t_next = ts_buf[idx_next];
        
        // Check if target is between prev and next
        // Note: Logic handles rollover if we assume (t_next - t_prev) is small positive
        if ((int32_t)(target_time - t_prev) >= 0 && (int32_t)(t_next - target_time) >= 0) {
            break; 
        }
        
        idx_prev = idx_next;
        idx_next = (idx_next + 1) % BUF_N;
        local_scan_idx++;
        scan_limit--;
    }

    // 2. Calculate Interpolation & Confidence
    uint32_t t_prev = ts_buf[idx_prev];
    uint32_t t_next = ts_buf[idx_next];
    float y_prev = samp_buf[idx_prev];
    float y_next = samp_buf[idx_next];

    int32_t dt_meas = t_next - t_prev; // The actual gap between ADC reads
    int32_t dt_targ = target_time - t_prev;
    
    // Sanity check for div/0
    if (dt_meas <= 0) return {y_prev, 1.0f};

    // Linear Interpolation
    float ratio = (float)dt_targ / (float)dt_meas;
    float val = y_prev + (ratio * (y_next - y_prev));

    // 3. Calculate Confidence
    // If the actual gap (dt_meas) is close to the ideal ticks_per_sample, confidence is high.
    // If the CPU hiccuped and we have a huge gap, confidence drops.
    float gap_ratio = (float)dt_meas / (float)sogi.ticks_per_sample;
    
    // Example Weighting Function:
    // Gap <= 1.5x ideal -> 100% confidence
    // Gap > 1.5x ideal  -> Confidence decays
    float conf = 1.0f;
    if (gap_ratio > 1.5f) {
        conf = 1.5f / gap_ratio; // e.g. if gap is 3x larger, trust it half as much
        if (conf < 0.0f) conf = 0.0f;
    }

    return {val, conf};
}

void process_sogi_resampled(int start_idx, int count) {
    if (count <= 1) return;

    // --- Pre-calc Coefficients (Fixed Time Base) ---
    // We calculate these ONCE based on the IDEAL sampling rate.
    // This provides the stable filter poles we need.
    float ts = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;
    float os = sogi.omega;
    float k = SOGI_K; // e.g., 1.414

    float wts = os * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    float a_b0 = 2.0f * k_wts * det;
    float a_b2 = -2.0f * k_wts * det;
    float a_a1 = 2.0f * (wts2 - 4.0f) * det;
    float a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;

    float b_b0 = k * wts2 * det;
    float b_b1 = 2.0f * b_b0;
    float b_b2 = b_b0;

    // --- Time Reconstruction Loop ---
    // Start exactly at the timestamp of the first sample
    uint32_t virtual_time = ts_buf[start_idx];
    
    // We assume the buffer covers roughly 'count' samples worth of time
    // But we iterate based on the IDEAL grid, not the buffer index.
    for (int i = 0; i < count; ++i) {
        
        // 1. Get Resampled Data with Confidence
        InterpResult meas = get_interpolated_sample(virtual_time, start_idx, count);
        
        // 2. Apply Confidence Weighting (The "Kalman-Lite" Step)
        // If confidence is 1.0, we feed the measured 'u' directly.
        // If confidence is 0.0, we feed the SOGI's own output (v_alpha) back into it.
        // This makes the error term = 0, causing the SOGI to "coast" (resonate).
        float u_weighted = (meas.value * meas.confidence) + 
                           (sogi.v_alpha * (1.0f - meas.confidence));

        // 3. SOGI Update (Standard DFII with Weighted Input)
        // Alpha Update
        float in_a = u_weighted - (a_a1 * sogi.wz1_a) - (a_a2 * sogi.wz2_a);
        sogi.v_alpha = (a_b0 * in_a) + (a_b2 * sogi.wz2_a); 
        sogi.wz2_a = sogi.wz1_a;
        sogi.wz1_a = in_a;

        // Beta Update
        float in_b = u_weighted - (a_a1 * sogi.wz1_b) - (a_a2 * sogi.wz2_b);
        sogi.v_beta = (b_b0 * in_b) + (b_b1 * sogi.wz1_b) + (b_b2 * sogi.wz2_b);
        sogi.wz2_b = sogi.wz1_b;
        sogi.wz1_b = in_b;

        // 4. Advance Virtual Time
        virtual_time += sogi.ticks_per_sample;
    }
}
