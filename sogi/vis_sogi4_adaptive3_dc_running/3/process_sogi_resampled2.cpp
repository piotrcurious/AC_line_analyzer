// Assumptions:
//  - BUF_N, ts_buf[], samp_buf[] and 'sogi' (with ticks_per_sample) are available.
//  - timestamps in ts_buf are monotonically increasing in buffer order (circularly).
//  - intervals between adjacent timestamps are << 2^31 (so unsigned diff logic is safe).

InterpResult get_interpolated_sample(uint32_t target_time, int start_idx, int count) {
    if (count <= 0) return {0.0f, 0.0f};

    // --- persistent scan state (keeps O(N) over a batch) ---
    static int local_scan_idx = 0;
    static int prev_start_idx = -1;

    // Reset scan if start index changed (start of new batch)
    if (start_idx != prev_start_idx) {
        local_scan_idx = 0;
        prev_start_idx = start_idx;
    }
    // Bound local scan index so it doesn't drift outside the current batch
    if (local_scan_idx >= count) local_scan_idx = 0;

    int idx_prev = (start_idx + local_scan_idx) % BUF_N;
    int idx_next = (idx_prev + 1) % BUF_N;
    int scan_limit = count;

    bool bracket_found = false;
    uint32_t t_prev = 0, t_next = 0;
    // Forward scan to find [t_prev, t_next] that contains target_time
    while (scan_limit > 0) {
        t_prev = ts_buf[idx_prev];
        t_next = ts_buf[idx_next];

        // unsigned diffs — works correctly across 32-bit wrap for small intervals
        uint32_t dt_meas = t_next - t_prev;
        uint32_t dt_targ = target_time - t_prev;

        // If target_time is between t_prev and t_next (inclusive), dt_targ <= dt_meas
        if (dt_targ <= dt_meas) {
            bracket_found = true;
            break;
        }

        // advance window
        idx_prev = idx_next;
        idx_next = (idx_next + 1) % BUF_N;
        local_scan_idx++;
        scan_limit--;
    }

    // If we didn't find a bracket, return nearest sample with low confidence
    if (!bracket_found) {
        // use idx_prev as the nearest-known sample
        float val = samp_buf[idx_prev];
        // very low confidence because we couldn't bracket the target
        return { val, 0.0f };
    }

    // Read sample values
    float y_prev = samp_buf[idx_prev];
    float y_next = samp_buf[idx_next];

    // Unsigned diffs (safe for wrap as long as real gaps << 2^31)
    uint32_t dt_meas_u = t_next - t_prev;
    uint32_t dt_targ_u = target_time - t_prev;

    // If the measured gap is zero (duplicate timestamps) we cannot interpolate
    if (dt_meas_u == 0) {
        // choose previous sample and low confidence
        return { y_prev, 0.0f };
    }

    // Linear interpolation ratio (0..1)
    float ratio = (float)dt_targ_u / (float)dt_meas_u;
    float val = y_prev + ratio * (y_next - y_prev);

    // Confidence based on measured gap vs ideal ticks_per_sample
    float gap_ratio = (float)dt_meas_u / (float)sogi.ticks_per_sample;
    float conf;
    if (gap_ratio <= 1.5f) {
        conf = 1.0f;
    } else {
        conf = 1.5f / gap_ratio;
        if (conf < 0.0f) conf = 0.0f;
        if (conf > 1.0f) conf = 1.0f;
    }

    return { val, conf };
}
