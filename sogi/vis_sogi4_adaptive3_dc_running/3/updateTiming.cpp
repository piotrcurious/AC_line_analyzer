void updateTimingParameters(float frequency) {
    // 1. Constrain and cache CPU speed
    const float f_clamped = constrain(frequency, 40.0f, 60.0f);
    const float cpu_hz = (float)ESP.getCpuFreqMHz() * 1e6f;
    
    // 2. Pre-calculate terms to save cycles later
    sogi.inv_cpu_freq = 1.0f / cpu_hz;
    const uint32_t old_ticks = sogi.ticks_per_sample;
    
    // Use double for the intermediate calculation if high precision is needed
    sogi.ticks_per_sample = (uint32_t)(cpu_hz / (f_clamped * SAMPLES_PER_CYCLE));
    single_cycle_cycles = (uint32_t)(cpu_hz / f_clamped);

    // 3. Phase Continuity Adjustment
    const uint32_t now = get_cycle_count();
    if (old_ticks > 0 && last_sample_cycles != 0) {
        uint32_t elapsed = now - last_sample_cycles;
        
        // Ensure we don't divide by zero or handle weird edge cases
        float progress = (float)elapsed / (float)old_ticks;
        
        // Clamp progress to 1.0 to prevent jumping into the future
        if (progress > 1.0f) progress = 1.0f; 
        
        last_sample_cycles = now - (uint32_t)(progress * (float)sogi.ticks_per_sample);
    }
}
