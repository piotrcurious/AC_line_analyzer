# SOGI-PLL Grid Synchronization System
## Production Documentation v2.0

---

## Table of Contents
1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Algorithm Explanation](#algorithm-explanation)
4. [Configuration Guide](#configuration-guide)
5. [Tuning Parameters](#tuning-parameters)
6. [Troubleshooting](#troubleshooting)
7. [Performance Metrics](#performance-metrics)
8. [Safety & Fault Handling](#safety--fault-handling)

---

## Overview

### What is SOGI-PLL?

**SOGI-PLL** (Second Order Generalized Integrator - Phase Locked Loop) is a digital control algorithm used to:
- Synchronize with single-phase AC grid signals
- Extract phase angle and frequency information
- Generate orthogonal (90° shifted) signal components
- Reject harmonics and noise

### Key Features of This Implementation

✅ **Production-Ready**
- Thread-safe data sharing with mutexes
- Comprehensive error handling
- Watchdog timer integration
- Stack overflow monitoring

✅ **Robust**
- DC offset removal with adaptive filtering
- Input validation and saturation detection
- Anti-windup PI controller
- Frequency clamping for safety

✅ **Well-Documented**
- Inline comments explaining each step
- Named constants instead of magic numbers
- Diagnostic outputs for debugging

---

## System Architecture

### Hardware Components

```
┌─────────────┐
│  Grid       │
│  Signal     │ ──► ADC (GPIO36) ──┐
│  50Hz AC    │                    │
└─────────────┘                    │
                                   │
┌─────────────────────────────────▼──────────────────────────┐
│                    ESP32 SOGI-PLL                          │
│                                                             │
│  ┌──────────────┐    ┌────────────────┐                   │
│  │   Timer ISR  │───►│  FreeRTOS Task │                   │
│  │   10 kHz     │    │  (Core 1)      │                   │
│  └──────────────┘    └────────┬───────┘                   │
│                               │                            │
│                               │                            │
│  ┌────────────────────────────▼─────────────────────┐     │
│  │  SOGI-PLL Algorithm                              │     │
│  │  1. DC Offset Removal                            │     │
│  │  2. SOGI Filter (α-β transformation)             │     │
│  │  3. Phase Detector (Park transform)              │     │
│  │  4. PI Controller                                │     │
│  │  5. Phase Integrator (VCO)                       │     │
│  └───────────────────────────┬──────────────────────┘     │
│                               │                            │
└───────────────────────────────┼────────────────────────────┘
                                │
                                ▼
                         DAC (GPIO25) ──► Synchronized
                                          Output Signal
```

### Software Architecture

```
┌─────────────────┐
│  Hardware Timer │  @ 10 kHz
│  (ISR)          │
└────────┬────────┘
         │ Semaphore
         ▼
┌─────────────────┐
│  SOGI Task      │  Priority 24, Core 1
│  (FreeRTOS)     │
└────────┬────────┘
         │ Mutex
         ▼
┌─────────────────┐
│  Shared Data    │  Thread-safe access
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Loop()         │  Serial monitoring
│  (Core 0)       │
└─────────────────┘
```

---

## Algorithm Explanation

### Step-by-Step Processing

#### 1. **DC Offset Removal**

**Problem:** Real-world signals have DC bias that must be removed.

**Solution:** Exponential moving average (EMA) filter
```
dc_offset = dc_offset × (1 - α) + v_raw × α
v_clean = v_raw - dc_offset
```

**Parameters:**
- α = 0.001 (time constant τ ≈ 1 second)
- Tracks slow drift without affecting AC component

---

#### 2. **SOGI Filter**

**Purpose:** Generate orthogonal (quadrature) signal components

**Equations:**
```
v̇_α = ω × v_β + k × ω × (v_in - v_α)
v̇_β = -ω × v_α
```

**Why Orthogonal?**
- v_α tracks input signal
- v_β is 90° phase-shifted version
- Together they form a "rotating phasor" representation

**Damping Factor (k):**
- k = √2 (1.414) provides critical damping
- Balances response speed vs. overshoot

**⚠️ CRITICAL:** Update both states simultaneously to prevent coupling:
```cpp
// WRONG - causes numerical instability
s_v_alpha += delta_alpha;
s_v_beta += -omega * s_v_alpha;  // Uses NEW alpha!

// CORRECT - independent updates
float new_alpha = s_v_alpha + delta_alpha;
float new_beta = s_v_beta + delta_beta;
s_v_alpha = new_alpha;
s_v_beta = new_beta;
```

---

#### 3. **Phase Detector (Park Transform)**

**Purpose:** Convert to rotating reference frame (dq-frame)

**Transformation:**
```
v_d = v_α × cos(θ) + v_β × sin(θ)
v_q = -v_α × sin(θ) + v_β × cos(θ)
```

**When Locked:**
- v_q → 0 (no quadrature error)
- v_d → amplitude (DC component)

**Error Signal:**
- v_q represents phase/frequency error
- Feed to PI controller to correct

---

#### 4. **PI Controller (Loop Filter)**

**Purpose:** Drive v_q to zero by adjusting ω

**Equations:**
```
integrator += Ki × Ts × v_q
ω = integrator + Kp × v_q
```

**Anti-Windup:**
- Clamp integrator to [ω_min, ω_max]
- Prevents saturation during large errors

**Tuning Guidelines:**
- **Kp** (proportional): Fast response, but can cause overshoot
  - Typical range: 1.0 - 5.0
  - Higher → faster but less stable
  
- **Ki** (integral): Eliminates steady-state error
  - Typical range: 10.0 - 30.0
  - Higher → faster convergence but potential oscillation

---

#### 5. **Phase Integrator (VCO)**

**Purpose:** Generate phase angle from frequency

**Equation:**
```
θ += ω × Ts
θ = θ mod 2π
```

**Normalization:**
- Use `fmodf()` to prevent floating-point drift
- Keeps θ ∈ [0, 2π) for numerical stability

---

#### 6. **Output Generation**

**Purpose:** Create synchronized sine wave

```
v_out = V_bias + A × sin(θ)
DAC_value = v_out × (255 / 3.3V)
```

**Applications:**
- Grid-tied inverter synchronization
- Power factor correction
- Active filtering reference

---

## Configuration Guide

### Pin Assignments

| Function | Pin | Description |
|----------|-----|-------------|
| ADC Input | GPIO36 | ADC1_CH0, Grid voltage sense |
| DAC Output | GPIO25 | DAC_CH1, Synchronized output |

### Voltage Ranges

**ADC Input:**
- Range: 0 - 3.3V
- Recommended AC: 1.0V peak (around 1.65V bias)
- Protection: Add series resistor + Zener clamps

**DAC Output:**
- Range: 0 - 3.3V
- Output: 1.65V ± 1.0V sine wave
- Load: High impedance (> 10kΩ)

### Sample Circuit

```
Grid Signal ──┬──[ 100kΩ ]──┬──┬──► GPIO36 (ADC)
              │              │  │
            [≈]            [3.3V Zener]
          Isolation       [GND Zener]
                            │  │
                           GND GND
                           
GPIO25 (DAC) ──[ 1kΩ ]──► To Load/Scope
```

---

## Tuning Parameters

### Quick Tuning Guide

**Symptom:** Slow convergence
- ↑ Increase `PLL_KI` (try 20-25)
- ↑ Increase `PLL_KP` (try 3-4)

**Symptom:** Oscillation/ringing
- ↓ Decrease `PLL_KP` (try 1.0-1.5)
- ↓ Decrease `PLL_KI` (try 10-12)

**Symptom:** High-frequency noise
- ↓ Decrease `SOGI_GAIN` (try 1.0-1.2)
- Consider input filtering

**Symptom:** Poor harmonic rejection
- ↑ Increase `SOGI_GAIN` (try 1.5-1.8)
- Check input signal quality

### Advanced Tuning

**DC Filter Time Constant:**
```cpp
#define DC_FILTER_ALPHA 0.001f  // τ = 1s
```
- Smaller α → slower, more filtering
- Larger α → faster, less filtering

**Frequency Limits:**
```cpp
#define GRID_FREQ_MIN 45.0f  // Hz
#define GRID_FREQ_MAX 55.0f  // Hz
```
- Adjust for regional standards (e.g., 59-61 Hz for 60 Hz grids)

---

## Performance Metrics

### Expected Performance

| Metric | Typical Value |
|--------|---------------|
| Settling Time | 2-4 cycles (40-80ms @ 50Hz) |
| Frequency Accuracy | ±0.01 Hz |
| Phase Error (steady-state) | <1° |
| Harmonic Rejection | -40 dB @ 3rd harmonic |
| DC Offset Rejection | >60 dB |
| Processing Time | ~50-70 µs per sample |

### Benchmarking

Monitor these indicators in Serial output:

```
Freq: 50.023 Hz | Theta: 0.123 V | Out: 2.234 V | DC: 1.651 V | Errors: 0 [OK]
[DIAG] Stack remaining: 6234 words (76.0%)
[DIAG] Free heap: 245632 bytes
```

**Good Health Indicators:**
- ✅ Frequency stable within ±0.1 Hz
- ✅ DC offset near 1.65V (±0.1V)
- ✅ Error count not increasing
- ✅ Stack > 30% remaining
- ✅ Status shows [OK]

---

## Troubleshooting

### Problem: Output Not Synchronizing

**Symptoms:**
- Frequency reading unstable
- Large phase error

**Checks:**
1. Verify input signal amplitude (should be 0.5-1.5V peak)
2. Check ADC voltage range (raw values 1000-3000)
3. Increase PLL gains (Kp, Ki)
4. Verify SOGI_GAIN = 1.414

**Debug:**
```cpp
// Add to Serial output
Serial.print(" | v_alpha: "); Serial.print(state.v_alpha);
Serial.print(" | v_beta: "); Serial.print(state.v_beta);
```

---

### Problem: High Error Count

**Symptoms:**
- Error count continuously increasing
- [FAULT] status indicator

**Possible Causes:**
1. **ADC Saturation**
   - Check: `adc_raw` out of range [100, 3995]
   - Fix: Reduce input amplitude or add attenuation

2. **DC Offset Drift**
   - Check: DC offset far from 1.65V (±0.5V)
   - Fix: Check input coupling, verify bias circuit

3. **Noisy Input**
   - Check: Signal with high-frequency components
   - Fix: Add hardware low-pass filter (< 1kHz)

---

### Problem: System Crashes/Resets

**Symptoms:**
- Watchdog timeout
- Random resets

**Checks:**
1. **Stack Overflow**
   ```
   [DIAG] Stack remaining: 128 words (1.6%)  ← DANGER!
   ```
   - Fix: Increase `STACK_SIZE_WORDS` to 10240

2. **Mutex Deadlock**
   - Check: Both tasks blocked
   - Fix: Verify mutex timeout values

3. **Timer Misconfiguration**
   - Check: Interrupt rate too high
   - Fix: Verify `TIMER_ALARM_US` calculation

---

### Problem: Output Distorted

**Symptoms:**
- Non-sinusoidal DAC output
- Clipped waveform

**Checks:**
1. **DAC Saturation**
   - Check: `v_out` outside [0.65, 2.65]V
   - Fix: Reduce `V_OUT_AMPLITUDE`

2. **Insufficient DAC Resolution**
   - ESP32 DAC is 8-bit (256 levels)
   - Expected: Some quantization noise

3. **Load Impedance Too Low**
   - DAC output impedance ~1-2kΩ
   - Fix: Use buffer op-amp or increase load

---

## Safety & Fault Handling

### Built-in Protections

1. **Watchdog Timer**
   - Timeout: 3 seconds
   - Resets system if task hangs
   - Auto-feed in main processing loop

2. **Input Validation**
   - ADC range checking
   - Invalid readings rejected
   - Fallback to safe output (V_BIAS)

3. **Frequency Clamping**
   - Hard limits: 45-55 Hz
   - Prevents runaway oscillation
   - Integrator anti-windup

4. **Stack Monitoring**
   - Periodic high-water-mark check
   - Warning at <30% remaining
   - Prevents silent overflow

### Fail-Safe Behaviors

**On ADC Failure:**
- Output DC bias voltage (1.65V)
- Increment error counter
- Maintain last valid DC offset

**On Loss of Lock:**
- PI controller continues trying to converge
- Frequency stays clamped to valid range
- System remains stable, no oscillation

**On Heap Exhaustion:**
- System prints diagnostic
- Existing tasks continue running
- New allocations fail gracefully

---

## Testing Procedures

### Initial Verification

1. **Power-On Test**
   ```
   Expected Serial Output:
   ================================
   SOGI-PLL v2.0 - Production Ready
   ================================
   [INIT] Configuring ADC...
   [INIT] Configuring DAC...
   ...
   [READY] System initialized successfully!
   ```

2. **No-Input Test**
   - Disconnect ADC input (floating)
   - Should see: DC offset converging to ~1.65V
   - Output: Clean 50 Hz sine wave (from PLL free-running)

3. **Signal Injection Test**
   - Apply 50 Hz, 1V peak sine wave to ADC
   - Monitor convergence (2-4 cycles)
   - Verify: `Freq: 50.000 Hz` (±0.1 Hz)

### Performance Testing

1. **Frequency Step Response**
   - Change input from 50 Hz → 51 Hz
   - Measure settling time
   - Verify: <100ms to new frequency

2. **Harmonic Rejection**
   - Add 150 Hz (3rd harmonic) to input
   - Check: Output remains clean 50 Hz
   - Expected: >30 dB rejection

3. **Noise Immunity**
   - Add white noise to input
   - Verify: System remains locked
   - Acceptable: Frequency jitter <0.1 Hz

---

## Maintenance & Updates

### Version History

**v2.0.0** (2026-01-27) - Production Release
- Complete refactoring for production use
- Added comprehensive error handling
- Thread-safe data sharing with mutexes
- Watchdog integration
- Fixed SOGI integration coupling issue
- Removed magic numbers
- Added extensive documentation

**v1.0.0** (Original)
- Basic SOGI-PLL implementation
- Known issues with race conditions

### Future Enhancements

**Planned Features:**
- [ ] Runtime parameter adjustment via Serial commands
- [ ] Amplitude tracking for voltage sag detection
- [ ] THD (Total Harmonic Distortion) measurement
- [ ] Data logging to SD card
- [ ] Web interface for monitoring

**Optimization Opportunities:**
- [ ] Replace `sinf/cosf` with lookup table (3x faster)
- [ ] Fixed-point arithmetic (avoid float overhead)
- [ ] SIMD vectorization (if supported by ESP32-S3)

---

## References

### Academic Papers
1. Rodriguez, P., et al. (2006). "New positive-sequence voltage detector for grid synchronization of power converters under faulty grid conditions"
2. Ciobotaru, M., et al. (2006). "A new single-phase PLL structure based on second order generalized integrator"

### Application Notes
- Espressif ESP32 Technical Reference Manual
- FreeRTOS Real-Time Kernel Documentation

### Related Links
- ESP32 ADC Calibration: https://docs.espressif.com/projects/esp-idf/
- PLL Design Guidelines: Texas Instruments SLAA589

---

## Support

For questions or issues:
1. Check troubleshooting section
2. Review Serial output diagnostics
3. Measure signals with oscilloscope
4. Verify hardware connections

**Common Gotchas:**
- Input signal must be AC-coupled
- Ensure proper ground reference
- ADC needs time to stabilize (100ms after boot)
- DAC cannot drive low-impedance loads directly

---

*End of Documentation*
