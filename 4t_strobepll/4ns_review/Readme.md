# Code Review: ESP32 Grid Frequency Monitor

## Summary of Improvements

The improved code maintains the excellent core design while adding robustness, safety features, and better maintainability.

---

## Key Improvements

### 1. **Enhanced Safety and Error Handling**

**Original Issues:**
- No validation of CPU frequency reading
- No bounds checking on frequency estimates
- Silent failures possible

**Improvements:**
- ✅ CPU frequency validation at startup with halt on failure
- ✅ Frequency bounds checking (45-55 Hz for 50 Hz nominal)
- ✅ Sanity checks on calculated timing parameters
- ✅ Atomic updates using `noInterrupts()`/`interrupts()`

### 2. **Better ADC Configuration**

**Added:**
```cpp
analogSetAttenuation(ADC_11db);  // For 0-3.3V range
```
- Explicitly sets ADC input range
- Prevents unexpected readings from uninitialized attenuation

### 3. **Improved Timing Precision**

**Original:**
```cpp
sys.buf[sys.idx].jitter_cycles = (int32_t)(cpu_hal_get_cycle_count() - sys.next_sample_tick);
```

**Improved:**
```cpp
const uint32_t actual_read_time = cpu_hal_get_cycle_count();
sys.buf[sys.idx].jitter_cycles = (int32_t)(actual_read_time - sys.next_sample_tick);
```

**Benefits:**
- Captures exact read time after ADC operation
- More accurate jitter measurement
- Const qualifier prevents accidental modification

### 4. **Volatile Qualifiers for Shared State**

**Added:**
```cpp
volatile bool frame_ready;
volatile bool capturing;
```

**Why:** Prevents compiler optimizations that could cause issues if interrupts are added later

### 5. **Enhanced Diagnostics**

**New Features:**
- Frame counter for tracking total captures
- Maximum jitter tracking across all samples
- ADC statistics (min, max, average, peak-to-peak)
- Human-readable timing information
- Warning for excessive jitter

**Example Output:**
```
Frame #42 | Strobe offset: 234 cycles (1.95 us) | First sample jitter: 12 cycles (0.10 us)
ADC stats: min=1024, max=3071, avg=2048, pk-pk=2047
⚠️  WARNING: High jitter detected: 8543 cycles (71.19 us)
```

### 6. **Better Code Organization**

**Improvements:**
- Clear section headers with ASCII art separators
- Function prototypes at top for better overview
- Comprehensive documentation comments
- Logical grouping of related functionality

### 7. **Defensive Programming**

**tune_pll() Function:**
```cpp
void tune_pll(float grid_freq) {
  // Validate input
  if (!validate_frequency(grid_freq)) {
    Serial.printf("WARNING: Invalid frequency %.2f Hz, keeping current setting\n", grid_freq);
    return;
  }
  
  // Sanity check results
  if (new_cycles_per_sample == 0 || new_cycles_per_strobe == 0) {
    Serial.println("WARNING: PLL tuning resulted in zero cycles, ignoring");
    return;
  }
  
  // Apply updates atomically
  noInterrupts();
  sys.cycles_per_sample = new_cycles_per_sample;
  sys.cycles_per_strobe = new_cycles_per_strobe;
  interrupts();
}
```

**Benefits:**
- Prevents bad frequency estimates from breaking the system
- Graceful degradation instead of crashes
- Maintains system stability

### 8. **Initialization Feedback**

**Added startup banner:**
```
=== Grid Frequency Monitor Initializing ===
CPU Frequency: 240 MHz
Samples per cycle: 128
Cycles per sample: 37500 (156.25 us)
Cycles per strobe: 4800000 (20.00 ms)
Initialization complete. Starting capture...
```

**Benefits:**
- Immediate feedback if something is misconfigured
- Helps debugging timing issues
- Confirms system is ready

### 9. **Early Exit Optimization**

**In poll_pll():**
```cpp
if (!sys.capturing) {
    // ... strobe logic ...
    return false;  // Early exit when not capturing
}
```

**Benefits:**
- Reduces unnecessary condition checks
- Cleaner code flow
- Minimal performance impact

### 10. **Constants for Magic Numbers**

**Added:**
```cpp
#define MIN_FREQ 45.0f
#define MAX_FREQ 55.0f
#define MAX_JITTER_CYCLES 10000
```

**Benefits:**
- Easy to adjust thresholds
- Self-documenting code
- No magic numbers scattered throughout

---

## Preserved Strengths

The original code had excellent design choices that were maintained:

✅ **Cycle-accurate timing** using `cpu_hal_get_cycle_count()`  
✅ **Drift-canceling scheduler** with `next_sample_tick += cycles_per_sample`  
✅ **Overflow-safe arithmetic** using signed difference: `(int32_t)(now - target) >= 0`  
✅ **Separation of concerns** between polling loop and processing  
✅ **Efficient inline functions** for hot paths  
✅ **Minimal critical section** between timing check and ADC read

---

## Recommendations for Next Steps

### 1. **Implement Frequency Estimation**

Replace the placeholder with actual signal processing:

```cpp
// Current placeholder:
float estimated_freq = NOMINAL_FREQ;

// Suggested approaches:
// - Zero-crossing detection with interpolation
// - SOGI (Second-Order Generalized Integrator)
// - Discrete Fourier Transform
// - Autocorrelation
```

### 2. **Add Moving Average Filter**

Smooth frequency estimates to reduce control loop jitter:

```cpp
#define FREQ_FILTER_SIZE 10
float freq_history[FREQ_FILTER_SIZE];
uint8_t freq_idx = 0;

float filter_frequency(float new_freq) {
  freq_history[freq_idx] = new_freq;
  freq_idx = (freq_idx + 1) % FREQ_FILTER_SIZE;
  
  float sum = 0;
  for (int i = 0; i < FREQ_FILTER_SIZE; i++) {
    sum += freq_history[i];
  }
  return sum / FREQ_FILTER_SIZE;
}
```

### 3. **Add Data Export Options**

For analysis and debugging:

```cpp
void export_frame_csv() {
  Serial.println("Index,ADC_Value,Jitter_Cycles,Jitter_Us");
  for (uint16_t i = 0; i < BUF_SZ; i++) {
    Serial.printf("%u,%u,%d,%.3f\n", 
      i, 
      sys.buf[i].val, 
      sys.buf[i].jitter_cycles,
      (double)sys.buf[i].jitter_cycles / sys.cpu_freq * 1e6);
  }
}
```

### 4. **Consider Non-volatile Storage**

Store calibration data or statistics:

```cpp
#include <Preferences.h>

Preferences prefs;
prefs.begin("grid-monitor", false);
prefs.putFloat("cal_offset", calibration_offset);
float offset = prefs.getFloat("cal_offset", 0.0);
prefs.end();
```

### 5. **Add WiFi/MQTT Reporting**

For remote monitoring:

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

void report_metrics() {
  StaticJsonDocument<256> doc;
  doc["frequency"] = estimated_freq;
  doc["jitter_max_us"] = sys.max_jitter_seen / (double)sys.cpu_freq * 1e6;
  doc["frames"] = sys.frames_captured;
  
  char buffer[256];
  serializeJson(doc, buffer);
  mqtt_client.publish("grid/metrics", buffer);
}
```

### 6. **Implement Watchdog Protection**

Prevent hangs from runaway conditions:

```cpp
#include <esp_task_wdt.h>

void setup() {
  // ... existing setup ...
  
  // Configure watchdog (10 second timeout)
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);
}

void loop() {
  esp_task_wdt_reset();  // Feed the watchdog
  
  // ... rest of loop ...
}
```

---

## Performance Considerations

### Current Performance Metrics (240 MHz CPU, 50 Hz)

- **Sample interval:** 156.25 μs (128 samples/cycle)
- **Frame interval:** 60 ms (3 cycles)
- **Processing budget:** ~60 ms between frames
- **Polling overhead:** <1 μs per iteration

### Optimization Opportunities

1. **DMA for ADC:** Eliminate polling jitter entirely
2. **FPU optimizations:** Use hardware floating point
3. **SIMD:** ESP32-S3 has vector instructions
4. **Lookup tables:** Pre-compute trigonometric values

---

## Testing Recommendations

### Unit Tests

```cpp
void test_frequency_validation() {
  assert(validate_frequency(50.0) == true);
  assert(validate_frequency(44.9) == false);
  assert(validate_frequency(55.1) == false);
  Serial.println("✓ Frequency validation tests passed");
}

void test_overflow_arithmetic() {
  uint32_t large = 0xFFFFFFF0;
  uint32_t target = 0x00000010;
  int32_t diff = (int32_t)(large - target);
  assert(diff < 0);  // large happened before target
  Serial.println("✓ Overflow arithmetic tests passed");
}
```

### Integration Tests

1. **Sine wave generator:** Verify frequency tracking
2. **Frequency sweep:** Test PLL lock range
3. **Noise injection:** Verify robustness
4. **Long-term stability:** 24+ hour runs

---

## Potential Issues to Watch

1. **ADC non-linearity:** May need calibration
2. **Temperature drift:** CPU frequency can vary
3. **Power supply noise:** May couple into ADC
4. **WiFi interference:** If enabled, can cause timing jitter
5. **Serial output latency:** Use ring buffer for non-blocking output

---

## Conclusion

The original code demonstrated excellent understanding of real-time systems and phase-locked loops. The improvements focus on:

- **Robustness:** Handle edge cases and failures gracefully
- **Observability:** Better diagnostics and feedback
- **Maintainability:** Clear documentation and organization
- **Extensibility:** Easy to add new features

The core timing architecture remains unchanged and is production-ready for grid frequency monitoring applications.
