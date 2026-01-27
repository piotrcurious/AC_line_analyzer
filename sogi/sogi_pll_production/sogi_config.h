/**
 * @file sogi_config.h
 * @brief Configuration Header for SOGI-PLL System
 * 
 * Centralized configuration file for easy system tuning.
 * Modify values here instead of the main code.
 * 
 * @version 2.0.0
 * @date 2026-01-27
 */

#ifndef SOGI_CONFIG_H
#define SOGI_CONFIG_H

// ================================================================
// SYSTEM IDENTIFICATION
// ================================================================
#define SYSTEM_VERSION      "2.0.0"
#define SYSTEM_NAME         "SOGI-PLL Grid Sync"

// ================================================================
// HARDWARE CONFIGURATION
// ================================================================

/**
 * ADC Configuration
 * GPIO36 (ADC1_CH0) on most ESP32 boards
 */
#define ADC_CHANNEL         ADC1_CHANNEL_0
#define ADC_ATTEN           ADC_ATTEN_DB_11     // 0-3.3V range
#define ADC_WIDTH           ADC_WIDTH_BIT_12    // 12-bit resolution
#define ADC_MAX_VALUE       4095.0f
#define ADC_VREF            3.3f                // Reference voltage

/**
 * DAC Configuration
 * GPIO25 (DAC_CH1) on most ESP32 boards
 */
#define DAC_CHANNEL         DAC_CHANNEL_1
#define DAC_MAX_VALUE       255                 // 8-bit resolution
#define DAC_VREF            3.3f                // Reference voltage

/**
 * Signal Conditioning
 */
#define V_BIAS              1.65f               // Mid-point bias (Vref/2)
#define V_OUT_AMPLITUDE     1.0f                // Output amplitude (Vpeak)

// Valid ADC input range (with margin for noise)
#define ADC_MIN_VALID       100                 // ~0.08V
#define ADC_MAX_VALID       3995                // ~3.22V

// DC offset drift detection threshold
#define MAX_OFFSET_DRIFT    0.5f                // Maximum deviation from bias (V)

// ================================================================
// TIMING CONFIGURATION
// ================================================================

/**
 * Sampling Configuration
 * CRITICAL: Must be high enough to capture grid harmonics
 * Nyquist: fs > 2 × f_max
 * Recommended: fs ≥ 10 kHz for 50/60 Hz grids
 */
#define SAMPLING_FREQ_HZ    10000.0f            // Sampling frequency
#define SAMPLE_TIME_S       (1.0f / SAMPLING_FREQ_HZ)

/**
 * Timer Configuration
 * For ESP32: Hardware timer base frequency = 1 MHz typical
 */
#define TIMER_FREQ_HZ       1000000UL           // 1 MHz timer base
#define TIMER_ALARM_US      (TIMER_FREQ_HZ / SAMPLING_FREQ_HZ)

// ================================================================
// GRID PARAMETERS
// ================================================================

/**
 * Grid Frequency Configuration
 * Adjust based on your region:
 * - Europe/Asia/Africa: 50 Hz (most common)
 * - Americas: 60 Hz
 * - Japan: 50 Hz (East) / 60 Hz (West)
 */
#define GRID_FREQ_NOMINAL   50.0f               // Nominal grid frequency (Hz)
#define GRID_FREQ_MIN       45.0f               // Minimum valid frequency (Hz)
#define GRID_FREQ_MAX       55.0f               // Maximum valid frequency (Hz)

// For 60 Hz systems, use:
// #define GRID_FREQ_NOMINAL   60.0f
// #define GRID_FREQ_MIN       57.0f
// #define GRID_FREQ_MAX       63.0f

/**
 * Derived Angular Frequencies
 * ω = 2π × f
 */
#define OMEGA_NOMINAL       (2.0f * PI * GRID_FREQ_NOMINAL)
#define OMEGA_MIN           (2.0f * PI * GRID_FREQ_MIN)
#define OMEGA_MAX           (2.0f * PI * GRID_FREQ_MAX)

// ================================================================
// SOGI FILTER PARAMETERS
// ================================================================

/**
 * SOGI Damping Factor
 * 
 * Theory: Second-order system damping coefficient
 * - k = √2 (1.414): Critical damping (optimal for most cases)
 * - k < √2: Under-damped (faster, more overshoot)
 * - k > √2: Over-damped (slower, no overshoot)
 * 
 * Tuning Guide:
 * - Clean signals: Use 1.414 (default)
 * - Noisy signals: Increase to 1.6-2.0 (more filtering)
 * - Fast transients: Decrease to 1.0-1.2 (faster response)
 */
#define SOGI_GAIN           1.414f

// ================================================================
// PLL CONTROLLER PARAMETERS
// ================================================================

/**
 * PI Controller Gains
 * 
 * These gains determine the PLL tracking performance:
 * - Kp (Proportional): Immediate response to error
 * - Ki (Integral): Eliminates steady-state error
 * 
 * Tuning Guide for 50 Hz grid:
 * 
 * SLOW (Conservative - Best for noisy signals):
 * #define PLL_KP 1.0f
 * #define PLL_KI 10.0f
 * Settling time: ~6-8 cycles (120-160ms)
 * 
 * MEDIUM (Balanced - Default):
 * #define PLL_KP 2.0f
 * #define PLL_KI 15.0f
 * Settling time: ~3-4 cycles (60-80ms)
 * 
 * FAST (Aggressive - Clean signals only):
 * #define PLL_KP 4.0f
 * #define PLL_KI 25.0f
 * Settling time: ~2-3 cycles (40-60ms)
 * 
 * WARNING: Too high gains cause oscillation!
 */
#define PLL_KP              2.0f                // Proportional gain
#define PLL_KI              15.0f               // Integral gain

// ================================================================
// FILTER PARAMETERS
// ================================================================

/**
 * DC Offset Removal Filter
 * 
 * Exponential moving average (EMA) coefficient:
 * α = Ts / (Ts + τ)
 * 
 * Where:
 * - Ts = Sample time = 0.0001s (for 10kHz sampling)
 * - τ = Time constant (seconds)
 * 
 * Current settings:
 * - α = 0.001 → τ ≈ 1 second
 * 
 * Tuning:
 * - Faster tracking: α = 0.002 (τ ≈ 0.5s)
 * - Slower, more stable: α = 0.0005 (τ ≈ 2s)
 */
#define DC_FILTER_ALPHA     0.001f

// ================================================================
// RTOS CONFIGURATION
// ================================================================

/**
 * Task Parameters
 */
#define TASK_PRIORITY       24                  // High priority (max is 25)
#define TASK_CORE           1                   // Pin to Core 1
#define STACK_SIZE_WORDS    8192                // Stack size in 32-bit words

/**
 * Stack size guide:
 * - 4096: Minimum (tight, may overflow with debugging)
 * - 8192: Recommended (safe margin) - DEFAULT
 * - 16384: Large (plenty of headroom for future features)
 * 
 * Monitor with: uxTaskGetStackHighWaterMark()
 * Keep > 30% free for safety
 */

/**
 * Watchdog Timer
 */
#define WDT_TIMEOUT_S       3                   // Watchdog timeout (seconds)

// ================================================================
// DIAGNOSTICS CONFIGURATION
// ================================================================

/**
 * Serial Output
 */
#define SERIAL_BAUD         115200              // Baud rate
#define PRINT_INTERVAL_MS   100                 // Status print interval

/**
 * Debug Features
 * Uncomment to enable specific debug outputs
 */
// #define DEBUG_SOGI_STATES                    // Print α, β components
// #define DEBUG_PLL_INTERNALS                  // Print PI controller internals
// #define DEBUG_TIMING                         // Print execution time per sample
// #define DEBUG_ADC_RAW                        // Print raw ADC values

/**
 * Stack Monitoring
 */
#define STACK_CHECK_INTERVAL_MS  5000           // Check every 5 seconds
#define STACK_WARNING_THRESHOLD  30.0f          // Warn if < 30% free

// ================================================================
// ADVANCED TUNING (EXPERTS ONLY)
// ================================================================

/**
 * Data Sharing Update Rate
 * Update shared data every N samples to reduce mutex overhead
 * 10 = update at 1kHz (every 1ms)
 */
#define SHARED_DATA_UPDATE_DIVIDER  10

/**
 * Numerical Stability
 * These should generally not be changed
 */
#define THETA_NORMALIZE_METHOD  1               // 0=subtraction, 1=fmodf

/**
 * Anti-Windup Method
 * 0 = Simple clamping
 * 1 = Back-calculation (future feature)
 */
#define ANTIWINDUP_METHOD   0

// ================================================================
// FEATURE FLAGS
// ================================================================

/**
 * Enable/Disable Features
 */
#define ENABLE_WATCHDOG         1               // Watchdog timer
#define ENABLE_STACK_MONITOR    1               // Stack usage monitoring
#define ENABLE_ERROR_COUNTING   1               // Count and report errors
#define ENABLE_SIGNAL_VALIDATION 1              // Input signal validation

// ================================================================
// COMPILE-TIME CHECKS
// ================================================================

// Ensure sampling frequency is reasonable
#if SAMPLING_FREQ_HZ < 1000.0f
#error "Sampling frequency too low! Must be >= 1 kHz"
#endif

#if SAMPLING_FREQ_HZ > 50000.0f
#warning "Sampling frequency very high! Check timer configuration"
#endif

// Ensure frequency limits are valid
#if GRID_FREQ_MIN >= GRID_FREQ_NOMINAL
#error "GRID_FREQ_MIN must be less than GRID_FREQ_NOMINAL"
#endif

#if GRID_FREQ_MAX <= GRID_FREQ_NOMINAL
#error "GRID_FREQ_MAX must be greater than GRID_FREQ_NOMINAL"
#endif

// Ensure stack size is adequate
#if STACK_SIZE_WORDS < 4096
#error "Stack size too small! Minimum 4096 words required"
#endif

// ================================================================
// DERIVED CONSTANTS (DO NOT MODIFY)
// ================================================================

// ADC conversion factor
#define ADC_TO_VOLTS        (ADC_VREF / ADC_MAX_VALUE)

// DAC conversion factor
#define VOLTS_TO_DAC        (DAC_MAX_VALUE / DAC_VREF)

// Angular frequency to Hz conversion
#define OMEGA_TO_HZ         (1.0f / (2.0f * PI))

// ================================================================
// CONFIGURATION PRESETS
// ================================================================

/**
 * Uncomment ONE of these presets for quick configuration
 * Then fine-tune individual parameters if needed
 */

// PRESET 1: LABORATORY (Clean signals, fast response)
// #define CONFIG_PRESET_LAB
#ifdef CONFIG_PRESET_LAB
    #undef PLL_KP
    #undef PLL_KI
    #undef SOGI_GAIN
    #define PLL_KP 4.0f
    #define PLL_KI 25.0f
    #define SOGI_GAIN 1.414f
#endif

// PRESET 2: INDUSTRIAL (Noisy environment, robust)
// #define CONFIG_PRESET_INDUSTRIAL
#ifdef CONFIG_PRESET_INDUSTRIAL
    #undef PLL_KP
    #undef PLL_KI
    #undef SOGI_GAIN
    #undef DC_FILTER_ALPHA
    #define PLL_KP 1.5f
    #define PLL_KI 12.0f
    #define SOGI_GAIN 1.6f
    #define DC_FILTER_ALPHA 0.0005f
#endif

// PRESET 3: PORTABLE (Battery-powered, lower sampling)
// #define CONFIG_PRESET_PORTABLE
#ifdef CONFIG_PRESET_PORTABLE
    #undef SAMPLING_FREQ_HZ
    #undef SAMPLE_TIME_S
    #define SAMPLING_FREQ_HZ 5000.0f
    #define SAMPLE_TIME_S (1.0f / SAMPLING_FREQ_HZ)
#endif

#endif // SOGI_CONFIG_H
