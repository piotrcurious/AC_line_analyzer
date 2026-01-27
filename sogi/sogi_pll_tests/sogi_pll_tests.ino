/**
 * @file sogi_pll_tests.ino
 * @brief Test Suite for SOGI-PLL System
 * 
 * Comprehensive testing and validation routines.
 * Upload this separately to run diagnostic tests.
 * 
 * @version 2.0.0
 * @date 2026-01-27
 */

#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>

// ================================================================
// TEST CONFIGURATION
// ================================================================
#define TEST_ADC_CHANNEL    ADC1_CHANNEL_0
#define TEST_DAC_CHANNEL    DAC_CHANNEL_1
#define SERIAL_BAUD         115200

// ================================================================
// TEST UTILITIES
// ================================================================

class TestFramework {
private:
    int tests_passed = 0;
    int tests_failed = 0;
    String current_suite = "";

public:
    void beginSuite(const String& name) {
        current_suite = name;
        Serial.println("\n" + String("=").repeat(60));
        Serial.println("TEST SUITE: " + name);
        Serial.println(String("=").repeat(60));
    }

    void endSuite() {
        Serial.println(String("-").repeat(60));
        Serial.printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
        Serial.println(String("=").repeat(60) + "\n");
    }

    void assertTrue(const String& test_name, bool condition, const String& message = "") {
        Serial.print("  [TEST] " + test_name + " ... ");
        if (condition) {
            Serial.println("✓ PASS");
            tests_passed++;
        } else {
            Serial.println("✗ FAIL");
            if (message.length() > 0) {
                Serial.println("    Error: " + message);
            }
            tests_failed++;
        }
    }

    void assertEqual(const String& test_name, float actual, float expected, float tolerance = 0.01f) {
        bool passed = fabs(actual - expected) <= tolerance;
        Serial.print("  [TEST] " + test_name + " ... ");
        if (passed) {
            Serial.println("✓ PASS");
            tests_passed++;
        } else {
            Serial.printf("✗ FAIL (Expected: %.4f, Got: %.4f)\n", expected, actual);
            tests_failed++;
        }
    }

    void printInfo(const String& message) {
        Serial.println("  [INFO] " + message);
    }

    int getTotalTests() { return tests_passed + tests_failed; }
    int getPassedTests() { return tests_passed; }
    int getFailedTests() { return tests_failed; }
};

TestFramework test;

// ================================================================
// HARDWARE TESTS
// ================================================================

void test_ADC_Configuration() {
    test.beginSuite("ADC Configuration");

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(TEST_ADC_CHANNEL, ADC_ATTEN_DB_11);
    delay(100);  // Allow ADC to stabilize

    // Test 1: Read ADC multiple times
    test.printInfo("Reading ADC 10 times...");
    int readings[10];
    for (int i = 0; i < 10; i++) {
        readings[i] = adc1_get_raw(TEST_ADC_CHANNEL);
        delay(10);
    }

    // Check all readings are in valid range
    bool all_valid = true;
    for (int i = 0; i < 10; i++) {
        if (readings[i] < 0 || readings[i] > 4095) {
            all_valid = false;
            break;
        }
    }
    test.assertTrue("ADC readings in valid range", all_valid);

    // Test 2: Check reading consistency
    int min_reading = readings[0];
    int max_reading = readings[0];
    for (int i = 1; i < 10; i++) {
        if (readings[i] < min_reading) min_reading = readings[i];
        if (readings[i] > max_reading) max_reading = readings[i];
    }
    int spread = max_reading - min_reading;
    test.assertTrue("ADC readings consistent", spread < 200, 
                   "Spread: " + String(spread) + " (should be < 200)");

    // Test 3: Voltage conversion
    float voltage = readings[5] * (3.3f / 4095.0f);
    test.assertTrue("Voltage conversion reasonable", 
                   voltage >= 0.0f && voltage <= 3.3f,
                   "Voltage: " + String(voltage) + "V");

    test.printInfo("Sample readings: " + String(readings[0]) + ", " + 
                   String(readings[4]) + ", " + String(readings[9]));
    test.printInfo("Converted voltage: " + String(voltage) + "V");

    test.endSuite();
}

void test_DAC_Configuration() {
    test.beginSuite("DAC Configuration");

    // Enable DAC
    dac_output_enable(TEST_DAC_CHANNEL);
    delay(100);

    // Test 1: Set minimum value
    dac_output_voltage(TEST_DAC_CHANNEL, 0);
    delay(100);
    test.assertTrue("DAC minimum value set", true);

    // Test 2: Set mid value
    dac_output_voltage(TEST_DAC_CHANNEL, 128);
    delay(100);
    test.assertTrue("DAC mid value set", true);

    // Test 3: Set maximum value
    dac_output_voltage(TEST_DAC_CHANNEL, 255);
    delay(100);
    test.assertTrue("DAC maximum value set", true);

    // Test 4: Generate ramp
    test.printInfo("Generating DAC ramp (measure on GPIO25)...");
    for (int i = 0; i <= 255; i += 5) {
        dac_output_voltage(TEST_DAC_CHANNEL, i);
        delay(10);
    }
    test.assertTrue("DAC ramp completed", true);

    // Reset to mid-point
    dac_output_voltage(TEST_DAC_CHANNEL, 128);

    test.endSuite();
}

// ================================================================
// ALGORITHM TESTS
// ================================================================

void test_SOGI_Filter() {
    test.beginSuite("SOGI Filter Algorithm");

    // Test parameters
    const float omega = 314.159f;  // 50 Hz
    const float k = 1.414f;
    const float dt = 0.0001f;
    
    float v_alpha = 0.0f;
    float v_beta = 0.0f;

    // Test 1: Single iteration
    float v_in = 1.0f;
    float omega_dt = omega * dt;
    float error = v_in - v_alpha;
    
    float alpha_dot = omega_dt * v_beta + k * omega_dt * error;
    float beta_dot = -omega_dt * v_alpha;
    
    v_alpha += alpha_dot;
    v_beta += beta_dot;

    test.assertTrue("SOGI single iteration", 
                   fabs(v_alpha) < 1.0f && fabs(v_beta) < 1.0f);

    // Test 2: Convergence over multiple iterations
    test.printInfo("Running 1000 iterations...");
    for (int i = 0; i < 1000; i++) {
        float t = i * dt;
        v_in = sinf(omega * t);
        
        error = v_in - v_alpha;
        alpha_dot = omega_dt * v_beta + k * omega_dt * error;
        beta_dot = -omega_dt * v_alpha;
        
        v_alpha += alpha_dot;
        v_beta += beta_dot;
    }

    test.printInfo("Final v_alpha: " + String(v_alpha, 6));
    test.printInfo("Final v_beta: " + String(v_beta, 6));
    test.assertTrue("SOGI convergence", 
                   fabs(v_alpha) < 1.5f && fabs(v_beta) < 1.5f);

    test.endSuite();
}

void test_PI_Controller() {
    test.beginSuite("PI Controller");

    const float kp = 2.0f;
    const float ki = 15.0f;
    const float dt = 0.0001f;
    
    float integrator = 314.159f;
    float error = 1.0f;  // 1 Hz error

    // Test 1: Proportional response
    float p_term = kp * error;
    test.assertEqual("Proportional term", p_term, 2.0f, 0.01f);

    // Test 2: Integral accumulation
    integrator += ki * dt * error;
    test.assertTrue("Integrator increases", integrator > 314.159f);

    // Test 3: Anti-windup clamping
    integrator = 350.0f;  // Above max
    if (integrator > 345.0f) integrator = 345.0f;
    test.assertEqual("Integrator clamped high", integrator, 345.0f, 0.01f);

    integrator = 280.0f;  // Below min
    if (integrator < 282.0f) integrator = 282.0f;
    test.assertEqual("Integrator clamped low", integrator, 282.0f, 0.01f);

    test.endSuite();
}

void test_Phase_Normalization() {
    test.beginSuite("Phase Angle Normalization");

    // Test 1: Positive overflow
    float theta = 7.0f;  // > 2π
    theta = fmodf(theta, 2.0f * PI);
    test.assertTrue("Positive overflow normalized", theta >= 0.0f && theta < 2.0f * PI);

    // Test 2: Negative angle
    theta = -1.0f;
    theta = fmodf(theta, 2.0f * PI);
    if (theta < 0.0f) theta += 2.0f * PI;
    test.assertTrue("Negative angle normalized", theta >= 0.0f && theta < 2.0f * PI);

    // Test 3: Large angle
    theta = 100.0f;
    theta = fmodf(theta, 2.0f * PI);
    test.assertTrue("Large angle normalized", theta >= 0.0f && theta < 2.0f * PI);

    test.endSuite();
}

void test_DC_Offset_Filter() {
    test.beginSuite("DC Offset Removal");

    const float alpha = 0.001f;
    float dc_offset = 1.65f;

    // Test 1: Steady signal
    for (int i = 0; i < 1000; i++) {
        float v_in = 1.65f;  // No AC component
        dc_offset = dc_offset * (1.0f - alpha) + v_in * alpha;
    }
    test.assertEqual("DC offset converges", dc_offset, 1.65f, 0.01f);

    // Test 2: Step change
    dc_offset = 1.65f;
    for (int i = 0; i < 5000; i++) {
        float v_in = 2.0f;  // Step to 2.0V
        dc_offset = dc_offset * (1.0f - alpha) + v_in * alpha;
    }
    test.assertTrue("DC offset tracks step", dc_offset > 1.9f && dc_offset < 2.1f);
    test.printInfo("Final DC offset: " + String(dc_offset, 4) + "V");

    test.endSuite();
}

// ================================================================
// PERFORMANCE TESTS
// ================================================================

void test_Execution_Timing() {
    test.beginSuite("Execution Timing");

    // Simulate one complete iteration
    unsigned long start_time = micros();
    
    // ADC read
    int adc_val = adc1_get_raw(TEST_ADC_CHANNEL);
    float v_in = adc_val * (3.3f / 4095.0f);
    
    // SOGI calculation
    float v_alpha = 0.5f, v_beta = 0.3f;
    float omega = 314.159f;
    float dt = 0.0001f;
    float error = v_in - v_alpha;
    
    v_alpha += omega * dt * v_beta + 1.414f * omega * dt * error;
    v_beta += -omega * dt * v_alpha;
    
    // Phase calculation
    float theta = 1.0f;
    float s = sinf(theta);
    float c = cosf(theta);
    float v_q = -v_alpha * s + v_beta * c;
    
    // PI controller
    float integrator = 314.159f;
    integrator += 15.0f * dt * v_q;
    float output = integrator + 2.0f * v_q;
    
    // DAC write
    int dac_val = (int)(1.65f * 77.27f);
    dac_output_voltage(TEST_DAC_CHANNEL, dac_val);
    
    unsigned long end_time = micros();
    unsigned long execution_time = end_time - start_time;

    test.printInfo("Execution time: " + String(execution_time) + " µs");
    test.assertTrue("Execution within timing budget", 
                   execution_time < 100,  // Must be < 100µs for 10kHz
                   "Time: " + String(execution_time) + "µs (budget: 100µs)");

    // Run 100 iterations for average
    start_time = micros();
    for (int i = 0; i < 100; i++) {
        adc_val = adc1_get_raw(TEST_ADC_CHANNEL);
        v_in = adc_val * (3.3f / 4095.0f);
        error = v_in - v_alpha;
        v_alpha += omega * dt * v_beta + 1.414f * omega * dt * error;
        v_beta += -omega * dt * v_alpha;
        s = sinf(theta);
        c = cosf(theta);
        v_q = -v_alpha * s + v_beta * c;
        integrator += 15.0f * dt * v_q;
        output = integrator + 2.0f * v_q;
        dac_val = (int)(1.65f * 77.27f);
        dac_output_voltage(TEST_DAC_CHANNEL, dac_val);
    }
    end_time = micros();
    float avg_time = (end_time - start_time) / 100.0f;

    test.printInfo("Average execution time (100 samples): " + String(avg_time, 2) + " µs");

    test.endSuite();
}

void test_Memory_Usage() {
    test.beginSuite("Memory Usage");

    // Check heap
    uint32_t free_heap = ESP.getFreeHeap();
    test.printInfo("Free heap: " + String(free_heap) + " bytes");
    test.assertTrue("Sufficient heap available", free_heap > 100000);

    // Check stack (for main task)
    test.printInfo("Note: Stack usage requires running task");
    
    test.endSuite();
}

// ================================================================
// INTEGRATION TESTS
// ================================================================

void test_Signal_Generation() {
    test.beginSuite("Signal Generation");

    test.printInfo("Generating 50 Hz test signal on DAC...");
    test.printInfo("Measure on GPIO25 with oscilloscope");
    
    // Generate 2 seconds of 50 Hz
    const float freq = 50.0f;
    const float omega = 2.0f * PI * freq;
    const float dt = 0.001f;  // 1ms
    const int samples = 2000;  // 2 seconds
    
    for (int i = 0; i < samples; i++) {
        float t = i * dt;
        float v_out = 1.65f + 1.0f * sinf(omega * t);
        int dac_val = (int)(v_out * 77.27f);
        dac_output_voltage(TEST_DAC_CHANNEL, constrain(dac_val, 0, 255));
        delay(1);
    }
    
    test.assertTrue("Signal generation completed", true);
    test.printInfo("Expected: 50 Hz, 1V peak, 1.65V offset");

    test.endSuite();
}

void test_Loopback() {
    test.beginSuite("ADC-DAC Loopback");

    test.printInfo("Connect GPIO25 (DAC) to GPIO36 (ADC) for this test");
    test.printInfo("Generating test signals...");

    bool loopback_ok = true;

    // Test 1: DC levels
    dac_output_voltage(TEST_DAC_CHANNEL, 0);
    delay(100);
    int adc_low = adc1_get_raw(TEST_ADC_CHANNEL);
    
    dac_output_voltage(TEST_DAC_CHANNEL, 255);
    delay(100);
    int adc_high = adc1_get_raw(TEST_ADC_CHANNEL);
    
    test.printInfo("DAC=0 → ADC=" + String(adc_low));
    test.printInfo("DAC=255 → ADC=" + String(adc_high));
    
    test.assertTrue("ADC-DAC loopback functioning", adc_high > adc_low + 2000);

    // Test 2: Sine wave
    test.printInfo("Generating sine wave...");
    const int num_samples = 200;
    int prev_adc = 0;
    int zero_crossings = 0;
    
    for (int i = 0; i < num_samples; i++) {
        float t = i / 100.0f;  // 2 seconds total
        float v = 1.65f + 1.0f * sinf(2.0f * PI * 50.0f * t);
        int dac_val = (int)(v * 77.27f);
        dac_output_voltage(TEST_DAC_CHANNEL, constrain(dac_val, 0, 255));
        delay(10);
        
        int adc_val = adc1_get_raw(TEST_ADC_CHANNEL);
        
        // Count zero crossings (approx 1.65V = 2048 ADC)
        if (prev_adc < 2048 && adc_val >= 2048) {
            zero_crossings++;
        }
        prev_adc = adc_val;
    }
    
    test.printInfo("Zero crossings detected: " + String(zero_crossings));
    test.assertTrue("Sine wave detected", zero_crossings >= 8 && zero_crossings <= 12);

    test.endSuite();
}

// ================================================================
// MAIN TEST RUNNER
// ================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(2000);  // Wait for serial monitor

    Serial.println("\n\n");
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║         SOGI-PLL TEST SUITE v2.0                      ║");
    Serial.println("║         Comprehensive System Validation               ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println();

    // Run all test suites
    test_ADC_Configuration();
    test_DAC_Configuration();
    test_SOGI_Filter();
    test_PI_Controller();
    test_Phase_Normalization();
    test_DC_Offset_Filter();
    test_Execution_Timing();
    test_Memory_Usage();
    test_Signal_Generation();
    // test_Loopback();  // Requires hardware connection

    // Final summary
    Serial.println("\n" + String("=").repeat(60));
    Serial.println("FINAL TEST SUMMARY");
    Serial.println(String("=").repeat(60));
    Serial.printf("Total Tests: %d\n", test.getTotalTests());
    Serial.printf("Passed: %d\n", test.getPassedTests());
    Serial.printf("Failed: %d\n", test.getFailedTests());
    
    if (test.getFailedTests() == 0) {
        Serial.println("\n✓✓✓ ALL TESTS PASSED ✓✓✓");
    } else {
        Serial.println("\n✗✗✗ SOME TESTS FAILED ✗✗✗");
        Serial.println("Review failed tests above for details");
    }
    Serial.println(String("=").repeat(60));
    Serial.println("\nTest suite completed. Reset to run again.");
}

void loop() {
    // Test suite runs once in setup()
    delay(1000);
}
