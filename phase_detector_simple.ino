// Include the necessary libraries
#include <Arduino.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <math.h>

// Define the pins for the ADC and DAC
#define ADC_PIN 36 // GPIO36
#define DAC_PIN 25 // GPIO25

// Define the parameters for the PLL
#define Kp 0.1 // Proportional gain
#define Ki 0.01 // Integral gain
#define Wn 314.16 // Nominal angular frequency (50 Hz)
#define Ts 0.0001 // Sampling time (10 kHz)

// Declare the global variables
float theta = 0; // Phase angle
float omega = Wn; // Angular frequency
float vgrid = 0; // Grid voltage
float vout = 0; // Output voltage
float error = 0; // Phase error
float integral = 0; // Integral of the phase error

// Setup function
void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Initialize the ADC
  adc1_config_width(ADC_WIDTH_BIT_12); // 12-bit resolution
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // 0-3.6V range

  // Initialize the DAC
  dac_output_enable(DAC_CHANNEL_1); // Enable DAC channel 1
}

// Loop function
void loop() {
  // Read the grid voltage from the ADC
  vgrid = adc1_get_raw(ADC1_CHANNEL_0) * 3.6 / 4095.0; // Convert to volts

  // Calculate the phase error using a phase detector
  error = vgrid * sin(theta);

  // Update the integral of the phase error
  integral = integral + error * Ts;

  // Update the angular frequency using a PI controller
  omega = Wn + Kp * error + Ki * integral;

  // Update the phase angle using a phase accumulator
  theta = theta + omega * Ts;

  // Wrap the phase angle to the range [-pi, pi]
  if (theta > PI) {
    theta = theta - 2 * PI;
  }
  else if (theta < -PI) {
    theta = theta + 2 * PI;
  }

  // Generate the output voltage using a sine wave generator
  vout = 1.8 * sin(theta); // 1.8V peak amplitude

  // Write the output voltage to the DAC
  dac_output_voltage(DAC_CHANNEL_1, (vout + 1.8) * 127.5 / 3.6); // Convert to 8-bit value

  // Print the results to the serial monitor
  Serial.print("vgrid = ");
  Serial.print(vgrid, 3);
  Serial.print(" V, vout = ");
  Serial.print(vout, 3);
  Serial.print(" V, theta = ");
  Serial.print(theta, 3);
  Serial.print(" rad, omega = ");
  Serial.print(omega, 3);
  Serial.println(" rad/s");
}
