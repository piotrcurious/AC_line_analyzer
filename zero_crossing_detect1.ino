// Define the pins for the analog input and the LED output
const int analogPin = 34; // Analog input pin for the AC signal
const int ledPin = 2; // Digital output pin for the LED

// Define the parameters for the PLL and the Kalman filter
const float Kp = 0.1; // Proportional gain for the PLL
const float Ki = 0.01; // Integral gain for the PLL
const float Q = 0.001; // Process noise covariance for the Kalman filter
const float R = 0.01; // Measurement noise covariance for the Kalman filter

// Define some global variables for the PLL and the Kalman filter
float theta = 0; // Phase angle of the PLL
float omega = 2 * PI * 50; // Frequency of the PLL (initially set to 50 Hz)
float P = 1; // Error covariance of the Kalman filter
float x = 0; // State variable of the Kalman filter

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Initialize the analog input pin
  analogReadResolution(12); // Set the resolution to 12 bits
  analogSetAttenuation(ADC_11db); // Set the attenuation to 11 dB
}

void loop() {
  // Read the analog input value
  int analogValue = analogRead(analogPin);

  // Convert the analog value to a voltage
  float voltage = analogValue * (3.3 / 4095.0);

  // Update the Kalman filter
  // Predict
  x = x + omega; // Predict the state variable
  P = P + Q; // Predict the error covariance
  // Update
  float y = voltage - sin(x); // Calculate the measurement residual
  float K = P / (P + R); // Calculate the Kalman gain
  x = x + K * y; // Update the state variable
  P = (1 - K) * P; // Update the error covariance

  // Update the PLL
  // Calculate the phase error
  float e = voltage * sin(x);

  // Calculate the control signal
  float u = Kp * e + Ki * e * 0.01; // 0.01 is the sampling time

  // Update the phase and frequency
  theta = theta + omega + u;
  omega = omega + u;

  // Normalize the phase to the range [0, 2*PI]
  theta = fmod(theta, 2 * PI);
  if (theta < 0) {
    theta = theta + 2 * PI;
  }

  // Print the results to the serial monitor
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Phase: ");
  Serial.print(theta);
  Serial.print(" rad, Frequency: ");
  Serial.print(omega / (2 * PI));
  Serial.println(" Hz");

  // Turn on the LED when the phase is close to zero
  if (abs(theta) < 0.1 || abs(theta - 2 * PI) < 0.1) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // Wait for 10 ms
  delay(10);
}

