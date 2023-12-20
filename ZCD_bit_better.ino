// Define pins for analog input, zero crossing detector, and simulated output
const int AC_IN = 36; // Analog input pin for AC voltage
const int ZCD_IN = 2; // Digital input pin for zero crossing detector
const int ZC_OUT = 4; // Digital output pin for simulated zero crossing

// Define constants for PLL and Kalman filter
const float Kp = 0.1; // Proportional gain for PLL
const float Ki = 0.01; // Integral gain for PLL
const float Q = 0.001; // Process noise covariance for Kalman filter
const float R = 0.1; // Measurement noise covariance for Kalman filter

// Define variables for PLL and Kalman filter
float phase = 0; // Estimated phase of AC signal
float freq = 50; // Estimated frequency of AC signal
float bias = 0; // Estimated bias of AC signal
float P = 1; // Error covariance of phase estimate
float K = 0; // Kalman gain
float y = 0; // Measurement residual
float dt = 0; // Time interval between measurements
unsigned long prevTime = 0; // Previous measurement time
unsigned long currTime = 0; // Current measurement time

// Define variables for simulated output
bool zcState = false; // State of simulated zero crossing output
unsigned long zcTime = 0; // Time of last zero crossing
unsigned long halfPeriod = 10000; // Half period of AC signal in microseconds

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize pins
  pinMode(AC_IN, INPUT);
  pinMode(ZCD_IN, INPUT_PULLUP);
  pinMode(ZC_OUT, OUTPUT);
  digitalWrite(ZC_OUT, zcState);

  // Attach interrupt handler for zero crossing detector
  attachInterrupt(digitalPinToInterrupt(ZCD_IN), zeroCrossing, RISING);
}

void loop() {
  // Read analog input and map it to -1 to 1 range
  float acValue = map(analogRead(AC_IN), 0, 4095, -1, 1);

  // Update current time and time interval
  currTime = micros();
  dt = (currTime - prevTime) / 1000000.0;
  prevTime = currTime;

  // Update phase estimate
  phase = phase + freq * dt;

  // Update error covariance
  P = P + Q;

  // Check if zero crossing detected
  if (digitalRead(ZCD_IN) == HIGH) {
    // Calculate measurement residual
    y = phase - 0;

    // Calculate Kalman gain
    K = P / (P + R);

    // Update phase, frequency, and bias estimates
    phase = phase - K * y;
    freq = freq - K * bias;
    bias = bias - K * y / dt;

    // Update error covariance
    P = (1 - K) * P;
  }

  // Update simulated output
  if (currTime - zcTime >= halfPeriod) {
    // Toggle output state
    zcState = !zcState;
    digitalWrite(ZC_OUT, zcState);

    // Update zero crossing time and half period
    zcTime = currTime;
    halfPeriod = 1000000 / (2 * freq);
  }

  // Print estimated frequency
  Serial.println(freq);
}
