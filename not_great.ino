// include the necessary libraries
#include <Arduino.h>
#include <math.h>

// define the analog input pin
#define AC_PIN 34

// define the sampling frequency (Hz)
#define FS 1000

// define the number of samples to capture
#define N 100

// define the AC line frequency (Hz)
#define F 50

// define the reference voltage (V)
#define VREF 3.3

// define the voltage divider ratio
#define RATIO 10

// define the ADC resolution
#define ADC_BITS 12

// define the ADC counts per volt
#define ADC_COUNTS (pow(2, ADC_BITS) / VREF)

// define the pi constant
#define PI 3.14159

// declare the global variables
float samples[N]; // array to store the samples
float frequency; // variable to store the frequency
float thd; // variable to store the total harmonic distortion
float rms; // variable to store the root mean square voltage
float peak; // variable to store the peak voltage
float integrated; // variable to store the integrated voltage
float difference; // variable to store the difference between rms and integrated

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 9600 bits per second
  Serial.begin(9600);

  // set the analog input pin as input
  pinMode(AC_PIN, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  // capture the samples
  captureSamples();

  // calculate the frequency
  frequency = calculateFrequency();

  // calculate the total harmonic distortion
  thd = calculateTHD();

  // calculate the root mean square voltage
  rms = calculateRMS();

  // calculate the peak voltage
  peak = calculatePeak();

  // calculate the integrated voltage
  integrated = calculateIntegrated();

  // calculate the difference between rms and integrated
  difference = calculateDifference();

  // print the results
  printResults();

  // wait for one second
  delay(1000);
}

// function to capture the samples
void captureSamples() {
  // calculate the sampling period (ms)
  float T = 1000.0 / FS;

  // loop through the samples
  for (int i = 0; i < N; i++) {
    // read the analog value
    int value = analogRead(AC_PIN);

    // convert the value to voltage
    float voltage = value / ADC_COUNTS * VREF / RATIO;

    // store the voltage in the array
    samples[i] = voltage;

    // wait for the sampling period
    delay(T);
  }
}

// function to calculate the frequency
float calculateFrequency() {
  // initialize the variables
  float freq = 0;
  int count = 0;
  bool rising = false;

  // loop through the samples
  for (int i = 0; i < N - 1; i++) {
    // check if the current sample is rising
    if (samples[i + 1] > samples[i]) {
      // set the rising flag to true
      rising = true;
    }

    // check if the current sample is falling
    if (samples[i + 1] < samples[i]) {
      // check if the previous sample was rising
      if (rising) {
        // increment the count of zero crossings
        count++;

        // set the rising flag to false
        rising = false;
      }
    }
  }

  // calculate the frequency using the count of zero crossings
  freq = count / 2.0 * FS / N;

  // return the frequency
  return freq;
}

// function to calculate the total harmonic distortion
float calculateTHD() {
  // initialize the variables
  float thd = 0;
  float a0 = 0;
  float a1 = 0;
  float b1 = 0;
  float c1 = 0;
  float an = 0;
  float bn = 0;
  float cn = 0;
  float sum = 0;

  // calculate the coefficients of the fundamental frequency
  for (int i = 0; i < N; i++) {
    // calculate the angle
    float angle = 2 * PI * F * i / FS;

    // calculate the cosine and sine terms
    float cosTerm = cos(angle);
    float sinTerm = sin(angle);

    // update the coefficients
    a0 += samples[i];
    a1 += samples[i] * cosTerm;
    b1 += samples[i] * sinTerm;
  }

  // normalize the coefficients
  a0 /= N;
  a1 *= 2.0 / N;
  b1 *= 2.0 / N;

  // calculate the amplitude and phase of the fundamental frequency
  c1 = sqrt(a1 * a1 + b1 * b1);
  float phi1 = atan2(b1, a1);

  // loop through the harmonics
  for (int n = 2; n <= N / 2; n++) {
    // reset the coefficients
    an = 0;
    bn = 0;

    // calculate the coefficients of the nth harmonic
    for (int i = 0; i < N; i++) {
      // calculate the angle
      float angle = 2 * PI * n * F * i / FS;

      // calculate the cosine and sine terms
      float cosTerm = cos(angle);
      float sinTerm = sin(angle);

      // update the coefficients
      an += samples[i] * cosTerm;
      bn += samples[i] * sinTerm;
    }

    // normalize the coefficients
    an *= 2.0 / N;
    bn *= 2.0 / N;

    // calculate the amplitude and phase of the nth harmonic
    cn = sqrt(an * an + bn * bn);
    float phin = atan2(bn, an);

    // calculate the harmonic distortion
    float hd = cn / c1;

    // update the sum of harmonic distortions
    sum += hd * hd;
  }

  // calculate the total harmonic distortion
  thd = sqrt(sum) * 100;

  // return the total harmonic distortion
  return thd;
}

// function to calculate the root mean square voltage
float calculateRMS() {
  // initialize the variable
  float rms = 0;

  // loop through the samples
  for (int i = 0; i < N; i++) {
    // update the sum of squares
    rms += samples[i] * samples[i];
  }

  // divide the sum by the number of samples
  rms /= N;

  // take the square root
  rms = sqrt(rms);

  // return the root mean square voltage
  return rms;
}

// function to calculate the peak voltage
float calculatePeak() {
  // initialize the variable
  float peak = 0;

  // loop through the samples
  for (int i = 0; i < N; i++) {
    // check if the current sample is greater than the peak
    if (samples[i] > peak) {
      // update the peak
      peak = samples[i];
    }
  }

  // return the peak voltage
  return peak;
}

// function to calculate the integrated voltage
float calculateIntegrated() {
  // initialize the variable
  float integrated = 0;

  // loop through the samples
  for (int i = 0; i < N; i++) {
    // update the sum of samples
    integrated += samples[i];
  }

  // divide the sum by the number of samples
  integrated /= N;

  // return the integrated voltage
  return integrated;
}

// function to calculate the difference between rms and integrated
float calculateDifference() {
  // calculate the difference
  float difference = rms - integrated;

  // return the difference
  return difference;
}

// function to print the results
void printResults() {
  // print the frequency
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.println(" Hz");

  // print the total harmonic distortion
  Serial.print("Total Harmonic Distortion: ");
  Serial.print(thd);
  Serial.println(" %");

  // print the root mean square voltage
  Serial.print("Root Mean Square Voltage: ");
  Serial.print(rms);
  Serial.println(" V");

  // print the peak voltage
  Serial.print("Peak Voltage: ");
  Serial.print(peak);
  Serial.println(" V");

  // print the integrated voltage
  Serial.print("Integrated Voltage: ");
  Serial.print(integrated);
  Serial.println(" V");

  // print the difference between rms and integrated
  Serial.print("Difference: ");
  Serial.print(difference);
  Serial.println(" V");
}
