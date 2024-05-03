#include <Arduino.h>
#include "FFT.h" // Include the library for Fast Fourier Transform

// Define the size of the arrays and the FFT
const int N = 256; // Size of the arrays and FFT
double array1[N];
double array2[N];
double fft1[N];
double fft2[N];
double result[N];

// Function to fill the arrays with sample data
void fillArrays() {
  // Fill array1 and array2 with your data
}

// Function to perform the FFT on both arrays
void performFFT() {
  FFT.fft(array1, fft1, N);
  FFT.fft(array2, fft2, N);
}

// Function to perform phase correlation
void phaseCorrelation() {
  for (int i = 0; i < N; i++) {
    double real1 = fft1[i * 2]; // Real part of the first array
    double imag1 = fft1[i * 2 + 1]; // Imaginary part of the first array
    double real2 = fft2[i * 2]; // Real part of the second array
    double imag2 = fft2[i * 2 + 1]; // Imaginary part of the second array

    // Compute the cross-power spectrum
    double magnitude = sqrt(real1 * real1 + imag1 * imag1) * sqrt(real2 * real2 + imag2 * imag2);
    if (magnitude == 0) magnitude = 1; // Avoid division by zero

    // Normalize and compute the conjugate product
    real1 = (real1 * real2 + imag1 * imag2) / magnitude;
    imag1 = (imag1 * real2 - real1 * imag2) / magnitude;

    // Inverse FFT to get the correlation
    fft1[i * 2] = real1;
    fft1[i * 2 + 1] = imag1;
  }

  FFT.ifft(fft1, result, N);
}

void setup() {
  Serial.begin(115200);
  fillArrays();
  performFFT();
  phaseCorrelation();

  // Find the index of the maximum value in the result
  int maxIndex = 0;
  for (int i = 1; i < N; i++) {
    if (result[i] > result[maxIndex]) {
      maxIndex = i;
    }
  }

  // The index of the maximum value corresponds to the phase shift
  Serial.print("Phase shift: ");
  Serial.println(maxIndex);
}

void loop() {
  // Your code here
}
