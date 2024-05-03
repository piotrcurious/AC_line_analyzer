#include <Arduino.h>

const int arraySize = 10; // Adjust this to your actual array size
const int searchWindow = 5; // Adjust search window size (odd number recommended)

float array1[arraySize];
float array2[arraySize];

float calculatePearsonCorrelation(float* array1, float* array2, int size) {
  float sum1 = 0, sum2 = 0, product = 0;
  float sum1Sq = 0, sum2Sq = 0;

  for (int i = 0; i < size; i++) {
    sum1 += array1[i];
    sum2 += array2[i];
    product += array1[i] * array2[i];
    sum1Sq += array1[i] * array1[i];
    sum2Sq += array2[i] * array2[i];
  }

  float numerator = product - (sum1 * sum2) / size;
  float denominator = sqrt((sum1Sq - pow(sum1, 2.0) / size) * (sum2Sq - pow(sum2, 2.0) / size));

  // Handle division by zero
  if (denominator < 0.00001) {
    return 0.0;
  }

  return numerator / denominator;
}

int findPeak(float* array, int size, int searchWindow) {
  int peakIndex = 0;
  float peakValue = -INFINITY; // Initialize with negative infinity

  for (int i = 0; i < size; i++) {
    float windowSum = 0.0;
    // Calculate average correlation within search window
    for (int j = 0; j < searchWindow; j++) {
      int index = (i + j + size) % size; // Handle wraparound for cyclic data
      windowSum += calculatePearsonCorrelation(array, &array[index], searchWindow);
    }

    float averageCorrelation = windowSum / searchWindow;
    if (averageCorrelation > peakValue) {
      peakIndex = i;
      peakValue = averageCorrelation;
    }
  }

  return peakIndex;
}

void setup() {
  Serial.begin(115200);

  // Fill your arrays with signal data here
  // For example:
  for (int i = 0; i < arraySize; i++) {
    array1[i] = sin(2 * PI * i / 10.0); // Replace with your actual signal data
    array2[i] = sin(2 * PI * (i + 2) / 10.0); // Replace with your actual signal data (shifted version)
  }
}

void loop() {
  // Calculate Pearson correlation coefficient
  float correlation = calculatePearsonCorrelation(array1, array2, arraySize);

  Serial.print("Pearson correlation: ");
  Serial.println(correlation);

  // Peak detection using Pearson correlation within search window
  int peakIndex1 = findPeak(array1, arraySize, searchWindow);
  int peakIndex2 = findPeak(array2, arraySize, searchWindow);

  // Calculate shift based on peak indices
  int shift = peakIndex2 - peakIndex1;
  if (shift < 0) {
    shift += arraySize; // Handle negative shift for cyclic data
  }

  Serial.print("Best shift (peak alignment): ");
  Serial.println(shift);

  delay(1000);
}
