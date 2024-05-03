#include <Arduino.h>

const int arraySize = 10; // Adjust this to your actual array size

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

int findPeakIndex(float* array, int size) {
  int peakIndex = 0;
  float peakValue = array[0];

  for (int i = 1; i < size; i++) {
    if (array[i] > peakValue) {
      peakIndex = i;
      peakValue = array[i];
    }
  }

  return peakIndex;
}

int binarySearchPeak(float* array, int size, float target) {
  int low = 0;
  int high = size - 1;
  int mid;

  while (low <= high) {
    mid = low + (high - low) / 2;

    if (array[mid] == target) {
      return mid;
    } else if (array[mid] > target) {
      high = mid - 1;
    } else {
      low = mid + 1;
    }
  }

  // If not found exactly, return closest index
  if (array[low] > target) {
    return low;
  } else {
    return high;
  }
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

  // Exponent detection (replace with your preferred method)
  // This example simply finds the peak index
  int peakIndex1 = findPeakIndex(array1, arraySize);
  int peakIndex2 = findPeakIndex(array2, arraySize);

  // Calculate shift using binary search to find peak alignment
  float correlationValues[arraySize];
  for (int i = 0; i < arraySize; i++) {
    float shiftedArray2[arraySize];
    for (int j = 0; j < arraySize; j++) {
      shiftedArray2[j] = array2[(j + i) % arraySize];
    }
    correlationValues[i] = calculatePearsonCorrelation(array1, shiftedArray2, arraySize);
  }

  int bestShift = binarySearchPeak(correlationValues, arraySize, correlation);

  Serial.print("Best shift (peak alignment): ");
  Serial.println(bestShift);

  delay(1000);
}
