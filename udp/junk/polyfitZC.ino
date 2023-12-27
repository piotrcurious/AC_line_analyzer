#include <Polynomial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi settings
const char* ssid = "yourNetwork";
const char* password = "yourPassword";

// UDP settings
WiFiUDP udp;
const int localPort = 2000; // port to listen on
const int remotePort = 2000; // port to send to

// ADC settings
const int analogPin = 34; // pin to read AC voltage from
const int samples = 256; // number of samples to take per period
const float vref = 3.3; // reference voltage of ADC
const int adcResolution = 4096; // resolution of ADC
const float voltageFactor = 0.5; // voltage divider factor

// Polynomial settings
const int degree = 5; // degree of polynomial function
float coeffs[degree + 1]; // array to store polynomial coefficients
Polynomial poly(coeffs, degree); // polynomial object

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Initialize UDP
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());

  // Initialize ADC
  analogReadResolution(12); // set ADC resolution to 12 bits
  analogSetAttenuation(ADC_11db); // set ADC attenuation to 11 dB
}

void loop() {
  // Read and store samples
  float x[samples]; // array to store x values
  float y[samples]; // array to store y values
  for (int i = 0; i < samples; i++) {
    int adcValue = analogRead(analogPin); // read ADC value
    float voltage = adcValue * vref / adcResolution / voltageFactor; // convert to voltage
    x[i] = i; // store x value
    y[i] = voltage; // store y value
  }

  // Fit polynomial function to samples
  polyFit(x, y, samples, degree, coeffs); // find polynomial coefficients using least squares method
  Serial.print("Polynomial function: ");
  poly.printPoly(); // print polynomial function

  // Find zero crossing positions
  float roots[degree]; // array to store roots of polynomial function
  int numRoots = poly.solve(roots); // find roots of polynomial function
  Serial.print("Zero crossing positions: ");
  for (int i = 0; i < numRoots; i++) {
    Serial.print(roots[i]); // print root value
    Serial.print(" ");
  }
  Serial.println();
}

// Polynomial fitting function using least squares method
// Based on https://forum.arduino.cc/t/how-to-use-a-polynomial-fit-equation-for-compensating-a-value/194668/2[^1^][1]
void polyFit(float x[], float y[], int n, int m, float coeffs[]) {
  // initialize arrays
  float X[2 * m + 1];
  float Y[m + 1];
  float B[m + 1][m + 2];
  float a[m + 1];

  // calculate the values of X[]
  for (int i = 0; i < 2 * m + 1; i++) {
    X[i] = 0;
    for (int j = 0; j < n; j++) {
      X[i] += pow(x[j], i);
    }
  }

  // calculate the values of Y[] and B[][]
  for (int i = 0; i <= m; i++) {
    Y[i] = 0;
    for (int j = 0; j < n; j++) {
      Y[i] += pow(x[j], i) * y[j];
    }
    for (int j = 0; j <= m; j++) {
      B[i][j] = X[i + j];
    }
  }

  // calculate the values of B[][] and a[]
  for (int i = 0; i <= m; i++) {
    B[i][m + 1] = Y[i];
  }
  m = m + 1;
  for (int i = 0; i < m; i++) {
    for (int k = i + 1; k < m; k++) {
      if (B[i][i] < B[k][i]) {
        for (int j = 0; j <= m; j++) {
          float temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }
      }
    }
  }
  for (int i = 0; i < m - 1; i++) {
    for (int k = i + 1; k < m; k++) {
      float t = B[k][i] / B[i][i];
      for (int j = 0; j <= m; j++) {
        B[k][j] -= t * B[i][j];
      }
    }
  }
  for (int i = m - 1; i >= 0; i--) {
    a[i] = B[i][m];
    for (int j = 0; j < m; j++) {
      if (j != i) {
        a[i] -= B[i][j] * a[j];
      }
    }
    a[i] /= B[i][i];
  }

  // store the coefficients
  for (int i = 0; i < m; i++) {
    coeffs[i] = a[i];
  }
}
