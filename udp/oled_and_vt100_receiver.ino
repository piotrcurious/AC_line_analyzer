#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi settings
const char* ssid = "yourNetwork";
const char* password = "yourPassword";

// UDP settings
WiFiUDP udp;
const int localPort = 2000; // port to listen on
const int remotePort = 2000; // port to send to

// OLED settings
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// VT100 settings
#define ESC 27 // escape character
#define CR 13 // carriage return
#define LF 10 // line feed
#define MAX_ROWS 24 // number of rows on terminal
#define MAX_COLS 80 // number of columns on terminal
char screen[MAX_ROWS][MAX_COLS]; // buffer to store terminal screen
int row = 0; // current cursor row
int col = 0; // current cursor column
int fg = 1; // current foreground color (0 = black, 1 = white)
int bg = 0; // current background color (0 = black, 1 = white)

// AC wave settings
const int MAX_SAMPLES = 256; // number of samples per period
int wave[MAX_SAMPLES]; // buffer to store wave samples
int waveIndex = 0; // index of wave buffer
int packetIndex = 0; // index of received packet
int expectedIndex = 0; // index of expected packet
int lostPackets = 0; // number of lost packets

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

  // Initialize OLED
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // initialize with the I2C addr 0x3C
  display.clearDisplay(); // clear the buffer
  display.display(); // show the buffer on the screen

  // Initialize VT100
  clearScreen(); // clear the screen buffer
  printScreen(); // print the screen buffer on the OLED
}

void loop() {
  // Receive UDP packet
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read packet data
    byte data[packetSize];
    udp.read(data, packetSize);

    // Check packet index
    int receivedIndex = data[0];
    if (receivedIndex != packetIndex) {
      // Count lost packets
      lostPackets += receivedIndex - packetIndex;
      // Update packet index
      packetIndex = receivedIndex;
    }

    // Store wave samples
    for (int i = 1; i < packetSize; i += 2) {
      byte high = data[i];
      byte low = data[i + 1];
      int value = word(high, low);
      float voltage = value / 100.0;
      wave[waveIndex++] = voltage;
    }

    // Increment packet index
    packetIndex++;

    // Check if wave buffer is full
    if (waveIndex == MAX_SAMPLES) {
      // Draw wave graph on OLED
      drawWaveGraph();

      // Calculate peak, RMS, THD, frequency, and integrated voltage
      float peak = maxWave();
      float rms = rmsWave();
      float thd = thdWave();
      float freq = freqWave();
      float integ = integWave();

      // Calculate lost packets percentage
      float lostPercent = 100.0 * lostPackets / expectedIndex;

      // Print values on VT100 terminal
      printVT100(peak, rms, thd, freq, integ, lostPercent);

      // Reset wave index and packet index
      waveIndex = 0;
      packetIndex = 0;
      expectedIndex = 0;
      lostPackets = 0;
    }
  }
}

// Draw wave graph on OLED
void drawWaveGraph() {
  display.clearDisplay(); // clear the buffer
  display.drawFastVLine(0, 0, 64, WHITE); // draw y-axis
  display.drawFastHLine(0, 63, 128, WHITE); // draw x-axis
  display.setCursor(0, 0); // set cursor position
  display.print("V"); // print y-axis label
  for (int i = 0; i < MAX_SAMPLES; i++) {
    int x = map(i, 0, MAX_SAMPLES, 0, 128); // map sample index to x coordinate
    int y = map(wave[i], 0, 5, 63, 0); // map voltage value to y coordinate
    display.drawPixel(x, y, WHITE); // draw pixel on buffer
  }
  display.display(); // show the buffer on the screen
}

// Calculate maximum value of wave
float maxWave() {
  float max = 0;
  for (int i = 0; i < MAX_SAMPLES; i++) {
    if (wave[i] > max) {
      max = wave[i];
    }
  }
  return max;
}

// Calculate root mean square of wave
float rmsWave() {
  float sum = 0;
  for (int i = 0; i < MAX_SAMPLES; i++) {
    sum += sq(wave[i]);
  }
  float mean = sum / MAX_SAMPLES;
  float rms = sqrt(mean);
  return rms;
}

// Calculate total harmonic distortion of wave
float thdWave() {
  float fundamental = 0;
  float harmonics = 0;
  for (int k = 1; k <= 10; k++) {
    float ak = 0;
    float bk = 0;
    for (int n = 0; n < MAX_SAMPLES; n++) {
      float theta = 2 * PI * n * k / MAX_SAMPLES;
      ak += wave[n] * cos(theta);
      bk += wave[n] * sin(theta);
    }
    ak = 2 * ak / MAX_SAMPLES;
    bk = 2 * bk / MAX_SAMPLES;
    float ck = sqrt(sq(ak) + sq(bk));
    if (k == 1) {
      fundamental = ck;
    } else {
      harmonics += sq(ck);
    }
  }
  harmonics = sqrt(harmonics);
  float thd = harmonics / fundamental;
  return thd;
}

// Calculate frequency of wave
float freqWave() {
  int zeroCrossings = 0;
  for (int i = 1; i < MAX_SAMPLES; i++) {
    if (wave[i - 1] < 2.5 && wave[i] > 2.5) {
      zeroCrossings++;
    }
  }
  float freq = zeroCrossings / 2.0;
  return freq;
}

// Calculate integrated voltage of wave
float integWave() {
  float sum = 0;
  for (int i = 0; i < MAX_SAMPLES; i++) {
    sum += wave[i];
  }
  float integ = sum / MAX_SAMPLES;
  return integ;
}

// Print values on VT100 terminal
void printVT100(float peak, float rms, float thd, float freq, float integ, float lost) {
  clearScreen(); // clear the screen buffer
  setCursor(0, 0); // set cursor position
  print("Peak voltage: "); // print text
  print(peak); // print value
  print(" V"); // print unit
  setCursor(1, 0); // set cursor position
  print("RMS voltage: "); // print text
  print(rms); // print value
  print(" V"); // print unit
  setCursor(2, 0); // set cursor position
  print("THD: "); // print text
  print(thd); // print value
  print(" %"); // print unit
  setCursor(3, 0); // set cursor position
  print("Frequency: "); // print text
  print(freq); // print value
  print(" Hz"); // print unit
  setCursor(4, 0); // set cursor position
  print("Integrated voltage: "); // print text
  print(integ); // print value
  print(" V"); // print unit
  setCursor(5, 0); // set cursor position
  print("Lost packets: "); // print text
  print(lost); // print value
  print(" %"); // print unit
  printScreen(); // print the screen buffer on the OLED
}

// Clear the screen buffer
void clearScreen() {
  for (int r = 0; r < MAX_ROWS; r++) {
    for (int c = 0; c < MAX_COLS; c++) {
      screen[r][c] = ' ';
    }
  }
}

// Set the cursor position
void setCursor(int r, int c) {
  row = constrain(r, 0, MAX_ROWS - 1);
  col = constrain(c, 0, MAX_COLS - 1);
}

// Print a character on the screen buffer
void print(char ch) {
  screen[row][col] = ch;
  col++;
  if (col >= MAX_COLS) {
    col = 0;
    row++;
    if (row >= MAX_ROWS) {
      row = 0;
    }
  }
}

// Print a string on the screen buffer
void print(const char* str) {
  while (*str) {
    print(*str);
    str++;
  }
}

// Print a number on the screen buffer
void print(float num) {
  char buffer[10];
  dtostrf(num, 4, 2, buffer);
  print(buffer);
}

// Print the screen buffer on the OLED
void printScreen() {
  display.clearDisplay(); // clear the buffer
  display.setTextSize(1); // set text size
  display.setTextColor(WHITE); // set text color
  display.setCursor(0, 0); // set cursor position
  for (int r = 0; r < MAX_ROWS; r++) {
    for (int c = 0; c < MAX_COLS; c++) {
      display.write(screen[r][c]); // write character on buffer
    }
    display.println(); // print new line on buffer
  }
  display.display(); // show the buffer on the screen
}
