#include <WiFi.h>
#include "lwip/udp.h"
#include "lwip/pbuf.h"

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* multicast_ip = "239.1.2.3";
const uint16_t port = 1234;

struct udp_pcb *pcb;
ip_addr_t target_addr;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  // 1. Create a new UDP Control Block (PCB)
  pcb = udp_new();
  ipaddr_aton(multicast_ip, &target_addr);

  // 2. Optimization: Turn off WiFi sleep to stop the radio from power-cycling
  WiFi.setSleep(false); 
  
  Serial.println("LwIP Raw UDP Sender Initialized");
}

void sendLargePacket() {
  const size_t payloadSize = 1450;
  static uint8_t frameCounter = 0;

  // 3. Allocate a 'pbuf' (Packet Buffer) 
  // PBUF_RAM allocates the memory in one contiguous chunk
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, payloadSize, PBUF_RAM);

  if (p != NULL) {
    // Fill buffer with dummy data
    memset(p->payload, frameCounter++, payloadSize);

    // 4. Send the packet
    // This returns an 'err_t' which is your feedback!
    err_t err = udp_sendto(pcb, p, &target_addr, port);

    if (err == ERR_OK) {
      // Success! Packet handed off to the WiFi hardware buffer
    } else {
      Serial.printf("Send failed with error: %d\n", err);
    }

    // 5. CRITICAL: You must free the pbuf or you will leak memory instantly
    pbuf_free(p);
  }
}

void loop() {
  // Push the hardware to its limit
  sendLargePacket();
}
