#include <WiFi.h>
#include "esp_wifi.h"
#include "lwip/udp.h"

// Tracking variables
volatile uint32_t startTime = 0;
volatile uint32_t endTime = 0;
volatile bool txFinished = false;

// 1. The Hardware Callback
// This is triggered by the WiFi High-Priority Interrupt
void IRAM_ATTR wifi_tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t data_len, bool status) {
    endTime = micros(); // Capture the exact microsecond
    txFinished = true;
}

struct udp_pcb *pcb;
ip_addr_t target_addr;

void setup() {
    Serial.begin(115200);
    WiFi.begin("SSID", "PASS");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    // 2. Register the hardware TX Done callback
    esp_wifi_set_tx_done_cb(wifi_tx_done_cb);

    pcb = udp_new();
    ipaddr_aton("239.1.2.3", &target_addr);
    WiFi.setSleep(false); // Critical for timing consistency
}

void sendMeasuredPacket() {
    const size_t len = 1450;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    
    if (p) {
        txFinished = false;
        startTime = micros(); // Start timing before handoff
        
        // 3. Hand off to LwIP
        err_t err = udp_sendto(pcb, p, &target_addr, 1234);
        
        if (err == ERR_OK) {
            // Wait for the hardware callback (don't do this in production loops!)
            uint32_t timeout = millis();
            while(!txFinished && (millis() - timeout < 100)); 
            
            if(txFinished) {
                uint32_t duration = endTime - startTime;
                Serial.printf("Hardware Sent 1450B in: %u microseconds\n", duration);
            }
        }
        pbuf_free(p);
    }
}

void loop() {
    sendMeasuredPacket();
    delay(1000);
}
