#include <WiFi.h>
#include "esp_wifi.h"
#include "lwip/udp.h"

// Hardware Synchronization
SemaphoreHandle_t txSemaphore;
volatile uint32_t lastTxDuration = 0;
volatile uint32_t startTime = 0;

// 1. Hardware Interrupt Callback
void IRAM_ATTR wifi_tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t data_len, bool status) {
    lastTxDuration = micros() - startTime;
    
    // Wake up the processing task immediately
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(txSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

struct udp_pcb *pcb;
ip_addr_t target_addr;

void setup() {
    Serial.begin(115200);
    WiFi.begin("SSID", "PASS");
    while (WiFi.status() != WL_CONNECTED) delay(500);

// Force WiFi to use 24Mbps for Non-Management frames (experimental)
esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20); 
  

  
    // 2. Initialize Semaphore and WiFi Hook
    txSemaphore = xSemaphoreCreateBinary();
    esp_wifi_set_tx_done_cb(wifi_tx_done_cb);

    pcb = udp_new();
    ipaddr_aton("239.1.2.3", &target_addr);
    WiFi.setSleep(false);

    // Create a dedicated High-Priority Task for Transmission
    xTaskCreatePinnedToCore(udpTask, "udp_task", 4096, NULL, 20, NULL, 1);
}

void udpTask(void *pvParameters) {
    const size_t len = 1450;
    
    for (;;) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p) {
            startTime = micros();
            
            // Push to LwIP
            if (udp_sendto(pcb, p, &target_addr, 1234) == ERR_OK) {
                // 3. WAIT here until the hardware clears the buffer
                // This blocks the task, letting other code run.
                if (xSemaphoreTake(txSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Success! Optional: Log the speed
                    // Serial.printf("Time: %u us\n", lastTxDuration);
                }
            }
            pbuf_free(p);
        }
        // Small yield to prevent watchdog if hardware fails
        vTaskDelay(1); 
    }
}

void loop() { 
    // Loop stays empty; all work is in the high-priority task
}
