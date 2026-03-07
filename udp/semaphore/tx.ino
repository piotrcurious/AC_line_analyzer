#include <WiFi.h>
#include "esp_wifi.h"
#include "lwip/udp.h"
#include "esp_timer.h"
#include "freertos/semphr.h"

// Hardware Synchronization
static SemaphoreHandle_t txSemaphore = NULL;
volatile int64_t lastTxDurationUs = 0;
volatile int64_t startTimeUs = 0;

// 1. Hardware Interrupt Callback
// NOTE: check your SDK's required prototype for tx-done callback and enable in menuconfig if needed.
void IRAM_ATTR wifi_tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t data_len, bool status) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Use esp_timer_get_time() which returns int64_t microseconds and is safe in ISR
    int64_t now = esp_timer_get_time();
    lastTxDurationUs = now - startTimeUs;

    if (txSemaphore) {
        xSemaphoreGiveFromISR(txSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

static struct udp_pcb *pcb = NULL;
static ip_addr_t target_addr;

void udpTask(void *pvParameters) {
    const size_t len = 1450;
    const TickType_t waitTicks = pdMS_TO_TICKS(100);

    // Make sure PCB exists and is connected
    if (!pcb) {
        vTaskDelay(pdMS_TO_TICKS(10));
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        // Allocate pbuf and fill it
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (!p) {
            // Allocation failed; back off to avoid busy loop
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Fill payload with real data (example: simple pattern). p->payload points to first fragment.
        // If payload is chained, you must fill each fragment using p->payload and p->len
        memset(p->payload, 0xAA, p->len); // replace with real data fill logic

        // Capture start time (64-bit). Use volatile shared variable so ISR can read it.
        startTimeUs = esp_timer_get_time();

        // Use udp_send (pcb already connected) for slightly simpler API
        err_t res = udp_send(pcb, p);
        if (res == ERR_OK) {
            // Wait for ISR to signal actual HW completion
            if (xSemaphoreTake(txSemaphore, waitTicks) == pdTRUE) {
                // Success — lastTxDurationUs holds the duration
                // Serial.printf("HW tx time: %lld us\n", (long long)lastTxDurationUs);
            } else {
                // Timeout — no tx-done interrupt arrived
                // Handle as needed (drop, retry, log)
                // Serial.println("TX done timeout");
            }
        } else {
            // udp_send failed
            // Serial.printf("udp_send failed: %d\n", res);
        }

        pbuf_free(p);

        // Small delay to yield
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.begin("SSID", "PASS");
    while (WiFi.status() != WL_CONNECTED) delay(500);

    // Try to set bandwidth / other wifi params if supported by SDK
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);

    // Create semaphore and check
    txSemaphore = xSemaphoreCreateBinary();
    if (!txSemaphore) {
        Serial.println("Failed to create txSemaphore");
        while (1) delay(1000);
    }

    // Register callback (ensure SDK supports it and it's enabled)
    esp_err_t cb_res = esp_wifi_set_tx_done_cb(wifi_tx_done_cb);
    if (cb_res != ESP_OK) {
        Serial.printf("esp_wifi_set_tx_done_cb failed: %d\n", cb_res);
        // Optionally continue without callback or abort
    }

    // Prepare pcb and remote address
    pcb = udp_new();
    if (!pcb) {
        Serial.println("udp_new failed");
        while (1) delay(1000);
    }

    if (!ipaddr_aton("239.1.2.3", &target_addr)) {
        Serial.println("invalid target address");
        while (1) delay(1000);
    }

    // Connect PCB once to simplify sends (use udp_send)
    err_t con = udp_connect(pcb, &target_addr, 1234);
    if (con != ERR_OK) {
        Serial.printf("udp_connect failed: %d\n", con);
        // decide to continue or abort
    }

    // If multicast, you may need to set TTL and/or use IGMP to join groups for recv. For TX, setting route is normally enough.

    WiFi.setSleep(false);

    // Create task pinned to core 1 with a high priority but below critical RTOS levels
    BaseType_t t = xTaskCreatePinnedToCore(udpTask, "udp_task", 4096, NULL, 20, NULL, 1);
    if (t != pdPASS) {
        Serial.println("Task creation failed");
        while (1) delay(1000);
    }
}

void loop() {
    // empty: work happens in task
}
