/* ESP32 Arduino core 3.3.12 compatible example
   - Uses esp_wifi_set_tx_done_cb() (check return)
   - Measures HW TX finish time using esp_timer_get_time()
   - Protects 64-bit timestamp via portMUX critical section (ISR-safe)
   - Uses UDP pcb connected once + pbuf allocation + safe payload fill
*/

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "lwip/err.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

// ---- CONFIG ----
const char* SSID = "SSID";
const char* PASS = "PASS";
const char* TARGET_IP = "239.1.2.3";
const uint16_t TARGET_PORT = 1234;
const size_t UDP_PAYLOAD_LEN = 1450;
const TickType_t TX_DONE_TIMEOUT = pdMS_TO_TICKS(100);

// ---- Sync primitives and shared state ----
static SemaphoreHandle_t txSemaphore = NULL;
static struct udp_pcb *pcb = NULL;
static ip_addr_t target_addr;

// Protect 64-bit shared variables (startTimeUs, lastTxDurationUs)
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int64_t startTimeUs = 0;
volatile int64_t lastTxDurationUs = 0;

// --------------------- ISR callback ---------------------
// Signature used by esp_wifi_set_tx_done_cb
void IRAM_ATTR wifi_tx_done_cb(uint8_t ifidx, uint8_t *data, uint16_t data_len, bool status) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Enter an ISR-safe critical section
    portENTER_CRITICAL_ISR(&timerMux);
    int64_t now = esp_timer_get_time();
    lastTxDurationUs = now - startTimeUs;
    portEXIT_CRITICAL_ISR(&timerMux);

    // Give the semaphore to wake the waiting task
    if (txSemaphore) {
        xSemaphoreGiveFromISR(txSemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

// --------------------- UDP TX task ---------------------
void udpTask(void *pvParameters) {
    (void) pvParameters;

    if (!pcb) {
        Serial.println("udpTask: pcb is NULL, exiting task");
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, UDP_PAYLOAD_LEN, PBUF_RAM);
        if (!p) {
            // allocation failed; back off a bit
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        // Fill chained fragments safely (pbuf may be chained)
        for (struct pbuf *q = p; q != NULL; q = q->next) {
            // Example pattern fill; replace with your real payload
            memset(q->payload, 0xAA, q->len);
        }

        // Capture 64-bit start time atomically
        portENTER_CRITICAL(&timerMux);
        startTimeUs = esp_timer_get_time();
        portEXIT_CRITICAL(&timerMux);

        // Send (pcb was connected in setup so use udp_send)
        err_t res = udp_send(pcb, p);
        if (res == ERR_OK) {
            // Wait for the ISR to notify real HW completion
            if (xSemaphoreTake(txSemaphore, TX_DONE_TIMEOUT) == pdTRUE) {
                // Read lastTxDurationUs atomically
                int64_t duration;
                portENTER_CRITICAL(&timerMux);
                duration = lastTxDurationUs;
                portEXIT_CRITICAL(&timerMux);

                // Optionally sample/log (avoid heavy logging in tight loops)
                Serial.printf("HW tx done (us): %lld\n", (long long)duration);
            } else {
                // Timeout — TX-done didn't arrive in time
                Serial.println("TX-done timeout (no ISR)");
            }
        } else {
            Serial.printf("udp_send error: %d\n", res);
        }

        pbuf_free(p);

        // tiny yield so watchdog and lower-priority tasks can run
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --------------------- Arduino setup/loop ---------------------
void setup() {
    Serial.begin(115200);
    delay(10);

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print('.');
        delay(500);
    }
    Serial.println();
    Serial.print("IP: "); Serial.println(WiFi.localIP());

    // Reduce sleep to avoid WiFi going to low-power
    WiFi.setSleep(false);

    // Try to set bandwidth (no-op if driver ignores it)
    esp_err_t bres = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (bres != ESP_OK) {
        Serial.printf("esp_wifi_set_bandwidth err %d\n", bres);
    }

    // Create semaphore
    txSemaphore = xSemaphoreCreateBinary();
    if (!txSemaphore) {
        Serial.println("Failed to create txSemaphore");
        while (1) delay(1000);
    }

    // Register TX-done callback (check return value)
    esp_err_t cb_res = esp_wifi_set_tx_done_cb(wifi_tx_done_cb);
    if (cb_res != ESP_OK) {
        Serial.printf("esp_wifi_set_tx_done_cb failed: %d\n", cb_res);
        // If this fails, the rest of the timing mechanism won't work reliably.
        // You can continue (but no ISR notifications) or halt depending on your needs.
    } else {
        Serial.println("Registered tx_done callback");
    }

    // prepare UDP pcb and connect once for simpler sends
    pcb = udp_new();
    if (!pcb) {
        Serial.println("udp_new failed");
        while (1) delay(1000);
    }

    if (!ipaddr_aton(TARGET_IP, &target_addr)) {
        Serial.println("invalid target address");
        while (1) delay(1000);
    }

    err_t con = udp_connect(pcb, &target_addr, TARGET_PORT);
    if (con != ERR_OK) {
        Serial.printf("udp_connect failed: %d\n", con);
        // Can continue — udp_sendto could still be used — but log it.
    }

    // Create high-priority task pinned to core 1 (adjust priority as needed)
    BaseType_t t = xTaskCreatePinnedToCore(udpTask, "udp_task", 6144, NULL, 20, NULL, 1);
    if (t != pdPASS) {
        Serial.println("Task creation failed");
        while (1) delay(1000);
    }
}

void loop() {
    // nothing here — work is in the task
    vTaskDelay(pdMS_TO_TICKS(1000));
}
