#include <stdio.h>
#include "driver/i2s.h"
//#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <stdlib.h>
#include <sys/param.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/timers.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
//#include "protocol_examples_common.h"
#include "esp_err.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include <math.h>





#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/* ------- I²S / Ringbuffer ------- */
#define RB_CHUNK_BYTES   128            // ∑ genau ein DMA-Block
#define RB_NUM_CHUNKS    32             // 64 × 128 B = 8 kB (anpassen ≙ Pufferlänge)
static RingbufHandle_t audio_rb;        // globales Handle
//----------------------------------------

#define PIN_LED      2
#define I2S_NUM         I2S_NUM_0
#define I2S_CLK      3      // Bit Clock (SCK)
#define I2S_WS       8      // Word Select (WS)
#define I2S_SD       38      // Serial Data (SD)

static const char *TAG = "UDP_unicast";
#define WIFI_SSID     "iPhone von Leonard " //"FRITZ!Box 7412"
#define WIFI_PASS      "12341234"//"03829891029701011627"
#define HOST_IP_ADDR "172.20.10.2"//"192.168.178.41"   // Empfänger
#define DEST_PORT    7000


// Event-Gruppe für Verbindungsstatus
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static const char *TAG_WIFI = "wifi_init";

/* ---------- LED helper ---------- */
static inline void led_on (void) { gpio_set_level(PIN_LED, 1); }
static inline void led_off(void) { gpio_set_level(PIN_LED, 0); }
//-----------------------------------------------

static void io_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode         = GPIO_MODE_OUTPUT
    };
    gpio_config(&io);
}

void i2s_init(void)
{
    // 44,1 kHz × 32 bit × 2 Kanäle → 2,8224 MHz BCLK
    i2s_config_t cfg = {
        .mode                 = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate          = 16000,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1 | ESP_INTR_FLAG_IRAM,
        .dma_buf_count        = 16,
        .dma_buf_len          = 64,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,     
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM, &cfg, 0, NULL));

    const i2s_pin_config_t pins = {
        .bck_io_num   = I2S_CLK,
        .ws_io_num    = I2S_WS,
        .data_out_num = -1,
        .data_in_num  = I2S_SD
    };
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM, &pins));

    ESP_ERROR_CHECK(i2s_set_clk(I2S_NUM, 16000,
                                I2S_BITS_PER_SAMPLE_32BIT,
                                I2S_CHANNEL_STEREO));

    audio_rb = xRingbufferCreate(RB_CHUNK_BYTES * RB_NUM_CHUNKS,RINGBUF_TYPE_BYTEBUF);
       
    assert(audio_rb);

    ESP_ERROR_CHECK(i2s_start(I2S_NUM));
}



static void i2s_reader_task(void *arg)
{
    int32_t dma_buf[RB_CHUNK_BYTES / 4];       // DMA-Puffer (32-bit)
    uint8_t be_buf[RB_CHUNK_BYTES / 2];        // Big-Endian Ausgabe-Puffer (16-bit PCM)
    size_t bytes_read;

    const int extra_shift = 2;  // Je größer, desto leiser und weniger Übersteuerung.

    while (1) {
        if (i2s_read(I2S_NUM, dma_buf, sizeof(dma_buf),
                     &bytes_read, portMAX_DELAY) != ESP_OK) continue;

        uint8_t *w = be_buf;
        int32_t *r = dma_buf;
        int frames = bytes_read / 8;   // 8 Bytes pro Stereo-Frame (32-bit Stereo)

        for (int i = 0; i < frames; ++i) {
            int16_t l16 = (int16_t)((r[0] >> (8 + extra_shift)));
            int16_t r16 = (int16_t)((r[1] >> (8 + extra_shift)));


            *w++ = (l16 >> 8) & 0xFF; *w++ = l16 & 0xFF;
            *w++ = (r16 >> 8) & 0xFF; *w++ = r16 & 0xFF;

            r += 2;
        }

        // Daten in den Ringpuffer senden:
        xRingbufferSend(audio_rb, be_buf, frames * 4, portMAX_DELAY);
    }
}


// Event-Handler für Wi-Fi Events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG_WIFI, "Disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "Waiting for IP...");
    // Warte, bis die Verbindung hergestellt ist
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG_WIFI, "Connected!");
}
static void udp_stream_task(void *arg)
{
    //uint32_t pkt_cnt = 0;
    uint32_t pkt_cnt = 0, pkt_sec = 0;
    int64_t last = esp_timer_get_time();
    

    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);



    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ESP_LOGE(TAG, "socket fail"); vTaskDelete(NULL); }

    
    const size_t MTU_PAYLOAD = 512;            // maximal ohne Fragmentierung
    uint8_t pkt[MTU_PAYLOAD];

    while (1) {
        /* 1472 Byte ≈ 8,3 ms Audio → ~120 Pakete/s                   */
        size_t grabbed = 0;
        while (grabbed < MTU_PAYLOAD) {
            size_t got;
            uint8_t *p = xRingbufferReceiveUpTo(audio_rb, &got,
                                                portMAX_DELAY,
                                                MTU_PAYLOAD - grabbed);
            memcpy(pkt + grabbed, p, got);
            grabbed += got;
            vRingbufferReturnItem(audio_rb, p);
        }

        /* schicken, ggf. mit Back-off warten                           */
        while (1) {
            ssize_t sent = sendto(sock, pkt, MTU_PAYLOAD, 0,
                (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            
            if (sent >= 0) {
                    // nach jedem erfolgreichen sendto:
                    pkt_sec++;
                    int64_t now = esp_timer_get_time();
                    if (now - last >= 1000000) { // jede Sekunde
                    printf("UDP-Pakete gesendet pro Sekunde: %ld\n", pkt_sec);
                    pkt_sec = 0;
                    last = now;
                    }

                 /* ---------- LED heartbeat ---------- */
                if (++pkt_cnt >= 50) {                 // alle 10 Pakete = 5 Hz
                pkt_cnt = 0;
                static bool led = false;
                led = !led;
                gpio_set_level(PIN_LED, led);
                }
                break;                    // Erfolg
            }
            if (errno == ENOMEM) {                   // Puffer voll
                vTaskDelay(pdMS_TO_TICKS(5));        // 5 ms Pause
            } else {
                ESP_LOGE(TAG, "sendto errno=%d", errno);
                vTaskDelay(pdMS_TO_TICKS(20));       // andere Fehler
            }
              
        }
    }
}





void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
   

    io_init();
    i2s_init();           // Legacy-API -> driver/i2s.h benutzen oder umstellen
    wifi_init_sta();      // eigenes STA-Setup
    vTaskDelay(pdMS_TO_TICKS(100));  // 100 ms warten, bevor Tasks starten

    xTaskCreatePinnedToCore(i2s_reader_task, "i2s_read",
                            4096, NULL, 22, NULL, 1);
    xTaskCreate(udp_stream_task, "udp_send",
                4096, NULL, 10, NULL);
}
