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
#include "protocol_examples_common.h"
#include "esp_err.h"
#include "freertos/ringbuf.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/* ------- I²S / Ringbuffer ------- */
#define RB_CHUNK_BYTES   128            // ∑ genau ein DMA-Block
#define RB_NUM_CHUNKS    64             // 64 × 128 B = 8 kB (anpassen ≙ Pufferlänge)
static RingbufHandle_t audio_rb;        // globales Handle
//----------------------------------------

#define PIN_LED      33
#define I2S_NUM         I2S_NUM_0
#define I2S_CLK      26      // Bit Clock (SCK)
#define I2S_WS       25      // Word Select (WS)
#define I2S_SD       19      // Serial Data (SD)

static const char *TAG = "UDP_BC";
#define DEST_IP   "192.168.4.255"   // Broadcast im /24-Netz des Soft-AP
#define DEST_PORT 7000


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
    int32_t  dma_buf[RB_CHUNK_BYTES / 4];          // 128 x 32 bit
    uint8_t  be_buf[RB_CHUNK_BYTES / 2];           // 256 Bytes (BE-PCM)
    size_t   bytes_read;

    while (1) {
        if (i2s_read(I2S_NUM, dma_buf, sizeof(dma_buf),
                     &bytes_read, portMAX_DELAY) != ESP_OK) continue;

        /* 24-bit → 16-bit + LE→BE + Stereo interleaved (LR LR …) */
        uint8_t *w = be_buf;
        int32_t *r = dma_buf;
        int frames = bytes_read / 8;               // 8 Bytes = 1 Stereo-Frame
        for (int i = 0; i < frames; ++i) {
            int16_t l   = (int16_t)(r[0] >> 8);     // Sample extrahieren
            int16_t r16 = (int16_t)(r[1] >> 8);
        
            // Dämpfung:
            float gain = 0.8f;  // Oder int gain = 4, dann Division unten
            l   = (int16_t)(l * gain);       // oder (l * 4) / 10 für Int
            r16 = (int16_t)(r16 * gain);
        
            // Serialisierung wie gehabt:
            *w++ = (l >> 8) & 0xFF;   // MSB
            *w++ =  l       & 0xFF;   // LSB
            *w++ = (r16 >> 8) & 0xFF; // MSB
            *w++ =  r16       & 0xFF; // LSB
        
            r += 2;
        }
        
        

        /* 256 Byte (= frames × 4) in den Ringpuffer legen */
        xRingbufferSend(audio_rb, be_buf, frames * 4, portMAX_DELAY);
    }
}


static void start_softap(void)
{
    /* 1. TCP/IP-Stack initialisieren  */
    ESP_ERROR_CHECK(esp_netif_init());         

    /* 2. Standard-AP-Netif anlegen  */
    esp_netif_create_default_wifi_ap();          

    /* 3. Wi-Fi-Treiber starten  */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));        

    /* 4. AP-Parameter setzen  */
    wifi_config_t ap = {
        .ap = {
            .ssid           = "Merle hat gefurzt",
            .password       = "12345678",
            .channel        = 1,
            .max_connection = 2,
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK,
        }
    };
    if (strlen((char *)ap.ap.password) == 0)
        ap.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Soft-AP gestartet: SSID=\"%s\"", ap.ap.ssid);
}



static void udp_stream_task(void *arg)
{
    uint32_t pkt_cnt = 0;
    struct sockaddr_in dest = {0};
    dest.sin_family      = AF_INET;
    dest.sin_port        = htons(DEST_PORT);
    dest.sin_addr.s_addr = inet_addr(DEST_IP);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ESP_LOGE(TAG, "socket fail"); vTaskDelete(NULL); }

    int yes = 1;                          // Broadcast erlauben
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    
    const size_t MTU_PAYLOAD = 1472;            // maximal ohne Fragmentierung
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
                                  (struct sockaddr *)&dest, sizeof(dest));
            if (sent >= 0) {
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


      
    
    


static void udp_broadcast_task(void *arg)
{
    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port   = htons(DEST_PORT);
    dest_addr.sin_addr.s_addr = inet_addr(DEST_IP);

    if (dest_addr.sin_addr.s_addr == INADDR_NONE) {
        ESP_LOGE(TAG, "Ungültige IP-Adresse");
        vTaskDelete(NULL);
    }

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) { ESP_LOGE(TAG, "socket() failed"); vTaskDelete(NULL); }

    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    while (1) {
        uint32_t ts = esp_log_timestamp();
        uint32_t ts_net = htonl(ts);

        ssize_t sent = sendto(sock, &ts_net, sizeof(ts_net), 0,
                              (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            ESP_LOGE(TAG, "sendto errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(200));         // Back-off bei Error
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(20));             // 50 P/s
    }
}


void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    io_init();
    i2s_init();
    xTaskCreatePinnedToCore(i2s_reader_task, "i2s_read", 4096, NULL, 22, NULL, 1);

    start_softap();
    xTaskCreate(udp_stream_task, "udp_send", 4096, NULL, 5, NULL);
}
