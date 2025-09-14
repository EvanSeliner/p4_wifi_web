// main.c — ESP32-P4 host + ESP32-C6 (Hosted) + simple HTTP + Speaker toggle
// IDF: v5.5.x
// Requires managed components: espressif/esp_wifi_remote, espressif/esp_hosted, espressif/es8311

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_http_server.h"

// --- WiFi via Hosted/Remote ---
#include "esp_wifi_remote.h"                  // provided by esp_wifi_remote (host API)


// --- Driver / GPIO / I2C / I2S ---
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"

// --- ES8311 codec ---
#include "es8311.h"


static const char *TAG = "P4+C6+SND";

//
// ===== Pins per Waveshare ESP32-P4-WIFI6 board =====
// I2C0 (ES8311 control)
#define I2C0_SDA      7
#define I2C0_SCL      8
#define I2C0_PORT     I2C_NUM_0
// I2S (audio data to ES8311)
#define I2S_MCLK      13
#define I2S_BCLK      12  // SCLK
#define I2S_WS        10  // LRCK
#define I2S_DOUT      11  // ASDOUT from P4 to codec
// Amp enable (NS4150B) — active high
#define AMP_ENABLE    53

// Wi-Fi AP fallback SSID
#define AP_SSID       "ESP-CAM"
#define AP_PASS       ""        // open AP (keep empty for no pass)
#define AP_CHAN       6
#define HOSTNAME      "esp-p4"

/* ---------------- HTTP server ---------------- */

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/* ---------------- Wi-Fi Hosted glue (P4 host talks to C6 over SDIO) ---------------- */

static EventGroupHandle_t s_ev;
#define BIT_GOT_IP BIT0

static void ip_evt(void *arg, esp_event_base_t base, int32_t id, void *data) {
  if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(s_ev, BIT_GOT_IP);
  }
}

static void wifi_sta_start(const char* ssid, const char* pass)
{
  ESP_LOGI(TAG, "WiFi STA start (Hosted/Remote)");
  // esp_wifi_remote provides esp_wifi_* API that forwards calls to the C6.
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  wifi_config_t cfg = { 0 };
  strncpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid)-1);
  strncpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password)-1);
  cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

  ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &cfg) );
  ESP_ERROR_CHECK( esp_wifi_start() );
  ESP_ERROR_CHECK( esp_wifi_connect() );
}

static void wifi_ap_start(void)
{
  ESP_LOGW(TAG, "AP fallback -> SSID='%s'", AP_SSID);
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_AP) );
  wifi_config_t ap = { 0 };
  strncpy((char*)ap.ap.ssid, AP_SSID, sizeof(ap.ap.ssid)-1);
  ap.ap.ssid_len = strlen(AP_SSID);
  ap.ap.channel = AP_CHAN;
  ap.ap.max_connection = 4;
  ap.ap.authmode = WIFI_AUTH_OPEN;
  ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_AP, &ap) );
  ESP_ERROR_CHECK( esp_wifi_start() );
}

/* ---------------- Audio: ES8311 + I²S ---------------- */


static i2s_chan_handle_t s_i2s_tx = NULL;
static es8311_handle_t s_es = NULL;
static volatile bool s_play = false;

static esp_err_t audio_hw_init(void)
{
  // Amp enable pin
  gpio_config_t io = {
    .pin_bit_mask = 1ULL << AMP_ENABLE,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
  };
  ESP_ERROR_CHECK( gpio_config(&io) );
  gpio_set_level(AMP_ENABLE, 0); // start muted

  // I2C0 for ES8311 control (addr 0x18 by default)
  i2c_config_t i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C0_SDA,
    .scl_io_num = I2C0_SCL,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
  };
  ESP_ERROR_CHECK( i2c_param_config(I2C0_PORT, &i2c_cfg) );
  ESP_ERROR_CHECK( i2c_driver_install(I2C0_PORT, i2c_cfg.mode, 0, 0, 0) );

  // I2S TX setup (44.1 kHz, 16-bit stereo)
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK( i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL) );

  i2s_std_config_t std = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(16, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = I2S_MCLK,
      .bclk = I2S_BCLK,
      .ws   = I2S_WS,
      .dout = I2S_DOUT,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  ESP_ERROR_CHECK( i2s_channel_init_std_mode(s_i2s_tx, &std) );

  // ES8311 init
  s_es = es8311_create(I2C0_PORT, ES8311_ADDRESS_0);

  es8311_clock_config_t clk = {
    .mclk_from_mclk_pin = true,
    .mclk_frequency = 44100 * 256,
    .sample_frequency = 44100
  };

  ESP_ERROR_CHECK( es8311_init(s_es, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) );
  ESP_ERROR_CHECK( es8311_voice_volume_set(s_es, 80, NULL) ); // 0-100

  ESP_ERROR_CHECK( i2s_channel_enable(s_i2s_tx) );

  return ESP_OK;
}

static void audio_set_play(bool on)
{
  s_play = on;
  gpio_set_level(AMP_ENABLE, on ? 1 : 0);
}

static void audio_task(void *arg)
{
  const float f = 440.0f;
  const int fs = 44100, nsamp = 512;
  int16_t buf[nsamp*2]; // stereo

  while (1) {
    if (!s_play) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    for (int i = 0; i < nsamp; ++i) {
      float t = (float)i / fs;
      int16_t s = (int16_t)(sin(2*M_PI*f*t) * 3000.0f);
      buf[2*i+0] = s;
      buf[2*i+1] = s;
    }
    size_t w = 0;
    i2s_channel_write(s_i2s_tx, buf, sizeof(buf), &w, 100 / portTICK_PERIOD_MS);
  }
}

/* ---------------- Simple metrics + HTTP server ---------------- */

typedef struct {
  uint32_t uptime_s, heap_free, heap_min, psram_free;
} metrics_t;

static void get_metrics(metrics_t *m) {
  m->uptime_s  = (uint32_t)(esp_timer_get_time()/1000000ULL);
  m->heap_free = esp_get_free_heap_size();
  m->heap_min  = esp_get_minimum_free_heap_size();
  m->psram_free= 0; // (no off-chip PSRAM on this kit)
}

static esp_err_t root_get(httpd_req_t *r)
{
  httpd_resp_set_type(r, "text/html");
  const size_t len = index_html_end - index_html_start;
  return httpd_resp_send(r, (const char*)index_html_start, len);
}

static esp_err_t status_get(httpd_req_t *r)
{
  metrics_t m; get_metrics(&m);
  char out[256];
  int n = snprintf(out, sizeof(out),
                   "uptime: %" PRIu32 "s\nheap: %" PRIu32 " (min %" PRIu32 ")\npsram: %" PRIu32 "\nSound: %s\n",
                   m.uptime_s, m.heap_free, m.heap_min, m.psram_free, s_play ? "On":"Off");
  httpd_resp_set_type(r, "text/plain");
  return httpd_resp_send(r, out, n);
}

// Minimal MJPEG placeholder: if you had an earlier /video endpoint, keep it;
// otherwise this keeps <video> from erroring until you wire H.264 later.
static esp_err_t video_get(httpd_req_t *r)
{
  httpd_resp_set_type(r, "video/mp4"); // placeholder MIME; replace when H.264 MSE is added
  httpd_resp_set_hdr(r, "Cache-Control", "no-store");
  // For now, stream nothing (browser will sit idle). Later, feed H.264 segments.
  return httpd_resp_send(r, "", 0);
}

static esp_err_t sound_toggle_post(httpd_req_t *r)
{
  audio_set_play(!s_play);
  const char *resp = s_play ? "On" : "Off";
  httpd_resp_set_type(r, "text/plain");
  return httpd_resp_send(r, resp, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_web(void)
{
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  httpd_handle_t h = NULL;
  ESP_ERROR_CHECK( httpd_start(&h, &cfg) );

  httpd_uri_t u_root = { .uri="/", .method=HTTP_GET, .handler=root_get };

  httpd_register_uri_handler(h, &u_root);
  return h;
}

/* --------------------------------- app_main -------------------------------- */

void app_main(void)
{
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  ESP_LOGI(TAG, "Boot!");

  // Basic init
  ESP_ERROR_CHECK(nvs_flash_init());
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  s_ev = xEventGroupCreate();

  // Bring up netif for STA + AP
  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();

  // Hosted base transport (C6 over SDIO) is initialized by components auto-init.
  // Try STA for ~10s, then AP fallback (matches what you saw in logs).
  wifi_sta_start("DaeganSmells", "0123456789");
  esp_event_handler_instance_t inst;
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_evt, NULL, &inst));
  EventBits_t r = xEventGroupWaitBits(s_ev, BIT_GOT_IP, pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
  if (!(r & BIT_GOT_IP)) {
    ESP_LOGW(TAG, "STA timeout -> AP fallback");
    wifi_ap_start();
  }

  // Audio init + task
  ESP_ERROR_CHECK( audio_hw_init() );
  xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 5, NULL, 0);

  // HTTP
  start_web();
  ESP_LOGW(TAG, "Browse http://<ip>/  (status, /sound/toggle)");
}
