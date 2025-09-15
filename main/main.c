// main.c — ESP32-P4 host + ESP32-C6 (ESP-Hosted SDIO, legacy-friendly) + simple HTTP
// "Fail soft": never panic; log & continue
// IDF: v5.5.x
// Needs managed components: espressif/esp_wifi_remote, espressif/esp_hosted

#include <stdio.h>
#include <string.h>
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
#include "driver/gpio.h"

// WiFi via Hosted/Remote (forwards esp_wifi_* to C6 over SDIO)
#include "esp_wifi_remote.h"

static const char *TAG = "P4+C6";

#define AP_SSID       "ESP-CAM"
#define AP_PASS       ""
#define AP_CHAN       6

// ---------- HTTP: serve embedded index.html ----------
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// ---------- Events ----------
static EventGroupHandle_t s_ev;
#define BIT_GOT_IP  BIT0

// ---------- Fail-soft helpers ----------
#define LOGE(fmt, ...)   ESP_LOGE(TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...)   ESP_LOGW(TAG, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...)   ESP_LOGI(TAG, fmt, ##__VA_ARGS__)

// Try a call; on error, log and continue (never abort)
#define TRY(expr) do { \
    esp_err_t __e = (expr); \
    if (__e != ESP_OK) LOGE(#expr " -> %s (0x%x)", esp_err_to_name(__e), __e); \
} while (0)

// Try a call with custom label text
#define TRYL(label, expr) do { \
    esp_err_t __e = (expr); \
    if (__e != ESP_OK) LOGE("%s: %s (0x%x)", (label), esp_err_to_name(__e), __e); \
} while (0)

static bool s_wifi_inited = false;

/* ---------------------------------------------------------------------------
   SDIO legacy-compat shim (host only; you CANNOT flash the C6)
   - Pull-ups on CMD/D0
   - Long reset timing
   - Try opposite reset polarity once during probe
--------------------------------------------------------------------------- */
static void hosted_sdio_legacy_pre_init(void) {
    const int GPIO_SDIO_CMD = 19;
    const int GPIO_SDIO_D0  = 14;
    const int GPIO_SLAVE_RST= 54;

    // Pull-ups on CMD/D0
    gpio_pullup_en(GPIO_SDIO_CMD);
    gpio_pulldown_dis(GPIO_SDIO_CMD);
    gpio_pullup_en(GPIO_SDIO_D0);
    gpio_pulldown_dis(GPIO_SDIO_D0);

    // Reset GPIO as output
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << GPIO_SLAVE_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    TRYL("gpio_config(reset)", gpio_config(&io));

    // Assume active-LOW reset first
    gpio_set_level(GPIO_SLAVE_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(GPIO_SLAVE_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(350));
}

// Wait until Hosted responds; try opposite reset polarity mid-way
static void wifi_wait_ready_probe(void) {
    uint8_t mac[6];
    const int GPIO_SLAVE_RST=54;

    for (int i = 0; i < 100; ++i) { // ~10s
        if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
            LOGI("Hosted ready (MAC %02x:%02x:%02x:%02x:%02x:%02x)",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        if (i == 25) {
            // Try active-HIGH style reset once, in case board is wired that way
            gpio_set_level(GPIO_SLAVE_RST, 1);
            vTaskDelay(pdMS_TO_TICKS(80));
            gpio_set_level(GPIO_SLAVE_RST, 0);
            vTaskDelay(pdMS_TO_TICKS(350));
        }
    }
    LOGW("wifi_wait_ready_probe: MAC probe timed out; continuing anyway");
}

// ---------- NVS + Wi-Fi base ----------
static void ensure_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TRY(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) LOGE("nvs_flash_init final -> %s (0x%x)", esp_err_to_name(err), err);
}

static void ensure_wifi_inited(void) {
    if (s_wifi_inited) return;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    TRY(esp_wifi_init(&cfg));
    s_wifi_inited = true;
}

// ---------- IP event ----------
static void ip_evt(void *arg, esp_event_base_t base, int32_t id, void *data) {
    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t* event = (const ip_event_got_ip_t*) data;
        LOGI("Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_ev, BIT_GOT_IP);
    }
}

// ---------- Wi-Fi: STA + AP (non-fatal) ----------
static void wifi_sta_start(const char* ssid, const char* pass) {
    LOGI("WiFi STA start (Hosted/Remote)");
    ensure_wifi_inited();
    wifi_wait_ready_probe();

    wifi_config_t cfg = { 0 };
    strncpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid)-1);
    strncpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password)-1);
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    TRY( esp_wifi_set_mode(WIFI_MODE_STA) );
    TRY( esp_wifi_set_config(WIFI_IF_STA, &cfg) );
    TRY( esp_wifi_start() );
    TRY( esp_wifi_connect() );
}

static void wifi_ap_start(void) {
    LOGW("AP fallback -> SSID='%s'", AP_SSID);
    ensure_wifi_inited();
    wifi_wait_ready_probe();

    wifi_config_t ap = { 0 };
    strncpy((char*)ap.ap.ssid, AP_SSID, sizeof(ap.ap.ssid)-1);
    ap.ap.ssid_len = strlen(AP_SSID);
    ap.ap.channel = AP_CHAN;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.ssid_hidden = 0;

    TRY( esp_wifi_set_mode(WIFI_MODE_AP) );
    TRY( esp_wifi_set_config(WIFI_IF_AP, &ap) );
    TRY( esp_wifi_start() );
}

// ---------- Simple HTTP ----------
static esp_err_t root_get(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/html");
    const size_t len = index_html_end - index_html_start;
    return httpd_resp_send(r, (const char*)index_html_start, len);
}

static httpd_handle_t start_web(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    httpd_handle_t h = NULL;
    esp_err_t e = httpd_start(&h, &cfg);
    if (e != ESP_OK) {
        LOGE("httpd_start -> %s (0x%x)", esp_err_to_name(e), e);
        return NULL;
    }
    httpd_uri_t u_root = { .uri="/", .method=HTTP_GET, .handler=root_get };
    TRY( httpd_register_uri_handler(h, &u_root) );
    return h;
}

// --------------------------------- app_main ---------------------------------
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));       // power settle
    hosted_sdio_legacy_pre_init();        // host-only SDIO prep

    LOGI("Boot!");
    esp_log_level_set("wifi", ESP_LOG_DEBUG);
    esp_log_level_set("esp_wifi_remote", ESP_LOG_DEBUG);
    esp_log_level_set("H_API", ESP_LOG_DEBUG);

    ensure_nvs();
    TRY( esp_netif_init() );
    TRY( esp_event_loop_create_default() );

    // Create default netifs BEFORE esp_wifi_init()
    TRY( (esp_netif_create_default_wifi_sta() ? ESP_OK : ESP_FAIL) );
    TRY( (esp_netif_create_default_wifi_ap()  ? ESP_OK : ESP_FAIL) );

    s_ev = xEventGroupCreate();

    // Register IP event BEFORE starting Wi-Fi
    esp_event_handler_instance_t inst;
    TRY( esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, ip_evt, NULL, &inst) );

    // Try STA for ~12s, then AP fallback, but never abort app
    wifi_sta_start("DaeganSmells", "0123456789");
    EventBits_t r = xEventGroupWaitBits(s_ev, BIT_GOT_IP, pdTRUE, pdFALSE, pdMS_TO_TICKS(12000));
    if (!(r & BIT_GOT_IP)) {
        LOGW("STA timeout -> AP fallback");
        wifi_ap_start();
    }

    // HTTP always attempts to start
    start_web();
    LOGW("Browse http://<ip>/ (even if Wi-Fi failed, HTTP will run when IP is up)");
}
