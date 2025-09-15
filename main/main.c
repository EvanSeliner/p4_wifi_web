// main.c — ESP32-P4 host + ESP32-C6 (ESP-Hosted SDIO, legacy-friendly) + simple HTTP
// "Fail soft": never panic; log & continue
// IDF: v5.5.x
// Needs managed components: espressif/esp_wifi_remote, espressif/esp_hosted

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
// Camera/esp_video headers (device name via esp_video_device.h)
// Camera/esp_video probe
#include <fcntl.h>
#include <sys/ioctl.h>
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include <unistd.h>
// (build string via esp_get_idf_version())

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

static bool s_wifi_inited   = false;
static bool s_hosted_ready  = false;
static bool s_sta_got_ip    = false;

// ---- Forward declarations & globals ----
static esp_err_t root_get(httpd_req_t *r);
static esp_err_t status_get(httpd_req_t *r);
static esp_err_t latency_get(httpd_req_t *r);
static esp_err_t events_get(httpd_req_t *r);
static esp_err_t pad_handler(httpd_req_t *r);
#ifdef CONFIG_HTTPD_WS_SUPPORT
static esp_err_t ws_handler(httpd_req_t *req);
#endif
static void video_task(void *arg);

static httpd_handle_t g_httpd = NULL;
#ifdef CONFIG_HTTPD_WS_SUPPORT
static int s_ws_fd = -1;
#endif

// (duplicate start_web/ws_handler block removed)

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

    s_hosted_ready = false;
    for (int i = 0; i < 100; ++i) { // ~10s
        if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
            LOGI("Hosted ready (MAC %02x:%02x:%02x:%02x:%02x:%02x)",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
            s_hosted_ready = true;
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
        s_sta_got_ip = true;
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

// ---------- HTTP Handlers ----------
// NOTE: Added a minimal WebSocket endpoint at /ws so the page can connect
// without 404. For now it just echos any message (text/binary) back to the
// sender. We'll hook the camera encoder to push H.264 frames here next.
// Small JS fragment we append to the built-in index.html so the latency demo
// lives on the main page. It creates a dot + counters and uses /pad for RTT.
static const char k_index_inject[] = "<script src='/ui.js'></script>";

// --- /ui.js: adds latency widget + WS indicator + frame counter to any page ---
static const char k_ui_js[] =
"(()=>{" 
"  const mount=(sel,html)=>{const c=document.querySelector(sel)||document.body;const d=document.createElement('div');d.innerHTML=html;c.appendChild(d);};"
"  const ui = `"
"  <div id='latency-ui' style='position:fixed;right:12px;bottom:12px;background:#111;color:#eee;padding:10px 12px;border-radius:8px;font:14px system-ui,Arial;box-shadow:0 2px 8px rgba(0,0,0,.3);z-index:2147483647'>"
"    <div style='display:flex;align-items:center;gap:8px'>"
"      <span id='led' style='display:inline-block;width:10px;height:10px;border-radius:50%;background:#666'></span>"
"      <span>RTT:</span><b id='rtt'>--</b><span> ms</span>"
"      <span style='margin-left:10px'>count:</span><b id='count'>0</b>"
"      <span style='margin-left:10px'>WS:</span><b id='ws'>OFF</b>"
"      <span style='margin-left:6px'>msg:</span><b id='wsc'>0</b>"
"      <span style='margin-left:10px'>frames:</span><b id='vframes'>0</b>"
"    </div>"
"    <div id='lat-hint' style='opacity:.8;margin-top:4px;font-size:12px'>Press any gamepad button to measure round-trip</div>"
"  </div>`;"
"  mount('body', ui);"
"  const led=document.getElementById('led'), rtt=document.getElementById('rtt'), cnt=document.getElementById('count');"
"  const wsEl=document.getElementById('ws'), wscEl=document.getElementById('wsc'), vfEl=document.getElementById('vframes');"
"  const flash=(ok)=>{led.style.background=ok?'#0a0':'#a00'; setTimeout(()=>{led.style.background='#666';},120);};"
"  const ping=async(btn)=>{"
"    const t0=performance.now();"
"    try{"
"      const res=await fetch('/pad',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({btn,pressed:true,t_client_ms:Math.round(t0)})});"
"      const t1=performance.now(); const j=await res.json();"
"      rtt.textContent=(t1-t0).toFixed(2); cnt.textContent=j.count||0; flash(true);"
"    }catch(e){ flash(false); }"
"  };"
"  let last=0; const poll=()=>{"
"    const gps=navigator.getGamepads?navigator.getGamepads():[];"
"    for(const g of gps){ if(!g) continue; for(let i=0;i<g.buttons.length;i++){ if(g.buttons[i].pressed){ const now=performance.now(); if(now-last>90){ last=now; ping(i);} break; } } }"
"    requestAnimationFrame(poll);"
"  }; requestAnimationFrame(poll);"
"  try{"
"    const proto=location.protocol==='https:'?'wss':'ws';"
"    const ws=new WebSocket(`${proto}://${location.host}/ws`);"
"    ws.binaryType='arraybuffer';"
"    ws.onopen=()=>{ wsEl.textContent='OPEN'; try{ws.send('ping');}catch{} };"
"    setInterval(()=>{ if(ws&&ws.readyState===1){ try{ws.send('ping');}catch{} } },2000);"
"    ws.onmessage=(ev)=>{"
"      if(ev.data instanceof ArrayBuffer){ vfEl.textContent = (Number(vfEl.textContent)||0)+1; }"
"      else { wscEl.textContent = (Number(wscEl.textContent)||0)+1; }"
"    };"
"    ws.onerror = ()=>{ wsEl.textContent='ERROR'; };"
"    ws.onclose = ()=>{ wsEl.textContent='CLOSED'; };"
"    window._p4ws = ws;"
"  }catch(e){ wsEl.textContent='UNSUPPORTED'; }"
"})();";

static esp_err_t ui_get(httpd_req_t *r){
    httpd_resp_set_type(r, "application/javascript");
    return httpd_resp_send(r, k_ui_js, HTTPD_RESP_USE_STRLEN);
}

// (camera probe defined later, after globals)

static esp_err_t root_get(httpd_req_t *r) {
    // Serve the embedded index.html, then append our latency UI script so it
    // appears on the main page without modifying your original HTML blob.
    httpd_resp_set_type(r, "text/html");
    const char *p = (const char*)index_html_start;
    size_t len = (size_t)(index_html_end - index_html_start);
    esp_err_t e = httpd_resp_send_chunk(r, p, len);
    if (e != ESP_OK) return e;
    e = httpd_resp_send_chunk(r, k_index_inject, HTTPD_RESP_USE_STRLEN);
    if (e != ESP_OK) return e;
    return httpd_resp_send_chunk(r, NULL, 0);
}

// --- /latency: built-in gamepad RTT demo
// Small self-contained HTML+JS page that measures round-trip latency via /pad
static const char k_latency_html[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<title>Latency Tester</title>"
    "<style>body{font-family:system-ui,Arial;margin:24px}"
    "#rtt{font:700 24px/1 monospace;margin-left:8px}"
    "#count{font:700 18px/1 monospace;margin-left:8px}"
    ".ok{color:#0a0}.bad{color:#a00}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#999;margin-right:8px}"
    "</style></head><body>"
    "<h1>ESP32-P4 Button Round-Trip</h1>"
    "<p>Press any gamepad button. The page POSTs to <code>/pad</code>, the ESP responds, and we show the round-trip time (RTT). Also opens a simple EventStream (<code>/events</code>) for server→client pushes without WebSockets.</p>"
    "<div><span class='dot' id='led'></span>"
    "<span>RTT:</span><span id='rtt'>--</span> ms"
    "<span style='margin-left:16px'>count:</span><span id='count'>0</span></div>"
    "<pre id='log' style='margin-top:12px;color:#555;white-space:pre-wrap'></pre>"
    "<script>(function(){"
    "const led=document.getElementById('led');"
    "const rtt=document.getElementById('rtt');"
    "const cnt=document.getElementById('count');"
    "const log=document.getElementById('log');"
    "let gpIndex=null;"
    "function flash(ok){led.style.background=ok?'#0a0':'#a00';setTimeout(()=>{led.style.background='#999';},120);}"
    "async function ping(btn){"
    "  const t0=performance.now();"
    "  try{"
    "    const body=JSON.stringify({btn,pressed:true,t_client_ms:Math.round(t0)});"
    "    const res=await fetch('/pad',{method:'POST',headers:{'Content-Type':'application/json'},body});"
    "    const t1=performance.now();"
    "    const j=await res.json();"
    "    rtt.textContent=(t1-t0).toFixed(2);"
    "    cnt.textContent=j.count||0;"
    "    flash(true);"
    "  }catch(e){flash(false);log.textContent=String(e);}"
    "}"
    "function poll(){const gps=navigator.getGamepads?navigator.getGamepads():[];const g=gpIndex!=null?gps[gpIndex]:null;"
    "  if(g){for(let i=0;i<g.buttons.length;i++){if(g.buttons[i].pressed){ping(i);break;}}}"
    "  requestAnimationFrame(poll);}"
    "window.addEventListener('gamepadconnected',e=>{gpIndex=e.gamepad.index;});"
    "window.addEventListener('gamepaddisconnected',e=>{if(gpIndex===e.gamepad.index)gpIndex=null;});"
    "// Simple server-sent events stream (no WebSocket needed)"
    "try{const es=new EventSource('/events');es.onmessage=(ev)=>{console.log('SSE',ev.data)};es.onerror=()=>{try{es.close()}catch{}}}catch{}"
    "requestAnimationFrame(poll);"
    "})();</script></body></html>";

static esp_err_t latency_get(httpd_req_t *r) {
    httpd_resp_set_type(r, "text/html");
    return httpd_resp_send(r, k_latency_html, HTTPD_RESP_USE_STRLEN);
}

// Minimal /pad sink (prevents 404 spam). Accepts POST/GET, returns 204.
static volatile uint32_t g_pad_count = 0;
static volatile uint64_t g_pad_last_us = 0;
// Camera probe state
static bool     s_cam_ok = false;
static char     s_cam_card[32] = {0};
static char     s_cam_bus[32]  = {0};
static uint16_t s_cam_w = 0, s_cam_h = 0;

// Safer JSON format strings (avoid heavy escaping in snprintf lines)
static const char PAD_JSON_FMT[] =
"{"
"\"ok\":true,"
"\"srv_us\":%llu,"
"\"count\":%u,"
"\"echo\":%.*s"
"}";

static const char STATUS_JSON_FMT[] =
"{"
"\"uptime_s\":%" PRIu32 ","
"\"heap_free\":%" PRIu32 ","
"\"heap_min\":%" PRIu32 ","
"\"hosted_ready\":%s,"
"\"sta_got_ip\":%s,"
"\"build\":\"%s\","
"\"pad_count\":%" PRIu32 ","
"\"pad_last_us\":%" PRIu64 ","
"\"cam_ok\":%s,"
"\"cam_card\":\"%s\","
"\"cam_bus\":\"%s\","
"\"cam_w\":%u,"
"\"cam_h\":%u"
"}";

static esp_err_t pad_handler(httpd_req_t *r) {
    char buf[256];
    int total = 0;
    while (total < (int)sizeof(buf) - 1 && total < r->content_len) {
        int ret = httpd_req_recv(r, buf + total, (int)sizeof(buf) - 1 - total);
        if (ret <= 0) break;
        total += ret;
    }
    buf[total] = '\0';

    uint64_t now_us = esp_timer_get_time();
    g_pad_last_us = now_us;
    g_pad_count++;

    char out[256];
    int n = snprintf(out, sizeof(out), PAD_JSON_FMT,
        (unsigned long long)now_us, (unsigned)g_pad_count, total, buf);

    httpd_resp_set_type(r, "application/json");
    return httpd_resp_send(r, out, n > 0 ? n : 0);
}

static esp_err_t status_get(httpd_req_t *r) {
    const char *build = esp_get_idf_version();

    uint32_t up_s   = (uint32_t)(esp_timer_get_time()/1000000ULL);
    uint32_t heap_f = esp_get_free_heap_size();
    uint32_t heap_m = esp_get_minimum_free_heap_size();

    char out[480];
    int n = snprintf(out, sizeof(out), STATUS_JSON_FMT,
        up_s, heap_f, heap_m,
        s_hosted_ready ? "true" : "false",
        s_sta_got_ip   ? "true" : "false",
        build,
        g_pad_count, (uint64_t)g_pad_last_us,
        s_cam_ok ? "true" : "false",
        s_cam_card[0] ? s_cam_card : "",
        s_cam_bus[0]  ? s_cam_bus  : "",
        (unsigned)s_cam_w, (unsigned)s_cam_h);

    if (n < 0) {
        httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "snprintf failed");
        return ESP_OK;
    }
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_send(r, out, n);
}

// Lightweight server→client push without WebSockets: Server-Sent Events (SSE)
// Works with plain httpd (no menuconfig flags). The browser connects to /events
// and we periodically push a line. Useful as a keepalive or to prove
// server→client delivery alongside the /pad POST round-trip.
static esp_err_t events_get(httpd_req_t *r) {
    // SSE requires special headers
    httpd_resp_set_type(r, "text/event-stream");
    httpd_resp_set_hdr(r, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(r, "Connection", "keep-alive");

    // Send a few heartbeats quickly, then hand off to a tiny loop (~2s)
    for (int i = 0; i < 3; ++i) {
        char line[64];
        int n = snprintf(line, sizeof(line), "data: beat %d\\n\\n", i);
        if (httpd_resp_send_chunk(r, line, n) != ESP_OK) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // Short-lived stream so we don't tie up a worker forever
    for (int j = 0; j < 10; ++j) {
        uint64_t us = esp_timer_get_time();
        char line[96];
        int n = snprintf(line, sizeof(line), "data: t_us=%llu count=%u\\n\\n", (unsigned long long)us, (unsigned)g_pad_count);
        if (httpd_resp_send_chunk(r, line, n) != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    httpd_resp_send_chunk(r, NULL, 0);
    return ESP_OK;
}

// Optional tiny HTTP check for camera
static esp_err_t cam_ok_get(httpd_req_t *r){
    httpd_resp_set_type(r, "text/plain");
    if (s_cam_ok) return httpd_resp_sendstr(r, "CAM=OK\n");
    return httpd_resp_sendstr(r, "CAM=FAIL\n");
}

// One-shot camera probe using V4L2 QUERYCAP via esp_video VFS device
static void cam_probe_once(void) {
    const char *dev = ESP_VIDEO_MIPI_CSI_DEVICE_NAME; // e.g. "/dev/video0"
    int fd = open(dev, O_RDWR);
    if (fd < 0) {
        LOGE("camera: open(%s) failed", dev);
        s_cam_ok = false;
        return;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        snprintf(s_cam_card, sizeof(s_cam_card), "%s", (const char*)cap.card);
        snprintf(s_cam_bus,  sizeof(s_cam_bus),  "%s", (const char*)cap.bus_info);
        s_cam_ok = true;
        LOGI("camera: QUERYCAP ok | driver=%s card=%s bus=%s", cap.driver, cap.card, cap.bus_info);
    } else {
        LOGE("camera: VIDIOC_QUERYCAP failed");
        s_cam_ok = false;
        close(fd);
        LOGE("CAMERA PROBE: FAIL (check ribbon + I2C pins)");
        return;
    }

    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width  = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == 0) {
        s_cam_w = fmt.fmt.pix.width;
        s_cam_h = fmt.fmt.pix.height;
        LOGI("camera: S_FMT ok -> %ux%u fourcc=0x%08x", s_cam_w, s_cam_h, fmt.fmt.pix.pixelformat);
    } else {
        LOGW("camera: S_FMT not applied (continuing)");
    }

    close(fd);
    if (s_cam_ok) {
        LOGI("CAMERA PROBE: PASS (OV5647 expected) card=%s bus=%s %ux%u",
             s_cam_card[0]?s_cam_card:"?", s_cam_bus[0]?s_cam_bus:"?", s_cam_w, s_cam_h);
    } else {
        LOGE("CAMERA PROBE: FAIL (check ribbon + I2C pins)");
    }
}


#ifdef CONFIG_HTTPD_WS_SUPPORT
// Forward declaration so start_web() can reference ws_handler
static esp_err_t ws_handler(httpd_req_t *req);
#endif

static httpd_handle_t start_web(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_uri_handlers = 16;      // avoid ESP_ERR_HTTPD_HANDLERS_FULL
    cfg.lru_purge_enable = true;    // purge old handlers if needed
    httpd_handle_t h = NULL;
    esp_err_t e = httpd_start(&h, &cfg);
    if (e != ESP_OK) {
        LOGE("httpd_start -> %s (0x%x)", esp_err_to_name(e), e);
        return NULL;
    }
    httpd_uri_t u_root    = { .uri="/",        .method=HTTP_GET,  .handler=root_get    };
    httpd_uri_t u_status  = { .uri="/status",  .method=HTTP_GET,  .handler=status_get  };
    httpd_uri_t u_cam_ok  = { .uri="/cam",     .method=HTTP_GET,  .handler=cam_ok_get  };
    httpd_uri_t u_latency = { .uri="/latency", .method=HTTP_GET,  .handler=latency_get };
    httpd_uri_t u_ui      = { .uri="/ui.js",  .method=HTTP_GET,  .handler=ui_get      };
    httpd_uri_t u_events  = { .uri="/events",  .method=HTTP_GET,  .handler=events_get  };
    httpd_uri_t u_pad_g   = { .uri="/pad",     .method=HTTP_GET,  .handler=pad_handler };
    httpd_uri_t u_pad_p   = { .uri="/pad",     .method=HTTP_POST, .handler=pad_handler };
    TRY( httpd_register_uri_handler(h, &u_root) );
    TRY( httpd_register_uri_handler(h, &u_status) );
    TRY( httpd_register_uri_handler(h, &u_latency) );
    TRY( httpd_register_uri_handler(h, &u_ui) );
    TRY( httpd_register_uri_handler(h, &u_events) );
    TRY( httpd_register_uri_handler(h, &u_cam_ok) );
#ifdef CONFIG_HTTPD_WS_SUPPORT
    httpd_uri_t u_ws = { .uri="/ws", .method=HTTP_GET, .handler=ws_handler, .user_ctx=NULL, .is_websocket = true };
    TRY( httpd_register_uri_handler(h, &u_ws) );
    g_httpd = h;
    // Launch lightweight video sender task (will push JPEG frames when available)
    xTaskCreatePinnedToCore(video_task, "vid", 4096, NULL, 5, NULL, tskNO_AFFINITY);
#else
    LOGW("WebSocket not enabled (CONFIG_HTTPD_WS_SUPPORT). /ws will 404.");
#endif
    TRY( httpd_register_uri_handler(h, &u_pad_g) );
    TRY( httpd_register_uri_handler(h, &u_pad_p) );
    return h;
}

#ifdef CONFIG_HTTPD_WS_SUPPORT
// --- WebSocket echo handler (will carry H.264 later) ---
static esp_err_t ws_handler(httpd_req_t *req){
    if (req->method == HTTP_GET) {
        // WebSocket handshake complete — remember this client socket
        s_ws_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }
    // Non-handshake frames: receive (text/binary) and echo back
    httpd_ws_frame_t frame = {
        .final = true,
        .type = HTTPD_WS_TYPE_BINARY,
        .payload = NULL,
        .len = 0
    };
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;
    if (frame.len) {
        frame.payload = (uint8_t*)malloc(frame.len);
        if (!frame.payload) return ESP_ERR_NO_MEM;
        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK) { free(frame.payload); return ret; }
    }
    httpd_ws_frame_t resp = {
        .final = true,
        .type  = frame.type,
        .payload = frame.payload,
        .len = frame.len
    };
    ret = httpd_ws_send_frame(req, &resp);
    if (frame.payload) free(frame.payload);
    return ret;
}
#endif

#ifdef CONFIG_HTTPD_WS_SUPPORT
static void video_task(void *arg){
    // Dummy binary tick so the page proves WS->browser path works.
    // Replace with real camera frames (H.264/JPEG) later.
    static const uint8_t tick[4] = {0,1,2,3};
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (g_httpd && s_ws_fd >= 0) {
            httpd_ws_frame_t f = {
                .final = true,
                .type = HTTPD_WS_TYPE_BINARY,
                .payload = (uint8_t*)tick,
                .len = sizeof(tick)
            };
            // Non-blocking push from task context
            httpd_ws_send_frame_async(g_httpd, s_ws_fd, &f);
        }
    }
}
#endif

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
    s_sta_got_ip = false;
    wifi_sta_start("DaeganSmells", "0123456789");
    EventBits_t r = xEventGroupWaitBits(s_ev, BIT_GOT_IP, pdTRUE, pdFALSE, pdMS_TO_TICKS(12000));
    if (!(r & BIT_GOT_IP)) {
        LOGW("STA timeout -> AP fallback");
        wifi_ap_start();
    }

    // HTTP always attempts to start
    start_web();
    // Probe the CSI camera once and report status in logs and /status
    // Increase esp_video logs for troubleshooting (device registration, probe)
    esp_log_level_set("esp_video", ESP_LOG_DEBUG);
    esp_log_level_set("esp_video_vfs", ESP_LOG_DEBUG);
    esp_log_level_set("esp_cam_sensor", ESP_LOG_DEBUG);
    cam_probe_once();
    LOGW("Browse http://<ip>/status (JSON) and http://<ip>/latency for the gamepad RTT demo");
}
