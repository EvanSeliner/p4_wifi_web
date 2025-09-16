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
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
// I2C for SCCB smoke test
#include "driver/i2c.h"
// Camera/esp_video headers (device name via esp_video_device.h)
// Camera/esp_video probe
#include <fcntl.h>
#include <sys/ioctl.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include "linux/videodev2.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
// H.264 HW encoder (ESP32-P4)
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_alloc.h"
#include "esp_heap_caps.h"
#ifdef CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#ifndef MAP_FAILED
#define MAP_FAILED ((void*)-1)
#endif

static void log_ram(void){
    size_t i_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
    size_t i_min  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024;
    size_t x_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    size_t x_size = 0;
#ifdef CONFIG_SPIRAM
    x_size = esp_psram_get_size() / 1024;
#endif
    ESP_LOGI("RAM", "int_free=%u KB psram_size=%u KB psram_free=%u KB",
             (unsigned)i_free, (unsigned)x_size, (unsigned)x_free);
}

// Fallback SCCB defaults for esp_video_init when project Kconfig is not set
#ifndef CONFIG_P4_SCCB_PORT
#define CONFIG_P4_SCCB_PORT 0
#endif
#ifndef CONFIG_P4_SCCB_SDA_GPIO
#define CONFIG_P4_SCCB_SDA_GPIO 7
#endif
#ifndef CONFIG_P4_SCCB_SCL_GPIO
#define CONFIG_P4_SCCB_SCL_GPIO 8
#endif
#ifndef CONFIG_P4_SCCB_RESET_GPIO
#define CONFIG_P4_SCCB_RESET_GPIO -1
#endif
#ifndef CONFIG_P4_SCCB_PWDN_GPIO
#define CONFIG_P4_SCCB_PWDN_GPIO -1
#endif
#include <unistd.h>
// (build string via esp_get_idf_version())

// WiFi via Hosted/Remote (forwards esp_wifi_* to C6 over SDIO)
#include "esp_wifi_remote.h"
#include "lwip/inet.h"

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
static esp_err_t dev_get(httpd_req_t *r);
static esp_err_t i2c_get(httpd_req_t *r);
static esp_err_t latency_get(httpd_req_t *r);
static esp_err_t events_get(httpd_req_t *r);
static esp_err_t pad_handler(httpd_req_t *r);
static esp_err_t h264_stats_get(httpd_req_t *r);
#ifdef CONFIG_HTTPD_WS_SUPPORT
static esp_err_t ws_handler(httpd_req_t *req);
#endif
static void video_task(void *arg);
static void h264_task(void *arg);
static void video_stack_init(void);
static esp_err_t webrtc_offer_post(httpd_req_t *r);
static esp_err_t webrtc_js_get(httpd_req_t *r);

static httpd_handle_t g_httpd = NULL;
#ifdef CONFIG_HTTPD_WS_SUPPORT
static int s_ws_fd = -1;
#endif
static char s_ip_str[16] = {0};

// ---------------------- H.264 encode stats ----------------------
static volatile uint32_t g_h264_frames = 0;
static volatile uint32_t g_h264_bytes  = 0;
static volatile bool     g_h264_running = false;

static esp_err_t h264_stats_get(httpd_req_t *r){
    char buf[96];
    int n = snprintf(buf, sizeof(buf), "{\"running\":%s,\"frames\":%u,\"bytes\":%u}\n",
        g_h264_running?"true":"false", (unsigned)g_h264_frames, (unsigned)g_h264_bytes);
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_send(r, buf, n);
}

// Initialize esp_video stack (CSI + ISP registration)
static void video_stack_init(void){
    // Provide explicit SCCB/I2C + optional RESET/PWDN so init doesn't rely on menuconfig coupling
    int sda = CONFIG_P4_SCCB_SDA_GPIO;
    int scl = CONFIG_P4_SCCB_SCL_GPIO;
    int port = CONFIG_P4_SCCB_PORT;
    if (sda < 0 || scl < 0) {
        sda = 7;
        scl = 8;
        LOGW("SCCB pins unset in Kconfig; defaulting to SDA=%d SCL=%d on port %d", sda, scl, port);
    } else {
        LOGI("SCCB pins: SDA=%d SCL=%d on port %d", sda, scl, port);
    }
    esp_video_init_sccb_config_t sccb = {
        .init_sccb = true,
        .i2c_config = {
            .port = port,
            .scl_pin = scl,
            .sda_pin = sda,
        },
        .freq = 400000,
    };
    esp_video_init_csi_config_t csi = {
        .sccb_config = sccb,
        .reset_pin = CONFIG_P4_SCCB_RESET_GPIO,
        .pwdn_pin  = CONFIG_P4_SCCB_PWDN_GPIO,
    };
    esp_video_init_config_t vc = {
        .csi = &csi,
        .dvp = NULL,
        .jpeg = NULL,
    };
    esp_err_t e = esp_video_init(&vc);
    if (e != ESP_OK) LOGE("esp_video_init -> %s (0x%x)", esp_err_to_name(e), e);
    else LOGI("esp_video_init: OK");
}

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
        ip4addr_ntoa_r((const ip4_addr_t*)&event->ip_info.ip, s_ip_str, sizeof(s_ip_str));
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
static const char k_webrtc_js[] =
"(async()=>{\n"
"  try{\n"
"    const v=document.createElement('video');v.autoplay=true;v.playsInline=true;v.muted=true;v.style.width='100%';v.style.maxWidth='640px';document.body.appendChild(v);\n"
"    const pc=new RTCPeerConnection({iceServers:[]});\n"
"    pc.addTransceiver('video',{direction:'recvonly'});\n"
"    pc.ontrack=(e)=>{v.srcObject=e.streams[0];};\n"
"    const offer=await pc.createOffer();\n"
"    await pc.setLocalDescription(offer);\n"
"    const res=await fetch('/webrtc/offer',{method:'POST',headers:{'Content-Type':'application/sdp'},body:offer.sdp});\n"
"    const answer=await res.text();\n"
"    await pc.setRemoteDescription({type:'answer',sdp:answer});\n"
"  }catch(e){console.error('webrtc',e);}\n"
"})();\n";

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
static char     s_devnode[16]  = {0};

// Helper: list /dev and locate a /dev/videoX node
static void dump_dev_dir(void){
    DIR *d = opendir("/dev");
    if (!d) { LOGW("opendir(/dev) failed"); return; }
    struct dirent *e;
    LOGI("/dev contents:");
    while ((e = readdir(d)) != NULL) LOGI("  %s", e->d_name);
    closedir(d);
}

static const char* find_video_node(void){
    static char path[32];
    for (int i=0;i<4;i++){
        snprintf(path, sizeof(path), "/dev/video%d", i);
        int fd = open(path, O_RDWR);
        if (fd >= 0){ close(fd); return path; }
    }
    return NULL;
}

// --------------------- I2C/SCCB smoke test helpers ---------------------
#if CONFIG_P4_DEBUG_ENABLE_I2C_SMOKE
static esp_err_t i2c_smoke_init(void){
    int sda = CONFIG_P4_SCCB_SDA_GPIO;
    int scl = CONFIG_P4_SCCB_SCL_GPIO;
    int port = CONFIG_P4_SCCB_PORT;
    if (sda < 0 || scl < 0) { sda = 7; scl = 8; LOGW("I2C smoke: defaulting pins to SDA=%d SCL=%d (port %d)", sda, scl, port); }
    i2c_config_t c = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_P4_I2C_SMOKE_SPEED_HZ
    };
    esp_err_t e = i2c_param_config(port, &c);
    if (e != ESP_OK) return e;
    return i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
}

static bool i2c_probe_addr(uint8_t addr7){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7<<1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t e = i2c_master_cmd_begin(CONFIG_P4_SCCB_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return e == ESP_OK;
}

static bool ov5647_read_id(uint8_t addr7, uint8_t *id_high, uint8_t *id_low){
    // Read 0x300A then 0x300B
    uint8_t reg = 0x30, sub = 0x0A;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, sub, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7<<1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, id_high, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    if (i2c_master_cmd_begin(CONFIG_P4_SCCB_PORT, cmd, pdMS_TO_TICKS(50)) != ESP_OK){ i2c_cmd_link_delete(cmd); return false; }
    i2c_cmd_link_delete(cmd);

    reg = 0x30; sub = 0x0B;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7<<1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, sub, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7<<1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, id_low, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    bool ok = (i2c_master_cmd_begin(CONFIG_P4_SCCB_PORT, cmd, pdMS_TO_TICKS(50)) == ESP_OK);
    i2c_cmd_link_delete(cmd);
    return ok;
}

static void sccb_smoke_test(void){
    // Determine pins (use defaults if unset)
    int sda = CONFIG_P4_SCCB_SDA_GPIO;
    int scl = CONFIG_P4_SCCB_SCL_GPIO;
    int port = CONFIG_P4_SCCB_PORT;
    if (sda < 0 || scl < 0) { sda = 7; scl = 8; }
    if (i2c_smoke_init() != ESP_OK) return;
#if CONFIG_P4_DEBUG_TOGGLE_RESET
    if (CONFIG_P4_SCCB_RESET_GPIO >= 0){
        gpio_set_direction(CONFIG_P4_SCCB_RESET_GPIO, GPIO_MODE_OUTPUT);
        // Try an active-low pulse then release
        gpio_set_level(CONFIG_P4_SCCB_RESET_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(CONFIG_P4_SCCB_RESET_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(120));
    }
    if (CONFIG_P4_SCCB_PWDN_GPIO >= 0){
        gpio_set_direction(CONFIG_P4_SCCB_PWDN_GPIO, GPIO_MODE_OUTPUT);
        // Toggle PWDN both ways
        gpio_set_level(CONFIG_P4_SCCB_PWDN_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(CONFIG_P4_SCCB_PWDN_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(120));
    }
#endif
    LOGW("I2C smoke: scanning on port %d SDA=%d SCL=%d speed=%d",
         port, sda, scl, CONFIG_P4_I2C_SMOKE_SPEED_HZ);
    for (uint8_t a=0x08; a<0x78; ++a){ if (i2c_probe_addr(a)) LOGI("I2C ACK at 0x%02X", a); }
    uint8_t cand[2] = {0x36, 0x21};
    for (int i=0;i<2;i++){
        uint8_t hi=0, lo=0; uint8_t a=cand[i];
        if (i2c_probe_addr(a) && ov5647_read_id(a, &hi, &lo))
            LOGI("OV5647? addr=0x%02X ID=0x%02X 0x%02X (expect 0x56 0x47)", a, hi, lo);
        else
            LOGW("addr 0x%02X: no ID", a);
    }
}
#endif // CONFIG_P4_DEBUG_ENABLE_I2C_SMOKE

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
"\"cam_h\":%u,"
#ifdef CONFIG_CAMERA_OV5647
"\"kcfg_ov5647\":true,"
#else
"\"kcfg_ov5647\":false,"
#endif
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
"\"kcfg_mipi_csi\":true"
#else
"\"kcfg_mipi_csi\":false"
#endif
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

// HTTP: /i2c — returns scan + optional ID
static esp_err_t i2c_get(httpd_req_t *r){
#if CONFIG_P4_DEBUG_ENABLE_I2C_SMOKE
    httpd_resp_set_type(r, "application/json");
    char buf[768];
    size_t off=0;
    off += snprintf(buf+off, sizeof(buf)-off,
        "{\"port\":%d,\"sda\":%d,\"scl\":%d,\"speed\":%d,\"scan\":[",
        CONFIG_P4_SCCB_PORT, CONFIG_P4_SCCB_SDA_GPIO, CONFIG_P4_SCCB_SCL_GPIO, CONFIG_P4_I2C_SMOKE_SPEED_HZ);
    bool first=true;
    if (i2c_smoke_init()==ESP_OK){
        for (uint8_t a=0x08; a<0x78; ++a){ if (i2c_probe_addr(a)){
            off += snprintf(buf+off, sizeof(buf)-off, "%s\"0x%02X\"", first?"":",", a);
            first=false;
        }}
        uint8_t hi=0, lo=0; bool ok=false; uint8_t addr=0;
        uint8_t cand[2]={0x36,0x21};
        for(int i=0;i<2;i++){ addr=cand[i]; if (i2c_probe_addr(addr) && ov5647_read_id(addr,&hi,&lo)){ ok=true; break; } }
        off += snprintf(buf+off, sizeof(buf)-off,
            "],\"ov5647\":{\"ok\":%s,\"addr\":%u,\"id_hi\":%u,\"id_lo\":%u}}",
            ok?"true":"false", (unsigned)addr, (unsigned)hi, (unsigned)lo);
    } else {
        off += snprintf(buf+off, sizeof(buf)-off, "]}");
    }
    return httpd_resp_send(r, buf, off);
#else
    httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "P4_DEBUG_ENABLE_I2C_SMOKE disabled");
    return ESP_OK;
#endif
}

// HTTP: /dev — list nodes, show candidate videos
static esp_err_t dev_get(httpd_req_t *r){
    httpd_resp_set_type(r, "text/plain");
    DIR *d = opendir("/dev");
    if (!d){ return httpd_resp_sendstr(r, "opendir(/dev) failed\n"); }
    char line[64];
    httpd_resp_sendstr_chunk(r, "DEV LIST:\n");
    struct dirent *e; while ((e=readdir(d))!=NULL){
        int n = snprintf(line, sizeof(line), "  %s\n", e->d_name);
        httpd_resp_send_chunk(r, line, n);
    }
    closedir(d);
    const char* vid = find_video_node();
    if (vid){
        int n = snprintf(line, sizeof(line), "video: %s\n", vid);
        httpd_resp_send_chunk(r, line, n);
    }
    return httpd_resp_send_chunk(r, NULL, 0);
}

// Minimal WebRTC client helper script
static esp_err_t webrtc_js_get(httpd_req_t *r){
    httpd_resp_set_type(r, "application/javascript");
    return httpd_resp_send(r, k_webrtc_js, HTTPD_RESP_USE_STRLEN);
}

// --- Minimal ICE-lite SDP answer builder ---
static char hexrand()
{ const char* h="0123456789abcdef"; return h[esp_random() & 0xF]; }

static void make_rand_token(char *out, size_t n)
{ for(size_t i=0;i+1<n;i++) out[i]=hexrand(); out[n-1]='\0'; }

static char* build_sdp_answer(const char *ip, uint16_t rtp_port)
{
    char ufrag[9], pwd[33];
    make_rand_token(ufrag, sizeof(ufrag));
    make_rand_token(pwd, sizeof(pwd));
    // Dummy fingerprint (placeholder). DTLS implementation will replace this.
    const char *fp = "00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00:00";
    char *s = malloc(1024);
    if (!s) return NULL;
    int n = snprintf(s, 1024,
        "v=0\r\n"
        "o=- 0 0 IN IP4 %s\r\n"
        "s=ESP32-P4\r\n"
        "t=0 0\r\n"
        "a=ice-lite\r\n"
        "a=group:BUNDLE 0\r\n"
        "a=msid-semantic: WMS\r\n"
        "m=video %u UDP/TLS/RTP/SAVPF 96\r\n"
        "c=IN IP4 %s\r\n"
        "a=rtcp-mux\r\n"
        "a=setup:passive\r\n"
        "a=mid:0\r\n"
        "a=sendonly\r\n"
        "a=ice-ufrag:%s\r\n"
        "a=ice-pwd:%s\r\n"
        "a=fingerprint:sha-256 %s\r\n"
        "a=rtpmap:96 H264/90000\r\n"
        "a=fmtp:96 packetization-mode=1;profile-level-id=42e01f\r\n"
        "a=ssrc:12345678 cname:esp\r\n"
        "a=candidate:1 1 udp 2130706431 %s %u typ host generation 0\r\n",
        ip, (unsigned)rtp_port, ip, ufrag, pwd, fp, ip, (unsigned)rtp_port);
    if (n <= 0) { free(s); return NULL; }
    return s;
}

// HTTP: /webrtc/offer — accept browser SDP offer and reply with ICE-lite answer
static esp_err_t webrtc_offer_post(httpd_req_t *r)
{
    int len = r->content_len;
    if (len <= 0 || len > 8192) return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "bad sdp");
    char *offer = (char*)malloc(len+1);
    if (!offer) return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    int got = httpd_req_recv(r, offer, len);
    if (got < 0) { free(offer); return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "recv fail"); }
    offer[got] = '\0';
    LOGI("/webrtc/offer: got %d bytes", got);
    // TODO: parse minimal fields if needed. For now, we choose a static RTP port.
    uint16_t rtp_port = 52000;
    const char *ip = s_ip_str[0] ? s_ip_str : "192.168.0.2";
    char *answer = build_sdp_answer(ip, rtp_port);
    free(offer);
    if (!answer) return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "sdp build fail");
    httpd_resp_set_type(r, "application/sdp");
    esp_err_t rc = httpd_resp_sendstr(r, answer);
    free(answer);
    return rc;
}

// HTTP: /probe — re-run camera probe and return JSON
static void cam_probe_once(void); // forward
static esp_err_t probe_get(httpd_req_t *r){
    httpd_resp_set_type(r, "application/json");
    s_cam_ok=false; s_cam_card[0]=s_cam_bus[0]=0; s_cam_w=s_cam_h=0; s_devnode[0]=0;
    cam_probe_once();
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"dev\":\"%s\",\"ok\":%s,\"card\":\"%s\",\"bus\":\"%s\",\"w\":%u,\"h\":%u}",
        s_devnode, s_cam_ok?"true":"false", s_cam_card, s_cam_bus, (unsigned)s_cam_w, (unsigned)s_cam_h);
    return httpd_resp_send(r, buf, n);
}

// HTTP: /kcfg — dump key Kconfig flags
static esp_err_t kcfg_get(httpd_req_t *r){
    httpd_resp_set_type(r, "application/json");
    char buf[256];
    int n = snprintf(buf, sizeof(buf),
        "{\"OV5647\":%s,\"MIPI_CSI\":%s,\"XCLK_ROUTER\":%s}",
#ifdef CONFIG_CAMERA_OV5647
        "true",
#else
        "false",
#endif
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
        "true",
#else
        "false",
#endif
#ifdef CONFIG_CAMERA_XCLK_USE_ESP_CLOCK_ROUTER
        "true"
#else
        "false"
#endif
    );
    return httpd_resp_send(r, buf, n);
}

// Optional tiny HTTP check for camera
static esp_err_t cam_ok_get(httpd_req_t *r){
    httpd_resp_set_type(r, "text/plain");
    if (s_cam_ok) return httpd_resp_sendstr(r, "CAM=OK\n");
    return httpd_resp_sendstr(r, "CAM=FAIL\n");
}

// One-shot camera probe using V4L2 QUERYCAP via esp_video VFS device
static void cam_probe_once(void) {
    // Give the driver a brief moment to register VFS device
    const char *dev = NULL;
    for (int tries=0; tries<20 && !dev; ++tries){
        dev = find_video_node();
        if (!dev) vTaskDelay(pdMS_TO_TICKS(100));
    }
    dump_dev_dir();
    if (!dev){
        LOGE("camera: no /dev/video* found (sensor not registered)");
        s_cam_ok = false;
        return;
    }
    LOGI("camera: opening %s", dev);
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


// -------------------------- Minimal H.264 encode task --------------------------
static inline void i420_to_ouyy_evyy(uint8_t *dst, const uint8_t *src, int w, int h){
    const uint8_t *Y = src;
    const uint8_t *U = Y + (size_t)w*h;
    const uint8_t *V = U + ((size_t)w*h>>2);
    uint8_t *dp = dst;
    for (int j=0; j<h; j+=2){
        const uint8_t *y0 = Y + (size_t)j*w;
        const uint8_t *u0 = U + (size_t)(j/2)*(w/2);
        for (int i=0; i<w; i+=2){ *dp++ = u0[i/2]; *dp++ = y0[i+0]; *dp++ = y0[i+1]; }
        const uint8_t *y1 = Y + (size_t)(j+1)*w;
        const uint8_t *v0 = V + (size_t)(j/2)*(w/2);
        for (int i=0; i<w; i+=2){ *dp++ = v0[i/2]; *dp++ = y1[i+0]; *dp++ = y1[i+1]; }
    }
}
static void h264_task(void *arg){
    const char *dev = find_video_node();
    if (!dev){ vTaskDelay(pdMS_TO_TICKS(1000)); dev = find_video_node(); }
    if (!dev){ LOGW("h264: no /dev/video* (camera not detected) — skipping"); vTaskDelete(NULL); return; }

    int fd = open(dev, O_RDWR);
    if (fd < 0){ LOGW("h264: open(%s) failed", dev); vTaskDelete(NULL); return; }

    int w = 640, h = 480;
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width  = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420; // I420
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) != 0){ LOGE("h264: S_FMT I420 %dx%d failed", w, h); close(fd); vTaskDelete(NULL); return; }
    w = fmt.fmt.pix.width; h = fmt.fmt.pix.height;
    LOGI("h264: capture fmt -> %ux%u fourcc=0x%08x", w, h, fmt.fmt.pix.pixelformat);

    struct v4l2_requestbuffers req = {0};
    req.count = 3; req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2){ LOGE("h264: REQBUFS MMAP failed (count=%u)", req.count); close(fd); vTaskDelete(NULL); return; }
    void *buf_ptr[4] = {0}; size_t buf_len[4] = {0};
    for (uint32_t i=0;i<req.count;i++){
        struct v4l2_buffer b = {0}; b.type = req.type; b.memory = req.memory; b.index = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0){ LOGE("h264: QUERYBUF %u failed", i); goto stop_close; }
        buf_ptr[i] = mmap(NULL, b.length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        buf_len[i] = b.length;
        if (buf_ptr[i] == MAP_FAILED){ LOGE("h264: mmap failed idx=%u", i); goto stop_close; }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0){ LOGE("h264: QBUF %u failed", i); goto stop_close; }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0){ LOGE("h264: STREAMON failed"); goto stop_close; }

    esp_h264_enc_cfg_hw_t cfg = {0};
    cfg.pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY; // packed 4:2:0 expected by HW
    cfg.gop = 30; cfg.fps = 30;
    cfg.res.width = w; cfg.res.height = h;
    cfg.rc.bitrate = 2*1024*1024; cfg.rc.qp_min = 20; cfg.rc.qp_max = 45;

    esp_h264_enc_handle_t enc = NULL;
    if (esp_h264_enc_hw_new(&cfg, &enc) != ESP_H264_ERR_OK || !enc){ LOGE("h264: enc_new failed"); goto stop_stream; }
    if (esp_h264_enc_open(enc) != ESP_H264_ERR_OK){ LOGE("h264: enc_open failed"); goto del_enc; }

    // Buffers used by HW encoder must be cacheline-aligned (64B) and sizes often
    // rounded to cacheline multiples to satisfy esp_cache_msync requirements.
    size_t yuv_len = (size_t)w*h + ((size_t)w*h>>1);
    const size_t ALIGN = 64;
    size_t out_cap = 512*1024;
    // Allocate with 64B alignment in PSRAM
    uint8_t *wrk    = (uint8_t*)heap_caps_aligned_alloc(ALIGN, yuv_len,   MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    uint8_t *outbuf = (uint8_t*)heap_caps_aligned_alloc(ALIGN, out_cap,   MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
    if (!wrk || !outbuf){ LOGE("h264: SPIRAM alloc failed"); goto close_enc; }

    g_h264_running = true;
    TickType_t last_log = xTaskGetTickCount();
    for(;;){
        struct v4l2_buffer b = {0}; b.type = req.type; b.memory = req.memory;
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0){ LOGW("h264: DQBUF failed"); break; }

        // Convert I420 (Y,U,V planes) read from MMAP buffer
        i420_to_ouyy_evyy(wrk, (const uint8_t*)buf_ptr[b.index], w, h);
        esp_h264_enc_in_frame_t in = { .raw_data = { .buffer = wrk,    .len = yuv_len }, .pts = 0 };
        esp_h264_enc_out_frame_t out = { .raw_data = { .buffer = outbuf, .len = out_cap } };
        esp_h264_err_t er = esp_h264_enc_process(enc, &in, &out);
        if (er == ESP_H264_ERR_OK){ g_h264_frames++; g_h264_bytes += out.length; }
        else { LOGW("h264: process err=%d", er); }

        if (ioctl(fd, VIDIOC_QBUF, &b) != 0){ LOGW("h264: QBUF after DQBUF failed"); break; }
        if (xTaskGetTickCount() - last_log > pdMS_TO_TICKS(2000)){
            last_log = xTaskGetTickCount();
            LOGI("h264: frames=%u bytes=%u", (unsigned)g_h264_frames, (unsigned)g_h264_bytes);
        }
    }

    g_h264_running = false;
    if (wrk) heap_caps_free(wrk);
    if (outbuf) heap_caps_free(outbuf);
close_enc:
    esp_h264_enc_close(enc);
del_enc:
    esp_h264_enc_del(enc);
stop_stream:
    type = req.type; ioctl(fd, VIDIOC_STREAMOFF, &type);
stop_close:
    for (uint32_t i=0;i<req.count;i++) if (buf_ptr[i]) munmap(buf_ptr[i], buf_len[i]);
    close(fd);
    vTaskDelete(NULL);
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
    httpd_uri_t u_dev     = { .uri="/dev",     .method=HTTP_GET,  .handler=dev_get     };
    httpd_uri_t u_i2c     = { .uri="/i2c",     .method=HTTP_GET,  .handler=i2c_get     };
    httpd_uri_t u_probe   = { .uri="/probe",   .method=HTTP_GET,  .handler=probe_get   };
    httpd_uri_t u_h264    = { .uri="/h264.stats", .method=HTTP_GET, .handler=h264_stats_get };
    httpd_uri_t u_kcfg    = { .uri="/kcfg",    .method=HTTP_GET,  .handler=kcfg_get    };
    httpd_uri_t u_webo    = { .uri="/webrtc/offer", .method=HTTP_POST, .handler=webrtc_offer_post };
    httpd_uri_t u_webrtc  = { .uri="/webrtc.js", .method=HTTP_GET, .handler=webrtc_js_get };
    httpd_uri_t u_pad_g   = { .uri="/pad",     .method=HTTP_GET,  .handler=pad_handler };
    httpd_uri_t u_pad_p   = { .uri="/pad",     .method=HTTP_POST, .handler=pad_handler };
    TRY( httpd_register_uri_handler(h, &u_root) );
    TRY( httpd_register_uri_handler(h, &u_status) );
    TRY( httpd_register_uri_handler(h, &u_latency) );
    TRY( httpd_register_uri_handler(h, &u_ui) );
    TRY( httpd_register_uri_handler(h, &u_events) );
    TRY( httpd_register_uri_handler(h, &u_cam_ok) );
    TRY( httpd_register_uri_handler(h, &u_dev) );
    TRY( httpd_register_uri_handler(h, &u_i2c) );
    TRY( httpd_register_uri_handler(h, &u_probe) );
    TRY( httpd_register_uri_handler(h, &u_kcfg) );
    TRY( httpd_register_uri_handler(h, &u_h264) );
    TRY( httpd_register_uri_handler(h, &u_webo) );
    TRY( httpd_register_uri_handler(h, &u_webrtc) );
#ifdef CONFIG_HTTPD_WS_SUPPORT
    httpd_uri_t u_ws = { .uri="/ws", .method=HTTP_GET, .handler=ws_handler, .user_ctx=NULL, .is_websocket = true };
    TRY( httpd_register_uri_handler(h, &u_ws) );
    g_httpd = h;
    // Launch lightweight WS tick task (placeholder)
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
    // Turn up camera stack logs
    esp_log_level_set("esp_video", ESP_LOG_DEBUG);
    esp_log_level_set("esp_video_vfs", ESP_LOG_DEBUG);
    esp_log_level_set("esp_cam_sensor", ESP_LOG_DEBUG);
    esp_log_level_set("esp_driver_cam", ESP_LOG_DEBUG);
    esp_log_level_set("cam_ctlr_mipi_csi", ESP_LOG_DEBUG);
    esp_log_level_set("isp", ESP_LOG_DEBUG);

    ensure_nvs();
    log_ram();
    // Bring up esp_video stack before probing CSI
    video_stack_init();
    // Quick platform dump
    esp_chip_info_t chip; esp_chip_info(&chip);
    LOGI("Chip: model=%d cores=%d rev=%d features=0x%x", chip.model, chip.cores, chip.revision, chip.features);
    // Show key Kconfig camera/video flags at runtime
#ifdef CONFIG_CAMERA_OV5647
    LOGI("Kconfig: CONFIG_CAMERA_OV5647 = y");
#else
    LOGW("Kconfig: CONFIG_CAMERA_OV5647 = n (enable in menuconfig)");
#endif
#ifdef CONFIG_ESP_VIDEO_ENABLE_MIPI_CSI_VIDEO_DEVICE
    LOGI("Kconfig: MIPI-CSI video device enabled");
#else
    LOGW("Kconfig: MIPI-CSI video device DISABLED");
#endif
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

    // Optional: run SCCB smoke test early to see ACKs/ID
#if CONFIG_P4_DEBUG_ENABLE_I2C_SMOKE
    sccb_smoke_test();
#endif
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
    // Start minimal H.264 encode loop (NV12 -> HW encoder), non-fatal if camera absent
    xTaskCreatePinnedToCore(h264_task, "h264", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    LOGW("Browse http://<ip>/status (JSON) and http://<ip>/latency for the gamepad RTT demo");
}
