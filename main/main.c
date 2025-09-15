// p4_wifi_web/main.c — Embedded UI + WebSocket streamer

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#include "cam.h"  // provides cam_init(), cam_get_jpeg()

#define TAG "P4+C6"

// --- Minimal embedded UI so we don't depend on external files ---
static const char INDEX_HTML[] =
"<!doctype html>\n"
"<meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
"<title>P4 Cam</title>\n"
"<style>"
"body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu,\"Helvetica Neue\",Arial,sans-serif;margin:0}"
"header,footer{padding:10px 14px;background:#111;color:#fff}main{padding:10px}"
"#row{display:flex;gap:14px;align-items:center;flex-wrap:wrap}"
"#cam{max-width:100%;height:auto;border:1px solid #ccc;border-radius:8px}"
".pill{display:inline-block;padding:2px 8px;border-radius:999px;background:#eee;margin-left:6px}"
"</style>\n"
"<header>P4 Camera <span class=\"pill\" id=\"ws\">WS: ?</span> <span class=\"pill\" id=\"fps\">frames: 0</span> <span class=\"pill\" id=\"msg\">msg: 0</span></header>\n"
"<main><div id=\"row\"><img id=\"cam\" alt=\"frame\"></div></main>\n"
"<footer>Open console for logs. <button id=\"start\">start</button> <button id=\"stop\">stop</button></footer>\n"
"<script src=\"/ui.js\"></script>\n";

static const char UI_JS[] =
"(()=>{\n"
"const wsEl=document.getElementById('ws');\n"
"const fpsEl=document.getElementById('fps');\n"
"const msgEl=document.getElementById('msg');\n"
"const img=document.getElementById('cam');\n"
"const btnS=document.getElementById('start');\n"
"const btnP=document.getElementById('stop');\n"
"let msgs=0,frames=0,ws;\n"
"function upd(){fpsEl.textContent='frames: '+frames;msgEl.textContent='msg: '+msgs;}\n"
"function openWS(){\n"
"  const proto=location.protocol==='https:'?'wss':'ws';\n"
"  ws=new WebSocket(proto+'://'+location.host+'/ws');\n"
"  ws.binaryType='arraybuffer';\n"
"  ws.onopen = ()=>{wsEl.textContent='WS: OPEN'};\n"
"  ws.onclose= ()=>{wsEl.textContent='WS: CLOSED'; setTimeout(openWS,1000)};\n"
"  ws.onerror= ()=>{wsEl.textContent='WS: ERROR'};\n"
"  ws.onmessage=(ev)=>{\n"
"    msgs++;\n"
"    if(ev.data instanceof ArrayBuffer){\n"
"      const blob=new Blob([ev.data],{type:'image/jpeg'});\n"
"      img.src=URL.createObjectURL(blob);\n"
"      frames++;\n"
"    }else{\n"
"      try{const t=ev.data.toString();if(t==='ping'&&ws.readyState===1){ws.send('pong');}}catch{}\n"
"    }\n"
"    upd();\n"
"  };\n"
"}\n"
"btnS.onclick=()=>{try{ws&&ws.readyState===1&&ws.send('start');}catch{}};\n"
"btnP.onclick=()=>{try{ws&&ws.readyState===1&&ws.send('stop');}catch{}};\n"
"openWS();\n"
"})();\n";

static httpd_handle_t g_httpd = NULL;
static int            s_ws_fd = -1;       // active client sockfd or -1
static volatile bool  s_stream_on = true; // UI toggles via WS text "start/stop"

// Forward decls
static esp_err_t root_get(httpd_req_t *r);
static esp_err_t ui_js_get(httpd_req_t *r);
static esp_err_t status_get(httpd_req_t *r);
static esp_err_t ws_handler(httpd_req_t *req);
static void      video_task(void *arg);
static httpd_handle_t start_web(void);

static esp_err_t root_get(httpd_req_t *r){
    httpd_resp_set_type(r, "text/html; charset=utf-8");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t ui_js_get(httpd_req_t *r){
    httpd_resp_set_type(r, "application/javascript");
    httpd_resp_set_hdr(r, "Cache-Control", "no-store");
    return httpd_resp_send(r, UI_JS, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get(httpd_req_t *r){
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "{\"ws\":%s,\"stream\":%s}",
                     (s_ws_fd>=0)?"true":"false", s_stream_on?"true":"false");
    httpd_resp_set_type(r, "application/json");
    return httpd_resp_send(r, buf, n);
}

// ================= WebSocket =================
static esp_err_t ws_handler(httpd_req_t *req){
#if !CONFIG_HTTPD_WS_SUPPORT
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "WS disabled");
    return ESP_OK;
#else
    if (req->method == HTTP_GET){
        s_ws_fd = httpd_req_to_sockfd(req);
        ESP_LOGW(TAG, "WS open on /ws (sock %d)", s_ws_fd);
        return ESP_OK;
    }

    httpd_ws_frame_t frame = {0};
    frame.type = HTTPD_WS_TYPE_TEXT;
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) return ret;
    if (frame.len){
        frame.payload = malloc(frame.len + 1);
        if (!frame.payload) return ESP_ERR_NO_MEM;
        ret = httpd_ws_recv_frame(req, &frame, frame.len);
        if (ret != ESP_OK){ free(frame.payload); return ret; }
        ((char*)frame.payload)[frame.len] = '\0';
    }

    if (frame.type == HTTPD_WS_TYPE_TEXT && frame.payload){
        const char *txt = (const char*)frame.payload;
        if (!strcmp(txt, "ping")){
            httpd_ws_frame_t resp = {.type=HTTPD_WS_TYPE_TEXT};
            resp.payload = (uint8_t*)"pong"; resp.len = 4;
            ret = httpd_ws_send_frame(req, &resp);
        } else if (!strcmp(txt, "start")){
            s_stream_on = true;
        } else if (!strcmp(txt, "stop")){
            s_stream_on = false;
        }
    }

    if (frame.payload) free(frame.payload);
    return ret;
#endif
}

// ================= Video push task =================
static void video_task(void *arg){
    bool tried_init=false;
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(30)); // ~33 fps cap
        if (!tried_init){ tried_init=true; (void)cam_init(); }
        if (!(g_httpd && s_ws_fd>=0)) continue;
        if (!s_stream_on) continue;

        const uint8_t *jpg=NULL; size_t jlen=0;
        if (!cam_get_jpeg(&jpg, &jlen) || !jpg || jlen==0) continue;

        httpd_ws_frame_t f = {0};
        f.type = HTTPD_WS_TYPE_BINARY;
        f.payload = (uint8_t*)jpg;
        f.len = jlen;
        (void)httpd_ws_send_frame_async(g_httpd, s_ws_fd, &f);
    }
}

// ================= Server start =================
static httpd_handle_t start_web(void){
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 12;
    cfg.max_uri_handlers = 16;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t h = NULL;
    esp_err_t e = httpd_start(&h, &cfg);
    if (e != ESP_OK){ ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(e)); return NULL; }

    httpd_uri_t u_root   = { .uri="/",       .method=HTTP_GET, .handler=root_get };
    httpd_uri_t u_ui     = { .uri="/ui.js",  .method=HTTP_GET, .handler=ui_js_get };
    httpd_uri_t u_status = { .uri="/status", .method=HTTP_GET, .handler=status_get };
    httpd_uri_t u_ws     = { .uri="/ws",     .method=HTTP_GET, .handler=ws_handler, .is_websocket = true };

    if ((e = httpd_register_uri_handler(h, &u_root))   != ESP_OK) ESP_LOGW(TAG, "register / -> %s",   esp_err_to_name(e));
    if ((e = httpd_register_uri_handler(h, &u_ui))     != ESP_OK) ESP_LOGW(TAG, "register /ui.js -> %s", esp_err_to_name(e));
    if ((e = httpd_register_uri_handler(h, &u_status)) != ESP_OK) ESP_LOGW(TAG, "register /status -> %s", esp_err_to_name(e));
    if ((e = httpd_register_uri_handler(h, &u_ws))     != ESP_OK) ESP_LOGW(TAG, "register /ws -> %s (is WS enabled in menuconfig?)", esp_err_to_name(e));

    g_httpd = h;
    xTaskCreatePinnedToCore(video_task, "vid", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    ESP_LOGW(TAG, "WS on /ws, UI at / (loads /ui.js). Camera frames stream when WS is open.");
    return h;
}

// ================= Minimal Wi‑Fi glue (STA) =================
static void wifi_sta_start(const char *ssid, const char *pass){
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t el = esp_event_loop_create_default();
    if (el != ESP_OK && el != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(el);

    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wicfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wicfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t cfg = {0};
    strlcpy((char*)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char*)cfg.sta.password, pass, sizeof(cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    const char *STA_SSID = "DaegainSmells";   // replace with your SSID
    const char *STA_PASS = "1234567890";      // replace with your password
    wifi_sta_start(STA_SSID, STA_PASS);
    start_web();
}

