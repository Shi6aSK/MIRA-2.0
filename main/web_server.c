#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_camera.h"
#include "web_server.h"
#include "vision_pipeline.h"
#include "vision_config.h"
#include "vision_types.h"
#include "training.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "servo_control.h"
#include "mic_capture.h"

static const char *TAG = "http";

/* ── Shared JPEG frame buffer (updated by vision_task) ──────── */
static uint8_t          *s_jpg_buf  = NULL;
static size_t            s_jpg_len  = 0;
static SemaphoreHandle_t s_jpg_lock = NULL;

void web_server_update_frame(const uint8_t *jpg, size_t len)
{
    if (!s_jpg_lock || !jpg || len == 0) return;
    if (xSemaphoreTake(s_jpg_lock, pdMS_TO_TICKS(10)) != pdTRUE) return;

    /* Reuse buffer if it's big enough, otherwise reallocate */
    if (!s_jpg_buf || len > s_jpg_len) {
        free(s_jpg_buf);
        s_jpg_buf = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_jpg_buf) s_jpg_buf = malloc(len);
    }
    if (s_jpg_buf) {
        memcpy(s_jpg_buf, jpg, len);
        s_jpg_len = len;
    }
    xSemaphoreGive(s_jpg_lock);
}

static void set_cors(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
}

/* ── MJPEG streaming task (runs off the HTTP server task) ─────────────── */
#define STREAM_BOUNDARY  "mjpegframe"
#define STREAM_CT        "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY

static void stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    char      hdr[96];
    esp_err_t res = ESP_OK;

    while (res == ESP_OK) {
        if (xSemaphoreTake(s_jpg_lock, pdMS_TO_TICKS(1000)) != pdTRUE) continue;
        if (!s_jpg_buf || s_jpg_len == 0) {
            xSemaphoreGive(s_jpg_lock);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        size_t   llen  = s_jpg_len;
        uint8_t *local = malloc(llen);
        if (local) memcpy(local, s_jpg_buf, llen);
        xSemaphoreGive(s_jpg_lock);

        if (!local) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        int hlen = snprintf(hdr, sizeof(hdr),
            "--" STREAM_BOUNDARY "\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %u\r\n\r\n", (unsigned)llen);
        res = httpd_resp_send_chunk(req, hdr, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (char *)local, (ssize_t)llen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n", 2);
        free(local);

        vTaskDelay(pdMS_TO_TICKS(100));  /* ~10 fps */
    }

    httpd_req_async_handler_complete(req);  /* release back to server */
    vTaskDelete(NULL);
}

/* ── GET /stream  – hands off to async task immediately ──────────────── */
static esp_err_t stream_handler(httpd_req_t *req)
{
    if (!s_jpg_lock) { httpd_resp_send_500(req); return ESP_FAIL; }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    set_cors(async_req);
    httpd_resp_set_type(async_req, STREAM_CT);
    httpd_resp_set_hdr(async_req, "Cache-Control", "no-store");

    /* Spawn the stream task – stack 6 KB is enough (no inference here) */
    if (xTaskCreate(stream_task, "mjpeg", 6144, async_req, 4, NULL) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ── GET /detect ─────────────────────────────────────────────── */
static esp_err_t detect_handler(httpd_req_t *req)
{
    detection_t d = vision_get_detection();
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"obj\":%d,\"kind\":\"%s\",\"gesture\":\"%s\","
             "\"side\":\"%s\",\"bbox\":[%d,%d,%d,%d],"
             "\"score\":%.3f,\"frame_id\":%u}",
             d.object_present ? 1 : 0,
             d.kind, d.gesture, d.side,
             d.x1, d.y1, d.x2, d.y2,
             (double)d.score, (unsigned)d.frame_id);
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, buf);
    return ESP_OK;
}

/* ── GET /train  ?op=face_start|gesture_start|stop|status [&label=NAME] */
static esp_err_t train_handler(httpd_req_t *req)
{
    char query[160] = "";
    httpd_req_get_url_query_str(req, query, sizeof(query));

    char op[32]    = "";
    char label[48] = "";
    httpd_query_key_value(query, "op",    op,    sizeof(op));
    httpd_query_key_value(query, "label", label, sizeof(label));

    char resp[160];

    if (strcmp(op, "face_start") == 0) {
        if (!label[0]) strcpy(label, "person");
        training_start_face(label);
        snprintf(resp, sizeof(resp),
                 "{\"msg\":\"Face training started\",\"label\":\"%s\"}", label);

    } else if (strcmp(op, "gesture_start") == 0) {
        if (!label[0]) strcpy(label, "gesture");
        training_start_gesture(label);
        snprintf(resp, sizeof(resp),
                 "{\"msg\":\"Gesture training started\",\"label\":\"%s\"}", label);

    } else if (strcmp(op, "stop") == 0) {
        int cnt = training_get_count();
        training_stop();
        snprintf(resp, sizeof(resp),
                 "{\"msg\":\"Training stopped\",\"count\":%d}", cnt);

    } else {
        /* default: status */
        static const char *names[] = {"idle", "face", "gesture"};
        snprintf(resp, sizeof(resp),
                 "{\"mode\":\"%s\",\"label\":\"%s\",\"count\":%d}",
                 names[training_get_mode()],
                 training_get_label(),
                 training_get_count());
    }

    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/* ── GET /cam  ?brightness=N&contrast=N&saturation=N&sharpness=N&ae_level=N&gainceiling=N */
static esp_err_t cam_handler(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    char query[256] = "";
    httpd_req_get_url_query_str(req, query, sizeof(query));
    char val[8];
    if (s) {
        if (httpd_query_key_value(query, "brightness",  val, sizeof(val)) == ESP_OK) s->set_brightness(s,  atoi(val));
        if (httpd_query_key_value(query, "contrast",    val, sizeof(val)) == ESP_OK) s->set_contrast(s,    atoi(val));
        if (httpd_query_key_value(query, "saturation",  val, sizeof(val)) == ESP_OK) s->set_saturation(s,  atoi(val));
        if (httpd_query_key_value(query, "sharpness",   val, sizeof(val)) == ESP_OK) s->set_sharpness(s,   atoi(val));
        if (httpd_query_key_value(query, "ae_level",    val, sizeof(val)) == ESP_OK) s->set_ae_level(s,    atoi(val));
        if (httpd_query_key_value(query, "gainceiling", val, sizeof(val)) == ESP_OK) s->set_gainceiling(s, (gainceiling_t)atoi(val));
    }
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, s ? "{\"ok\":1}" : "{\"err\":\"no sensor\"}");
    return ESP_OK;
}

/* ── GET /servo  ?flip_pan=1 | ?flip_tilt=1 | ?center=1 ─────────── */
static esp_err_t servo_handler(httpd_req_t *req)
{
    char query[64] = "";
    httpd_req_get_url_query_str(req, query, sizeof(query));
    char val[8];
    bool pi, ti;
    servo_get_invert(&pi, &ti);
    if (httpd_query_key_value(query, "flip_pan",  val, sizeof(val)) == ESP_OK) pi = !pi;
    if (httpd_query_key_value(query, "flip_tilt", val, sizeof(val)) == ESP_OK) ti = !ti;
    servo_set_invert(pi, ti);
    if (httpd_query_key_value(query, "center",    val, sizeof(val)) == ESP_OK) servo_center();
    char pv[8], tv[8];
    if (httpd_query_key_value(query, "pan",  pv, sizeof(pv)) == ESP_OK) servo_set_pan(atoi(pv));
    if (httpd_query_key_value(query, "tilt", tv, sizeof(tv)) == ESP_OK) servo_set_tilt(atoi(tv));
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    char resp[64];
    snprintf(resp, sizeof(resp), "{\"pan_inv\":%d,\"tilt_inv\":%d}", pi ? 1 : 0, ti ? 1 : 0);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

/* ── GET /snap  – single JPEG snapshot (for Gemma proxy) ────────── */
static esp_err_t snap_handler(httpd_req_t *req)
{
    if (!s_jpg_lock || xSemaphoreTake(s_jpg_lock, pdMS_TO_TICKS(2000)) != pdTRUE) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    if (!s_jpg_buf || s_jpg_len == 0) {
        xSemaphoreGive(s_jpg_lock);
        httpd_resp_send_err(req, HTTPD_408_REQ_TIMEOUT, "No frame yet");
        return ESP_FAIL;
    }
    set_cors(req);
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = httpd_resp_send(req, (const char *)s_jpg_buf, (ssize_t)s_jpg_len);
    xSemaphoreGive(s_jpg_lock);
    return ret;
}

/* ── Gemma vision analysis ──────────────────────────────────── */
static char          s_gemma_result[512] = "";
static char          s_gemma_ctx[32]     = "describe";
static volatile bool s_gemma_pending     = false;

void web_server_auto_trigger_gemma(const char *ctx)
{
    if (s_gemma_pending) return;   /* already waiting – don't clobber */
    s_gemma_pending    = true;
    s_gemma_result[0]  = '\0';
    snprintf(s_gemma_ctx, sizeof(s_gemma_ctx), "%s", ctx ? ctx : "describe");
}

static esp_err_t gemma_handler(httpd_req_t *req)
{
    char query[64] = "";
    httpd_req_get_url_query_str(req, query, sizeof(query));
    char val[8] = "";
    httpd_query_key_value(query, "trigger", val, sizeof(val));
    if (val[0] == '1') {
        s_gemma_pending   = true;
        s_gemma_result[0] = '\0';
        snprintf(s_gemma_ctx, sizeof(s_gemma_ctx), "describe");
    }
    /* sanitise for JSON embedding */
    char safe[520]; int si = 0;
    for (int ri = 0; s_gemma_result[ri] && si < (int)sizeof(safe)-2; ri++) {
        char c = s_gemma_result[ri];
        if (c == '"') safe[si++] = '\'';
        else if (c == '\n' || c == '\r') safe[si++] = ' ';
        else safe[si++] = c;
    }
    safe[si] = '\0';
    char resp[640];
    snprintf(resp, sizeof(resp), "{\"pending\":%s,\"ctx\":\"%s\",\"result\":\"%s\"}",
             s_gemma_pending ? "true" : "false", s_gemma_ctx, safe);
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t gemma_result_handler(httpd_req_t *req)
{
    int len = req->content_len;
    if (len <= 0 || len >= (int)sizeof(s_gemma_result) - 1)
        len = (int)sizeof(s_gemma_result) - 1;
    int got = httpd_req_recv(req, s_gemma_result, len);
    if (got > 0) { s_gemma_result[got] = '\0'; s_gemma_pending = false; }
    set_cors(req);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":1}");
    return ESP_OK;
}

/* ── GET /  (dashboard) ──────────────────────────────────────── */
static const char ROOT_HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>ESP32 Vision</title>"
    "<style>"
    "body{margin:0;font-family:monospace;background:#111;color:#ccc;font-size:13px}"
    ".w{display:flex;flex-wrap:wrap;gap:8px;padding:8px;align-items:flex-start}"
    ".p{background:#1e1e1e;border:1px solid #333;border-radius:6px;padding:10px}"
    "h3{margin:0 0 8px;color:#5af;font-size:11px;text-transform:uppercase;letter-spacing:1px;"
    "border-bottom:1px solid #2a2a2a;padding-bottom:4px}"
    "#cam{display:block;width:240px;height:240px}"
    ".r{display:flex;align-items:center;gap:6px;margin:3px 0}"
    "label{width:90px;text-align:right;color:#888;font-size:11px;flex-shrink:0}"
    "input[type=range]{flex:1;accent-color:#5af;min-width:120px}"
    ".sv{width:26px;text-align:center;color:#5af;font-size:11px}"
    "input[type=text]{background:#111;border:1px solid #444;color:#ccc;"
    "padding:4px 6px;border-radius:4px;width:120px;font-family:monospace}"
    "button{padding:5px 10px;border:none;border-radius:4px;"
    "font-family:monospace;font-size:11px;cursor:pointer;color:#fff;margin:2px}"
    ".bf{background:#1a6632}.bg{background:#1553a0}.bs{background:#8b1c1c}"
    "#det{font-size:11px;line-height:1.8;color:#8f8;min-height:80px}"
    "#sts{font-size:11px;color:#fa5;margin-top:6px}"
    "</style></head><body>"
    "<div class='w'>"
    "<div class='p'><h3>Camera Feed</h3><img id='cam' src='/stream'></div>"
    "<div class='p'>"
    "<h3>Detection</h3><div id='det'>loading...</div>"
    "<h3 style='margin-top:10px'>Training</h3>"
    "<div class='r'><label>Label</label>"
    "<input type='text' id='lbl' value='person'></div>"
    "<div class='r' style='margin-top:4px'>"
    "<button class='bf' onclick='trn(\"face_start\")'>Train Face</button>"
    "<button class='bg' onclick='trn(\"gesture_start\")'>Train Gesture</button>"
    "<button class='bs' onclick='trn(\"stop\")'>Stop</button></div>"
    "<div id='sts'>idle</div></div>"
    "<div class='p'><h3>Camera Settings</h3><div id='sl'></div>"
    "<div style='margin-top:8px;border-top:1px solid #2a2a2a;padding-top:6px'>"
    "<h3 style='margin-top:4px'>Servo</h3>"
    "<div class='r'>"
    "<button class='bg' onclick='sf(\"pan\")' id='bp'>&#8596; Flip Pan</button>"
    "<button class='bg' onclick='sf(\"tilt\")' id='bt'>&#8597; Flip Tilt</button>"
    "<button style='background:#555;padding:5px 8px;border:none;border-radius:4px;font-family:monospace;font-size:11px;cursor:pointer;color:#fff' onclick='sc()'>&#8982; Centre</button></div>"
    "<div id='sinv' style='font-size:10px;color:#888;margin-top:3px'></div>"
    "</div></div>"
    "<div class='p'><h3>Gemma Vision</h3>"
    "<div class='r' style='justify-content:center;margin-bottom:6px'>"
    "<button class='bg' onclick='askGemma()'>&#128065; Describe Scene</button>"
    "<button class='bs' onclick='recMic()' id='bmic'>&#127908; Record</button></div>"
    "<div id='gsts' style='color:#888;font-size:10px'>idle &ndash; run tools/gemma_proxy.py on host PC</div>"
    "<div id='gdesc' style='font-size:11px;line-height:1.6;color:#ccc;margin-top:4px;min-height:50px'></div></div>"
    "</div>"
    "<script>"
    "var S=[['brightness',-2,2,2],['contrast',-2,2,0],"
    "['saturation',-2,2,1],['sharpness',-2,2,1],"
    "['ae_level',-2,2,2],['gainceiling',0,6,4]];"
    "function mk(){"
    "var d=document.getElementById('sl');"
    "S.forEach(function(s){"
    "var row=document.createElement('div');row.className='r';"
    "var lbl=document.createElement('label');lbl.textContent=s[0];"
    "var inp=document.createElement('input');"
    "inp.type='range';inp.min=s[1];inp.max=s[2];inp.value=s[3];"
    "var sp=document.createElement('span');sp.className='sv';sp.textContent=s[3];"
    "inp.oninput=function(){"
    "sp.textContent=inp.value;"
    "fetch('/cam?'+s[0]+'='+inp.value).catch(function(){});};"
    "row.appendChild(lbl);row.appendChild(inp);row.appendChild(sp);"
    "d.appendChild(row);});}"
    "function rd(){fetch('/detect').then(function(r){return r.json();}).then(function(d){"
    "document.getElementById('det').innerHTML="
    "'kind: <b>'+d.kind+'</b><br>'"
    "+'gesture: '+d.gesture+'<br>'"
    "+'side: '+d.side+'<br>'"
    "+'score: '+d.score.toFixed(3)+'<br>'"
    "+'bbox: ['+d.bbox+']';"
    "}).catch(function(){});}"
    "function rs(){fetch('/train?op=status').then(function(r){return r.json();}).then(function(s){"
    "var t=s.mode==='idle'?'idle'"
    ":s.mode+' \\u25b6 '+s.label+' ('+s.count+' samples)';"
    "document.getElementById('sts').textContent=t;"
    "}).catch(function(){});}"
    "function trn(op){"
    "var l=document.getElementById('lbl').value||'unknown';"
    "fetch('/train?op='+op+'&label='+encodeURIComponent(l))"
    ".then(function(r){return r.json();})"
    ".then(function(s){document.getElementById('sts').textContent=s.msg||'ok';})"
    ".catch(function(){});}"
    "var gPend=false;"
    "function askGemma(){fetch('/gemma?trigger=1').then(function(r){return r.json();}).then(function(){gPend=true;document.getElementById('gsts').textContent='waiting for proxy...';}).catch(function(){});}"
    "function recMic(){var b=document.getElementById('bmic');b.disabled=true;b.textContent='\\u23fa Recording...';fetch('/mic').then(function(){setTimeout(function(){b.disabled=false;b.textContent='&#127908; Record';},2500);}).catch(function(){b.disabled=false;b.textContent='&#127908; Record';});}"
    "function pollGemma(){if(!gPend)return;"
    "fetch('/gemma').then(function(r){return r.json();}).then(function(j){"
    "if(j.result&&j.result.length>0&&!j.pending){document.getElementById('gdesc').textContent=j.result;"
    "document.getElementById('gsts').textContent='done';gPend=false;}"
    "}).catch(function(){});}"
    "function sf(axis){"
    "fetch('/servo?flip_'+axis+'=1').then(function(r){return r.json();}).then(function(j){"
    "document.getElementById('sinv').textContent='pan_inv='+j.pan_inv+' tilt_inv='+j.tilt_inv;"
    "}).catch(function(){});}"
    "function sc(){fetch('/servo?center=1').catch(function(){});}"
    "mk();setInterval(rd,500);setInterval(rs,2000);setInterval(pollGemma,1000);"
    "</script></body></html>";

static esp_err_t mic_handler(httpd_req_t *req)
{
    set_cors(req);
    mic_capture_async(MIC_DURATION_MS);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"recording\":true}");
    return ESP_OK;
}

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, ROOT_HTML);
    return ESP_OK;
}

/* ── start ───────────────────────────────────────────────────── */
esp_err_t web_server_start(void)
{
    s_jpg_lock = xSemaphoreCreateMutex();
    if (!s_jpg_lock) { ESP_LOGE(TAG, "JPEG lock alloc failed"); return ESP_FAIL; }

    httpd_config_t cfg    = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = HTTP_PORT;
    cfg.stack_size        = 12288;
    cfg.max_uri_handlers  = 12;
    cfg.max_open_sockets  = 7;   /* 1 async stream + detect/train/cam/root + spares */
    cfg.lru_purge_enable  = true;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return ESP_FAIL;
    }

    const httpd_uri_t uris[] = {
        { .uri = "/",       .method = HTTP_GET, .handler = root_handler   },
        { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler },
        { .uri = "/frame",  .method = HTTP_GET, .handler = stream_handler  },
        { .uri = "/detect", .method = HTTP_GET, .handler = detect_handler },
        { .uri = "/train",  .method = HTTP_GET, .handler = train_handler  },
        { .uri = "/servo",        .method = HTTP_GET,  .handler = servo_handler        },
        { .uri = "/snap",         .method = HTTP_GET,  .handler = snap_handler         },
        { .uri = "/cam",          .method = HTTP_GET,  .handler = cam_handler          },
        { .uri = "/gemma",        .method = HTTP_GET,  .handler = gemma_handler        },
        { .uri = "/gemma_result", .method = HTTP_POST, .handler = gemma_result_handler },
        { .uri = "/mic",          .method = HTTP_GET,  .handler = mic_handler          },
    };
    for (int i = 0; i < 11; i++)
        httpd_register_uri_handler(server, &uris[i]);

    ESP_LOGW(TAG, "HTTP server on :%d  –  open http://<IP>/ in a browser", HTTP_PORT);
    return ESP_OK;
}
