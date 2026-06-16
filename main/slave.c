/**
 ****************************************************************************************************
 * @file        slave.c
 * @author      ONE
 * @version     V1.5
 * @date        2026-06-16
 * @brief       从机模块 — WiFi STA + HTTP上传 (无USB, 回调安全)
 *
 *   ★ 安全规则: WiFi事件回调中只用xEventGroupSetBits, 不做printf ★
 *   所有日志和重连逻辑在slave_loop_tick主循环中完成
 ****************************************************************************************************
 */

#include "slave.h"
#include "led.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_http_server.h"
#include "esp_http_client.h"

#define SLAVE_DEFAULT_SSID  "BOSSCOM_USB_AP"
#define SLAVE_DEFAULT_PASS  "012345678"

/* ---- 全局状态 ---- */
static bool        g_wifi_connected  = false;
static int         g_files_received  = 0;
static httpd_handle_t g_srv_handle   = NULL;
static char        g_host_ip[16]     = "192.168.3.1";
static char        g_my_ip[16];
static EventGroupHandle_t g_evt = NULL;

/* 事件位: BIT0=STA_START, BIT1=DISCONNECTED, BIT2=GOT_IP */
#define EVT_STA_START      BIT0
#define EVT_DISCONNECTED   BIT1
#define EVT_GOT_IP         BIT2

/* ======================== WiFi 事件回调 (极简, 不调用printf!) ======================== */

static void slave_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                     int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xEventGroupSetBits(g_evt, EVT_STA_START);
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupSetBits(g_evt, EVT_DISCONNECTED);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        snprintf(g_my_ip, sizeof(g_my_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(g_evt, EVT_GOT_IP);
    }
}

/* ======================== WiFi STA 连接 ======================== */

static esp_err_t slave_wifi_connect(void)
{
    g_evt = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &slave_wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &slave_wifi_event_handler, NULL));

    /* 配置 STA */
    wifi_config_t wc = {0};
    memcpy(wc.sta.ssid, SLAVE_DEFAULT_SSID, strlen(SLAVE_DEFAULT_SSID));
    memcpy(wc.sta.password, SLAVE_DEFAULT_PASS, strlen(SLAVE_DEFAULT_PASS));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    wc.sta.pmf_cfg.capable    = true;
    wc.sta.pmf_cfg.required   = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("[SLAVE] WiFi started, SSID='%s'\n", SLAVE_DEFAULT_SSID);
    printf("[SLAVE] Waiting for connection (20s timeout)...\n");

    /* 等待 STA_START → 然后 connect */
    xEventGroupWaitBits(g_evt, EVT_STA_START, pdFALSE, pdTRUE, pdMS_TO_TICKS(5000));
    esp_wifi_connect();
    printf("[SLAVE] Connecting to AP...\n");

    /* 等 GOT_IP 或 超时 */
    EventBits_t bits = xEventGroupWaitBits(g_evt, EVT_GOT_IP, pdFALSE, pdFALSE, pdMS_TO_TICKS(20000));

    if (bits & EVT_GOT_IP) {
        g_wifi_connected = true;
        printf("[SLAVE] *** CONNECTED! IP=%s ***\n", g_my_ip);
        return ESP_OK;
    }

    printf("[SLAVE] WiFi timeout — AP not found\n");
    return ESP_FAIL;
}

/* ======================== HTTP 文件接收 ======================== */

static esp_err_t slave_root_handler(httpd_req_t *req)
{
    char html[512];
    snprintf(html, sizeof(html),
        "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>Slave</title></head>"
        "<body style=\"font-family:sans-serif;padding:20px\">"
        "<h2>ESP32-S3 SLAVE</h2>"
        "<p>WiFi: <strong>%s</strong></p><p>IP: %s</p><p>Files: <strong>%d</strong></p>"
        "</body></html>",
        g_wifi_connected?"OK":"NO", g_my_ip, g_files_received);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t slave_upload_handler(httpd_req_t *req)
{
    char query[256]={0}, raw_name[256]={0};
    size_t qlen = httpd_req_get_url_query_len(req)+1;
    if (qlen > sizeof(query)) qlen = sizeof(query);
    if (httpd_req_get_url_query_str(req, query, qlen) != ESP_OK) {
        httpd_resp_send_500(req); return ESP_FAIL;
    }
    if (httpd_query_key_value(query, "name", raw_name, sizeof(raw_name)) != ESP_OK) {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"missing name\"}");
        httpd_resp_set_type(req, "application/json");
        return ESP_FAIL;
    }
    if (strchr(raw_name, '/') || strchr(raw_name, '\\') || strstr(raw_name, "..") || raw_name[0]=='\0') {
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"bad name\"}");
        httpd_resp_set_type(req, "application/json");
        return ESP_FAIL;
    }

    char save_path[512];
    snprintf(save_path, sizeof(save_path), "/SD/%s", raw_name);
    FILE *fp = fopen(save_path, "wb");
    if (!fp) {
        printf("[SLAVE] fopen FAIL: %s\n", save_path);
        httpd_resp_send_500(req); return ESP_FAIL;
    }
    char buf[2048]; int recv, total=0;
    while ((recv = httpd_req_recv(req, buf, sizeof(buf))) > 0) {
        fwrite(buf, 1, recv, fp); total += recv;
    }
    fclose(fp);
    g_files_received++;
    printf("[SLAVE] *** RECEIVED: %s (%d bytes, total=%d) ***\n", raw_name, total, g_files_received);

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"bytes\":%d}", total);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t slave_http_server_start(void)
{
    httpd_config_t c = HTTPD_DEFAULT_CONFIG();
    c.max_uri_handlers = 6;
    c.stack_size       = 8192;
    c.server_port      = 80;
    esp_err_t ret = httpd_start(&g_srv_handle, &c);
    if (ret != ESP_OK) { printf("[SLAVE] HTTP start FAIL\n"); return ret; }
    httpd_uri_t r1 = {.uri="/", .method=HTTP_GET, .handler=slave_root_handler};
    httpd_register_uri_handler(g_srv_handle, &r1);
    httpd_uri_t r2 = {.uri="/api/upload", .method=HTTP_POST, .handler=slave_upload_handler};
    httpd_register_uri_handler(g_srv_handle, &r2);
    printf("[SLAVE] HTTP on port 80\n");
    return ESP_OK;
}

/* ======================== 主机注册 ======================== */

static esp_err_t slave_register_with_host(void)
{
    char url[80];
    snprintf(url, sizeof(url), "http://%s/api/register", g_host_ip);
    esp_http_client_config_t cfg = {.url=url, .method=HTTP_METHOD_POST, .timeout_ms=5000};
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    if (!cli) return ESP_FAIL;
    esp_http_client_set_header(cli, "Content-Type", "application/json");
    esp_http_client_set_post_field(cli, "{\"role\":\"slave\"}", 15);
    esp_err_t ret = esp_http_client_perform(cli);
    int st = esp_http_client_get_status_code(cli);
    esp_http_client_cleanup(cli);
    if (ret == ESP_OK && st == 200) {
        printf("[SLAVE] *** REGISTERED ***\n");
        led_set_pattern(LED_PAT_SLOW_BLINK);
        return ESP_OK;
    }
    printf("[SLAVE] Register FAIL: ret=%d HTTP=%d\n", ret, st);
    return ESP_FAIL;
}

/* ======================== 公开API ======================== */

esp_err_t slave_init(void)
{
    nvs_flash_init();

    esp_err_t ret = slave_wifi_connect();
    if (ret != ESP_OK)
        printf("[SLAVE] WiFi FAIL — AP not found. HTTP server will still run.\n");

    ret = slave_http_server_start();
    if (ret != ESP_OK) return ESP_FAIL;

    if (g_wifi_connected) {
        vTaskDelay(pdMS_TO_TICKS(1500));
        slave_register_with_host();
    }

    printf("[SLAVE] Init done. WiFi=%s\n", g_wifi_connected?"OK":"NO");
    return ESP_OK;
}

void slave_loop_tick(void)
{
    static uint32_t next_check = 5000, last_reg = 0, last_reconn = 0;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    /* ★ 处理WiFi事件 (从g_evt读取, 然后清除) ★ */
    EventBits_t ev = xEventGroupClearBits(g_evt, EVT_DISCONNECTED | EVT_GOT_IP);
    if (ev & EVT_DISCONNECTED) {
        g_wifi_connected = false;
        printf("[SLAVE] WiFi disconnected\n");
        led_set_pattern(LED_PAT_VERY_FAST_BLINK);
    }
    if (ev & EVT_GOT_IP) {
        g_wifi_connected = true;
        printf("[SLAVE] *** GOT IP: %s ***\n", g_my_ip);
    }

    /* 重连逻辑: 每3秒尝试 */
    if (!g_wifi_connected && (now - last_reconn > 3000)) {
        last_reconn = now;
        printf("[SLAVE] Reconnect attempt...\n");
        esp_wifi_connect();
    }

    if (now < next_check) return;
    next_check = now + 5000;

    wifi_ap_record_t ap;
    bool connected = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);

    if (connected && !g_wifi_connected) {
        g_wifi_connected = true;
        printf("[SLAVE] *** RECONNECTED! ***\n");
        slave_register_with_host();
        led_set_pattern(LED_PAT_SLOW_BLINK);
    } else if (!connected && g_wifi_connected) {
        g_wifi_connected = false;
        printf("[SLAVE] Connection lost\n");
        led_set_pattern(LED_PAT_VERY_FAST_BLINK);
    } else if (connected && (now - last_reg > 60000)) {
        last_reg = now;
        slave_register_with_host();
    }
}

bool slave_is_connected(void)       { return g_wifi_connected; }
int  slave_get_files_received(void) { return g_files_received; }
