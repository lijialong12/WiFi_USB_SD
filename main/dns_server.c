#include "dns_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <ctype.h>

static const char *TAG = "DNS_SRV";
#define DNS_PORT 53
#define DNS_BUF_SIZE 512

static void dns_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) { ESP_LOGE(TAG, "socket create failed"); vTaskDelete(NULL); return; }

    struct sockaddr_in server_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(DNS_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "bind port 53 failed"); close(sock); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "DNS server started on port %d", DNS_PORT);

    uint8_t buf[DNS_BUF_SIZE];
    while (1) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int len = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addr_len);
        if (len < 12) continue;

        /* ---- 解析问题区QNAME(偏移12起): 判断是否为Windows辅助检测域名 ---- */
        char qname[128];
        int qn = 0, p = 12;
        while (p < len) {
            uint8_t seg = buf[p++];
            if (seg == 0) break;                              /* 域名结束 */
            if (qn + seg + 2 >= (int)sizeof(qname)) { qn = -1; break; }
            memcpy(qname + qn, &buf[p], seg);
            qn += seg;
            qname[qn++] = '.';
            p += seg;
        }
        if (qn > 0 && qname[qn - 1] == '.') qn--;
        if (qn < 0) qn = 0;
        qname[qn] = '\0';
        for (int i = 0; i < qn; i++) qname[i] = (char)tolower((unsigned char)qname[i]);

        /* dns.msftncsi.com必须返回131.107.255.255, 否则Windows判定"强制门户"→Edge拦截POST */
        uint8_t rip[4] = {192, 168, 4, 1};
        if (strcmp(qname, "dns.msftncsi.com") == 0) {
            rip[0] = 131; rip[1] = 107; rip[2] = 255; rip[3] = 255;
        }

        int out_len = len;
        if (out_len > DNS_BUF_SIZE - 16) out_len = DNS_BUF_SIZE - 16;

        /* 复用查询缓冲: 改写标志位为响应+单条应答 */
        buf[2] = 0x81;
        buf[3] = 0x80;
        buf[4] = 0;
        buf[5] = 1;
        buf[6] = 0;
        buf[7] = 1;
        buf[8] = 0;
        buf[9] = 0;

        int ans_start = out_len;
        uint8_t *ans = buf + ans_start;
        ans[0] = 0xC0;
        ans[1] = 0x0C;
        ans[2] = 0;
        ans[3] = 1;
        ans[4] = 0;
        ans[5] = 1;
        ans[6] = 0;
        ans[7] = 0;
        ans[8] = 0;
        ans[9] = 0x3C;
        ans[10] = 0;
        ans[11] = 4;
        ans[12] = rip[0];
        ans[13] = rip[1];
        ans[14] = rip[2];
        ans[15] = rip[3];

        sendto(sock, buf, ans_start + 16, 0, (struct sockaddr *)&client_addr, addr_len);
    }
}

esp_err_t dns_server_start(void)
{
    return xTaskCreate(dns_task, "dns_srv", 4096, NULL, 5, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}
