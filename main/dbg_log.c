#include "dbg_log.h"
#include <stdarg.h>
#include <stdio.h>
#include "esp_timer.h"
#include "esp_system.h"

#define DBG_PATH "/sd/dbg.log"

/* 每次启动追加会话分隔行(不再截断: 保留跨重启的历史证据链) */
void dbg_log_boot(void)
{
    FILE *f = fopen(DBG_PATH, "a");
    if (f == NULL) return;
    fprintf(f, "\n[%llu ms] === SESSION BOOT reset_reason=%d ===\n",
            (unsigned long long)(esp_timer_get_time() / 1000),
            (int)esp_reset_reason());
    fclose(f);
}

/* 追加一行日志: 打开-写入-关闭(慢但崩溃前数据已落盘, 且跨模式切换安全) */
void dbg_log(const char *fmt, ...)
{
    FILE *f = fopen(DBG_PATH, "a");
    if (f == NULL) return;
    va_list ap;
    va_start(ap, fmt);
    fprintf(f, "[%llu ms] ", (unsigned long long)(esp_timer_get_time() / 1000));
    vfprintf(f, fmt, ap);
    fputc('\n', f);
    va_end(ap);
    fclose(f);
}
