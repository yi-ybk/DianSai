#include "bsp_log.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define BSP_LOG_FORMAT_BUFFER_SIZE 256U

void BSPLogInit()
{
    SEGGER_RTT_Init();
}

static int PrintLogV(const char *fmt, va_list args)
{
    char log_buffer[BSP_LOG_FORMAT_BUFFER_SIZE];
    int n;

    n = vsnprintf(log_buffer, sizeof(log_buffer), fmt, args);
    if (n > 0)
    {
        log_buffer[sizeof(log_buffer) - 1U] = '\0';
        SEGGER_RTT_WriteString(BUFFER_INDEX, log_buffer);
    }

    return n;
}

int PrintLog(const char *fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    n = PrintLogV(fmt, args);
    va_end(args);

    return n;
}

int printf_log(const char *fmt, ...)
{
    va_list args;
    int n;

    va_start(args, fmt);
    n = PrintLogV(fmt, args);
    va_end(args);

    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);

    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}
