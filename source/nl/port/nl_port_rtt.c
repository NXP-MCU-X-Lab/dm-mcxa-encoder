#include "rtthread.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

uint32_t nl_get_sys_ms(void)
{
    return rt_tick_get_millisecond();
}

int nl_mdelay(int ms)
{
    return rt_thread_mdelay(ms);
}


void *nl_malloc(int nbytes)
{
    return rt_malloc(nbytes);
}

void nl_free(void *ptr)
{
    rt_free(ptr);
}


void nl_printf(const char *format, ...)
{
    va_list args;
    char buf[256];
    
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    
    rt_kprintf("%s", buf);
}

void nl_enter_critical(void)
{
    rt_enter_critical();
}

void nl_exit_critical(void)
{
    rt_exit_critical();
}