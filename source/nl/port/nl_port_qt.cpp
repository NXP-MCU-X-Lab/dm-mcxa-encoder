#include <QDateTime>
#include <QDebug>
#include <QString>
#include <QMutex>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

static QMutex g_critical_mutex;

extern "C" {

uint32_t nl_get_sys_ms(void)
{
    return QDateTime::currentMSecsSinceEpoch() & 0xFFFFFFFF;
}

void *nl_malloc(int nbytes)
{
    return malloc(nbytes);
}

void nl_free(void *ptr)
{
    free(ptr);
}

void nl_printf(const char *format, ...)
{
    va_list args;
    char buf[256];

    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    QDebug debug = qDebug().noquote().nospace();
    debug << QString::fromLatin1(buf);
}

void nl_enter_critical(void)
{
    g_critical_mutex.lock();
}

void nl_exit_critical(void)
{
    g_critical_mutex.unlock();
}

} // extern "C"
