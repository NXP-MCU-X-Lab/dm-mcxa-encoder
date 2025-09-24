#ifndef __NL_DEBUG_H__
#define __NL_DEBUG_H__


#ifdef __cplusplus
extern "C"{
#endif

#include "nl_cfg.h"

/* DEBUG level */
#define NL_DBG_ERROR   0
#define NL_DBG_WARNING 1
#define NL_DBG_INFO    2
#define NL_DBG_LOG     3


/*
 * The color for terminal (foreground)
 * BLACK    30
 * RED      31
 * GREEN    32
 * YELLOW   33
 * BLUE     34
 * PURPLE   35
 * CYAN     36
 * WHITE    37
 */

#ifdef  nl_printf_COLOR_ENABLE
#define _NL_DBG_COLOR(n) nl_printf("\033[" #n "m")
#define _NL_DBG_LOG_HDR(lvl_name, color_n) \
    nl_printf("\033[" #color_n "m[" lvl_name "/" NL_DBG_TAG "] ")
#define _NL_DBG_LOG_X_END \
    nl_printf("\033[0m")
#else
#define _NL_DBG_COLOR(n)
#define _NL_DBG_LOG_HDR(lvl_name, color_n) \
    nl_printf("[" lvl_name "/" NL_DBG_TAG "] ")
#define _NL_DBG_LOG_X_END                                     \
    nl_printf("\n")
#endif

#define nl_dbg_log_line(lvl, color_n, fmt, ...) \
    do {                                         \
        _NL_DBG_LOG_HDR(lvl, color_n);          \
        nl_printf(fmt, ##__VA_ARGS__);              \
        _NL_DBG_LOG_X_END;                      \
    } while (0)

#if (NL_DBG_LVL >= NL_DBG_LOG)
#define NL_LOG_D(fmt, ...) nl_dbg_log_line("D", 0, fmt, ##__VA_ARGS__)
#else
#define NL_LOG_D(...)  {}
#endif

#if (NL_DBG_LVL >= NL_DBG_INFO)
#define NL_LOG_I(fmt, ...) nl_dbg_log_line("I", 32, fmt, ##__VA_ARGS__)
#else
#define NL_LOG_I(...) {}
#endif

#if (NL_DBG_LVL >= NL_DBG_WARNING)
#define NL_LOG_W(fmt, ...) nl_dbg_log_line("W", 33, fmt, ##__VA_ARGS__)
#else
#define NL_LOG_W(...) {}
#endif

#if (NL_DBG_LVL >= NL_DBG_ERROR)
#define NL_LOG_E(fmt, ...) nl_dbg_log_line("E", 31, fmt, ##__VA_ARGS__)
#else
#define NL_LOG_E(...) {}
#endif

#define NL_LOG_RAW(...) nl_printf(__VA_ARGS__)

void nl_assert(const char *filename, int linenum);
#define NL_ASSERT(f)                       \
    do {                                    \
        if (!(f))                           \
            nl_assert(__FILE__, __LINE__); \
    } while (0)



#ifdef __cplusplus
}
#endif

#endif



