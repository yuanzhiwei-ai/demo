#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

/* Log Levels */
#define LOG_LEVEL_DEBUG    0
#define LOG_LEVEL_WARNING  1
#define LOG_LEVEL_INFO     2
#define LOG_LEVEL_ERROR    3
#define LOG_LEVEL_NOTICE   4
#define LOG_LEVEL_FATAL    5

/* Module IDs */
typedef enum {
    MOD_ID_INIT = 0,
    MOD_ID_MAIN,
    MOD_ID_UART,
    MOD_ID_RS485,
    MOD_ID_CAN,
    MOD_ID_SYSTEM,
    MOD_ID_APP,
    MOD_ID_CNT
} module_id_t;

/* Trace Macros */
#define TRACE_DEBUG(module, ...)   TRACE_INTERNAL(LOG_LEVEL_DEBUG, module, __VA_ARGS__)
#define TRACE_WARNING(module, ...) TRACE_INTERNAL(LOG_LEVEL_WARNING, module, __VA_ARGS__)
#define TRACE_INFO(module, ...)    TRACE_INTERNAL(LOG_LEVEL_INFO, module, __VA_ARGS__)
#define TRACE_ERROR(module, ...)   TRACE_INTERNAL(LOG_LEVEL_ERROR, module, __VA_ARGS__)
#define TRACE_NOTICE(module, ...)  TRACE_INTERNAL(LOG_LEVEL_NOTICE, module, __VA_ARGS__)
#define TRACE_FATAL(module, ...)   TRACE_INTERNAL(LOG_LEVEL_FATAL, module, __VA_ARGS__)

/* HEX Trace Macros */
#define TRACE_HEX_DEBUG(module, pdata, length)   TRACE_HEX_INTERNAL(LOG_LEVEL_DEBUG, module, pdata, length)
#define TRACE_HEX_WARNING(module, pdata, length) TRACE_HEX_INTERNAL(LOG_LEVEL_WARNING, module, pdata, length)
#define TRACE_HEX_INFO(module, pdata, length)    TRACE_HEX_INTERNAL(LOG_LEVEL_INFO, module, pdata, length)
#define TRACE_HEX_ERROR(module, pdata, length)   TRACE_HEX_INTERNAL(LOG_LEVEL_ERROR, module, pdata, length)
#define TRACE_HEX_FATAL(module, pdata, length)   TRACE_HEX_INTERNAL(LOG_LEVEL_FATAL, module, pdata, length)

/* Short aliases */
#define TRACE(module, ...) TRACE_DEBUG(module, __VA_ARGS__)
#define TRACE_HEX(module, pdata, length) TRACE_HEX_DEBUG(module, pdata, length)

/* Internal Macros */
#define TRACE_INTERNAL(loglevel, module, ...)                      \
    do {                                                           \
        if (((1 << (module)) & gFilter)) {                       \
            DEBUG_LOG(loglevel, module, __VA_ARGS__);            \
        }                                                         \
    } while (0)

#define TRACE_HEX_INTERNAL(loglevel, module, pdata, length)       \
    do {                                                           \
        if (((1 << (module)) & gFilter)) {                       \
            DEBUG_HEX_LOG(loglevel, module, pdata, length);      \
        }                                                         \
    } while (0)

/* Global Variables */
extern unsigned int gFilter;

/* Function Prototypes */
void DEBUG_LOG(uint8_t loglevel, uint32_t module, const char *format, ...);
void DEBUG_HEX_LOG(uint8_t loglevel, uint32_t module, uint8_t *pdata, uint32_t length);
void debug_log_init(void);
void debug_log_set_filter(unsigned int filter);
void debug_log_enable_module(module_id_t module);
void debug_log_disable_module(module_id_t module);
void debug_log_enable_color(uint8_t enable);

#endif /* __DEBUG_LOG_H__ */