#include "debug_log.h"
#include "basic_func.h"
#include <stdio.h>
#include <string.h>

/* Color Enable Flag */
#define COLOR_ENABLE 1

/* Global Filter - Enable all modules by default */
unsigned int gFilter = 
    (1 << MOD_ID_INIT) |
    (1 << MOD_ID_MAIN) |
    (1 << MOD_ID_UART) |
    (1 << MOD_ID_RS485) |
    (1 << MOD_ID_CAN) |
    (1 << MOD_ID_SYSTEM) |
    (1 << MOD_ID_APP);

/* Color Enable Control */
static uint8_t color_enabled = COLOR_ENABLE;

/* ANSI Color Codes */
static const char *StringLevelColor[LOG_LEVEL_FATAL + 1] = {
    "\033[1m",        /* DEBUG - Bold White */
    "\033[1;33m",     /* WARNING - Yellow */
    "\033[1;32m",     /* INFO - Green */
    "\033[1;31m",     /* ERROR - Red */
    "\033[1;34m",     /* NOTICE - Blue */
    "\033[1;41m"      /* FATAL - Red Background */
};

/* Log Level Strings */
static const char *StringLevel[LOG_LEVEL_FATAL + 1] = {
    "[DEBUG]",
    "[WARNING]",
    "[INFO]",
    "[ERROR]",
    "[NOTICE]",
    "[FATAL]"
};

/* Color End Codes */
static const char *StringLevelEnd[LOG_LEVEL_FATAL + 1] = {
    "\033[0m\033[0m",
    "\033[0m\033[33m",
    "\033[0m\033[32m",
    "\033[0m\033[31m",
    "\033[0m\033[34m",
    "\033[0m\033[31m"
};

/* Module Name Strings */
static const char *StringModule[MOD_ID_CNT] = {
    "[INIT]",
    "[MAIN]",
    "[UART]",
    "[RS485]",
    "[CAN]",
    "[SYSTEM]",
    "[APP]"
};

/**
 * @brief Print log level with optional color
 */
static void print_log_level(uint8_t loglevel, uint16_t *len, char *buffer)
{
    (*len) += sprintf(buffer + (*len), "%s", StringLevel[loglevel]);
    if (color_enabled) {
        (*len) += sprintf(buffer + (*len), "%s", StringLevelEnd[loglevel]);
    }
}

/**
 * @brief Print module name
 */
static void print_module(uint32_t module, uint16_t *len, char *buffer)
{
    if (module < MOD_ID_CNT) {
        (*len) += sprintf(buffer + (*len), "%s", StringModule[module]);
    } else {
        (*len) += sprintf(buffer + (*len), "[MOD_%02X]", (unsigned int)module);
    }
}

/**
 * @brief Initialize debug log system
 */
void debug_log_init(void)
{
    /* Initialize with all modules enabled */
    gFilter = (1 << MOD_ID_CNT) - 1;
    color_enabled = COLOR_ENABLE;
}

/**
 * @brief Set log filter mask
 */
void debug_log_set_filter(unsigned int filter)
{
    gFilter = filter;
}

/**
 * @brief Enable specific module logging
 */
void debug_log_enable_module(module_id_t module)
{
    if (module < MOD_ID_CNT) {
        gFilter |= (1 << module);
    }
}

/**
 * @brief Disable specific module logging
 */
void debug_log_disable_module(module_id_t module)
{
    if (module < MOD_ID_CNT) {
        gFilter &= ~(1 << module);
    }
}

/**
 * @brief Enable/disable color output
 */
void debug_log_enable_color(uint8_t enable)
{
    color_enabled = enable;
}

/**
 * @brief Main debug log function
 */
void DEBUG_LOG(uint8_t loglevel, uint32_t module, const char *format, ...)
{
    char buffer[512];
    va_list args;
    uint16_t len = 0;
    uint16_t milliseconds = 0;
    
    /* Format timestamp */
    len += sprintf(buffer + len, "[");
    len += get_time_string(buffer + len, &milliseconds);
    len += sprintf(buffer + len, ":%03u] ", milliseconds);
    
    /* Add color if enabled */
    if (color_enabled && loglevel <= LOG_LEVEL_FATAL) {
        len += sprintf(buffer + len, "%s", StringLevelColor[loglevel]);
    }
    
    /* Add system tick */
    len += sprintf(buffer + len, "T%lu:", Get_SystemTick());
    
    /* Add log level */
    print_log_level(loglevel, &len, buffer);
    
    /* Add module name */
    print_module(module, &len, buffer);
    
    /* Format user message */
    va_start(args, format);
    len += vsnprintf(buffer + len, sizeof(buffer) - len - 10, format, args);
    va_end(args);
    
    /* Reset color if enabled */
    if (color_enabled) {
        len += sprintf(buffer + len, "\033[0m");
    }
    
    /* Add newline */
    len += sprintf(buffer + len, "\n");
    
    /* Output */
    printf("%s", buffer);
}

/**
 * @brief HEX data logging function
 */
void DEBUG_HEX_LOG(uint8_t loglevel, uint32_t module, uint8_t *pdata, uint32_t length)
{
    char buffer[2048];
    uint16_t len = 0;
    uint16_t milliseconds = 0;
    uint32_t i;
    
    /* Format timestamp */
    len += sprintf(buffer + len, "[");
    len += get_time_string(buffer + len, &milliseconds);
    len += sprintf(buffer + len, ":%03u] ", milliseconds);
    
    /* Add color if enabled */
    if (color_enabled && loglevel <= LOG_LEVEL_FATAL) {
        len += sprintf(buffer + len, "%s", StringLevelColor[loglevel]);
    }
    
    /* Add system tick */
    len += sprintf(buffer + len, "T%lu:", Get_SystemTick());
    
    /* Add log level */
    print_log_level(loglevel, &len, buffer);
    
    /* Add module name */
    print_module(module, &len, buffer);
    
    /* Add HEX label */
    len += sprintf(buffer + len, "HEX:");
    
    /* Print HEX data (limit to 600 bytes for safety) */
    if (length <= 600) {
        for (i = 0; i < length; i++) {
            len += sprintf(buffer + len, "%02X", pdata[i]);
            if ((i + 1) % 16 == 0 && i < length - 1) {
                len += sprintf(buffer + len, " ");
            }
        }
    } else {
        for (i = 0; i < 600; i++) {
            len += sprintf(buffer + len, "%02X", pdata[i]);
            if ((i + 1) % 16 == 0) {
                len += sprintf(buffer + len, " ");
            }
        }
        len += sprintf(buffer + len, "\n...data is %lu bytes (truncated to 600)", length);
    }
    
    /* Reset color if enabled */
    if (color_enabled) {
        len += sprintf(buffer + len, "\033[0m");
    }
    
    /* Add newline */
    len += sprintf(buffer + len, "\n");
    
    /* Output */
    printf("%s", buffer);
}