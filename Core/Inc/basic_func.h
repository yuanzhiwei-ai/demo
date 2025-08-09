#ifndef __BASIC_FUNC_H__
#define __BASIC_FUNC_H__

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

// System time structure
typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} system_time_t;

// System tick functions
uint32_t Get_SystemTick(void);
void System_Tick_Inc(void);

// Time string functions
int get_time_string(char *buffer, uint16_t *milliseconds);
void get_system_time(system_time_t *time);

// Utility functions
void delay_ms(uint32_t ms);
uint32_t get_elapsed_time(uint32_t start_tick);

#endif /* __BASIC_FUNC_H__ */