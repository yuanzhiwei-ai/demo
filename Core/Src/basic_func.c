#include "basic_func.h"

static volatile uint32_t system_tick_counter = 0;
static volatile uint32_t system_ms_counter = 0;

/**
 * @brief Get current system tick count
 * @return Current tick count
 */
uint32_t Get_SystemTick(void)
{
    return system_tick_counter;
}

/**
 * @brief Increment system tick (call from SysTick_Handler)
 */
void System_Tick_Inc(void)
{
    system_tick_counter++;
    system_ms_counter++;
}

/**
 * @brief Get formatted time string
 * @param buffer Output buffer for time string
 * @param milliseconds Output milliseconds
 * @return Number of characters written
 */
int get_time_string(char *buffer, uint16_t *milliseconds)
{
    uint32_t total_ms = system_ms_counter;
    uint32_t hours = (total_ms / 3600000) % 24;
    uint32_t minutes = (total_ms / 60000) % 60;
    uint32_t seconds = (total_ms / 1000) % 60;
    uint32_t ms = total_ms % 1000;
    
    if (milliseconds) {
        *milliseconds = ms;
    }
    
    return sprintf(buffer, "%02lu:%02lu:%02lu", hours, minutes, seconds);
}

/**
 * @brief Get system time structure
 * @param time Pointer to time structure
 */
void get_system_time(system_time_t *time)
{
    if (time == NULL) return;
    
    uint32_t total_ms = system_ms_counter;
    time->hour = (total_ms / 3600000) % 24;
    time->minute = (total_ms / 60000) % 60;
    time->second = (total_ms / 1000) % 60;
    time->millisecond = total_ms % 1000;
}

/**
 * @brief Delay in milliseconds
 * @param ms Milliseconds to delay
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = system_tick_counter;
    while ((system_tick_counter - start) < ms) {
        __NOP();
    }
}

/**
 * @brief Get elapsed time since start tick
 * @param start_tick Start tick value
 * @return Elapsed time in milliseconds
 */
uint32_t get_elapsed_time(uint32_t start_tick)
{
    return (system_tick_counter - start_tick);
}