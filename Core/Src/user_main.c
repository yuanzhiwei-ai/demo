#include "user_main.h"
#include "debug_log.h"
#include "rs485_driver.h"

static uint32_t tick = 0;
static uint8_t test_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
static uint8_t received_data[256];
static uint32_t received_data_length = 0;

void user_init(void)
{
    debug_log_init();
    rs485_init();
}

void user_main(void)
{
    if (HAL_GetTick() - tick > 1000)
    {
        tick = HAL_GetTick();
        TRACE_DEBUG(MOD_ID_MAIN, "Tick: %d", tick);
        rs485_send_packet_dma(test_data, sizeof(test_data));
    }

    received_data_length = rs485_receive_packet(received_data, sizeof(received_data));
    if (received_data_length > 0)
    {
        TRACE_DEBUG(MOD_ID_MAIN, "Received data: %d", received_data_length);
        TRACE_HEX_DEBUG(MOD_ID_MAIN, received_data, received_data_length);
        // echo back
        rs485_send_packet_dma(received_data, received_data_length);
    }
}