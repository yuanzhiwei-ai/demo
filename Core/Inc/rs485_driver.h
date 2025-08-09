#ifndef __RS485_DRIVER_H__
#define __RS485_DRIVER_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "uart_comm.h"

/* RS485 Configuration */
#define RS485_UART_HANDLE           huart5
#define RS485_DMA_RX_BUFFER_SIZE    256
#define RS485_RX_BUFFER_SIZE        512
#define RS485_TX_BUFFER_SIZE        512

/* RS485 Control Pin - PD11 */
#define RS485_CTR1_Pin              GPIO_PIN_11
#define RS485_CTR1_GPIO_Port        GPIOD

/* RS485 Direction */
typedef enum {
    RS485_DIR_RX = 0,   /* Receive mode */
    RS485_DIR_TX = 1    /* Transmit mode */
} rs485_direction_t;

/* RS485 Status */
typedef enum {
    RS485_OK = 0,
    RS485_ERROR,
    RS485_BUSY,
    RS485_TIMEOUT,
    RS485_BUFFER_FULL
} rs485_status_t;

/* RS485 Statistics */
typedef struct {
    uint32_t tx_packets;
    uint32_t rx_packets;
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t rx_errors;
    uint32_t tx_errors;
} rs485_stats_t;

/* RS485 Driver Structure */
typedef struct {
    uart_port_t uart_port;
    uint8_t *dma_rx_buffer;
    uint8_t *rx_buffer;
    uint8_t *tx_buffer;
    volatile rs485_direction_t direction;
    volatile uint8_t tx_pending;
    rs485_stats_t stats;
    void (*packet_received_callback)(uint8_t *data, uint16_t length);
} rs485_driver_t;

/* Global RS485 Instance */
extern rs485_driver_t rs485_driver;
extern UART_HandleTypeDef huart5;

/* Function Prototypes */

/* Initialization */
rs485_status_t rs485_init(void);
rs485_status_t rs485_deinit(void);

/* Direction Control */
void rs485_set_direction(rs485_direction_t direction);
rs485_direction_t rs485_get_direction(void);

/* Data Transmission */
rs485_status_t rs485_send_packet_dma(const uint8_t *data, uint16_t length);

/* Data Reception */
uint16_t rs485_receive_packet(uint8_t *buffer, uint16_t max_length);
uint16_t rs485_data_available(void);
void rs485_clear_rx_buffer(void);

/* Interrupt Handlers */
void rs485_idle_interrupt_handler(void);
void rs485_tx_complete_callback(void);
void rs485_rx_callback(uint8_t *data, uint16_t length);

/* Callbacks */
void rs485_set_packet_callback(void (*callback)(uint8_t *, uint16_t));

/* Statistics */
void rs485_get_stats(rs485_stats_t *stats);
void rs485_reset_stats(void);

/* Debug */
void rs485_print_stats(void);

#endif /* __RS485_DRIVER_H__ */