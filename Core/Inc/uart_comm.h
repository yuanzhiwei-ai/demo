#ifndef __UART_COMM_H__
#define __UART_COMM_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Buffer Sizes */
#define UART_RX_BUFFER_SIZE     256
#define UART_TX_BUFFER_SIZE     256
#define UART_DMA_BUFFER_SIZE    256

/* UART Status */
typedef enum {
    UART_OK = 0,
    UART_ERROR,
    UART_BUSY,
    UART_TIMEOUT,
    UART_BUFFER_FULL,
    UART_BUFFER_EMPTY
} uart_status_t;

/* Ring Buffer Structure */
typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
} ring_buffer_t;

/* UART Port Structure */
typedef struct {
    UART_HandleTypeDef *huart;
    ring_buffer_t rx_buffer;
    ring_buffer_t tx_buffer;
    uint8_t *dma_rx_buffer;
    uint16_t dma_rx_size;
    volatile uint8_t rx_complete;
    volatile uint8_t tx_busy;
    void (*rx_callback)(uint8_t *data, uint16_t length);
    void (*tx_complete_callback)(void);
} uart_port_t;

/* Function Prototypes */

/* Initialization */
uart_status_t uart_comm_init(uart_port_t *port, UART_HandleTypeDef *huart, 
                             uint8_t *rx_buf, uint16_t rx_size,
                             uint8_t *tx_buf, uint16_t tx_size);
uart_status_t uart_comm_start_rx_dma(uart_port_t *port);
uart_status_t uart_comm_stop_rx_dma(uart_port_t *port);

/* Ring Buffer Operations */
uart_status_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size);
uart_status_t ring_buffer_put(ring_buffer_t *rb, uint8_t data);
uart_status_t ring_buffer_get(ring_buffer_t *rb, uint8_t *data);
uart_status_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, uint16_t length);
uint16_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data, uint16_t max_length);
uint16_t ring_buffer_available(ring_buffer_t *rb);
uint16_t ring_buffer_free(ring_buffer_t *rb);
void ring_buffer_clear(ring_buffer_t *rb);

/* Data Transmission */
uart_status_t uart_comm_send(uart_port_t *port, const uint8_t *data, uint16_t length);
uart_status_t uart_comm_send_dma(uart_port_t *port, const uint8_t *data, uint16_t length);

/* Data Reception */
uint16_t uart_comm_receive(uart_port_t *port, uint8_t *buffer, uint16_t max_length);
uint16_t uart_comm_data_available(uart_port_t *port);

/* Interrupt Handlers */
void uart_comm_idle_handler(uart_port_t *port);
void uart_comm_dma_tx_complete(uart_port_t *port);

/* Callbacks */
void uart_comm_set_tx_callback(uart_port_t *port, void (*callback)(void));

/* Utility */
uint32_t uart_comm_stop_dma_safe(UART_HandleTypeDef *huart);

#endif /* __UART_COMM_H__ */