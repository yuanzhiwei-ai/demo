#include "uart_comm.h"
#include "debug_log.h"
#include <string.h>

/**
 * @brief Initialize ring buffer
 */
uart_status_t ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t size)
{
    if (rb == NULL || buffer == NULL || size == 0) {
        return UART_ERROR;
    }
    
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    
    return UART_OK;
}

/**
 * @brief Put single byte into ring buffer
 */
uart_status_t ring_buffer_put(ring_buffer_t *rb, uint8_t data)
{
    if (rb == NULL) {
        return UART_ERROR;
    }
    
    if (rb->count >= rb->size) {
        return UART_BUFFER_FULL;
    }
    
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
    
    return UART_OK;
}

/**
 * @brief Get single byte from ring buffer
 */
uart_status_t ring_buffer_get(ring_buffer_t *rb, uint8_t *data)
{
    if (rb == NULL || data == NULL) {
        return UART_ERROR;
    }
    
    if (rb->count == 0) {
        return UART_BUFFER_EMPTY;
    }
    
    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    
    return UART_OK;
}

/**
 * @brief Write multiple bytes to ring buffer
 */
uart_status_t ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, uint16_t length)
{
    uint16_t i;
    
    if (rb == NULL || data == NULL) {
        return UART_ERROR;
    }
    
    if (ring_buffer_free(rb) < length) {
        return UART_BUFFER_FULL;
    }
    
    for (i = 0; i < length; i++) {
        ring_buffer_put(rb, data[i]);
    }
    
    return UART_OK;
}

/**
 * @brief Read multiple bytes from ring buffer
 */
uint16_t ring_buffer_read(ring_buffer_t *rb, uint8_t *data, uint16_t max_length)
{
    uint16_t count = 0;
    uint8_t byte;
    
    if (rb == NULL || data == NULL) {
        return 0;
    }
    
    while (count < max_length && ring_buffer_get(rb, &byte) == UART_OK) {
        data[count++] = byte;
    }
    
    return count;
}

/**
 * @brief Get number of bytes available in ring buffer
 */
uint16_t ring_buffer_available(ring_buffer_t *rb)
{
    if (rb == NULL) {
        return 0;
    }
    return rb->count;
}

/**
 * @brief Get free space in ring buffer
 */
uint16_t ring_buffer_free(ring_buffer_t *rb)
{
    if (rb == NULL) {
        return 0;
    }
    return rb->size - rb->count;
}

/**
 * @brief Clear ring buffer
 */
void ring_buffer_clear(ring_buffer_t *rb)
{
    if (rb == NULL) {
        return;
    }
    
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief Initialize UART communication port
 */
uart_status_t uart_comm_init(uart_port_t *port, UART_HandleTypeDef *huart,
                             uint8_t *rx_buf, uint16_t rx_size,
                             uint8_t *tx_buf, uint16_t tx_size)
{
    if (port == NULL || huart == NULL) {
        return UART_ERROR;
    }
    
    port->huart = huart;
    port->rx_complete = 0;
    port->tx_busy = 0;
    port->rx_callback = NULL;
    port->tx_complete_callback = NULL;
    
    /* Initialize ring buffers */
    ring_buffer_init(&port->rx_buffer, rx_buf, rx_size);
    ring_buffer_init(&port->tx_buffer, tx_buf, tx_size);
    
    TRACE_INFO(MOD_ID_UART, "UART comm initialized");
    
    return UART_OK;
}

/**
 * @brief Start UART DMA reception
 */
uart_status_t uart_comm_start_rx_dma(uart_port_t *port)
{
    if (port == NULL || port->huart == NULL || port->dma_rx_buffer == NULL) {
        return UART_ERROR;
    }
    
    /* Enable IDLE interrupt */
    __HAL_UART_ENABLE_IT(port->huart, UART_IT_IDLE);
    
    /* Start DMA reception */
    if (HAL_UART_Receive_DMA(port->huart, port->dma_rx_buffer, port->dma_rx_size) != HAL_OK) {
        return UART_ERROR;
    }
    
    TRACE_DEBUG(MOD_ID_UART, "DMA RX started");
    
    return UART_OK;
}

/**
 * @brief Stop UART DMA reception safely
 */
uart_status_t uart_comm_stop_rx_dma(uart_port_t *port)
{
    if (port == NULL || port->huart == NULL) {
        return UART_ERROR;
    }
    
    uart_comm_stop_dma_safe(port->huart);
    
    return UART_OK;
}

/**
 * @brief Stop DMA safely with interrupt protection
 */
uint32_t uart_comm_stop_dma_safe(UART_HandleTypeDef *huart)
{
    uint32_t remaining = 0;
    
    /* Disable interrupts */
    __disable_irq();
    
    /* Check if DMA is enabled */
    if (HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR)) {
        /* Disable DMA request */
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
        
        /* Abort DMA transfer */
        if (huart->hdmarx != NULL) {
            remaining = huart->hdmarx->Instance->NDTR;
            HAL_DMA_Abort(huart->hdmarx);
        }
        
        /* Clear UART interrupts */
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
        
        /* Reset UART state */
        huart->RxState = HAL_UART_STATE_READY;
    }
    
    /* Enable interrupts */
    __enable_irq();
    
    return remaining;
}

/**
 * @brief Send data via UART (blocking)
 */
uart_status_t uart_comm_send(uart_port_t *port, const uint8_t *data, uint16_t length)
{
    if (port == NULL || data == NULL || length == 0) {
        return UART_ERROR;
    }
    
    if (HAL_UART_Transmit(port->huart, (uint8_t *)data, length, 1000) != HAL_OK) {
        return UART_ERROR;
    }
    
    return UART_OK;
}

/**
 * @brief Send data via UART DMA (non-blocking)
 */
uart_status_t uart_comm_send_dma(uart_port_t *port, const uint8_t *data, uint16_t length)
{
    if (port == NULL || data == NULL || length == 0) {
        return UART_ERROR;
    }
    
    if (port->tx_busy) {
        return UART_BUSY;
    }
    
    port->tx_busy = 1;
    
    if (HAL_UART_Transmit_DMA(port->huart, (uint8_t *)data, length) != HAL_OK) {
        port->tx_busy = 0;
        return UART_ERROR;
    }
    
    return UART_OK;
}

/**
 * @brief Receive data from UART buffer
 */
uint16_t uart_comm_receive(uart_port_t *port, uint8_t *buffer, uint16_t max_length)
{
    if (port == NULL || buffer == NULL) {
        return 0;
    }
    
    return ring_buffer_read(&port->rx_buffer, buffer, max_length);
}

/**
 * @brief Get number of bytes available in receive buffer
 */
uint16_t uart_comm_data_available(uart_port_t *port)
{
    if (port == NULL) {
        return 0;
    }
    
    return ring_buffer_available(&port->rx_buffer);
}

/**
 * @brief Handle UART IDLE interrupt
 */
void uart_comm_idle_handler(uart_port_t *port)
{
    uint32_t remaining;
    uint16_t received_size;
    
    if (port == NULL || port->huart == NULL) {
        return;
    }
    
    /* Clear IDLE flag */
    __HAL_UART_CLEAR_IDLEFLAG(port->huart);
    
    /* Stop DMA to get received data count */
    remaining = uart_comm_stop_dma_safe(port->huart);
    
    /* Calculate received size */
    received_size = port->dma_rx_size - remaining;
    
    if (received_size > 0) {
        /* Copy data to ring buffer */
        ring_buffer_write(&port->rx_buffer, port->dma_rx_buffer, received_size);
        
        /* Call user callback if set */
        if (port->rx_callback != NULL) {
            port->rx_callback(port->dma_rx_buffer, received_size);
        }
        
        TRACE_DEBUG(MOD_ID_UART, "IDLE: Received %d bytes", received_size);
    }
    
    /* Restart DMA reception */
    uart_comm_start_rx_dma(port);
}

/**
 * @brief Handle DMA TX complete interrupt
 */
void uart_comm_dma_tx_complete(uart_port_t *port)
{
    if (port == NULL) {
        return;
    }
    
    port->tx_busy = 0;
    
    /* Call user callback if set */
    if (port->tx_complete_callback != NULL) {
        port->tx_complete_callback();
    }
    
    TRACE_DEBUG(MOD_ID_UART, "DMA TX Complete");
}

/**
 * @brief Set TX complete callback function
 */
void uart_comm_set_tx_callback(uart_port_t *port, void (*callback)(void))
{
    if (port != NULL) {
        port->tx_complete_callback = callback;
    }
}