#include "rs485_driver.h"
#include "debug_log.h"
#include <string.h>

/* Global RS485 Instance */
rs485_driver_t rs485_driver;

/* Static Buffers */
static uint8_t rs485_dma_rx_buffer[RS485_DMA_RX_BUFFER_SIZE];
static uint8_t rs485_rx_ring_buffer[RS485_RX_BUFFER_SIZE];
static uint8_t rs485_tx_ring_buffer[RS485_TX_BUFFER_SIZE];

/**
 * @brief Initialize RS485 driver
 */
rs485_status_t rs485_init(void)
{
    uart_status_t status;
    
    /* Clear driver structure */
    memset(&rs485_driver, 0, sizeof(rs485_driver));
    
    /* Set buffer pointers */
    rs485_driver.dma_rx_buffer = rs485_dma_rx_buffer;
    rs485_driver.rx_buffer = rs485_rx_ring_buffer;
    rs485_driver.tx_buffer = rs485_tx_ring_buffer;
    
    /* Initialize UART port */
    status = uart_comm_init(&rs485_driver.uart_port, &huart5,
                           rs485_rx_ring_buffer, RS485_RX_BUFFER_SIZE,
                           rs485_tx_ring_buffer, RS485_TX_BUFFER_SIZE);
    
    if (status != UART_OK) {
        TRACE_ERROR(MOD_ID_RS485, "Failed to initialize UART port");
        return RS485_ERROR;
    }
    
    /* Configure DMA buffer */
    rs485_driver.uart_port.dma_rx_buffer = rs485_dma_rx_buffer;
    rs485_driver.uart_port.dma_rx_size = RS485_DMA_RX_BUFFER_SIZE;
    
    /* Set callbacks */
    uart_comm_set_tx_callback(&rs485_driver.uart_port, rs485_tx_complete_callback);
    
    /* Set initial direction to RX */
    rs485_set_direction(RS485_DIR_RX);
    
    /* Start DMA reception */
    if (uart_comm_start_rx_dma(&rs485_driver.uart_port) != UART_OK) {
        TRACE_ERROR(MOD_ID_RS485, "Failed to start DMA RX");
        return RS485_ERROR;
    }
    
    /* Reset statistics */
    rs485_reset_stats();
    
    TRACE_INFO(MOD_ID_RS485, "RS485 driver initialized");
    
    return RS485_OK;
}

/**
 * @brief Deinitialize RS485 driver
 */
rs485_status_t rs485_deinit(void)
{
    /* Stop DMA */
    uart_comm_stop_rx_dma(&rs485_driver.uart_port);
    
    /* Set to RX mode */
    rs485_set_direction(RS485_DIR_RX);
    
    TRACE_INFO(MOD_ID_RS485, "RS485 driver deinitialized");
    
    return RS485_OK;
}

/**
 * @brief Set RS485 transceiver direction
 */
void rs485_set_direction(rs485_direction_t direction)
{
    rs485_driver.direction = direction;
    
    if (direction == RS485_DIR_TX) {
        /* Set control pin high for transmit */
        HAL_GPIO_WritePin(RS485_CTR1_GPIO_Port, RS485_CTR1_Pin, GPIO_PIN_SET);
        TRACE_DEBUG(MOD_ID_RS485, "Direction: TX");
    } else {
        /* Set control pin low for receive */
        HAL_GPIO_WritePin(RS485_CTR1_GPIO_Port, RS485_CTR1_Pin, GPIO_PIN_RESET);
        TRACE_DEBUG(MOD_ID_RS485, "Direction: RX");
    }
}

/**
 * @brief Get current RS485 direction
 */
rs485_direction_t rs485_get_direction(void)
{
    return rs485_driver.direction;
}

/**
 * @brief Send packet via RS485 using DMA (non-blocking)
 */
rs485_status_t rs485_send_packet_dma(const uint8_t *data, uint16_t length)
{
    uart_status_t status;
    
    if (data == NULL || length == 0) {
        return RS485_ERROR;
    }
    
    if (rs485_driver.tx_pending) {
        TRACE_WARNING(MOD_ID_RS485, "TX busy");
        return RS485_BUSY;
    }
    
    /* Set pending flag */
    rs485_driver.tx_pending = 1;
    
    /* Send data via DMA */
    status = uart_comm_send_dma(&rs485_driver.uart_port, data, length);
    
    if (status == UART_OK) {
        rs485_driver.stats.tx_packets++;
        rs485_driver.stats.tx_bytes += length;
        TRACE_DEBUG(MOD_ID_RS485, "DMA TX started: %d bytes", length);
        return RS485_OK;
    } else {
        rs485_driver.stats.tx_errors++;
        rs485_driver.tx_pending = 0;
        TRACE_ERROR(MOD_ID_RS485, "DMA TX failed");
        return RS485_ERROR;
    }
}

/**
 * @brief Receive packet from RS485
 */
uint16_t rs485_receive_packet(uint8_t *buffer, uint16_t max_length)
{
    uint16_t received;
    
    if (buffer == NULL || max_length == 0) {
        return 0;
    }
    
    received = uart_comm_receive(&rs485_driver.uart_port, buffer, max_length);
    
    if (received > 0) {
        rs485_driver.stats.rx_bytes += received;
        TRACE_DEBUG(MOD_ID_RS485, "Received %d bytes", received);
    }
    
    return received;
}

/**
 * @brief Get number of bytes available in receive buffer
 */
uint16_t rs485_data_available(void)
{
    return uart_comm_data_available(&rs485_driver.uart_port);
}

/**
 * @brief Clear receive buffer
 */
void rs485_clear_rx_buffer(void)
{
    ring_buffer_clear(&rs485_driver.uart_port.rx_buffer);
    TRACE_DEBUG(MOD_ID_RS485, "RX buffer cleared");
}

/**
 * @brief Handle IDLE interrupt for RS485
 */
void rs485_idle_interrupt_handler(void)
{
    uart_comm_idle_handler(&rs485_driver.uart_port);
    rs485_driver.stats.rx_packets++;
}

/**
 * @brief TX complete callback
 */
void rs485_tx_complete_callback(void)
{
    /* Clear pending flag */
    rs485_driver.tx_pending = 0;
    
    TRACE_DEBUG(MOD_ID_RS485, "TX complete, switched to RX");
}

/**
 * @brief RX callback from UART layer
 */
void rs485_rx_callback(uint8_t *data, uint16_t length)
{
    /* Call user callback if set */
    if (rs485_driver.packet_received_callback != NULL) {
        rs485_driver.packet_received_callback(data, length);
    }
    
    TRACE_DEBUG(MOD_ID_RS485, "RX callback: %d bytes", length);
    TRACE_HEX_DEBUG(MOD_ID_RS485, data, length);
}

/**
 * @brief Set packet received callback
 */
void rs485_set_packet_callback(void (*callback)(uint8_t *, uint16_t))
{
    rs485_driver.packet_received_callback = callback;
}

/**
 * @brief Get RS485 statistics
 */
void rs485_get_stats(rs485_stats_t *stats)
{
    if (stats != NULL) {
        memcpy(stats, &rs485_driver.stats, sizeof(rs485_stats_t));
    }
}

/**
 * @brief Reset RS485 statistics
 */
void rs485_reset_stats(void)
{
    memset(&rs485_driver.stats, 0, sizeof(rs485_stats_t));
    TRACE_INFO(MOD_ID_RS485, "Statistics reset");
}

/**
 * @brief Print RS485 statistics
 */
void rs485_print_stats(void)
{
    TRACE_INFO(MOD_ID_RS485, "=== RS485 Statistics ===");
    TRACE_INFO(MOD_ID_RS485, "TX Packets: %lu", rs485_driver.stats.tx_packets);
    TRACE_INFO(MOD_ID_RS485, "RX Packets: %lu", rs485_driver.stats.rx_packets);
    TRACE_INFO(MOD_ID_RS485, "TX Bytes: %lu", rs485_driver.stats.tx_bytes);
    TRACE_INFO(MOD_ID_RS485, "RX Bytes: %lu", rs485_driver.stats.rx_bytes);
    TRACE_INFO(MOD_ID_RS485, "TX Errors: %lu", rs485_driver.stats.tx_errors);
    TRACE_INFO(MOD_ID_RS485, "RX Errors: %lu", rs485_driver.stats.rx_errors);
}