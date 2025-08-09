# STM32F407 Communication Driver Demo

## Overview

This project is an STM32F407-based embedded system that implements RS485 and UART communication drivers with DMA support. The project is built using STM32 HAL (Hardware Abstraction Layer) and provides a modular communication framework.

## Features

- **RS485 Communication Driver** - Full-duplex RS485 communication with automatic direction control
- **UART Communication** - Multiple UART interfaces (UART1, UART5) with DMA support
- **CAN Bus Interface** - CAN1 peripheral configuration
- **Debug Logging** - Structured debug logging system with multiple trace levels
- **DMA Support** - Efficient data transfer using Direct Memory Access
- **Ring Buffer Implementation** - Circular buffers for UART RX/TX operations

## Hardware Requirements

- STM32F407 microcontroller
- RS485 transceiver
- CAN transceiver (optional)
- UART interfaces for communication and debugging

## Project Structure

```
demo/
├── Core/                       # Application core files
│   ├── Inc/                   # Header files
│   │   ├── main.h             # Main application header
│   │   ├── rs485_driver.h     # RS485 driver interface
│   │   ├── uart_comm.h        # UART communication layer
│   │   ├── debug_log.h        # Debug logging utilities
│   │   ├── basic_func.h       # Basic utility functions
│   │   └── user_main.h        # User application entry point
│   └── Src/                   # Source files
│       ├── main.c             # Main application
│       ├── rs485_driver.c     # RS485 driver implementation
│       ├── uart_comm.c        # UART communication implementation
│       ├── debug_log.c        # Debug logging implementation
│       ├── basic_func.c       # Basic utility functions
│       └── user_main.c        # User application logic
├── Drivers/                    # STM32 HAL and CMSIS drivers
│   ├── STM32F4xx_HAL_Driver/  # HAL driver library
│   └── CMSIS/                 # CMSIS library
├── Makefile                   # Build configuration
├── STM32F407XX_FLASH.ld       # Linker script
├── startup_stm32f407xx.s      # Startup assembly code
└── demo.ioc                   # STM32CubeMX project file
```

## Building the Project

### Prerequisites

- ARM GCC Toolchain (arm-none-eabi-gcc)
- Make utility
- STM32CubeProgrammer or OpenOCD for flashing

### Build Commands

```bash
# Build the project
make

# Clean build files
make clean

# Build with debug symbols
make DEBUG=1
```

The build output will be generated in the `build/` directory.

## Configuration

### RS485 Configuration

The RS485 driver can be configured in `Core/Inc/rs485_driver.h`:
- Buffer sizes (DMA RX, Ring RX/TX)
- Baud rate and communication parameters
- Direction control GPIO pins

### UART Configuration

UART parameters are configured through STM32CubeMX (demo.ioc file):
- UART1: Debug/console interface
- UART5: RS485 communication

### Debug Logging

Debug levels can be configured in `Core/Inc/debug_log.h`:
- TRACE_ERROR: Error messages
- TRACE_WARNING: Warning messages
- TRACE_INFO: Informational messages
- TRACE_DEBUG: Debug messages

## Peripheral Configuration

- **System Clock**: Configured through SystemClock_Config()
- **CAN1**: CAN bus communication interface
- **UART1**: Debug console with DMA support
- **UART5**: RS485 communication with DMA support
- **GPIO**: Various pins for LED indicators and RS485 direction control
- **DMA**: Multiple DMA streams for UART RX/TX operations

## Library Documentation

### [RS485 Library - DMA Communication](#rs485-library-documentation)
### [Debug Log Library](#debug-log-library-documentation)

---

# RS485 Library Documentation

## Overview

The RS485 library provides robust, DMA-based communication for RS485 serial interfaces on STM32F407. It features automatic direction control, efficient DMA transfers, and comprehensive error handling.

## RS485 Architecture

### Layer Structure
```
┌─────────────────────────┐
│   Application Layer     │
├─────────────────────────┤
│   RS485 Driver Layer    │  ← rs485_driver.c/h
├─────────────────────────┤
│   UART Comm Layer       │  ← uart_comm.c/h
├─────────────────────────┤
│   STM32 HAL Layer       │  ← STM32 HAL UART/DMA
└─────────────────────────┘
```

### DMA Buffer Configuration
- **DMA RX Buffer**: 256 bytes (hardware DMA circular buffer)
- **RX Ring Buffer**: 512 bytes (software circular buffer)
- **TX Ring Buffer**: 512 bytes (transmission buffer)

## RS485 DMA Implementation Details

### DMA Reception Mechanism

The RS485 library uses a **dual-buffer strategy** for reception:

1. **Hardware DMA Buffer** (Circular Mode):
   - Size: 256 bytes
   - DMA continuously writes received data in circular mode
   - Never stops, automatically wraps around

2. **Software Ring Buffer**:
   - Size: 512 bytes
   - Application reads data from here
   - Decouples hardware reception from application processing

### How DMA Reception Works

```c
// DMA RX is configured in CIRCULAR mode
// Data flow: UART RX → DMA → DMA Buffer → Ring Buffer → Application

// The DMA continuously receives data:
// 1. UART RX interrupt triggers DMA transfer
// 2. DMA writes to circular buffer automatically
// 3. IDLE line detection triggers data processing
// 4. Data is copied from DMA buffer to ring buffer
// 5. Application reads from ring buffer
```

### DMA Transmission Mechanism

DMA transmission uses **NORMAL mode** (single-shot):

```c
// Data flow: Application → TX Buffer → DMA → UART TX

// Steps:
// 1. Application calls rs485_send_packet_dma()
// 2. Direction switches to TX automatically
// 3. DMA transfers data to UART
// 4. TX complete interrupt triggers
// 5. Direction switches back to RX
```

## RS485 API Reference

### Core Functions

#### Initialize RS485 with DMA
```c
rs485_status_t rs485_init(void);
```
This function:
- Initializes UART5 with DMA
- Configures circular DMA for RX
- Sets up ring buffers
- Starts continuous DMA reception

#### Send Data via DMA (Non-blocking)
```c
rs485_status_t rs485_send_packet_dma(const uint8_t *data, uint16_t length);
```
**Key Features**:
- Non-blocking transmission
- Automatic direction control
- Returns immediately (data sent in background)
- Check status for busy/error conditions

#### Receive Data from Ring Buffer
```c
uint16_t rs485_receive_packet(uint8_t *buffer, uint16_t max_length);
```
**Returns**: Number of bytes received (0 if no data available)

#### Check Available Data
```c
uint16_t rs485_data_available(void);
```

## DMA Usage Examples

### Example 1: Basic DMA Send and Receive Loop

```c
void rs485_dma_communication_example(void) {
    uint8_t tx_data[] = "Hello RS485";
    uint8_t rx_buffer[256];
    rs485_status_t status;
    
    // Initialize RS485 with DMA
    if (rs485_init() != RS485_OK) {
        Error_Handler();
    }
    
    // Main communication loop
    while (1) {
        // --- DMA SEND ---
        // Send data using DMA (non-blocking)
        // This example sends periodically, adjust as needed
        static uint32_t last_send_tick = 0;
        if (HAL_GetTick() - last_send_tick > 1000) {  // Send every 1 second
            status = rs485_send_packet_dma(tx_data, sizeof(tx_data));
            
            if (status == RS485_OK) {
                last_send_tick = HAL_GetTick();
                TRACE_INFO(MOD_ID_RS485, "Sent data");
            } else if (status == RS485_BUSY) {
                // Previous DMA transfer still in progress
                TRACE_DEBUG(MOD_ID_RS485, "TX busy, will retry");
            } else {
                // Handle error
                TRACE_ERROR(MOD_ID_RS485, "Send failed");
            }
        }
        
        // --- DMA RECEIVE LOOP ---
        // Continuously check for received data
        // DMA is always running in background
        if (rs485_data_available() > 0) {
            uint16_t received = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
            
            // Process received data
            TRACE_INFO(MOD_ID_RS485, "Received %d bytes", received);
            TRACE_HEX_INFO(MOD_ID_RS485, rx_buffer, received);
            
            // Your data processing here
            process_received_data(rx_buffer, received);
        }
        
        // Do other tasks while DMA works in background
        perform_other_tasks();
        
        // Yield to other processes (optional, for RTOS)
        // osDelay(1);  // For FreeRTOS
    }
}
```

### Example 2: Continuous DMA Reception Loop

```c
void rs485_continuous_reception(void) {
    uint8_t rx_buffer[256];
    uint16_t bytes_received;
    
    // Initialize once
    rs485_init();
    
    // DMA reception is ALWAYS active after init
    // No need to restart or reinitialize
    
    while (1) {
        // Method 1: Polling for data
        bytes_received = rs485_data_available();
        if (bytes_received > 0) {
            // Read all available data
            bytes_received = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
            
            // Process the data
            for (uint16_t i = 0; i < bytes_received; i++) {
                process_byte(rx_buffer[i]);
            }
        }
        
        // Method 2: Read with threshold
        if (rs485_data_available() >= 10) {  // Wait for at least 10 bytes
            bytes_received = rs485_receive_packet(rx_buffer, 10);
            process_packet(rx_buffer, bytes_received);
        }
        
        // The DMA continues receiving in background
        // even while you process data
    }
}
```

### Example 3: DMA with IDLE Line Detection

```c
// This is handled automatically by the driver
// IDLE interrupt indicates end of packet

void UART5_IRQHandler(void) {
    // IDLE line detection - packet complete
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart5);
        
        // This triggers automatic data transfer from DMA buffer to ring buffer
        rs485_idle_interrupt_handler();
        
        // Now data is available in ring buffer
        // Set a flag for main loop processing
        packet_ready_flag = 1;
    }
    
    HAL_UART_IRQHandler(&huart5);
}

// In main loop
void main_loop(void) {
    uint8_t packet[256];
    
    while (1) {
        if (packet_ready_flag) {
            packet_ready_flag = 0;
            
            // Read the complete packet
            uint16_t len = rs485_receive_packet(packet, sizeof(packet));
            if (len > 0) {
                process_complete_packet(packet, len);
            }
        }
    }
}
```

### Example 4: Non-blocking DMA Send with Status Check

```c
void rs485_send_with_retry(uint8_t *data, uint16_t length) {
    rs485_status_t status;
    uint8_t retry_count = 0;
    const uint8_t MAX_RETRIES = 10;
    
    while (retry_count < MAX_RETRIES) {
        status = rs485_send_packet_dma(data, length);
        
        switch (status) {
            case RS485_OK:
                TRACE_INFO(MOD_ID_RS485, "Send started successfully");
                return;  // Success
                
            case RS485_BUSY:
                // DMA is busy with previous transfer
                TRACE_DEBUG(MOD_ID_RS485, "DMA busy, retry %d", retry_count);
                retry_count++;
                // Small delay or yield for retry
                for (volatile uint32_t i = 0; i < 10000; i++);
                break;
                
            case RS485_ERROR:
                TRACE_ERROR(MOD_ID_RS485, "Send error");
                return;  // Fatal error
        }
    }
    
    TRACE_ERROR(MOD_ID_RS485, "Send failed after %d retries", MAX_RETRIES);
}
```

### Example 5: Efficient DMA Loop for Protocol Processing

```c
typedef struct {
    uint8_t header[2];
    uint8_t length;
    uint8_t data[253];
} packet_t;

void rs485_protocol_handler(void) {
    static uint8_t rx_buffer[512];
    static uint16_t buffer_pos = 0;
    uint8_t temp_buffer[256];
    uint16_t bytes_available;
    
    // Initialize RS485 with DMA
    rs485_init();
    
    while (1) {
        // Check for new data from DMA
        bytes_available = rs485_data_available();
        
        if (bytes_available > 0) {
            // Read from DMA ring buffer
            uint16_t received = rs485_receive_packet(temp_buffer, 
                                    MIN(bytes_available, sizeof(temp_buffer)));
            
            // Accumulate in protocol buffer
            memcpy(&rx_buffer[buffer_pos], temp_buffer, received);
            buffer_pos += received;
            
            // Try to parse complete packets
            while (buffer_pos >= 3) {  // Minimum packet size
                // Check for valid header
                if (rx_buffer[0] == 0xAA && rx_buffer[1] == 0x55) {
                    uint8_t packet_length = rx_buffer[2] + 3;  // header + length + data
                    
                    if (buffer_pos >= packet_length) {
                        // Complete packet received
                        packet_t *packet = (packet_t*)rx_buffer;
                        process_packet(packet);
                        
                        // Remove processed packet from buffer
                        memmove(rx_buffer, &rx_buffer[packet_length], 
                               buffer_pos - packet_length);
                        buffer_pos -= packet_length;
                    } else {
                        // Wait for more data
                        break;
                    }
                } else {
                    // Invalid header, skip one byte
                    memmove(rx_buffer, &rx_buffer[1], buffer_pos - 1);
                    buffer_pos--;
                }
            }
        }
        
        // Perform other tasks
        // DMA continues receiving in background
        handle_other_tasks();
        
        // Yield CPU (optional for better performance)
        __WFI();  // Wait for interrupt
    }
}
```

### Example 6: Full-Duplex DMA Communication

```c
void rs485_full_duplex_dma(void) {
    uint8_t rx_buffer[256];
    uint8_t tx_queue[10][64];  // TX queue
    uint8_t tx_queue_head = 0;
    uint8_t tx_queue_tail = 0;
    rs485_status_t tx_status = RS485_OK;
    
    rs485_init();
    
    while (1) {
        // --- RECEIVE PROCESSING ---
        // DMA RX is always active
        if (rs485_data_available() > 0) {
            uint16_t len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
            
            // Generate response and add to TX queue
            if (should_respond(rx_buffer, len)) {
                prepare_response(&tx_queue[tx_queue_tail][0]);
                tx_queue_tail = (tx_queue_tail + 1) % 10;
            }
        }
        
        // --- TRANSMIT PROCESSING ---
        // Send from queue when DMA is ready
        if (tx_queue_head != tx_queue_tail && tx_status != RS485_BUSY) {
            tx_status = rs485_send_packet_dma(tx_queue[tx_queue_head], 64);
            
            if (tx_status == RS485_OK) {
                tx_queue_head = (tx_queue_head + 1) % 10;
            }
        }
        
        // Check if previous TX completed
        if (tx_status == RS485_BUSY) {
            // Check if DMA is still busy or reset status
            // In real implementation, check actual DMA state
            static uint32_t tx_timeout = 0;
            if (HAL_GetTick() - tx_timeout > 100) {  // 100ms timeout
                tx_status = RS485_OK;  // Reset for next attempt
                tx_timeout = HAL_GetTick();
            }
        }
        
        // Yield to scheduler or save power
        __WFI();  // Wait for interrupt
    }
}
```

## DMA Configuration Details

### Hardware Setup (STM32CubeMX)

```
UART5 Configuration:
- Mode: Asynchronous
- Baud Rate: 115200
- Word Length: 8 bits
- Stop Bits: 1
- Parity: None

DMA Configuration:
- UART5_RX: DMA1 Stream 0
  - Mode: Circular
  - Priority: High
  - Data Width: Byte
  
- UART5_TX: DMA1 Stream 7
  - Mode: Normal
  - Priority: High
  - Data Width: Byte

Interrupts:
- UART5 global interrupt: Enabled
- DMA1 Stream 0 (RX): Enabled
- DMA1 Stream 7 (TX): Enabled
```

## Important DMA Notes

1. **RX DMA Never Stops**: Once initialized, RX DMA runs continuously in circular mode
2. **No Data Loss**: Ring buffer prevents data loss even during processing
3. **Automatic Direction Control**: TX/RX direction switches automatically
4. **Non-blocking Operations**: All DMA operations are asynchronous
5. **IDLE Detection**: Hardware detects packet boundaries automatically

---

# Debug Log Library Documentation

## Overview

The Debug Log library provides comprehensive logging with color support, module filtering, and HEX data dumping for embedded systems.

## Features

- **6 Log Levels**: DEBUG, WARNING, INFO, ERROR, NOTICE, FATAL
- **Module-based Filtering**: Enable/disable per module
- **Colored Output**: ANSI color codes for terminals
- **HEX Data Logging**: Binary data visualization
- **Timestamps**: Millisecond precision timing

## Log Levels

| Level | Macro | Color | Usage |
|-------|-------|-------|-------|
| DEBUG | `TRACE_DEBUG` | White | Detailed debugging |
| WARNING | `TRACE_WARNING` | Yellow | Warnings |
| INFO | `TRACE_INFO` | Green | Information |
| ERROR | `TRACE_ERROR` | Red | Errors |
| NOTICE | `TRACE_NOTICE` | Blue | Significant events |
| FATAL | `TRACE_FATAL` | Red BG | Fatal errors |

## Module System

```c
typedef enum {
    MOD_ID_INIT = 0,    // Initialization
    MOD_ID_MAIN,        // Main application
    MOD_ID_UART,        // UART communication
    MOD_ID_RS485,       // RS485 driver
    MOD_ID_CAN,         // CAN bus
    MOD_ID_SYSTEM,      // System functions
    MOD_ID_APP,         // Application layer
    MOD_ID_CNT          // Module count
} module_id_t;
```

## Debug Log API

### Initialization
```c
void debug_log_init(void);
```

### Logging Macros
```c
TRACE_DEBUG(module, format, ...)    // Debug level
TRACE_INFO(module, format, ...)     // Info level
TRACE_WARNING(module, format, ...)  // Warning level
TRACE_ERROR(module, format, ...)    // Error level
TRACE_FATAL(module, format, ...)    // Fatal level

// HEX data logging
TRACE_HEX_DEBUG(module, data, length)
TRACE_HEX_INFO(module, data, length)
```

### Filter Control
```c
void debug_log_enable_module(module_id_t module);
void debug_log_disable_module(module_id_t module);
void debug_log_set_filter(unsigned int filter);
void debug_log_enable_color(uint8_t enable);
```

## Debug Log Examples

### Basic Logging
```c
// Initialize
debug_log_init();

// Log messages
TRACE_INFO(MOD_ID_MAIN, "System started");
TRACE_DEBUG(MOD_ID_RS485, "Sending %d bytes", count);
TRACE_ERROR(MOD_ID_UART, "Timeout after %dms", timeout);

// Log HEX data
uint8_t data[] = {0xAA, 0x55, 0x01, 0x02};
TRACE_HEX_DEBUG(MOD_ID_RS485, data, sizeof(data));
```

### Module Filtering
```c
// Enable only specific modules
debug_log_set_filter(0);  // Disable all
debug_log_enable_module(MOD_ID_RS485);
debug_log_enable_module(MOD_ID_ERROR);

// These will be shown
TRACE_INFO(MOD_ID_RS485, "This is visible");

// This will be filtered
TRACE_INFO(MOD_ID_MAIN, "This is hidden");
```

### Function Tracing
```c
#define FUNC_ENTER() TRACE_DEBUG(MOD_ID_APP, ">>> %s", __func__)
#define FUNC_EXIT()  TRACE_DEBUG(MOD_ID_APP, "<<< %s", __func__)

void my_function(void) {
    FUNC_ENTER();
    // Function body
    FUNC_EXIT();
}
```

## Output Format

### Standard Log
```
[HH:MM:SS:mmm] T<tick>:[LEVEL][MODULE] message
```

### HEX Log
```
[HH:MM:SS:mmm] T<tick>:[LEVEL][MODULE]HEX:AA550102...
```

## Complete Integration Example

```c
// main.c
int main(void) {
    // Initialize HAL
    HAL_Init();
    SystemClock_Config();
    
    // Initialize debug logging
    debug_log_init();
    TRACE_INFO(MOD_ID_INIT, "System starting...");
    
    // Initialize RS485 with DMA
    if (rs485_init() != RS485_OK) {
        TRACE_FATAL(MOD_ID_INIT, "RS485 init failed");
        Error_Handler();
    }
    TRACE_INFO(MOD_ID_INIT, "RS485 initialized");
    
    // Main loop with DMA communication
    uint8_t rx_buffer[256];
    uint8_t tx_data[] = "Test";
    
    while (1) {
        // Send data via DMA
        if (need_to_send()) {
            rs485_status_t status = rs485_send_packet_dma(tx_data, sizeof(tx_data));
            if (status == RS485_OK) {
                TRACE_DEBUG(MOD_ID_APP, "Sent %d bytes", sizeof(tx_data));
            }
        }
        
        // Check for received data (DMA runs continuously)
        if (rs485_data_available() > 0) {
            uint16_t len = rs485_receive_packet(rx_buffer, sizeof(rx_buffer));
            TRACE_INFO(MOD_ID_APP, "Received %d bytes", len);
            TRACE_HEX_DEBUG(MOD_ID_APP, rx_buffer, len);
        }
        
        // Main loop continues...
    }
}
```

## Development Tools

- **STM32CubeMX**: For peripheral configuration and code generation
- **GCC ARM Toolchain**: For compilation
- **Make**: For build automation
- **STM32CubeProgrammer**: For flashing firmware

## License

This project uses STMicroelectronics' HAL drivers which are licensed under their respective terms. See individual LICENSE files in the Drivers directory for details.

## Contributing

When contributing to this project:
1. Follow the existing code style and conventions
2. Test all changes thoroughly
3. Update documentation as needed
4. Ensure all peripherals are properly initialized

## Debugging

The project includes comprehensive debug logging. Connect to UART1 to view debug output. Use the debug_log module to add custom trace messages in your code.

## Contact

For questions or issues, please create an issue in the project repository.