# NXP MC33771B/MC33772B Battery Cell Controller Driver

Driver library for controlling NXP MC33771B/MC33772B Battery Cell Controllers via the MC33664 TPL/SPI transceiver. This library provides a comprehensive interface for battery monitoring, cell balancing, fault detection, and system control in battery management applications.

## Features

- **Communication Modes**: Supports both SPI and TPL (Transformer Physical Layer) communication
- **Multi-Device Support**: Control up to 15 devices in a daisy chain
- **Cell Monitoring**: Read individual cell voltages (up to 14 cells per device)
- **Current Sensing**: Coulomb counter and current sense voltage measurement
- **Temperature Monitoring**: IC temperature and external analog inputs (ANx)
- **Cell Balancing**: Configurable passive cell balancing with timer control
- **Fault Detection**: Comprehensive fault status monitoring and clearing
- **GPIO Control**: Configure and control GPIO pins
- **EEPROM Access**: Read/write EEPROM for persistent storage
- **Low Power Modes**: Sleep and low power mode support

## Hardware Requirements

- NXP MC33771B or MC33772B Battery Cell Controller IC(s)
- NXP MC33664 Transceiver IC (for TPL mode)
- STM32F4 microcontroller (currently tested platform)
- SPI interface

## Architecture

### Device Types
- **MC33771B**: 14-cell battery monitoring IC
- **MC33772B**: 6-cell battery monitoring IC

### Communication Modes
- **SPI Mode**: Direct SPI communication to a single device
- **TPL Mode**: Daisy chains via TPL communication

## Installation

This library is designed for use with PlatformIO. Add it to your project's `platformio.ini`:

```ini
lib_deps =
    bcc
```

## Basic Usage

### 1. SPI Mode (Single Device)

```cpp
#include "BatteryCellController.h"
#include "SPI.h"

// Define pins
#define BCC_ENABLE_PIN  PA0
#define BCC_INTB_PIN    PA1
#define BCC_CS_PIN      PA2

SPIClass *spi;
BatteryCellController *bcc;

void setup() {
    // Initialize SPI
    spi = new SPIClass(MOSI, MISO, SCK);

    // Create BCC instance for a single MC33772 with 6 cells
    bcc = new BatteryCellController(
        spi,                    // SPI instance
        BCC_DEVICE_MC33772,     // Device type
        6,                      // Number of cells
        BCC_ENABLE_PIN,         // Enable pin
        BCC_INTB_PIN,           // Interrupt pin
        BCC_CS_PIN,             // Chip select pin
        false                   // Loopback mode disabled
    );

    // Initialize the BCC
    bcc_status_t status = bcc->begin();
    if (status != BCC_STATUS_SUCCESS) {
        Serial.printf("BCC initialization failed: %d\n", status);
    }
}

void loop() {
    // Read cell voltages
    uint32_t cell_voltages[6];
    bcc_status_t status = bcc->get_cell_voltages(BCC_CID_DEV1, cell_voltages);

    if (status == BCC_STATUS_SUCCESS) {
        for (int i = 0; i < 6; i++) {
            float voltage_v = cell_voltages[i] / 1000000.0f;  // Convert uV to V
            Serial.printf("Cell %d: %.3f V\n", i+1, voltage_v);
        }
    }

    delay(1000);
}
```

### 2. TPL Mode (Multiple Devices)

```cpp
#include "BatteryCellController.h"
#include "TPLSPI.h"

// Define pins
#define BCC_TX_CS       PA0
#define BCC_RX_CS       PA1
#define BCC_TX_DATA     PA2
#define BCC_TX_SCK      PA3
#define BCC_RX_DATA     PA4
#define BCC_RX_SCK      PA5
#define BCC_ENABLE_PIN  PA6
#define BCC_INTB_PIN    PA7

#define MAX_DEVICES 8
#define CELL_COUNT 6

SPIClass *tx_spi, *rx_spi;
TPLSPI *tpl;
BatteryCellController *bcc;
bcc_device_t devices[MAX_DEVICES];

void setup() {
    // Setup device types (all MC33772 in this example)
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        devices[i] = BCC_DEVICE_MC33772;
    }

    // Initialize TX and RX SPI
    tx_spi = new SPIClass(BCC_TX_DATA, NC, BCC_TX_SCK, NC);
    rx_spi = new SPIClass(BCC_RX_DATA, NC, BCC_RX_SCK, BCC_RX_CS);

    // Initialize TPL
    tpl = new TPLSPI(tx_spi, rx_spi, BCC_TX_CS);

    // Create BCC instance for TPL chain
    bcc = new BatteryCellController(
        tpl,                    // TPL instance
        devices,                // Device type array
        MAX_DEVICES,            // Number of devices
        CELL_COUNT,             // Cells per device
        BCC_ENABLE_PIN,         // Enable pin
        BCC_INTB_PIN,           // Interrupt pin
        false                   // Loopback mode disabled
    );

    // Initialize the BCC chain
    bcc_status_t status = bcc->begin();
    if (status != BCC_STATUS_SUCCESS) {
        Serial.printf("BCC initialization failed: %d\n", status);
    }
}
```

## Key Functions

### Initialization

```cpp
// Initialize BCC with optional device configuration
bcc_status_t begin(const uint16_t devConf[][BCC_INIT_CONF_REG_CNT] = nullptr);

// Wake up device from sleep
void wake_up();

// Put device to sleep
bcc_status_t sleep();
```

### Measurements

```cpp
// Start ADC conversion asynchronously
bcc_status_t start_conversion_async(bcc_cid_t cid);

// Check if conversion is complete
bcc_status_t is_converting(bcc_cid_t cid, bool *completed);

// Get cell voltages (in microvolts)
bcc_status_t get_cell_voltages(bcc_cid_t cid, uint32_t *cell_voltages);

// Get single cell voltage
bcc_status_t get_cell_voltage(bcc_cid_t cid, uint8_t cell_index, uint32_t *cell_voltage);

// Get stack voltage (sum of all cells)
bcc_status_t get_stack_voltage(bcc_cid_t cid, uint32_t *stack_voltage);

// Get current sense voltage
bcc_status_t get_current_sense_voltage(bcc_cid_t cid, int32_t *current_sense_voltage);

// Get coulomb counter data
bcc_status_t get_coulomb_counter(bcc_cid_t cid, bcc_cc_data_t* coulomb_counter);

// Get IC temperature
bcc_status_t get_ic_temperature(bcc_cid_t cid, bcc_temp_unit_t unit, int16_t *ic_temperature);

// Get analog input voltages
bcc_status_t get_an_voltages(bcc_cid_t cid, uint32_t *an_voltages);
```

### Cell Balancing

```cpp
// Enable/disable cell balancing globally
bcc_status_t enable_cell_balancing(bcc_cid_t cid, bool enable);

// Set balancing for specific cell
bcc_status_t set_cell_balancing(bcc_cid_t cid, uint8_t cell_index, bool enable, uint16_t timer);

// Pause/resume cell balancing
bcc_status_t pause_cell_balancing(bcc_cid_t cid, bool pause);
```

### Fault Monitoring

```cpp
// Get fault status (returns array of fault registers)
bcc_status_t get_fault_status(bcc_cid_t cid, uint16_t fault_status[]);

// Clear fault status
bcc_status_t clear_fault_status(bcc_cid_t cid, bcc_fault_status_t fault_status);
```

### Register Access

```cpp
// Read register(s)
bcc_status_t read_register(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val);

// Write register
bcc_status_t write_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val, uint16_t* ret_reg = nullptr);

// Write to all devices (global write, TPL mode only)
bcc_status_t write_register_global(uint8_t reg_addr, uint16_t reg_val);

// Update register (read-modify-write)
bcc_status_t update_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t mask, uint16_t val);
```

### GPIO Control

```cpp
// Set GPIO configuration
bcc_status_t set_gpio_config(bcc_cid_t cid, uint8_t gpio_sel, bool val);

// Set GPIO mode (input/output)
bcc_status_t set_gpio_mode(bcc_cid_t cid, uint8_t gpio_sel, bcc_pin_mode_t mode);

// Read GPIO value
bcc_status_t read_gpio(bcc_cid_t cid, uint8_t gpio_sel, bool *val);

// Write GPIO value
bcc_status_t write_gpio(bcc_cid_t cid, uint8_t gpio_sel, bool val);
```

### Device Information

```cpp
// Read device GUID (unique identifier)
bcc_status_t read_guid(bcc_cid_t cid, uint64_t *guid);

// Read fuse mirror value
bcc_status_t read_fuse_mirror(bcc_cid_t cid, uint8_t fuse_addr, uint16_t *value);

// Write fuse mirror value
bcc_status_t write_fuse_mirror(bcc_cid_t cid, uint8_t fuse_addr, uint16_t value);
```

### EEPROM Access

```cpp
// Read EEPROM byte
bcc_status_t read_eeprom(bcc_cid_t cid, uint8_t address, uint8_t *data);

// Write EEPROM byte
bcc_status_t write_eeprom(bcc_cid_t cid, uint8_t address, uint8_t data);
```

## Device Addressing (CID)

In TPL mode, devices are addressed using Cluster Identification (CID):
- `BCC_CID_DEV1` - Device 1 (closest to transceiver)
- `BCC_CID_DEV2` - Device 2
- ... up to `BCC_CID_DEV15`

## Error Handling

All functions return a `bcc_status_t` value:
- `BCC_STATUS_SUCCESS` (0) - Operation successful
- Other values indicate specific errors (see `bcc_error.h`)

Always check return values:

```cpp
bcc_status_t status = bcc->get_cell_voltages(BCC_CID_DEV1, cell_voltages);
if (status != BCC_STATUS_SUCCESS) {
    Serial.printf("Error reading cell voltages: %d\n", status);
    // Handle error...
}
```

## Examples

See the `examples/` directory for complete examples:
- `simple_monitoring` - Basic cell voltage monitoring

## Platform Support

Currently tested on:
- STM32F4 series microcontrollers

The library uses conditional compilation for STM32-specific features but may work on other Arduino-compatible platforms with SPI support.

## License

See LICENSE file for details.

## References

- [MC33771B Datasheet](https://www.nxp.com/docs/en/data-sheet/MC33771B.pdf)
- [MC33772B Datasheet](https://www.nxp.com/docs/en/data-sheet/MC33772B.pdf)
- [MC33664 Datasheet](https://www.nxp.com/docs/en/data-sheet/MC33664.pdf)

## Author

outlandnish

## Version

1.0.0
