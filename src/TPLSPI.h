#pragma once
#include "Arduino.h"
#include "SPI.h"
#include "bcc/bcc.h"
#include "bcc/bcc_defs.h"

#define assert(expr) assert_param(expr)

// Function pointer type for DMA configuration callback
// Takes TX and RX SPI handles and returns true on success
typedef bool (*DMAConfigCallback)(SPIClass* spi_tx, SPIClass* spi_rx);

// TPL (Two-wire Passive Line) SPI class for MC33664 communication
// Manages separate TX (full-duplex) and RX (receive-only) SPI buses
//
// NOTE: DMA configuration is processor-specific and must be provided
// as a callback function passed to begin(). The DMA setup should configure:
// - TX SPI DMA for memory-to-peripheral transfers
// - RX SPI DMA for peripheral-to-memory transfers
// - Appropriate interrupt handlers for both DMA streams and SPI peripherals
class TPLSPI {
  private:
    // STM32-specific members
    SPIClass *spi_tx;
    SPIClass *spi_rx;
    uint8_t cs_tx_pin;
    DMAConfigCallback dma_config_callback;

    // Fast GPIO access for CS pin (bypasses Arduino digitalWrite)
    GPIO_TypeDef *cs_port;
    uint32_t cs_pin_mask;

  public:
    // STM32 constructor
    TPLSPI(SPIClass *tx, SPIClass *rx, uint8_t cs_tx_pin, DMAConfigCallback dma_config_callback = nullptr);

    bool beginTransaction();
    void endTransaction();

    SPIClass* getTxSPI() { return spi_tx; }
    SPIClass* getRxSPI() { return spi_rx; }

    bool begin();
    void end();
    bcc_status_t transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t rx_transfer_count);

    uint8_t getCSPin() { return cs_tx_pin; }
};