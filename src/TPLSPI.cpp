#include "TPLSPI.h"

TPLSPI::TPLSPI(SPIClass *tx, SPIClass *rx, uint8_t cs_tx_pin, DMAConfigCallback dma_config_callback)
  : spi_tx(tx), spi_rx(rx), cs_tx_pin(cs_tx_pin), dma_config_callback(dma_config_callback)
{
  pinMode(cs_tx_pin, OUTPUT);
  digitalWrite(cs_tx_pin, HIGH);

  // Set up fast GPIO access for CS pin (STM32-specific)
  cs_port = get_GPIO_Port(STM_PORT(digitalPinToPinName(cs_tx_pin)));
  cs_pin_mask = STM_GPIO_PIN(digitalPinToPinName(cs_tx_pin));
}

bool TPLSPI::begin() {
  // Configure and initialize SPI peripherals
  spi_tx->setIsMaster(true);
  spi_rx->setIsMaster(false);
  spi_tx->setDirection(SPI_DIRECTION_2LINES);
  spi_rx->setDirection(SPI_DIRECTION_2LINES_RXONLY);
  Serial.println("Starting TX");
  spi_tx->begin();
  Serial.println("Starting RX");
  spi_rx->begin();

  // Configure DMA if callback provided (processor-specific)
  if (this->dma_config_callback != nullptr) {
    Serial.println("Initializing SPI DMA via callback...");
    if (!this->dma_config_callback(spi_tx, spi_rx)) {
      Serial.println("Failed to configure DMA");
      return false;
    }
  }

  return beginTransaction();
}

bool TPLSPI::beginTransaction() {
  bool tx_ok = spi_tx->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1, SPI_MASTER));
  bool rx_ok = spi_rx->beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1, SPI_SLAVE));
  return tx_ok && rx_ok;
}

void TPLSPI::endTransaction() {
  spi_tx->endTransaction();
  spi_rx->endTransaction();
}

void TPLSPI::end() {
  spi_tx->end();
  spi_rx->end();
}

bcc_status_t TPLSPI::transfer(uint8_t *tx_buf, uint8_t *rx_buf, uint16_t rx_transfer_count) {
  HAL_StatusTypeDef tx_error, rx_error;
  uint16_t i;
  uint32_t tx_timeout_ms = BCC_TX_COM_TIMEOUT_MS;
  uint32_t rx_timeout_ms = BCC_RX_COM_TIMEOUT_MS;

  assert(tx_buf != nullptr);
  assert(rx_buf != nullptr);

  SPI_HandleTypeDef *hspi_tx = spi_tx->getHandle();
  SPI_HandleTypeDef *hspi_rx = spi_rx->getHandle();

  // Start RX reception using DMA for all frames (echo + response frames)
  uint32_t total_rx_size = rx_transfer_count * BCC_MSG_SIZE;

  rx_error = HAL_SPI_Receive_DMA(hspi_rx, rx_buf, total_rx_size);
  if (rx_error != HAL_OK) {
    return rx_error == HAL_TIMEOUT ? BCC_STATUS_COM_TIMEOUT : BCC_STATUS_SPI_FAIL;
  }

  cs_port->BSRR = (cs_pin_mask << 16);
  delayMicroseconds(2); // Setup time

  // Start TX DMA transfer
  tx_error = HAL_SPI_Transmit_DMA(hspi_tx, tx_buf, BCC_MSG_SIZE);
  if (tx_error != HAL_OK) {
    Serial.printf("Error starting TX DMA: %d (state=%d, error=0x%08lX)\n",
                  tx_error, hspi_tx->State, hspi_tx->ErrorCode);
    cs_port->BSRR = cs_pin_mask;  // Set (HIGH)
    HAL_SPI_Abort_IT(hspi_rx);
    return BCC_STATUS_SPI_FAIL;
  }

  // Wait for TX to complete
  uint32_t tx_timeout_us = tx_timeout_ms * 1000;
  while (HAL_SPI_GetState(hspi_tx) != HAL_SPI_STATE_READY && tx_timeout_us > 0) {
    delayMicroseconds(10);
    tx_timeout_us -= 10;
  }

  delayMicroseconds(2); // Hold time
  cs_port->BSRR = cs_pin_mask;

  if (tx_timeout_us <= 0) {
    Serial.printf("Timeout in TX transmission (state: %d, error: 0x%08lX)\n",
                  hspi_tx->State, hspi_tx->ErrorCode);
    HAL_SPI_Abort_IT(hspi_rx);
    return BCC_STATUS_COM_TIMEOUT;
  }

  // Wait until all RX frames received
  uint32_t rx_timeout_us = rx_timeout_ms * rx_transfer_count * 1000;
  HAL_SPI_StateTypeDef rx_state = HAL_SPI_GetState(hspi_rx);

  // Wait for DMA to complete (NDTR reaches 0)
  while (hspi_rx->hdmarx->Instance->NDTR > 0 && rx_timeout_us > 0) {
    delayMicroseconds(10);
    rx_timeout_us -= 10;
  }

  rx_state = HAL_SPI_GetState(hspi_rx);
  uint32_t final_ndtr = hspi_rx->hdmarx->Instance->NDTR;

  if (final_ndtr > 0) {
    Serial.printf("Timeout in waiting for RX DMA (NDTR: %lu, state: %d, error: 0x%08lX)\n",
                  final_ndtr, rx_state, hspi_rx->ErrorCode);
      // Read DMA stream status register for detailed error info
      DMA_Stream_TypeDef *dma_stream = hspi_rx->hdmarx->Instance;
      uint32_t dma_cr = dma_stream->CR;
    HAL_SPI_Abort_IT(hspi_rx);
    return BCC_STATUS_COM_TIMEOUT;
  }

  return BCC_STATUS_SUCCESS;
}