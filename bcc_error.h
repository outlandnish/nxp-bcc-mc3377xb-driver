#ifndef BCC_ERROR_H
#define BCC_ERROR_H

#include "bcc.h"

/* Error context structure */
typedef struct {
  bcc_status_t status;
  bcc_cid_t cid;
  uint8_t register_addr;
  const char* operation;
  const char* description;
} bcc_error_context_t;

/* Error logging callback type */
typedef void (*bcc_error_callback_t)(const bcc_error_context_t* context);

/* Error handler class */
class BccErrorHandler {
public:
  static void setCallback(bcc_error_callback_t callback) {
    s_callback = callback;
  }
  
  static void logError(bcc_status_t status, bcc_cid_t cid, 
            uint8_t reg_addr, const char* operation) {
    if (s_callback) {
      bcc_error_context_t ctx = {
        .status = status,
        .cid = cid,
        .register_addr = reg_addr,
        .operation = operation,
        .description = getErrorDescription(status)
      };
      s_callback(&ctx);
    }
  }
  
  static const char* getErrorDescription(bcc_status_t status) {
    switch (status) {
      case BCC_STATUS_SUCCESS:        return "Success";
      case BCC_STATUS_SPI_INIT:       return "SPI initialization failed";
      case BCC_STATUS_SPI_BUSY:       return "SPI bus busy";
      case BCC_STATUS_SPI_TX:         return "SPI transmission error";
      case BCC_STATUS_SPI_RX:         return "SPI reception error";
      case BCC_STATUS_TIMEOUT:        return "Communication timeout";
      case BCC_STATUS_CRC:            return "CRC check failed";
      case BCC_STATUS_PARAM_RANGE:    return "Parameter out of range";
      case BCC_STATUS_COM_STATUS:     return "Communication status error";
      default:                        return "Unknown error";
    }
  }

private:
  static bcc_error_callback_t s_callback;
};

/* Macro for checking errors with context */
#define BCC_CHECK_ERROR(expr, cid, reg, op) do { \
  bcc_status_t _err = (expr); \
  if (_err != BCC_STATUS_SUCCESS) { \
    BccErrorHandler::logError(_err, cid, reg, op); \
    return _err; \
  } \
} while(0)

/* Macro for checking errors without return */
#define BCC_LOG_ERROR(expr, cid, reg, op) do { \
  bcc_status_t _err = (expr); \
  if (_err != BCC_STATUS_SUCCESS) { \
    BccErrorHandler::logError(_err, cid, reg, op); \
  } \
} while(0)

#endif /* BCC_ERROR_H */