#pragma once
#include "bcc_utils.h"

#define BCC_CRC_TBL_SIZE    256U

#define BCC_MSG_SIZE              5U  /* 40b frames */

/*! @brief Note: For 5-byte frames, CID and CMD are combined into a single byte.
 * The index definitions BCC_MSG_IDX_CID_CMD and BCC_MSG_IDX_CRC are provided
 * below based on endianness (see lines 425-451). */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Maximal number of BCC devices in SPI mode. */
#define BCC_DEVICE_CNT_MAX_SPI    1U
/*! @brief Maximal number of BCC devices in TPL mode. */
#define BCC_DEVICE_CNT_MAX_TPL    15U
/*! @brief Maximal number of BCC devices. */
#define BCC_DEVICE_CNT_MAX        BCC_DEVICE_CNT_MAX_TPL

/*! @brief Minimal battery cell count connected to MC33771C. */
#define MC33771_MIN_CELLS        7U
/*! @brief Maximal battery cell count connected to MC33771C. */
#define MC33771_MAX_CELLS        14U
/*! @brief Minimal battery cell count connected to MC33772C. */
#define MC33772_MIN_CELLS        3U
/*! @brief Maximal battery cell count connected to MC33772C. */
#define MC33772_MAX_CELLS        6U
/*! @brief Maximal battery cell count connected to any BCC device. */
#define BCC_MAX_CELLS             14U

/*! @brief Maximal battery cell count connected to the BCC device.
 *
 * @param dev BCC device type.
 */
#define BCC_MAX_CELLS_DEV(dev) \
  ((dev == BCC_DEVICE_MC33771) ? MC33771_MAX_CELLS : MC33772_MAX_CELLS)

/*! @brief  Number of MC33771 measurement registers.
 *
 * Note MC33772 contains 22 measurement registers. For compliance
 * with bcc_measurements_t indexes, BCC_Meas_GetRawValues function
 * requires 30 x uint16_t array for both BCC devices.
 */
#define BCC_MEAS_CNT              30U

/*! @brief  Number of BCC status (fault) registers. */
#define BCC_STAT_CNT              11U

/*! @brief Max. number of frames that can be read at once in TPL mode. */
#define BCC_RX_LIMIT_TPL          0x7FU

/*! @brief Size of buffer that is used for receiving via SPI in TPL mode. */
#define BCC_RX_BUF_SIZE_TPL \
  (BCC_MSG_SIZE * (BCC_RX_LIMIT_TPL + 1U))

/*! @brief Number of GPIO/temperature sensor inputs. */
#define BCC_GPIO_INPUT_CNT        7U

/*!
 * Returns True if value VAL is in the range defined by MIN and MAX values
 * (range includes the border values).
 *
 * @param val Comparison value.
 * @param min Minimal value of the range.
 * @param max Maximal value of the range.
 *
 * @return True if value is the range. False otherwise.
 */
#define BCC_IS_IN_RANGE(val, min, max)   (((val) >= (min)) && ((val) <= (max)))

/* Enum types definition. */
/*!
 * @addtogroup enum_group
 * @{
 */
/*! @brief Error codes. */
typedef enum
{
  BCC_STATUS_SUCCESS        = 0U,   /*!< No error. */
  BCC_STATUS_SPI_INIT       = 1U,   /*!< SPI initialization failure. */
  BCC_STATUS_SPI_BUSY       = 2U,   /*!< SPI instance is busy. */
  BCC_STATUS_PARAM_RANGE    = 4U,   /*!< Parameter out of range. */
  BCC_STATUS_CRC            = 5U,   /*!< Wrong CRC. */
  BCC_STATUS_COM_TAG_ID     = 6U,   /*!< Response Tag ID does not match with provided ID. */
  BCC_STATUS_COM_RC         = 7U,   /*!< Response Rolling Counter (RC) value does not match
                       with expected RC. */
  BCC_STATUS_COM_TIMEOUT    = 8U,   /*!< Communication timeout. */
  BCC_STATUS_DIAG_FAIL      = 9U,   /*!< It is not allowed to enter diagnostic mode. */
  BCC_STATUS_EEPROM_ERROR   = 10U,  /*!< An error occurred during the communication to EEPROM. */
  BCC_STATUS_EEPROM_PRESENT = 11U,  /*!< No EEPROM detected. */
  BCC_STATUS_NULL_RESP      = 12U,  /*!< Response frame of BCC device is equal to zero
                       (except CRC). This occurs only in SPI communication
                       mode during the very first message. */
  /* Custom extensions beyond NXP reference */
  BCC_STATUS_SPI_FAIL       = 13U,  /*!< Fail in the SPI communication. */
  BCC_STATUS_COM_ECHO       = 14U,  /*!< Received "echo" frame from MC33664 does not correspond
                       to the sent frame. */
  BCC_STATUS_COM_MSG_CNT    = 15U,  /*!< Received frame has a valid CRC but the message counter
                       value does not match to the expected one. */
  BCC_STATUS_DATA_RDY       = 16U,  /*!< A new sequence of conversions is currently running. */
  BCC_STATUS_TIMEOUT_START  = 17U   /*!< An error reported in timeout start function. */
} bcc_status_t;

/*!
 * @brief Returns a non-zero value when desired cell (cellNo) is connected
 * to the BCC specified by CID.
 *
 * @param drvConfig Pointer to driver instance configuration.
 * @param cid       Cluster Identification Address of the BCC device.
 * @param cellNo    Cell number (range is {1, ..., 14} for MC33771C and
 *                  {1, ..., 6} for MC33772C).
 *
 * @return Non-zero value if cell is connected, zero otherwise.
 */
#define BCC_IS_CELL_CONN(drvConfig, cid, cellNo) \
  ((drvConfig)->drvData.cellMap[(cid) - 1U] & (1U << ((cellNo) - 1U)))

/*! @brief Maximal frequency of SPI clock in SPI mode. */
#define BCC_SPI_FREQ_MC3377x_MAX  4200000U
/*! @brief Minimal frequency of SPI_TX clock in TPL mode. */
#define BCC_SPI_FREQ_MC33664_MIN  1900000U
/*! @brief Maximal frequency of SPI_TX clock in TPL mode. */
#define BCC_SPI_FREQ_MC33664_MAX  2100000U

/*! @brief  Number of MC33771 registers configured in the initialization with
 * user values.
 *
 * Note that the number of configured registers of MC33772 is 52 only.
 * See BCC_INIT_CONF_REG_ADDR in bcc.c for more details. */
#define BCC_INIT_CONF_REG_CNT     68U

/*! @brief Cluster Identification Address.
 *
 * Note that SPI communication mode allows one cluster/device only.
 * The maximum number of clusters/devices for TPL mode is 63. */
typedef enum
{
  BCC_CID_UNASSIG       = 0U,    /*!< ID of uninitialized BCC devices. */
  BCC_CID_DEV1          = 1U,    /*!< Cluster ID of device 1.
                    In SPI mode, it is the only connected device.
                    In TPL mode, it is the first device in daisy
                    chain (connected directly to MC33664). */
  BCC_CID_DEV2          = 2U,    /*!< Cluster ID of device 2. */
  BCC_CID_DEV3          = 3U,    /*!< Cluster ID of device 3. */
  BCC_CID_DEV4          = 4U,    /*!< Cluster ID of device 4. */
  BCC_CID_DEV5          = 5U,    /*!< Cluster ID of device 5. */
  BCC_CID_DEV6          = 6U,    /*!< Cluster ID of device 6. */
  BCC_CID_DEV7          = 7U,    /*!< Cluster ID of device 7. */
  BCC_CID_DEV8          = 8U,    /*!< Cluster ID of device 8. */
  BCC_CID_DEV9          = 9U,    /*!< Cluster ID of device 9. */
  BCC_CID_DEV10         = 10U,   /*!< Cluster ID of device 10. */
  BCC_CID_DEV11         = 11U,   /*!< Cluster ID of device 11. */
  BCC_CID_DEV12         = 12U,   /*!< Cluster ID of device 12. */
  BCC_CID_DEV13         = 13U,   /*!< Cluster ID of device 13. */
  BCC_CID_DEV14         = 14U,   /*!< Cluster ID of device 14. */
  BCC_CID_DEV15         = 15U,   /*!< Cluster ID of device 15. */
  BCC_CID_DEV16         = 16U,   /*!< Cluster ID of device 16. */
  BCC_CID_DEV17         = 17U,   /*!< Cluster ID of device 17. */
  BCC_CID_DEV18         = 18U,   /*!< Cluster ID of device 18. */
  BCC_CID_DEV19         = 19U,   /*!< Cluster ID of device 19. */
  BCC_CID_DEV20         = 20U,   /*!< Cluster ID of device 20. */
  BCC_CID_DEV21         = 21U,   /*!< Cluster ID of device 21. */
  BCC_CID_DEV22         = 22U,   /*!< Cluster ID of device 22. */
  BCC_CID_DEV23         = 23U,   /*!< Cluster ID of device 23. */
  BCC_CID_DEV24         = 24U,   /*!< Cluster ID of device 24. */
  BCC_CID_DEV25         = 25U,   /*!< Cluster ID of device 25. */
  BCC_CID_DEV26         = 26U,   /*!< Cluster ID of device 26. */
  BCC_CID_DEV27         = 27U,   /*!< Cluster ID of device 27. */
  BCC_CID_DEV28         = 28U,   /*!< Cluster ID of device 28. */
  BCC_CID_DEV29         = 29U,   /*!< Cluster ID of device 29. */
  BCC_CID_DEV30         = 30U,   /*!< Cluster ID of device 30. */
  BCC_CID_DEV31         = 31U,   /*!< Cluster ID of device 31. */
  BCC_CID_DEV32         = 32U,   /*!< Cluster ID of device 32. */
  BCC_CID_DEV33         = 33U,   /*!< Cluster ID of device 33. */
  BCC_CID_DEV34         = 34U,   /*!< Cluster ID of device 34. */
  BCC_CID_DEV35         = 35U,   /*!< Cluster ID of device 35. */
  BCC_CID_DEV36         = 36U,   /*!< Cluster ID of device 36. */
  BCC_CID_DEV37         = 37U,   /*!< Cluster ID of device 37. */
  BCC_CID_DEV38         = 38U,   /*!< Cluster ID of device 38. */
  BCC_CID_DEV39         = 39U,   /*!< Cluster ID of device 39. */
  BCC_CID_DEV40         = 40U,   /*!< Cluster ID of device 40. */
  BCC_CID_DEV41         = 41U,   /*!< Cluster ID of device 41. */
  BCC_CID_DEV42         = 42U,   /*!< Cluster ID of device 42. */
  BCC_CID_DEV43         = 43U,   /*!< Cluster ID of device 43. */
  BCC_CID_DEV44         = 44U,   /*!< Cluster ID of device 44. */
  BCC_CID_DEV45         = 45U,   /*!< Cluster ID of device 45. */
  BCC_CID_DEV46         = 46U,   /*!< Cluster ID of device 46. */
  BCC_CID_DEV47         = 47U,   /*!< Cluster ID of device 47. */
  BCC_CID_DEV48         = 48U,   /*!< Cluster ID of device 48. */
  BCC_CID_DEV49         = 49U,   /*!< Cluster ID of device 49. */
  BCC_CID_DEV50         = 50U,   /*!< Cluster ID of device 50. */
  BCC_CID_DEV51         = 51U,   /*!< Cluster ID of device 51. */
  BCC_CID_DEV52         = 52U,   /*!< Cluster ID of device 52. */
  BCC_CID_DEV53         = 53U,   /*!< Cluster ID of device 53. */
  BCC_CID_DEV54         = 54U,   /*!< Cluster ID of device 54. */
  BCC_CID_DEV55         = 55U,   /*!< Cluster ID of device 55. */
  BCC_CID_DEV56         = 56U,   /*!< Cluster ID of device 56. */
  BCC_CID_DEV57         = 57U,   /*!< Cluster ID of device 57. */
  BCC_CID_DEV58         = 58U,   /*!< Cluster ID of device 58. */
  BCC_CID_DEV59         = 59U,   /*!< Cluster ID of device 59. */
  BCC_CID_DEV60         = 60U,   /*!< Cluster ID of device 60. */
  BCC_CID_DEV61         = 61U,   /*!< Cluster ID of device 61. */
  BCC_CID_DEV62         = 62U,   /*!< Cluster ID of device 62. */
  BCC_CID_DEV63         = 63U    /*!< Cluster ID of device 63. */
} bcc_cid_t;

// define ++ operator for bcc_cid_t to simplify loops
inline bcc_cid_t& operator++(bcc_cid_t& cid)
{
    cid = static_cast<bcc_cid_t>(static_cast<uint8_t>(cid) + 1U);
    return cid;
}

/*! @brief BCC communication mode.  */
typedef enum
{
  BCC_MODE_SPI          = 0U,    /*!< SPI communication mode. */
  BCC_MODE_TPL          = 1U     /*!< TPL communication mode. */
} bcc_mode_t;

/*! @brief BCC device.  */
typedef enum
{
  BCC_DEVICE_MC33771   = 0U,    /*!< MC33771. */
  BCC_DEVICE_MC33772   = 1U     /*!< MC33772. */
} bcc_device_t;

/*! @brief Measurements provided by battery cell controller.
 *
 * Note that MC33772 doesn't have MEAS_CELL7 - MEAS_CELL14 registers.
 * Function BCC_Meas_GetRawValues returns 0x0000 at these positions.
 */
typedef enum
{
  BCC_MSR_CC_NB_SAMPLES = 0U,   /*!< Number of samples in Coulomb counter (register CC_NB_SAMPLES). */
  BCC_MSR_COULOMB_CNT1  = 1U,   /*!< Coulomb counting accumulator (register COULOMB_CNT1). */
  BCC_MSR_COULOMB_CNT2  = 2U,   /*!< Coulomb counting accumulator (register COULOMB_CNT2). */
  BCC_MSR_ISENSE1       = 3U,   /*!< ISENSE measurement (register MEAS_ISENSE1). */
  BCC_MSR_ISENSE2       = 4U,   /*!< ISENSE measurement (register MEAS_ISENSE2). */
  BCC_MSR_STACK_VOLT    = 5U,   /*!< Stack voltage measurement (register MEAS_STACK). */
  BCC_MSR_CELL_VOLT14   = 6U,   /*!< Cell 14 voltage measurement (register MEAS_CELL14). */
  BCC_MSR_CELL_VOLT13   = 7U,   /*!< Cell 13 voltage measurement (register MEAS_CELL13). */
  BCC_MSR_CELL_VOLT12   = 8U,   /*!< Cell 12 voltage measurement (register MEAS_CELL12). */
  BCC_MSR_CELL_VOLT11   = 9U,   /*!< Cell 11 voltage measurement (register MEAS_CELL11). */
  BCC_MSR_CELL_VOLT10   = 10U,  /*!< Cell 10 voltage measurement (register MEAS_CELL10). */
  BCC_MSR_CELL_VOLT9    = 11U,  /*!< Cell 9 voltage measurement (register MEAS_CELL9). */
  BCC_MSR_CELL_VOLT8    = 12U,  /*!< Cell 8 voltage measurement (register MEAS_CELL8). */
  BCC_MSR_CELL_VOLT7    = 13U,  /*!< Cell 7 voltage measurement (register MEAS_CELL7). */
  BCC_MSR_CELL_VOLT6    = 14U,  /*!< Cell 6 voltage measurement (register MEAS_CELL6). */
  BCC_MSR_CELL_VOLT5    = 15U,  /*!< Cell 5 voltage measurement (register MEAS_CELL5). */
  BCC_MSR_CELL_VOLT4    = 16U,  /*!< Cell 4 voltage measurement (register MEAS_CELL4). */
  BCC_MSR_CELL_VOLT3    = 17U,  /*!< Cell 3 voltage measurement (register MEAS_CELL3). */
  BCC_MSR_CELL_VOLT2    = 18U,  /*!< Cell 2 voltage measurement (register MEAS_CELL2). */
  BCC_MSR_CELL_VOLT1    = 19U,  /*!< Cell 1 voltage measurement (register MEAS_CELL1). */
  BCC_MSR_AN6           = 20U,  /*!< Analog input 6 voltage measurement (register MEAS_AN6). */
  BCC_MSR_AN5           = 21U,  /*!< Analog input 5 voltage measurement (register MEAS_AN5). */
  BCC_MSR_AN4           = 22U,  /*!< Analog input 4 voltage measurement (register MEAS_AN4). */
  BCC_MSR_AN3           = 23U,  /*!< Analog input 3 voltage measurement (register MEAS_AN3). */
  BCC_MSR_AN2           = 24U,  /*!< Analog input 2 voltage measurement (register MEAS_AN2). */
  BCC_MSR_AN1           = 25U,  /*!< Analog input 1 voltage measurement (register MEAS_AN1). */
  BCC_MSR_AN0           = 26U,  /*!< Analog input 0 voltage measurement (register MEAS_AN0). */
  BCC_MSR_ICTEMP        = 27U,  /*!< IC temperature measurement (register MEAS_IC_TEMP). */
  BCC_MSR_VBGADC1A      = 28U,  /*!< ADCIA Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1A). */
  BCC_MSR_VBGADC1B      = 29U   /*!< ADCIB Band Gap Reference measurement (register MEAS_VBG_DIAG_ADC1B). */
} bcc_measurements_t;

/*! @brief Number of samples to be averaged in the conversion request. */
typedef enum
{
  BCC_AVG_1             = MC33771C_ADC_CFG_AVG_NO_AVERAGING_ENUM_VAL, /*!< No averaging, the result is taken as is. */
  BCC_AVG_2             = MC33771C_ADC_CFG_AVG_2_SAMPLES_ENUM_VAL,    /*!< Averaging of 2 consecutive samples. */
  BCC_AVG_4             = MC33771C_ADC_CFG_AVG_4_SAMPLES_ENUM_VAL,    /*!< Averaging of 4 consecutive samples. */
  BCC_AVG_8             = MC33771C_ADC_CFG_AVG_8_SAMPLES_ENUM_VAL,    /*!< Averaging of 8 consecutive samples. */
  BCC_AVG_16            = MC33771C_ADC_CFG_AVG_16_SAMPLES_ENUM_VAL,   /*!< Averaging of 16 consecutive samples. */
  BCC_AVG_32            = MC33771C_ADC_CFG_AVG_32_SAMPLES_ENUM_VAL,   /*!< Averaging of 32 consecutive samples. */
  BCC_AVG_64            = MC33771C_ADC_CFG_AVG_64_SAMPLES_ENUM_VAL,   /*!< Averaging of 64 consecutive samples. */
  BCC_AVG_128           = MC33771C_ADC_CFG_AVG_128_SAMPLES_ENUM_VAL,  /*!< Averaging of 126 consecutive samples. */
  BCC_AVG_256           = MC33771C_ADC_CFG_AVG_256_SAMPLES_ENUM_VAL   /*!< Averaging of 256 consecutive samples. */
} bcc_avg_t;

/*! @brief Status provided by battery cell controller. */
typedef enum
{
  BCC_FS_CELL_OV        = 0U,   /*!< CT overvoltage fault (register CELL_OV_FLT). */
  BCC_FS_CELL_UV        = 1U,   /*!< CT undervoltage fault (register CELL_UV_FLT). */
  BCC_FS_CB_OPEN        = 2U,   /*!< Open CB fault (register CB_OPEN_FLT). */
  BCC_FS_CB_SHORT       = 3U,   /*!< Short CB fault (register CB_SHORT_FLT). */
  BCC_FS_GPIO_STATUS    = 4U,   /*!< GPIO status (register GPIO_STS). */
  BCC_FS_AN_OT_UT       = 5U,   /*!< AN over and undertemperature (register AN_OT_UT_FLT). */
  BCC_FS_GPIO_SHORT     = 6U,   /*!< Short GPIO/open AN diagnostic (register GPIO_SHORT_ANx_OPEN_STS). */
  BCC_FS_COMM           = 7U,   /*!< Number of communication errors detected (register COM_STATUS). */
  BCC_FS_FAULT1         = 8U,   /*!< Fault status (register FAULT1_STATUS). */
  BCC_FS_FAULT2         = 9U,   /*!< Fault status (register FAULT2_STATUS). */
  BCC_FS_FAULT3         = 10U   /*!< Fault status (register FAULT3_STATUS). */
} bcc_fault_status_t;

/*! @brief Mode of a GPIOx/ANx pin. */
typedef enum
{
  BCC_PIN_ANALOG_IN_RATIO = 0U,   /*!< Analog input for ratiometric measurement). */
  BCC_PIN_ANALOG_IN_ABS   = 1U,   /*!< Analog input for absolute measurement. */
  BCC_PIN_DIGITAL_IN      = 2U,   /*!< Digital input. */
  BCC_PIN_DIGITAL_OUT     = 3U,   /*!< Digital output. */
  BCC_PIN_WAKE_UP_IN      = 4U,   /*!< Digital input with a wake-up capability (only GPIO0).
                     Wakes-up BCC from sleep to normal mode on any edge of GPIO0. */
  BCC_PIN_CONVERT_TR_IN   = 5U    /*!< Digital input with a convert trigger capability (only GPIO2).
                     A rising edge on GPIO2 triggers an ADC1-A and ADC1-B
                     conversion when BCC is in normal mode. */
} bcc_pin_mode_t;

/*! @brief Unit of the result from BCC_Meas_GetIcTemperature function. */
typedef enum
{
  BCC_TEMP_KELVIN         = 0U,   /*!< Result resolution: 0.1 K. */
  BCC_TEMP_CELSIUS        = 1U,   /*!< Result resolution: 0.1 �C. */
  BCC_TEMP_FAHRENHEIT     = 2U    /*!< Result resolution: 0.1 �F. */
} bcc_temp_unit_t;
/*! @} */

/* Configure struct types definition. */
/*!
 * @addtogroup struct_group
 * @{
 */
/*! @brief Coulomb counter data. */
typedef struct
{
  int32_t ccAccumulator;                   /*!< Coulomb counting accumulator with V_2RES resolution. */
  uint16_t nbSamples;                      /*!< Number of samples accumulated in the Coulomb counter. */
} bcc_cc_data_t;

/*!
 * @brief Driver internal data.
 *
 * Note that it is initialized in BCC_Init function by the driver
 * and the user mustn't change it at any time.
 */
typedef struct
{
  uint16_t cellMap[BCC_DEVICE_CNT_MAX];    /*!< Bit map of used cells of each BCC device. */
  uint8_t rcTbl[BCC_DEVICE_CNT_MAX];       /*!< Rolling counter index (0-4). */
  uint8_t tagId[BCC_DEVICE_CNT_MAX];       /*!< TAG IDs of BCC devices. */
  uint8_t rxBuf[BCC_RX_BUF_SIZE_TPL];      /*!< Buffer for receiving data in TPL mode. */
} bcc_drv_data_t;

/*! @brief Index to memory data field of SPI frame (higher byte). */
#define BCC_MSG_IDX_DATA_H        0U
/*! @brief Index to memory data field of SPI frame (lower byte). */
#define BCC_MSG_IDX_DATA_L        1U
/*! @brief Index to memory address field of SPI frame. */
#define BCC_MSG_IDX_ADDR          2U
/*! @brief Index to physical address (CID) and command fields of SPI frame. */
#define BCC_MSG_IDX_CID_CMD       3U
/*! @brief Index to CRC field of SPI frame. */
#define BCC_MSG_IDX_CRC           4U

/** BCC Commands. */
/*! @brief No operation command. */
#define BCC_CMD_NOOP              0x00U
/*! @brief Read command. */
#define BCC_CMD_READ              0x01U
/*! @brief Write command. */
#define BCC_CMD_WRITE             0x02U
/*! @brief Global write command. */
#define BCC_CMD_GLOB_WRITE        0x03U

/*!
 * @brief Returns Memory Data field from a frame.
 *
 * @param msg Pointer to the frame.
 * @return Memory data field.
 */
#define BCC_GET_MSG_DATA(msg) \
  (((uint16_t)*((msg) + BCC_MSG_IDX_DATA_H) << 8U) | \
    (uint16_t)*((msg) + BCC_MSG_IDX_DATA_L))

/*! @brief Mask for memory address field of frame. */
#define BCC_MSG_ADDR_MASK   0x7FU
/*! @brief Mask for RC field of frame. */
#define BCC_MSG_RC_MASK     0x0CU
/*! @brief Mask for TAG ID field of frame. */
#define BCC_MSG_TAGID_MASK  0x0FU

/*!
 * @brief Converts binary value to Gray code used for Rolling Counter.
 * It is a 2-bit Gray code. Final value is shifted left by 2 bits.
 *
 * @param binVal Binary value to be converted.
 * @return Converted value shifted left by 2 bits.
 */
#define BCC_GET_RC(binVal) \
  ((((binVal) >> 1) ^ (binVal)) << 2)

/*!
 * @brief Increments rcIdx value and executes modulo 4.
 *
 * @param rcIdx Index to be incremented.
 * @return Incremented rcIdx value.
 */
#define BCC_INC_RC_IDX(rcIdx) \
  (((rcIdx) + 1U) & 0x03U)