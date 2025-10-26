#include "BatteryCellController.h"
#include "bcc/bcc_defs.h"

#if ARDUINO_ARCH_STM32
#define assert(expr) assert_param(expr)
#endif

/** Addresses of configurable registers.
 *
 * Note that MC33772 does not have CB7_CFG - CB14_CFG (0x12 - 0x19) and
 * TH_CT14 - TH_CT7 (0x4c - 0x53) registers. These values are ignored
 * in the initialization. */
static const uint8_t BCC_INIT_CONF_REG_ADDR[BCC_INIT_CONF_REG_CNT] = {
    /* Note: INIT register is initialized automatically. SYS_CFG_GLOBAL register
     *       contains only command GO2SLEEP (no initialization needed). */

    BCC_REG_GPIO_CFG1_ADDR,
    BCC_REG_GPIO_CFG2_ADDR,
    BCC_REG_TH_ALL_CT_ADDR,
    BCC_REG_TH_CT14_ADDR,
    BCC_REG_TH_CT13_ADDR,
    BCC_REG_TH_CT12_ADDR,
    BCC_REG_TH_CT11_ADDR,
    BCC_REG_TH_CT10_ADDR,
    BCC_REG_TH_CT9_ADDR,
    BCC_REG_TH_CT8_ADDR,
    BCC_REG_TH_CT7_ADDR,
    BCC_REG_TH_CT6_ADDR,
    BCC_REG_TH_CT5_ADDR,
    BCC_REG_TH_CT4_ADDR,
    BCC_REG_TH_CT3_ADDR,
    BCC_REG_TH_CT2_ADDR,
    BCC_REG_TH_CT1_ADDR,
    BCC_REG_TH_AN6_OT_ADDR,
    BCC_REG_TH_AN5_OT_ADDR,
    BCC_REG_TH_AN4_OT_ADDR,
    BCC_REG_TH_AN3_OT_ADDR,
    BCC_REG_TH_AN2_OT_ADDR,
    BCC_REG_TH_AN1_OT_ADDR,
    BCC_REG_TH_AN0_OT_ADDR,
    BCC_REG_TH_AN6_UT_ADDR,
    BCC_REG_TH_AN5_UT_ADDR,
    BCC_REG_TH_AN4_UT_ADDR,
    BCC_REG_TH_AN3_UT_ADDR,
    BCC_REG_TH_AN2_UT_ADDR,
    BCC_REG_TH_AN1_UT_ADDR,
    BCC_REG_TH_AN0_UT_ADDR,
    BCC_REG_TH_ISENSE_OC_ADDR,
    BCC_REG_TH_COULOMB_CNT_MSB_ADDR,
    BCC_REG_TH_COULOMB_CNT_LSB_ADDR,
    BCC_REG_CB1_CFG_ADDR,
    BCC_REG_CB2_CFG_ADDR,
    BCC_REG_CB3_CFG_ADDR,
    BCC_REG_CB4_CFG_ADDR,
    BCC_REG_CB5_CFG_ADDR,
    BCC_REG_CB6_CFG_ADDR,
    BCC_REG_CB7_CFG_ADDR,
    BCC_REG_CB8_CFG_ADDR,
    BCC_REG_CB9_CFG_ADDR,
    BCC_REG_CB10_CFG_ADDR,
    BCC_REG_CB11_CFG_ADDR,
    BCC_REG_CB12_CFG_ADDR,
    BCC_REG_CB13_CFG_ADDR,
    BCC_REG_CB14_CFG_ADDR,
    BCC_REG_OV_UV_EN_ADDR,
    BCC_REG_SYS_CFG1_ADDR,
    BCC_REG_SYS_CFG2_ADDR,
    BCC_REG_ADC_CFG_ADDR,
    BCC_REG_ADC2_OFFSET_COMP_ADDR,
    BCC_REG_FAULT_MASK1_ADDR,
    BCC_REG_FAULT_MASK2_ADDR,
    BCC_REG_FAULT_MASK3_ADDR,
    BCC_REG_WAKEUP_MASK1_ADDR,
    BCC_REG_WAKEUP_MASK2_ADDR,
    BCC_REG_WAKEUP_MASK3_ADDR,
    BCC_REG_CELL_OV_FLT_ADDR,
    BCC_REG_CELL_UV_FLT_ADDR,
    BCC_REG_AN_OT_UT_FLT_ADDR,
    BCC_REG_CB_SHORT_FLT_ADDR,
    BCC_REG_GPIO_STS_ADDR,
    BCC_REG_GPIO_SHORT_ADDR,
    BCC_REG_FAULT1_STATUS_ADDR,
    BCC_REG_FAULT2_STATUS_ADDR,
    BCC_REG_FAULT3_STATUS_ADDR,
};

static const uint8_t BCC_CRC_TABLE[BCC_CRC_TBL_SIZE] = {
    0x00U, 0x2fU, 0x5eU, 0x71U, 0xbcU, 0x93U, 0xe2U, 0xcdU,
    0x57U, 0x78U, 0x09U, 0x26U, 0xebU, 0xc4U, 0xb5U, 0x9aU,
    0xaeU, 0x81U, 0xf0U, 0xdfU, 0x12U, 0x3dU, 0x4cU, 0x63U,
    0xf9U, 0xd6U, 0xa7U, 0x88U, 0x45U, 0x6aU, 0x1bU, 0x34U,
    0x73U, 0x5cU, 0x2dU, 0x02U, 0xcfU, 0xe0U, 0x91U, 0xbeU,
    0x24U, 0x0bU, 0x7aU, 0x55U, 0x98U, 0xb7U, 0xc6U, 0xe9U,
    0xddU, 0xf2U, 0x83U, 0xacU, 0x61U, 0x4eU, 0x3fU, 0x10U,
    0x8aU, 0xa5U, 0xd4U, 0xfbU, 0x36U, 0x19U, 0x68U, 0x47U,
    0xe6U, 0xc9U, 0xb8U, 0x97U, 0x5aU, 0x75U, 0x04U, 0x2bU,
    0xb1U, 0x9eU, 0xefU, 0xc0U, 0x0dU, 0x22U, 0x53U, 0x7cU,
    0x48U, 0x67U, 0x16U, 0x39U, 0xf4U, 0xdbU, 0xaaU, 0x85U,
    0x1fU, 0x30U, 0x41U, 0x6eU, 0xa3U, 0x8cU, 0xfdU, 0xd2U,
    0x95U, 0xbaU, 0xcbU, 0xe4U, 0x29U, 0x06U, 0x77U, 0x58U,
    0xc2U, 0xedU, 0x9cU, 0xb3U, 0x7eU, 0x51U, 0x20U, 0x0fU,
    0x3bU, 0x14U, 0x65U, 0x4aU, 0x87U, 0xa8U, 0xd9U, 0xf6U,
    0x6cU, 0x43U, 0x32U, 0x1dU, 0xd0U, 0xffU, 0x8eU, 0xa1U,
    0xe3U, 0xccU, 0xbdU, 0x92U, 0x5fU, 0x70U, 0x01U, 0x2eU,
    0xb4U, 0x9bU, 0xeaU, 0xc5U, 0x08U, 0x27U, 0x56U, 0x79U,
    0x4dU, 0x62U, 0x13U, 0x3cU, 0xf1U, 0xdeU, 0xafU, 0x80U,
    0x1aU, 0x35U, 0x44U, 0x6bU, 0xa6U, 0x89U, 0xf8U, 0xd7U,
    0x90U, 0xbfU, 0xceU, 0xe1U, 0x2cU, 0x03U, 0x72U, 0x5dU,
    0xc7U, 0xe8U, 0x99U, 0xb6U, 0x7bU, 0x54U, 0x25U, 0x0aU,
    0x3eU, 0x11U, 0x60U, 0x4fU, 0x82U, 0xadU, 0xdcU, 0xf3U,
    0x69U, 0x46U, 0x37U, 0x18U, 0xd5U, 0xfaU, 0x8bU, 0xa4U,
    0x05U, 0x2aU, 0x5bU, 0x74U, 0xb9U, 0x96U, 0xe7U, 0xc8U,
    0x52U, 0x7dU, 0x0cU, 0x23U, 0xeeU, 0xc1U, 0xb0U, 0x9fU,
    0xabU, 0x84U, 0xf5U, 0xdaU, 0x17U, 0x38U, 0x49U, 0x66U,
    0xfcU, 0xd3U, 0xa2U, 0x8dU, 0x40U, 0x6fU, 0x1eU, 0x31U,
    0x76U, 0x59U, 0x28U, 0x07U, 0xcaU, 0xe5U, 0x94U, 0xbbU,
    0x21U, 0x0eU, 0x7fU, 0x50U, 0x9dU, 0xb2U, 0xc3U, 0xecU,
    0xd8U, 0xf7U, 0x86U, 0xa9U, 0x64U, 0x4bU, 0x3aU, 0x15U,
    0x8fU, 0xa0U, 0xd1U, 0xfeU, 0x33U, 0x1cU, 0x6dU, 0x42U
};

// Define static members
BatteryCellController* BatteryCellController::instances[2] = {nullptr, nullptr};

void BatteryCellController::isr_wrapper_0() {
  if (instances[0] != nullptr) {
    uint8_t state = digitalRead(instances[0]->intb_pin);
    instances[0]->interrupt_events.push(state);
    // Note: Avoid Serial.print in ISR - it can cause issues
  }
}

void BatteryCellController::isr_wrapper_1() {
  if (instances[1] != nullptr) {
    uint8_t state = digitalRead(instances[1]->intb_pin);
    instances[1]->interrupt_events.push(state);
    // Note: Avoid Serial.print in ISR - it can cause issues
  }
}

BatteryCellController::BatteryCellController(SPIClass *spi, bcc_device_t device, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, uint8_t cs_pin, bool loopback) {
  this->spi = spi;

  this->comm_mode = BCC_MODE_SPI;
  this->devices[0] = device;
  this->device_count = 1;
  this->cell_count = cell_count;
  this->loopback = loopback;

  this->enable_pin = enable_pin;
  this->intb_pin = intb_pin;
  this->cs_tx_pin = cs_pin;

  pinMode(this->enable_pin, OUTPUT);
  digitalWrite(this->enable_pin, HIGH);

  pinMode(this->intb_pin, INPUT_PULLUP);  // Try INPUT or INPUT_PULLDOWN if this doesn't work
  int_state = digitalRead(this->intb_pin);

  // Register instance in first available slot
  if (instances[0] == nullptr) {
    instances[0] = this;
    attachInterrupt(this->intb_pin, isr_wrapper_0, CHANGE);
  } else if (instances[1] == nullptr) {
    instances[1] = this;
    attachInterrupt(this->intb_pin, isr_wrapper_1, CHANGE);
  } else {
    Serial.println("ERROR: Maximum 2 BCC instances supported");
  }
}

BatteryCellController::BatteryCellController(TPLSPI *tpl, bcc_device_t devices[], uint8_t device_count, uint8_t cell_count, uint8_t enable_pin, uint8_t intb_pin, bool loopback) {
  this->tpl = tpl;
  this->tpl->begin();

  for (int i = 0; i < device_count; i++) {
    this->devices[i] = devices[i];
  }

  this->device_count = device_count;

  this->cell_count = cell_count;
  this->comm_mode = BCC_MODE_TPL;
  this->loopback = loopback;

  this->enable_pin = enable_pin;
  this->intb_pin = intb_pin;
  this->cs_tx_pin = tpl->getCSPin();

  pinMode(this->enable_pin, OUTPUT);
  digitalWrite(this->enable_pin, HIGH);

  pinMode(this->intb_pin, INPUT_PULLUP);
  int_state = digitalRead(this->intb_pin);

  // Register instance in first available slot
  if (instances[0] == nullptr) {
    instances[0] = this;
    Serial.println("BCC: Registered instance 0");
    attachInterrupt(this->intb_pin, isr_wrapper_0, CHANGE);
  } else if (instances[1] == nullptr) {
    instances[1] = this;
    Serial.println("BCC: Registered instance 1");
    attachInterrupt(this->intb_pin, isr_wrapper_1, CHANGE);
  } else {
    Serial.println("ERROR: Maximum 2 BCC instances supported");
  }
}

void BatteryCellController::on_interrupt() {
  uint8_t current_state = digitalRead(this->intb_pin);
  int_state = current_state;
  interrupt_events.push(current_state);
}

void BatteryCellController::reset_interrupt_events() {
  noInterrupts();
  while (!interrupt_events.empty()) {
    interrupt_events.pop();
  }
  interrupts();
}

bcc_status_t BatteryCellController::begin(const uint16_t devConf[][BCC_INIT_CONF_REG_CNT]) {
  uint8_t device;
  bcc_status_t status;

  if ((this->comm_mode != BCC_MODE_SPI) && (this->comm_mode != BCC_MODE_TPL)) {
    return BCC_STATUS_PARAM_RANGE;
  }

  if ((this->device_count == 0) || (this->device_count > ((this->comm_mode == BCC_MODE_SPI) ? BCC_DEVICE_CNT_MAX_SPI : BCC_DEVICE_CNT_MAX_TPL))) {
    return BCC_STATUS_PARAM_RANGE;
  }

  // Validate device types and cell counts
  for (device = 0; device < this->device_count; device++) {
    if (this->devices[device] == BCC_DEVICE_MC33771) {
      if (!BCC_IS_IN_RANGE(this->cell_count, MC33771_MIN_CELLS, MC33771_MAX_CELLS)) {
        return BCC_STATUS_PARAM_RANGE;
      }
    } else if (this->devices[device] == BCC_DEVICE_MC33772) {
      if (!BCC_IS_IN_RANGE(this->cell_count, MC33772_MIN_CELLS, MC33772_MAX_CELLS)) {
        return BCC_STATUS_PARAM_RANGE;
      }
    } else {
      return BCC_STATUS_PARAM_RANGE;
    }
  }

  // Build cell map bitmap for each device (matches NXP BCC_Init)
  for (device = 0; device < this->device_count; device++) {
    uint8_t cell;
    if (this->devices[device] == BCC_DEVICE_MC33771) {
      // Start with base 7-cell pattern, then add higher cells
      this->cell_map[device] = 0x007F;  // Base: cells 1-7
      for (cell = MC33771_MIN_CELLS; cell < this->cell_count; cell++) {
        this->cell_map[device] |= (0x0400U >> (cell - MC33771_MIN_CELLS));
      }
    } else {
      // MC33772C: Start with base 3-cell pattern, then add higher cells
      this->cell_map[device] = 0x0007;  // Base: cells 1-3
      for (cell = MC33772_MIN_CELLS; cell < this->cell_count; cell++) {
        this->cell_map[device] |= (0x0010U >> (cell - MC33772_MIN_CELLS));
      }
    }
  }

  // Initialize Rolling Counter and TAG ID tables
  for (device = 0; device < this->device_count; device++) {
    this->rc_table[device] = 0U;
    this->tag_id[device] = 0U;
  }

  /* Enable MC33664 device in TPL mode. */
  if (this->comm_mode == BCC_MODE_TPL) {
    if ((status = this->enable_tpl()) != BCC_STATUS_SUCCESS) {
      Serial.printf("TPL mode enable failed: %d\r\n", status);
      return status;
    }
  }

  Serial.println("Initializing devices...");

  return init_devices(devConf);
}

bcc_status_t BatteryCellController::enable_tpl() {
  int32_t timeout;

  // Clear any existing interrupt events before starting
  reset_interrupt_events();

  /* Set normal state (transition from low to high). */
  digitalWrite(this->enable_pin, LOW);
  /* Wait at least 100 us. */
  delayMicroseconds(150);
  digitalWrite(this->enable_pin, HIGH);

  /* Note: MC33664 has time t_Ready/t_INTB_PULSE_DELAY (max. 100 us) to take effect.
  * Wait for INTB transition from high to low (max. 100 us). */
  timeout = 200;
  bool queue_empty = true;
  while (queue_empty && timeout > 0) {
    __DSB();  // Data Synchronization Barrier - ensure ISR writes are visible
    noInterrupts();
    queue_empty = interrupt_events.empty();
    interrupts();
    if (queue_empty) {
      delayMicroseconds(1);
      timeout--;
    }
  }

  noInterrupts();
  bool is_empty = interrupt_events.empty();
  interrupts();

  if (is_empty) {
    Serial.println("Timeout waiting for INTB low after enabling TPL.");
    return BCC_STATUS_COM_TIMEOUT;
  }

  noInterrupts();
  interrupt_events.pop();
  interrupts();

  reset_interrupt_events();

  /* Wait for INTB transition from low to high (typ. 100 us).
  * Wait for at most 200 us. */
  timeout = 200;
  queue_empty = true;
  while (queue_empty && timeout > 0) {
    __DSB();  // Data Synchronization Barrier - ensure ISR writes are visible
    noInterrupts();
    queue_empty = interrupt_events.empty();
    interrupts();
    if (queue_empty) {
      delayMicroseconds(1);
      timeout--;
    }
  }

  noInterrupts();
  is_empty = interrupt_events.empty();
  interrupts();

  if (is_empty) {
    Serial.println("Timeout waiting for INTB high after enabling TPL.");
    return BCC_STATUS_COM_TIMEOUT;
  }

  noInterrupts();
  interrupt_events.pop();
  interrupts();

  reset_interrupt_events();

  /* Now the device should be in normal mode (i.e. after INTB low to high
  * transition). For sure wait for 150 us. */
  delayMicroseconds(150);

  return BCC_STATUS_SUCCESS;
}

void BatteryCellController::disable_tpl() {
  digitalWrite(this->enable_pin, LOW);
}

bcc_status_t BatteryCellController::init_devices(const uint16_t devConf[][BCC_INIT_CONF_REG_CNT]) {
  uint8_t device;
  bcc_status_t status;

  Serial.println("Waking up devices...");
  wake_up();

  Serial.println("Running software reset...");
  status = software_reset((comm_mode == BCC_MODE_TPL) ? BCC_CID_UNASSIG : BCC_CID_DEV1);
  if (status != BCC_STATUS_SUCCESS) {
    Serial.printf("Software reset failed: %d\r\n", status);
    return status;
  }

  delay(BCC_T_VPWR_READY_MS);

  Serial.println("Assigning CIDs...");
  status = assign_cid(BCC_CID_DEV1);
  if (status != BCC_STATUS_SUCCESS) {
    Serial.printf("CID assignment failed: %d\r\n", status);
    return status;
  }

  // Assign CIDs to remaining devices (CID 2 to devicesCnt)
  for (device = 2; device <= device_count; device++) {
    delay(2);

    // Wake up the next device in the daisy chain
    wake_up_pattern_tpl();

    status = assign_cid((bcc_cid_t)device);
    if (status != BCC_STATUS_SUCCESS) {
      Serial.printf("CID assignment failed for device %d: %d\r\n", device, status);
      return status;
    }
  }

  // Initialize registers if configuration is provided (matches NXP BCC_Init pattern)
  if (devConf != nullptr) {
    Serial.println("Initializing registers...");
    status = init_registers(devConf);
    if (status != BCC_STATUS_SUCCESS) {
      Serial.printf("Register initialization failed: %d\r\n", status);
      return status;
    }
  }

  return status;
}

bcc_status_t BatteryCellController::software_reset(bcc_cid_t cid) {
  bcc_status_t status;

  if ((((uint8_t)cid) > device_count) ||
    ((cid == BCC_CID_UNASSIG) && (comm_mode == BCC_MODE_SPI)))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (cid == BCC_CID_UNASSIG) {
    // TPL Global reset command
    status = write_register_global(BCC_REG_SYS_CFG1_ADDR, BCC_W_SOFT_RST_MASK);
  }
  else {
    status = write_register(cid, BCC_REG_SYS_CFG1_ADDR, BCC_W_SOFT_RST_MASK, NULL);
    // Device does not respond after reset - this is normal
    if (status == BCC_STATUS_COM_TIMEOUT) {
      status = BCC_STATUS_SUCCESS;
    }
  }

  return status;
}

void BatteryCellController::wake_up_pattern_spi() {
  digitalWrite(this->cs_tx_pin, LOW);
  delayMicroseconds(BCC_CSB_WU_FLT_US);

  digitalWrite(this->cs_tx_pin, HIGH);
  delayMicroseconds(BCC_T_WAKE_UP_US);
}

void BatteryCellController::wake_up_pattern_tpl() {
  digitalWrite(this->cs_tx_pin, LOW);
  delayMicroseconds(BCC_WAKE_PULSE_US);

  digitalWrite(this->cs_tx_pin, HIGH);
  delayMicroseconds(BCC_T_WAKE_DELAY_US);

  digitalWrite(this->cs_tx_pin, LOW);
  delayMicroseconds(BCC_WAKE_PULSE_US);

  digitalWrite(this->cs_tx_pin, HIGH);
  delayMicroseconds(BCC_T_WU_WAIT_US * this->cell_count);
}

bcc_status_t BatteryCellController::init_registers(const uint16_t devConf[][BCC_INIT_CONF_REG_CNT]) {
  uint8_t i, cid;
  bcc_status_t error;

  /* Initialize all registers according to according to the user values. */
  for (cid = 1; cid <= device_count; cid++)
  {
    for (i = 0; i < BCC_INIT_CONF_REG_CNT; i++)
    {
      if (devices[cid - 1U] == BCC_DEVICE_MC33772)
      {
        if (BCC_IS_IN_RANGE(BCC_INIT_CONF_REG_ADDR[i], BCC_REG_CB7_CFG_ADDR, BCC_REG_CB14_CFG_ADDR) ||
          BCC_IS_IN_RANGE(BCC_INIT_CONF_REG_ADDR[i], BCC_REG_TH_CT14_ADDR, BCC_REG_TH_CT7_ADDR))
        {
          continue;
        }
      }

      error = write_register((bcc_cid_t)cid, BCC_INIT_CONF_REG_ADDR[i], devConf[cid - 1U][i], nullptr);
      if (error != BCC_STATUS_SUCCESS)
      {
        return error;
      }
    }
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::update_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t mask, uint16_t val) {
  uint16_t temp_reg_value;
  bcc_status_t status;

  if (((uint8_t)cid) > device_count)
    return BCC_STATUS_PARAM_RANGE;

  status = read_register(cid, reg_addr, 1U, &temp_reg_value);
  if (status != BCC_STATUS_SUCCESS)
    return status;

  temp_reg_value = BCC_REG_UNSET_BIT_VALUE(temp_reg_value, mask);
  temp_reg_value = BCC_REG_SET_BIT_VALUE(temp_reg_value, (val & mask));

  status = write_register(cid, reg_addr, temp_reg_value, nullptr);
  return status;
}

void BatteryCellController::wake_up() {
  if (this->comm_mode == BCC_MODE_SPI) {
    wake_up_pattern_spi();
  } else {
    wake_up_pattern_tpl();
  }
}

bcc_status_t BatteryCellController::assign_cid(bcc_cid_t cid) {
  uint16_t writeVal, readVal;
  bcc_status_t error;

  /* Check if unassigned node replies. This is the first reading after device
    * reset. */
  Serial.println("Checking for unassigned CID...");
  error = read_register(BCC_CID_UNASSIG, BCC_REG_INIT_ADDR, 1U, &readVal);

  /* Note: in SPI communication mode the device responds with all zero and the
    * correct CRC (null response) during the very first message. */
  if ((error != BCC_STATUS_SUCCESS) && (error != BCC_STATUS_NULL_RESP))
    return error;

  Serial.printf("Unassigned CID response value: %d\r\n", readVal);

  /* Assign CID and close the bus switch to be able to initialize next BCC
    * device. Bus switch of the last BCC in chain stays opened. In SPI mode,
    * just CID needs to be written.
    * Note: It is forbidden to use global write command to assign CID (writing
    * into INIT register). */
  if ((uint8_t)cid < device_count)
    writeVal = BCC_SET_CID(readVal, (uint8_t)cid) | BCC_BUS_SWITCH_ENABLED | BCC_RTERM_COMM_SW;
  else
    writeVal = BCC_SET_CID(readVal, (uint8_t)cid) | BCC_BUS_SWITCH_DISABLED | BCC_RTERM_COMM_SW;

  Serial.printf("Assigning CID %d with value %d...\r\n", cid, writeVal);
  error = write_register(cid, BCC_REG_INIT_ADDR, writeVal, NULL);
  if (error == BCC_STATUS_SUCCESS)
  {
      /* Check if assigned node replies. */
      Serial.printf("Verifying CID %d...\r\n", cid);
      error = read_register(cid, BCC_REG_INIT_ADDR, 1U, &readVal);
  }

  if (error != BCC_STATUS_SUCCESS)
  {
      Serial.printf("CID assignment failed, retrying...\r\n");
      /* Wait and try to assign CID once again. */
      delayMicroseconds(750U);

      Serial.printf("Retrying CID assignment...\r\n");
      error = write_register(BCC_CID_UNASSIG, BCC_REG_INIT_ADDR, writeVal, NULL);
      if (error == BCC_STATUS_SUCCESS)
      {
          /* Check if assigned node replies. */
          Serial.println("Verifying CID on retry...");
          error = read_register(cid, BCC_REG_INIT_ADDR, 1U, &readVal);
      }
  }

  return error;
}

bcc_status_t BatteryCellController::read_register(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_cnt, uint16_t *reg_val) {
  bcc_status_t status;

  switch (comm_mode) {
    case BCC_MODE_SPI:
      status = read_register_spi(cid, reg_addr, reg_cnt, reg_val);
      break;
    case BCC_MODE_TPL:
      status = read_register_tpl(cid, reg_addr, reg_cnt, reg_val);
      break;
  }

  return status;
}

bcc_status_t BatteryCellController::write_register(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val, uint16_t* ret_reg) {
  bcc_status_t status;

  switch (comm_mode) {
    case BCC_MODE_SPI:
      status = write_register_spi(cid, reg_addr, reg_val, ret_reg);
      break;
    case BCC_MODE_TPL:
      status = write_register_tpl(cid, reg_addr, reg_val, ret_reg);
      break;
  }

  return status;
}

bcc_status_t BatteryCellController::write_register_global(uint8_t reg_addr, uint16_t reg_val) {
  bcc_status_t status;

  assert(comm_mode == BCC_MODE_TPL);

  return write_global_tpl(reg_addr, reg_val);
}

void BatteryCellController::pack_frame(uint16_t data, uint8_t addr, bcc_cid_t cid, uint8_t cmd, uint8_t *frame) {
  assert(frame != NULL);

  /* Memory Data field. */
  frame[BCC_MSG_IDX_DATA_H] = (uint8_t)(data >> 8U);
  frame[BCC_MSG_IDX_DATA_L] = (uint8_t)(data & 0xFFU);

  /* Memory Address fields. Master/Slave field is always 0 for sending. */
  frame[BCC_MSG_IDX_ADDR] = (addr & BCC_MSG_ADDR_MASK);

  /* Physical Address (Cluster ID). */
  frame[BCC_MSG_IDX_CID_CMD] = ((uint8_t)cid & 0x0FU) << 4U;

  /* Command field. */
  frame[BCC_MSG_IDX_CID_CMD] |= (cmd & 0x0FU);

  /* CRC field. */
  frame[BCC_MSG_IDX_CRC] = calculate_crc(frame, BCC_MSG_SIZE - 1U);
}

uint8_t BatteryCellController::calculate_crc(const uint8_t *data, uint8_t size) {
  uint8_t crc;      /* Result. */
  uint8_t tableIdx; /* Index to the CRC table. */
  uint8_t dataIdx;  /* Index to the data array (memory). */

  assert(data != nullptr);

  /* Expanding value. */
  crc = 0x42U;

  for (dataIdx = 0U; dataIdx < size; dataIdx++)
  {
    tableIdx = crc ^ (*(data + dataIdx));
    crc = BCC_CRC_TABLE[tableIdx];
  }

  return crc;
}

bcc_status_t BatteryCellController::check_crc(const uint8_t *data) {
   uint8_t frameCrc;  /* CRC value from resp. */
    uint8_t compCrc;   /* Computed CRC value. */

    assert(data != NULL);

    /* Check CRC. */
    frameCrc = *(uint8_t *)(data + BCC_MSG_IDX_CRC);
    compCrc = calculate_crc(data, BCC_MSG_SIZE - 1U);

    return (compCrc != frameCrc) ? BCC_STATUS_CRC : BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::check_rolling_counter_tag_id(bcc_device_t device_type, const uint8_t *data, uint8_t rolling_counter, uint8_t tag_id) {
  uint8_t field;    /* Command field of a frame. */
  uint8_t regAddr;  /* Address field from a frame. */

  assert(data != NULL);

  field = *(uint8_t *)(data + BCC_MSG_IDX_CID_CMD);
  regAddr = *(uint8_t *)(data + BCC_MSG_IDX_ADDR) & BCC_MSG_ADDR_MASK;

  if (((device_type == BCC_DEVICE_MC33771) && BCC_HAS_TAG_ID_MC33771(regAddr)) ||
    ((device_type == BCC_DEVICE_MC33772) && BCC_HAS_TAG_ID_MC33772(regAddr)))
  {
    /* Check TAG ID. */
    if ((field & BCC_MSG_TAGID_MASK) != (tag_id & BCC_MSG_TAGID_MASK))
      return BCC_STATUS_COM_TAG_ID;
  }
  else
  {
    /* Check Rolling Counter value. */
    if ((field & BCC_MSG_RC_MASK) != (rolling_counter & BCC_MSG_RC_MASK))
      return BCC_STATUS_COM_RC;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::check_echo_frame(uint8_t tx[], const uint8_t rx[]) {
  assert(tx != NULL);
  assert(rx != NULL);

  if ((tx[BCC_MSG_IDX_DATA_H] == rx[BCC_MSG_IDX_DATA_H]) &&
    (tx[BCC_MSG_IDX_DATA_L] == rx[BCC_MSG_IDX_DATA_L]) &&
    (tx[BCC_MSG_IDX_ADDR] == rx[BCC_MSG_IDX_ADDR]) &&
    (tx[BCC_MSG_IDX_CID_CMD] == rx[BCC_MSG_IDX_CID_CMD]) &&
    (tx[BCC_MSG_IDX_CRC] == rx[BCC_MSG_IDX_CRC]))
  {
      return BCC_STATUS_SUCCESS;
  }
  else
  {
      return BCC_STATUS_COM_ECHO;
  }
}

bcc_status_t BatteryCellController::read_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_count, uint16_t *reg_val) {
  bcc_status_t status;
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t rx_buffer[BCC_MSG_SIZE];
  uint8_t reg_index;

  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR) ||
    (reg_count == 0U) || ((reg_addr + reg_count - 1U) > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for request. */
  pack_frame(0x0001U, reg_addr, cid, BCC_CMD_READ, tx_buffer);

  /* Send request for data. Required data are returned with the following transfer. */
  status = transfer_spi(tx_buffer, rx_buffer);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check CRC of the response. */
  if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS) {
    return status;
  }

  if (BCC_IS_NULL_RESP(rx_buffer))
  {
    return BCC_STATUS_NULL_RESP;
  }

  /* Read required data. */
  for (reg_index = 0U; reg_index < reg_count; reg_index++)
  {
    /* Increment address of the register to be read. */
    reg_addr++;
    if (reg_addr > 0x7FU)
    {
      reg_addr = 0x00U;
    }

    pack_frame(0x0001U, reg_addr, cid, BCC_CMD_READ, tx_buffer);

    /* Send request for data. Required data are returned with the following transfer. */
    status = transfer_spi(tx_buffer, rx_buffer);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    /* Check CRC. */
    if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS)
    {
      return status;
    }

    if (BCC_IS_NULL_RESP(rx_buffer))
    {
      return BCC_STATUS_NULL_RESP;
    }

    /* Store data. */
    *reg_val++ = BCC_GET_MSG_DATA(rx_buffer);
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_register_spi(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val, uint16_t *reg_reg) {
  bcc_status_t status;
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t rx_buffer[BCC_MSG_SIZE];
  
  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing. */
  pack_frame(reg_val, reg_addr, cid, BCC_CMD_WRITE, tx_buffer);
  status = transfer_spi(tx_buffer, rx_buffer);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check CRC */
  if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check message counter */
  /* Check whether all fields except CRC and message counter are zero */
  if (BCC_IS_NULL_RESP(rx_buffer))
  {
    return BCC_STATUS_NULL_RESP;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::read_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint8_t reg_count, uint16_t *reg_val) {
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t *rx_buffer = NULL;
  uint8_t reg_index;
  uint8_t rolling_counter;
  bcc_status_t status;

  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR) ||
    (reg_count == 0U) || ((reg_addr + reg_count - 1U) > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Calculate Rolling Counter (RC) value and increment RC index. */
    if (cid != BCC_CID_UNASSIG)
    {
      /* RC is not intended for global messages. */
      rolling_counter = (uint8_t)BCC_GET_RC(rc_table[(uint8_t)cid - 1U]);
      rc_table[(uint8_t)cid - 1U] = BCC_INC_RC_IDX(rc_table[(uint8_t)cid - 1U]);
    }
    else
    {
      rolling_counter = 0U;
    }

  /* Create frame for request. */
  pack_frame((uint16_t)reg_count, reg_addr, cid, BCC_CMD_READ | rolling_counter, tx_buffer);

  status = transfer_tpl(tx_buffer, this->rx_buffer, reg_count + 1);  // +1 for echo frame
  if (status != BCC_STATUS_SUCCESS)
    return status;

  /* Check and store responses */
  for (reg_index = 0U; reg_index < reg_count; reg_index++)
  {
    rx_buffer = this->rx_buffer + ((1U + reg_index) * BCC_MSG_SIZE);

    /* Check CRC. */
    if ((status = check_crc(rx_buffer)) != BCC_STATUS_SUCCESS)
      return status;

    /* Check the Message counter value. */
    if (cid != BCC_CID_UNASSIG)
    {
      /* RC and TAG ID are not intended for global messages. */
      status = check_rolling_counter_tag_id(devices[(uint8_t)cid - 1U], rx_buffer, rolling_counter, tag_id[(uint8_t)cid - 1U]);
      if (status != BCC_STATUS_SUCCESS)
      {
        return status;
      }
    }

    /* Store data. */
    *(reg_val + reg_index) = BCC_GET_MSG_DATA(rx_buffer);
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_register_tpl(bcc_cid_t cid, uint8_t reg_addr, uint16_t reg_val, uint16_t* ret_reg) {
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t *rx_buffer;
  bcc_status_t status;
  uint8_t rolling_counter;                  /* Rolling counter value. */
  memset(tx_buffer, 0, BCC_MSG_SIZE);

  if (((uint8_t)cid > device_count) || (reg_addr > BCC_MAX_REG_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Calculate Rolling Counter (RC) and increment RC index. */
  if (cid != BCC_CID_UNASSIG)
  {
    /* RC is not intended for global messages. */
    rolling_counter = (uint8_t)BCC_GET_RC(rc_table[(uint8_t)cid - 1U]);
    rc_table[(uint8_t)cid - 1U] = BCC_INC_RC_IDX(rc_table[(uint8_t)cid - 1U]);
  }
  else
    rolling_counter = 0;

  /* Create frame for writing. */
  pack_frame(reg_val, reg_addr, cid, BCC_CMD_WRITE | rolling_counter, tx_buffer);

  // debug packed frame
  // Serial.print("Packed TX Frame: ");
  // for (int i = 0; i < BCC_MSG_SIZE; i++) {
  //   Serial.print(tx_buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  status = transfer_tpl(tx_buffer, this->rx_buffer, 2);  // 2 frames: echo + response
  /* Skip an echo frame. */
  rx_buffer = this->rx_buffer + BCC_MSG_SIZE;

  status = check_crc(rx_buffer);
  if (status != BCC_STATUS_SUCCESS)
    return status;

  /* Check Rolling Counter value. */
  if ((*(rx_buffer + BCC_MSG_IDX_CID_CMD) & BCC_MSG_RC_MASK) != rolling_counter)
    return BCC_STATUS_COM_RC;

  /* Store content of received frame. */
  if (ret_reg != nullptr)
    *ret_reg = BCC_GET_MSG_DATA(rx_buffer);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_global_tpl(uint8_t reg_addr, uint16_t reg_val) {
  uint8_t tx_buffer[BCC_MSG_SIZE]; /* Transmission buffer. */
  bcc_status_t status;

  /* Check input parameters. */
  if (reg_addr > BCC_MAX_REG_ADDR)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing. */
  pack_frame(reg_val, reg_addr, BCC_CID_UNASSIG, BCC_CMD_GLOB_WRITE, tx_buffer);

  // debug packed frame
  // Serial.print("Packed TX Frame: ");
  // for (int i = 0; i < BCC_MSG_SIZE; i++) {
  //   Serial.print(tx_buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();

  // Note: Global write commands do NOT return echo frames according to MC33664 protocol
  // We still need to call transfer_tpl to send the TX data, but we ignore the RX response
  status = transfer_tpl(tx_buffer, this->rx_buffer, 1);

  // For global writes, ignore timeout errors - the MC33664 might not send any response
  if (status == BCC_STATUS_COM_TIMEOUT) {
    Serial.println("Global write: No response (expected behavior)");
    return BCC_STATUS_SUCCESS;
  }

  if (status != BCC_STATUS_SUCCESS) {
    Serial.printf("Global register write failed: %d\r\n", status);
    return status;
  }

  // Don't check echo frame for global writes - they don't echo back
  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::transfer_spi(uint8_t tx[], uint8_t rx[]) {
  // not currently implemented
  return BCC_STATUS_SPI_FAIL;
}

bcc_status_t BatteryCellController::transfer_tpl(uint8_t tx[], uint8_t rx[], uint32_t rx_transfer_count) {
  assert(tx != NULL);
  assert(rx != NULL);
  assert(rx_transfer_count > 0);

  auto error = tpl->transfer(tx, rx, rx_transfer_count);

  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("Error in transfer: %d\n", error);
    return error;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::send_nop_tpl(bcc_cid_t cid) {
  uint8_t txBuf[BCC_MSG_SIZE]; /* Transmission buffer. */
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || ((uint8_t)cid > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing.
  * Note: Register Data, Register Address and Message counter fields can
  * contain any value. */
  pack_frame(0x0000U, 0x00U, cid, BCC_CMD_NOOP, txBuf);

  status = transfer_tpl(txBuf, rx_buffer, 2);  // 2 frames: echo + response
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Check the echo frame. */
  return check_echo_frame(txBuf, rx_buffer);
}

bcc_status_t BatteryCellController::send_nop_spi(bcc_cid_t cid) {
  uint8_t tx[BCC_MSG_SIZE]; /* Transmission buffer. */
  uint8_t rx[BCC_MSG_SIZE]; /* Buffer for receiving. */
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || ((uint8_t)cid > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Create frame for writing.
  * Note: Register Data, Register Address and Message counter fields can
  * contain any value. */
  pack_frame(0x0000U, 0x00U, cid, BCC_CMD_NOOP, tx);

  status = transfer_spi(tx, rx);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  /* Check CRC. */
  if ((status = check_crc(rx)) != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Check whether all fields except CRC are zero. */
  if (BCC_IS_NULL_RESP(rx))
  {
    return BCC_STATUS_NULL_RESP;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::send_nop(bcc_cid_t cid) {
  uint8_t tx_buffer[BCC_MSG_SIZE];
  uint8_t rx_buffer[BCC_MSG_SIZE];
  bcc_status_t status;
  

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::disable_cyclic_timer(bcc_cid_t cid) {
  return update_register(
    cid,
    MC33771C_SYS_CFG1_OFFSET,
    MC33771C_SYS_CFG1_CYCLIC_TIMER_MASK,
    MC33771C_SYS_CFG1_CYCLIC_TIMER(MC33771C_SYS_CFG1_CYCLIC_TIMER_DISABLED_ENUM_VAL)
  );
}

bcc_status_t BatteryCellController::sleep() {
  if (comm_mode == BCC_MODE_SPI) {
    return write_register(BCC_CID_DEV1, BCC_REG_SYS_CFG_GLOBAL_ADDR,
      BCC_W_GLOBAL_GO2SLEEP(BCC_GO2SLEEP_ENABLED));
  }
  else {
    return write_register_global(BCC_REG_SYS_CFG_GLOBAL_ADDR,
      BCC_W_GLOBAL_GO2SLEEP(BCC_GO2SLEEP_ENABLED));
  }
}

bcc_status_t BatteryCellController::enter_low_power_mode() {
  bcc_status_t status;

  // disable cyclic timer on all devices
  for (uint8_t cid = 1; cid <= device_count; cid++) {
    BCC_CHECK_ERROR(disable_cyclic_timer((bcc_cid_t)cid));
  }

  // send go to sleep command
  BCC_CHECK_ERROR(sleep());

  // disable TPL mode
  if (comm_mode == BCC_MODE_TPL) {
    disable_tpl();
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::start_conversion_async(bcc_cid_t cid) {
  uint16_t regVal;     /* Value of ADC_CFG register. */
  bcc_status_t error;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
    return BCC_STATUS_PARAM_RANGE;

  error = read_register(cid, BCC_REG_ADC_CFG_ADDR, 1U, &regVal);
  if (error != BCC_STATUS_SUCCESS)
    return error;

  /* Increment TAG ID (4 bit value). */
  tag_id[(uint8_t)cid - 1] = (tag_id[(uint8_t)cid - 1] + 1U) & 0x0FU;

  /* Set new TAG ID and Start of Conversion bit. */
  regVal = BCC_SET_TAG_ID(regVal, tag_id[(uint8_t)cid - 1]);
  regVal = BCC_REG_SET_BIT_VALUE(regVal, BCC_W_SOC_MASK);

  return write_register(cid, BCC_REG_ADC_CFG_ADDR, regVal, NULL);
}

bcc_status_t BatteryCellController::start_conversion_global_async(uint16_t adc_config_value) {
  uint8_t dev;
  assert(this->comm_mode == BCC_MODE_TPL);

  /* Increment & Use TAG ID (4 bit value) of the first node. */
  this->tag_id[0] = (this->tag_id[0] + 1U) & 0x0FU;

  /* Set Tag ID to all BCCs in the driver configuration structure. */
  for (dev = 1; dev < device_count; dev++)
    this->tag_id[dev] = this->tag_id[0];

  /* Set new TAG ID and Start of Conversion bit. */
  adc_config_value = BCC_SET_TAG_ID(adc_config_value, this->tag_id[0]);
  adc_config_value = BCC_REG_SET_BIT_VALUE(adc_config_value, BCC_W_SOC_MASK);

  return write_register_global(BCC_REG_ADC_CFG_ADDR, adc_config_value);
}

bcc_status_t BatteryCellController::is_converting(bcc_cid_t cid, bool *completed) {
  uint16_t regVal;     /* Value of ADC_CFG register. */
  bcc_status_t error;
  assert(completed != nullptr);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  error = read_register(cid, BCC_REG_ADC_CFG_ADDR, 1U, &regVal);

  regVal = regVal & BCC_R_EOC_N_MASK;
  *(completed) = (regVal == 0x00U);

  return error;
}

bcc_status_t BatteryCellController::get_raw_values(bcc_cid_t cid, uint16_t *measurements) {
  bcc_status_t error;
  uint8_t i;

  assert(measurements != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
    return BCC_STATUS_PARAM_RANGE;

  /* Read all the measurement registers.
  * Note: the order and number of registers conforms to the order of measured
  * values in Measurements array, see enumeration bcc_measurements_t. */
  if (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771)
    error = read_register(cid, BCC_REG_CC_NB_SAMPLES_ADDR, BCC_MEAS_CNT, measurements);
  else
  {
    error = read_register(cid, BCC_REG_CC_NB_SAMPLES_ADDR,
      (BCC_REG_MEAS_STACK_ADDR - BCC_REG_CC_NB_SAMPLES_ADDR) + 1, measurements);
      
    if (error != BCC_STATUS_SUCCESS)
        return error;

    /* Skip the reserved registers. */
    measurements[BCC_MSR_CELL_VOLT14] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT13] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT12] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT11] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT10] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT9] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT8] = 0x0000;
    measurements[BCC_MSR_CELL_VOLT7] = 0x0000;

    error = read_register(cid, BCC_REG_MEAS_CELLX_ADDR_MC33772_START,
      (BCC_REG_MEAS_VBG_DIAG_ADC1B_ADDR - BCC_REG_MEAS_CELLX_ADDR_MC33772_START) + 1,
      (uint16_t *)(measurements + ((uint8_t)BCC_MSR_CELL_VOLT6)));
  }

  /* Mask bits. */
  /* Nothing to mask in CC_NB_SAMPLES, COULOMB_CNT1 and COULOMB_CNT2 registers. */
  measurements[BCC_MSR_ISENSE1] &= MC33771C_MEAS_ISENSE1_MEAS_I_MSB_MASK;
  measurements[BCC_MSR_ISENSE2] &= MC33771C_MEAS_ISENSE2_MEAS_I_LSB_MASK;

  /* Mask the other registers (starting at 5th register). */
  for (i = 5U; i < BCC_MEAS_CNT; i++)
    measurements[i] &= BCC_R_MEAS_MASK;

  return error;
}

bcc_status_t BatteryCellController::get_coulomb_counter(bcc_cid_t cid, bcc_cc_data_t* coulomb_counter) {
  bcc_status_t status;
  uint16_t readVal[3];

  assert(coulomb_counter != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_CC_NB_SAMPLES_OFFSET, 3U, readVal);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  coulomb_counter->nbSamples = readVal[0];
  coulomb_counter->ccAccumulator = BCC_GET_COULOMB_CNT(readVal[1], readVal[2]);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_current_sense_voltage(bcc_cid_t cid, int32_t *current_sense_voltage) {
  bcc_status_t status;
  uint16_t readVal[2];

  assert(current_sense_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_ISENSE1_OFFSET, 2U, readVal);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((readVal[0] & readVal[1] & MC33771C_MEAS_ISENSE1_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  *current_sense_voltage = BCC_GET_ISENSE_VOLT(readVal[0], readVal[1]);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_stack_voltage(bcc_cid_t cid, uint32_t *stack_voltage) {
  bcc_status_t status;
  uint16_t readVal;

  assert(stack_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_STACK_OFFSET, 1U, &readVal);
  if (status != BCC_STATUS_SUCCESS)
  {
      return status;
  }

  if ((readVal & MC33771C_MEAS_STACK_DATA_RDY_MASK) == 0U)
  {
      return BCC_STATUS_DATA_RDY;
  }

  *stack_voltage = BCC_GET_STACK_VOLT(readVal);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_cell_voltages(bcc_cid_t cid, uint32_t *cell_voltages) {
  bcc_status_t status;
  uint16_t read_values[BCC_MAX_CELLS];
  uint8_t i, cell_count;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  cell_count = BCC_MAX_CELLS_DEV(devices[(uint8_t)cid - 1]);

  /* Read the measurement registers. */
  status = read_register(cid,
                        (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771) ?
                              MC33771C_MEAS_CELL14_OFFSET : MC33771C_MEAS_CELL6_OFFSET,
                        cell_count, read_values);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Convert measurements to [uV], change the cell order and check the data-ready flag. */
  for (i = 0; i < cell_count; i++)
  {
    cell_voltages[cell_count - (i + 1)] = BCC_GET_VOLT(read_values[i]);
    read_values[0] &= read_values[i];
  }

  if ((read_values[0] & MC33771C_MEAS_CELL1_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_cell_voltage(bcc_cid_t cid, uint8_t cell_index, uint32_t *cell_voltage) {
  bcc_status_t status;
  uint16_t read_value;

  assert(cell_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
      (cell_index >= BCC_MAX_CELLS_DEV(devices[(uint8_t)cid - 1])))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_CELL1_OFFSET - cell_index, 1U, &read_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((read_value & MC33771C_MEAS_CELL1_DATA_RDY_MASK) == 0U)
  {
      return BCC_STATUS_DATA_RDY;
  }

  *cell_voltage = BCC_GET_VOLT(read_value);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_an_voltages(bcc_cid_t cid, uint32_t *an_voltages) {
  bcc_status_t status;
  uint16_t read_values[BCC_GPIO_INPUT_CNT];
  uint8_t i;

  assert(an_voltages != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Read the measurement registers. */
  status = read_register(cid, MC33771C_MEAS_AN6_OFFSET,
    BCC_GPIO_INPUT_CNT, read_values);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Convert measurements to [uV] and check the data-ready flag. */
  for (i = 0; i < BCC_GPIO_INPUT_CNT; i++)
  {
    an_voltages[i] = BCC_GET_VOLT(read_values[i]);
    read_values[0] &= read_values[i];
  }

  if ((read_values[0] & MC33771C_MEAS_AN0_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_an_voltage(bcc_cid_t cid, uint8_t an_index, uint32_t *an_voltage) {
  bcc_status_t status;
  uint16_t read_value;

  assert(an_voltage != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (an_index >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_AN0_OFFSET - an_index, 1U, &read_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((read_value & MC33771C_MEAS_AN0_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  *an_voltage = BCC_GET_VOLT(read_value);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_ic_temperature(bcc_cid_t cid, bcc_temp_unit_t unit, int16_t *ic_temperature) {
  bcc_status_t status;
  uint16_t read_value;

  assert(ic_temperature != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (unit > BCC_TEMP_FAHRENHEIT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = read_register(cid, MC33771C_MEAS_IC_TEMP_OFFSET, 1U, &read_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  if ((read_value & MC33771C_MEAS_IC_TEMP_DATA_RDY_MASK) == 0U)
  {
    return BCC_STATUS_DATA_RDY;
  }

  if (unit == BCC_TEMP_CELSIUS)
  {
    *ic_temperature = BCC_GET_IC_TEMP_C(read_value);
  }
  else if (unit == BCC_TEMP_FAHRENHEIT)
  {
    *ic_temperature = BCC_GET_IC_TEMP_F(read_value);
  }
  else
  {
    *ic_temperature = BCC_GET_IC_TEMP_K(read_value);
  }

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::get_fault_status(bcc_cid_t cid, uint16_t fault_status[]) {
  bcc_status_t error;

  assert(fault_status != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
    return BCC_STATUS_PARAM_RANGE;

  /* Read CELL_OV_FLT and CELL_UV_FLT. */
  error = read_register(cid, BCC_REG_CELL_OV_FLT_ADDR, 2U, &fault_status[BCC_FS_CELL_OV]);
  if (error != BCC_STATUS_SUCCESS) return error;

  /* Read CB_OPEN_FLT, CB_SHORT_FLT. */
  error = read_register(cid, BCC_REG_CB_OPEN_FLT_ADDR, 2U, &fault_status[BCC_FS_CB_OPEN]);
  if (error != BCC_STATUS_SUCCESS) return error;

  /* Read GPIO_STS, AN_OT_UT_FLT, GPIO_SHORT_Anx_OPEN_STS. */
  error = read_register(cid, BCC_REG_GPIO_STS_ADDR, 3U, &fault_status[BCC_FS_GPIO_STATUS]);
  if (error != BCC_STATUS_SUCCESS) return error;

  /* Read COM_STATUS, FAULT1_STATUS, FAULT2_STATUS and FAULT3_STATUS. */
  error = read_register(cid, BCC_REG_COM_STATUS_ADDR, 4U, &fault_status[BCC_FS_COMM]);
  if (error != BCC_STATUS_SUCCESS) return error;

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::clear_fault_status(bcc_cid_t cid, bcc_fault_status_t fault_status) {
  /* This array is intended for conversion of bcc_fault_status_t value to
    * a BCC register address. */
  const uint8_t regAddrMap[BCC_STAT_CNT] = {
    MC33771C_CELL_OV_FLT_OFFSET, MC33771C_CELL_UV_FLT_OFFSET,
    MC33771C_CB_OPEN_FLT_OFFSET, MC33771C_CB_SHORT_FLT_OFFSET,
    MC33771C_GPIO_STS_OFFSET, MC33771C_AN_OT_UT_FLT_OFFSET,
    MC33771C_GPIO_SHORT_ANX_OPEN_STS_OFFSET, MC33771C_COM_STATUS_OFFSET,
    MC33771C_FAULT1_STATUS_OFFSET, MC33771C_FAULT2_STATUS_OFFSET,
    MC33771C_FAULT3_STATUS_OFFSET
  };

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    ((uint32_t)fault_status >= BCC_STAT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return write_register(cid, regAddrMap[fault_status], 0x0000U);
}

bcc_status_t BatteryCellController::set_gpio_mode(bcc_cid_t cid, uint8_t gpio_selection, bcc_pin_mode_t mode) {
  bcc_status_t status = BCC_STATUS_PARAM_RANGE;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (gpio_selection >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if ((mode == BCC_PIN_WAKE_UP_IN) && (gpio_selection == 0U))
  {
    /* Set GPIO0 to digital input and enable the wake-up capability. */
    status = set_gpio_config(cid, 0U, BCC_PIN_DIGITAL_IN);
    if (status == BCC_STATUS_SUCCESS)
    {
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO0_WU_MASK,
        MC33771C_GPIO_CFG2_GPIO0_WU(MC33771C_GPIO_CFG2_GPIO0_WU_WAKEUP_ENUM_VAL));
    }
  }
  else if ((mode == BCC_PIN_CONVERT_TR_IN) && (gpio_selection == 2U))
  {
    /* Set GPIO2 to digital input serving as a conversion trigger. */
    status = set_gpio_config(cid, 2U, BCC_PIN_DIGITAL_IN);
    if (status == BCC_STATUS_SUCCESS)
    {
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO2_SOC_MASK,
        MC33771C_GPIO_CFG2_GPIO2_SOC(MC33771C_GPIO_CFG2_GPIO2_SOC_ADC_TRG_ENABLED_ENUM_VAL));
    }
  }
  else if (mode <= BCC_PIN_DIGITAL_OUT)
  {
    status = BCC_STATUS_SUCCESS;
    if (gpio_selection == 0U)
    {
      /* Disable the wake-up capability. */
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO0_WU_MASK,
        MC33771C_GPIO_CFG2_GPIO0_WU(MC33771C_GPIO_CFG2_GPIO0_WU_NO_WAKEUP_ENUM_VAL));
    }
    else if (gpio_selection == 2U)
    {
      /* Disable the conversion trigger. */
      status = update_register(cid,
        MC33771C_GPIO_CFG2_OFFSET,
        MC33771C_GPIO_CFG2_GPIO2_SOC_MASK,
        MC33771C_GPIO_CFG2_GPIO2_SOC(MC33771C_GPIO_CFG2_GPIO2_SOC_ADC_TRG_DISABLED_ENUM_VAL));
    }

    if (status == BCC_STATUS_SUCCESS)
    {
      status = set_gpio_config(cid, gpio_selection, mode);
    }
  }

  return status;
}

bcc_status_t BatteryCellController::read_gpio(bcc_cid_t cid, uint8_t gpio_selection, bool *value) {
  bcc_status_t status;
  uint16_t gpio_sts_value;

  assert(value != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (gpio_selection >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Read and update content of GPIO_CFG2 register. */
  status = read_register(cid, MC33771C_GPIO_STS_OFFSET, 1U, &gpio_sts_value);
  *value = (gpio_sts_value & (1U << gpio_selection)) > 0U;

  return status;
}

bcc_status_t BatteryCellController::write_gpio(bcc_cid_t cid, uint8_t gpio_selection, bool value) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count) ||
    (gpio_selection >= BCC_GPIO_INPUT_CNT))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* Update the content of GPIO_CFG2 register. */
  return update_register(cid, MC33771C_GPIO_CFG2_OFFSET,
    (uint16_t)(1U << gpio_selection),
    (uint16_t)((value ? 1U : 0U) << gpio_selection));
}

bcc_status_t BatteryCellController::enable_cell_balancing(bcc_cid_t cid, bool enable) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return update_register(cid, MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_CB_DRVEN_MASK,
    enable ? MC33771C_SYS_CFG1_CB_DRVEN(MC33771C_SYS_CFG1_CB_DRVEN_ENABLED_ENUM_VAL) 
    : MC33771C_SYS_CFG1_CB_DRVEN(MC33771C_SYS_CFG1_CB_DRVEN_DISABLED_ENUM_VAL));
}

bcc_status_t BatteryCellController::set_cell_balancing(bcc_cid_t cid, uint8_t cell_index, bool enable, uint16_t timer) {
  uint16_t config_value;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  if (cell_index >= BCC_MAX_CELLS_DEV(devices[(uint8_t)cid - 1]))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  if (timer > MC33771C_CB1_CFG_CB_TIMER_MASK)
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  config_value = enable ? MC33771C_CB1_CFG_CB_EN(MC33771C_CB1_CFG_CB_EN_ENABLED_ENUM_VAL) 
                      : MC33771C_CB1_CFG_CB_EN(MC33771C_CB1_CFG_CB_EN_DISABLED_ENUM_VAL);
  config_value |= MC33771C_CB1_CFG_CB_TIMER(timer);

  return write_register(cid, MC33771C_CB1_CFG_OFFSET + cell_index, config_value);
}

bcc_status_t BatteryCellController::pause_cell_balancing(bcc_cid_t cid, bool pause) {
  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  return update_register(cid, MC33771C_SYS_CFG1_OFFSET, MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_MASK,
    (pause) ? MC33771C_SYS_CFG1_CB_MANUAL_PAUSE(MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_ENABLED_ENUM_VAL) 
            : MC33771C_SYS_CFG1_CB_MANUAL_PAUSE(MC33771C_SYS_CFG1_CB_MANUAL_PAUSE_DISABLED_ENUM_VAL));
}

bcc_status_t BatteryCellController::read_fuse_mirror(bcc_cid_t cid, uint8_t fuse_address, uint16_t* value) {
  bcc_status_t status;

  assert(value != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (fuse_address > ((devices[(uint8_t)cid - 1U] == BCC_DEVICE_MC33771) ?
          MC33771C_MAX_FUSE_READ_ADDR : MC33772C_MAX_FUSE_READ_ADDR))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  status = write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(fuse_address) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_LOCKED_ENUM_VAL) |
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_SPI_WRITE_ENABLE_ENUM_VAL));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  return read_register(cid, MC33771C_FUSE_MIRROR_DATA_OFFSET, 1U, value);
}

bcc_status_t BatteryCellController::write_fuse_mirror(bcc_cid_t cid, uint8_t fuse_address, uint16_t value) {
  bcc_status_t status;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (fuse_address > ((devices[(uint8_t)cid - 1U] == BCC_DEVICE_MC33771) ?
          MC33771C_MAX_FUSE_WRITE_ADDR : MC33772C_MAX_FUSE_WRITE_ADDR))
  {
      return BCC_STATUS_PARAM_RANGE;
  }

  /* FUSE_MIRROR_CNTL to enable writing. */
  status = write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(0U) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_UNLOCKED_ENUM_VAL) |
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_SPI_WRITE_ENABLE_ENUM_VAL));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Send the fuse address. */
  status = write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(fuse_address) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_UNLOCKED_ENUM_VAL) |
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_SPI_WRITE_ENABLE_ENUM_VAL));
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  status = write_register(cid, MC33771C_FUSE_MIRROR_DATA_OFFSET, value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* FUSE_MIRROR_CNTL to low power. */
  return write_register(cid, MC33771C_FUSE_MIRROR_CNTL_OFFSET,
    MC33771C_FUSE_MIRROR_CNTL_FMR_ADDR(0U) |
    MC33771C_FUSE_MIRROR_CNTL_FSTM(MC33771C_FUSE_MIRROR_CNTL_FSTM_UNLOCKED_ENUM_VAL) | 
    MC33771C_FUSE_MIRROR_CNTL_FST(MC33771C_FUSE_MIRROR_CNTL_FST_LP_ENUM_VAL));
}

bcc_status_t BatteryCellController::read_guid(bcc_cid_t cid, uint64_t *guid) {
  const uint8_t addr771c[3] = {
    MC33771C_FUSE_TR_0_OFFSET,
    MC33771C_FUSE_TR_1_OFFSET,
    MC33771C_FUSE_TR_2_OFFSET
  };
  const uint8_t addr772c[3] = {
    MC33772C_FUSE_TR_0_OFFSET,
    MC33772C_FUSE_TR_1_OFFSET,
    MC33772C_FUSE_TR_2_OFFSET
  };
  uint8_t const *readAddr;
  uint16_t readData[3];
  uint8_t i;
  bcc_status_t status;

  assert(guid != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  readAddr = (devices[(uint8_t)cid - 1] == BCC_DEVICE_MC33771) ? addr771c : addr772c;

  for (i = 0; i < 3; i++)
  {
    status = read_fuse_mirror(cid, readAddr[i], &(readData[i]));
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  *guid = (((uint64_t)(readData[0] & BCC_FUSE_TR_0_MASK)) << 21) |
    (((uint64_t)(readData[1] & BCC_FUSE_TR_1_MASK)) << 5) |
    ((uint64_t)(readData[2] & BCC_FUSE_TR_2_MASK));

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::read_eeprom(bcc_cid_t cid, uint8_t address, uint8_t *data) {
  bcc_status_t status;
  uint16_t reg_value;

  assert(data != NULL);

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (address > BCC_MAX_EEPROM_ADDR)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* EEPROM Read command. */
  reg_value = MC33771C_EEPROM_CTRL_R_W(MC33771C_EEPROM_CTRL_R_W_READ_ENUM_VAL) |
    MC33771C_EEPROM_CTRL_EEPROM_ADD(address);
  status = write_register(cid, MC33771C_EEPROM_CTRL_OFFSET, reg_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Wait while data is read from EEPROM. */
  status = start_timeout(BCC_EEPROM_READ_TIMEOUT_US);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  do
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  } while ((reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK) && (!has_timer_expired()));

  /* Check once more after timeout expiration because the read command takes
    * several tens/hundreds of microseconds (depends on user code efficiency)
    * and the last read command could be done relatively long before the
    * timeout expiration. */
  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    return BCC_STATUS_COM_TIMEOUT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_EE_PRESENT_MASK)
  {
    return BCC_STATUS_EEPROM_PRESENT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_ERROR_MASK)
  {
    return BCC_STATUS_EEPROM_ERROR;
  }

  /* Store read data to memory space defined by the pointer. */
  *data = (uint8_t)(reg_value & MC33771C_EEPROM_CTRL_READ_DATA_MASK);

  return BCC_STATUS_SUCCESS;
}

bcc_status_t BatteryCellController::write_eeprom(bcc_cid_t cid, uint8_t address, uint8_t data) {
  bcc_status_t status;
  uint16_t reg_value;

  if ((cid == BCC_CID_UNASSIG) || (((uint8_t)cid) > device_count))
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  if (address > BCC_MAX_EEPROM_ADDR)
  {
    return BCC_STATUS_PARAM_RANGE;
  }

  /* EEPROM Write command. */
  reg_value = MC33771C_EEPROM_CTRL_R_W(MC33771C_EEPROM_CTRL_R_W_WRITE_ENUM_VAL) |
    MC33771C_EEPROM_CTRL_EEPROM_ADD(address) |
    MC33771C_EEPROM_CTRL_DATA_TO_WRITE(data);
  status = write_register(cid, MC33771C_EEPROM_CTRL_OFFSET, reg_value);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  /* Wait while BCC sends the write command to EEPROM. */
  status = start_timeout(BCC_EEPROM_WRITE_TIMEOUT_US);
  if (status != BCC_STATUS_SUCCESS)
  {
    return status;
  }

  do
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  } while ((reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK) && (!has_timer_expired()));

  /* Check once more after timeout expiration because the read command takes
    * several tens/hundreds of microseconds (depends on user code efficiency)
    * and the last read command could be done relatively long before the
    * timeout expiration. */
  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    status = read_register(cid, MC33771C_EEPROM_CTRL_OFFSET, 1U, &reg_value);
    if (status != BCC_STATUS_SUCCESS)
    {
      return status;
    }
  }

  if (reg_value & MC33771C_EEPROM_CTRL_BUSY_MASK)
  {
    return BCC_STATUS_COM_TIMEOUT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_EE_PRESENT_MASK)
  {
    return BCC_STATUS_EEPROM_PRESENT;
  }

  if (reg_value & MC33771C_EEPROM_CTRL_ERROR_MASK)
  {
    return BCC_STATUS_EEPROM_ERROR;
  }

  return BCC_STATUS_SUCCESS;
}

BatteryCellController::~BatteryCellController() {
  detachInterrupt(digitalPinToInterrupt(this->intb_pin));

  // Unregister from instances array
  if (instances[0] == this) {
    instances[0] = nullptr;
  } else if (instances[1] == this) {
    instances[1] = nullptr;
  }
}