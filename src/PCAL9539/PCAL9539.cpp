#include "PCAL9539.h"
#include "StringBuilder.h"

/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/

static volatile bool pcal9539_isr_fired[2] = { false };

/** ISRs */
void pcal9539_isr_0() {  pcal9539_isr_fired[0] = true;  }
void pcal9539_isr_1() {  pcal9539_isr_fired[1] = true;  }


/* Real register addresses */
static const uint8_t PCAL9539_REG_ADDR[PCAL9539_NUM_OF_REGISTERS] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
  0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4F
};

static const uint8_t PCAL_REG_DEFAULTS[PCAL9539_NUM_OF_REGISTERS] = {
  0x00, 0x00,  // INPUT_PORT0,1
  0x00, 0x00,  // OUTPUT_PORT0,1
  0x00, 0x00,  // POLARITY_INVERSION_PORT0,1
  0xFF, 0xFF,  // CONFIGURATION_PORT0,1
  0x00, 0x00,  // OUTPUT_DRIVE_STRENGTH_PORT0,1
  0x00, 0x00,  // OUTPUT_DRIVE_STRENGTH_PORT2,3
  0x00, 0x00,  // INPUT_LATCH_PORT0,1
  0x00, 0x00,  // PULLUP_PULLDOWN_ENABLE_PORT0,1
  0xFF, 0xFF,  // PULLUP_PULLDOWN_SELECTION_PORT0,1
  0xFF, 0xFF,  // INTERRUPT_MASK_PORT0,1
  0x00, 0x00,  // INTERRUPT_STATUS_PORT0,1
  0x00   // OUPUT_PORT_CONFIGURATION
};

const PCAL9539RegId _reg_id_from_addr(uint8_t addr) {
  switch (addr) {
    case 0x00:  return PCAL9539RegId::INPUT_PORT0;
    case 0x01:  return PCAL9539RegId::INPUT_PORT1;
    case 0x02:  return PCAL9539RegId::OUTPUT_PORT0;
    case 0x03:  return PCAL9539RegId::OUTPUT_PORT1;
    case 0x04:  return PCAL9539RegId::POLAR_INV_PORT0;
    case 0x05:  return PCAL9539RegId::POLAR_INV_PORT1;
    case 0x06:  return PCAL9539RegId::CONFIG_PORT0;
    case 0x07:  return PCAL9539RegId::CONFIG_PORT1;
    case 0x40:  return PCAL9539RegId::OUTPUT_DRIVE_STR_0;
    case 0x41:  return PCAL9539RegId::OUTPUT_DRIVE_STR_1;
    case 0x42:  return PCAL9539RegId::OUTPUT_DRIVE_STR_2;
    case 0x43:  return PCAL9539RegId::OUTPUT_DRIVE_STR_3;
    case 0x44:  return PCAL9539RegId::INPUT_LATCH_0;
    case 0x45:  return PCAL9539RegId::INPUT_LATCH_1;
    case 0x46:  return PCAL9539RegId::PULL_ENABLE_0;
    case 0x47:  return PCAL9539RegId::PULL_ENABLE_1;
    case 0x48:  return PCAL9539RegId::PULL_SELECT_0;
    case 0x49:  return PCAL9539RegId::PULL_SELECT_1;
    case 0x4A:  return PCAL9539RegId::IRQ_MASK_0;
    case 0x4B:  return PCAL9539RegId::IRQ_MASK_1;
    case 0x4C:  return PCAL9539RegId::IRQ_STATUS_0;
    case 0x4D:  return PCAL9539RegId::IRQ_STATUS_1;
    case 0x4F:  return PCAL9539RegId::OUTPUT_CONF;
  }
  return PCAL9539RegId::INVALID;
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor
*/
PCAL9539::PCAL9539(I2CAdapter* bus, const uint8_t i2c_addr, const uint8_t irq_pin, const uint8_t reset_pin, const uint8_t* reg_config) :
  I2CDevice(i2c_addr, bus),
  _IRQ_PIN(irq_pin),
  _RESET_PIN(reset_pin),
  _CONFIG((nullptr == reg_config) ? PCAL_REG_DEFAULTS : reg_config) {}


/**
* Destructor
*/
PCAL9539::~PCAL9539() {
  if (255 != _IRQ_PIN) {
    unsetPinFxn(_IRQ_PIN);
  }
  if (!preserveOnDestroy() & (255 != _RESET_PIN)) {
    setPin(_RESET_PIN, 0);  // Leave the part in reset state.
  }
}


/**
* In-class accessor to check if the IRQ pin has asserted since last poll()'ing.
*
* @return true if the data registers need to be read.
*/
bool PCAL9539::isrFired() {
  return (255 != _IRQ_PIN) ? pcal9539_isr_fired[0] : true;
}


/**
* Initializes the hardware.
*
* This driver's proper operation requires that IRQ autoclears when the data
*   registers are read, and that all input pins generate an IRQ on state change
*   (rising AND falling edges). The driver will rationalize the pin state
*   against the application's notification preferences for each input pin.
*
* @param b Is the bus that is hosting the hardware.
* @return 0 on success
*        -1 on failure to setup pins
*        -2 on no assigned I2CAdapter
*        -3 on failure to restore serialized hardware state
*        -4 on failure to refresh
*        -5 on failure to reset
*/
int8_t PCAL9539::init(I2CAdapter* b) {
  int8_t ret = -1;
  _pcal_clear_flag(PCAL9539_FLAG_INITIALIZED);
  if (nullptr != b) {
    _bus = b;
  }
  if (0 == _ll_pin_init()) {
    ret = -2;
    if (nullptr != _bus) {
      if (preserveOnDestroy()) {
        ret = -3;
        if (0 == refresh()) {
          // We take no action against the present hardware state. Just read it.
          ret = 0;
        }
      }
      else if (0 == reset()) {
        // Reset ahead of baseline class configuration.
        ret = 0;
      }
      else {
        ret = -4;
      }
    }
  }

  // Update shadow registers to reflect hardware
  if (0 == ret) {
    ret = refresh();
  }

  if (0 == ret) {
    _pcal_set_flag(PCAL9539_FLAG_INITIALIZED);
  }

  return ret;
}


/**
* Resets the hardware by the most expedient means available.
*
* This driver's proper operation requires that IRQ autoclears when the data
*   registers are read, and that all input pins generate an IRQ on state change
*   (rising AND falling edges). The driver will rationalize the pin state
*   against the application's notification preferences for each input pin.
*
* @param b Is the bus that is hosting the hardware.
* @return 0 on success, non-zero otherwise.
*/
int8_t PCAL9539::reset() {
  int8_t ret = -1;
  _0_in_dat = 0;
  _1_in_dat = 0;
  _0_out_dat = 0xff;
  _1_out_dat = 0xff;
  _pcal_clear_flag(PCAL9539_FLAG_INITIALIZED);
  pcal9539_isr_fired[0] = false;
  // Rewrite the shadows with the default PCAL values.
  for (uint8_t i = 0; i < PCAL9539_NUM_OF_REGISTERS; i++) {
    _shadows[i] = PCAL_REG_DEFAULTS[i];
  }
  if (255 != _RESET_PIN) {
    setPin(_RESET_PIN, 0);
    sleep_us(1);   // Datasheet says 300ns.
    setPin(_RESET_PIN, 1);
    if (255 != _IRQ_PIN) {
      // Wait on the IRQ pin to go high.
      uint32_t millis_abort = millis() + 15;
      while ((millis() < millis_abort) && (!readPin(_IRQ_PIN))) {}
      if (readPin(_IRQ_PIN)) {
        ret = 0;
      }
    }
    else {
      // Without an IRQ pin, we have to take the datasheet on faith when it says
      //   the chip comes back in 7ms.
      sleep_ms(10);
      ret = 0;
    }
  }
  else {
    // Without a RESET pin, steamroll the hardware with the default values.
    ret = _write_registers(PCAL9539RegId::INPUT_PORT0, 0x08);
    ret = _write_registers(PCAL9539RegId::OUTPUT_DRIVE_STR_0, 0x0E);

    if (0 == ret) {
      _read_registers(PCAL9539RegId::INPUT_PORT0, 2);   // Read back initial input states.
    }
  }
  return ret;
}


/**
* Poll the class for updates.
* If the application has setup a callback for a given pin, its change will not
*   be counted in the return value. Thus, if callbacks are setup for every pin
*   that is configured for input, this function will never return a value (>0).
*
* @return
*   -3 if not initialized.
*   -1 if the hardware needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if a read was initiated.
*/
int8_t PCAL9539::poll() {
  int8_t ret = -3;
  if (initialized()) {
    ret = 0;
    if (255 == _IRQ_PIN) {
      // Without an IRQ pin, we hammer this every polling cycle.
      pcal9539_isr_fired[0] = true;
    }
    if (pcal9539_isr_fired[0] && !_pcal_flag(PCAL9539_FLAG_READ_IN_FLIGHT)) {
      ret--;
      _pcal_set_flag(PCAL9539_FLAG_READ_IN_FLIGHT);
      if (0 == _read_registers(PCAL9539RegId::INPUT_PORT0, 2)) {
        pcal9539_isr_fired[0] = false;
        ret = 1;
      }
    }
  }
  return ret;
}


/**
* Refreshes all register shadows from the hardware. The address space is
*   discontinuous, so this must be done in chunks.
*
* @return 0 if all bus reads were dispatched, nonzero otherwise.
*/
int8_t PCAL9539::refresh() {
  // NOTE: Chip does not appear to be able to do multi-register reads...
  int8_t ret = _read_registers(PCAL9539RegId::INPUT_PORT0, 1);
  uint8_t reg_idx = 0;
  while ((0 == ret) & ((uint8_t) PCAL9539RegId::INVALID > reg_idx)) {
    ret = _read_registers((PCAL9539RegId) reg_idx, 1);
    reg_idx++;
  }
  return ret;
}


/**
* Sets a callback for when a pins IRQ condition is met
*
* @param pin The pin whose callback to be initialized
* @param callback The callback to be called
* @param condition The triggering IRQCondition
* @return 0 on success
*/
int8_t PCAL9539::attachInterrupt(uint8_t pin, PinCallback cb, IRQCondition condition) {
  int8_t ret = -1;
  if (16 > pin) {
    _callbacks[pin]  = cb;
    ret = 0;
  }
  return ret;
}


/**
* Stops the driver from invoking the pin callback on pin change.
*
* @param pin The pin whose callback is to be removed.
* @return the number of interrupts removed.
*/
int8_t PCAL9539::detachInterrupt(uint8_t pin) {
  int8_t ret = (nullptr == _callbacks[pin]) ? 0 : 1;
  _callbacks[pin]  = nullptr;

  return ret;
}

/**
* Stops the driver from invoking the pin callback on pin change.
*
* @param cb The callback for the pin that is to be removed.
* @return the number of interrupts removed.
*/
int8_t PCAL9539::detachInterrupt(PinCallback cb) {
  int8_t ret = -1;
  for (uint8_t i = 0; i < 16; i++) {
    if (cb == _callbacks[i]) {
      _callbacks[i] = nullptr;
      ret = 0;
    }
  }
  return ret;
}


/*
* TODO: Implement open-drain.
*/
int8_t PCAL9539::digitalWrite(uint8_t pin, bool value) {
  int8_t ret = -2;
  PCAL9539RegId reg0;
  if (pin < 16) {
    ret = 0;
    if(pin < 8){
      reg0 = PCAL9539RegId::OUTPUT_PORT0;
      pin = pin & 0x07;
    }
    else{
      reg0 = PCAL9539RegId::OUTPUT_PORT1;
      pin = pin & 0x07;
    }
    uint8_t val0 = _get_shadow_value(reg0);
    uint8_t val1 = val0;
    val0 = value ? (val0 | (0x01 << pin)) : (val0 & ~(0x01 << pin));
    if (val1 != val0) {
      ret = _write_register(reg0, val0);
      if (0 == ret) {
        _pcal_set_flag(PCAL9539_FLAG_WRITE_IN_FLIGHT);
      }
    }
  }
  return ret;
}


/**
*
* @return
*   0 or 1 to reflect the logic level
*   -1 if the pin is unsupported
*/
int8_t PCAL9539::digitalRead(uint8_t pin) {
  int8_t ret = -1;
  if (pin < 8) {
    pin = pin & 0x07; // Put in range 0-7 for register
    ret = (_0_in_dat >> pin) & 0x01;
  }
  else if (pin < 16) {
    pin = pin & 0x07; // Put in range 0-7 for register
    ret = (_1_in_dat >> pin) & 0x01;
  }
  return ret;
}


/**
*
* @return The current verified state of the input GPIO pins.
*/
uint16_t PCAL9539::getInputPinValues() {
  return (_0_in_dat | ((uint16_t) _1_in_dat << 8));
}

/**
*
* @return The current verified state of the output GPIO pins.
*/
uint16_t PCAL9539::getOutputPinValues() {
  return (_0_out_dat | ((uint16_t) _1_out_dat << 8));
}


/**
*
* @return 0 on success. Fail if otherwise
*/
int8_t PCAL9539::setPinValues(uint16_t value) {
  uint8_t ret = -1;
  _set_shadow_value(PCAL9539RegId::OUTPUT_PORT0, (uint8_t) (value >> 8));
  _set_shadow_value(PCAL9539RegId::OUTPUT_PORT1, (uint8_t) (value & 0x00FF));
  if (0 == _write_registers(PCAL9539RegId::OUTPUT_PORT0, 2)) {
    _pcal_set_flag(PCAL9539_FLAG_WRITE_IN_FLIGHT);
    ret = 0;
  }
  return ret;
}


/**
* Set GPIO pin mode
*
* @return
*   -2 on unsupported mode
*   -1 on bad pin
*   0 on success
*/
int8_t PCAL9539::gpioMode(uint8_t pin, GPIOMode mode) {
  uint8_t ret = -1;
  if (pin < 16)  { // Valid pin?
    bool in = true;
    bool pu = false;
    bool pd = false;
    bool irq = false;
    switch (mode) {
      // TODO: This part is capable of supporting open-drain output, but it
      //   would take slieght-of-hand that is not yet demanded of this driver.
      case GPIOMode::OUTPUT:
        in = false;
        break;
      case GPIOMode::INPUT:
        irq = true;
        break;
      case GPIOMode::INPUT_PULLUP:
        pu = true;
        irq = true;
        break;
      case GPIOMode::INPUT_PULLDOWN:
        pd = true;
        irq = true;
        break;
      default:
        return -2;
    }
    ret = 0;

    // All potential registers that need to be written on GPIO change
    PCAL9539RegId conf_reg   = (pin < 8) ? PCAL9539RegId::CONFIG_PORT0  : PCAL9539RegId::CONFIG_PORT1;  // Configuration register
    PCAL9539RegId pe_reg     = (pin < 8) ? PCAL9539RegId::PULL_ENABLE_0 : PCAL9539RegId::PULL_ENABLE_1; // Pullup/down enable register
    PCAL9539RegId ps_reg     = (pin < 8) ? PCAL9539RegId::PULL_SELECT_0 : PCAL9539RegId::PULL_SELECT_1; // Pullup/down selector register
    PCAL9539RegId latch_reg  = (pin < 8) ? PCAL9539RegId::INPUT_LATCH_0 : PCAL9539RegId::INPUT_LATCH_1; // Input latch register
    PCAL9539RegId irq_reg    = (pin < 8) ? PCAL9539RegId::IRQ_MASK_0    : PCAL9539RegId::IRQ_MASK_1;   // Interupt mask register

    // Grab current values from shadow registers
    uint8_t conf_val  = _get_shadow_value(conf_reg);
    uint8_t pe_val    = _get_shadow_value(pe_reg);
    uint8_t ps_val    = _get_shadow_value(ps_reg);
    uint8_t latch_val = _get_shadow_value(latch_reg);
    uint8_t irq_val   = _get_shadow_value(irq_reg);

    // Restrict to 8-bits.
    if(pin >= 8){ pin = (pin - 0x08); }
    pin = pin & 0x07;

    // Set direction, pull resistors, latch
    conf_val  = (in)       ? (conf_val  | (0x01 << pin))  : (conf_val  & ~(0x01 << pin)); // 0 - OUTPUT, 1 - INPUT
    pe_val    = (pu || pd) ? (pe_val    | (0x01 << pin))  : (pe_val    & ~(0x01 << pin)); // 0 - DISABLED, 1 - ENABLED
    ps_val    = (pu)       ? (ps_val    | (0x01 << pin))  : (ps_val    & ~(0x01 << pin)); // 0 - PULLDOWN, 1 - PULLUP
    latch_val = (in)       ? (latch_val | (0x01 << pin))  : (latch_val & ~(0x01 << pin)); // 0 - Latch disabled, 1 - Latch enabled
    irq_val   = (!irq)     ? (irq_val   | (0x01 << pin))  : (irq_val   & ~(0x01 << pin)); // 0 - IRQ enabled, 1 - IRQ disabled

    if ((0 == ret) & (_get_shadow_value(conf_reg) != conf_val)) {   ret = _write_register(conf_reg, conf_val);   }
    if ((0 == ret) & (_get_shadow_value(pe_reg) != pe_val)) {       ret = _write_register(pe_reg, pe_val);       }
    if ((0 == ret) & (_get_shadow_value(ps_reg) != ps_val)) {       ret = _write_register(ps_reg, ps_val);       }
    if ((0 == ret) & (_get_shadow_value(latch_reg) != latch_val)) { ret = _write_register(latch_reg, latch_val); }
    if ((0 == ret) & (_get_shadow_value(irq_reg) != irq_val)) {     ret = _write_register(irq_reg, irq_val);     }

  }
  return ret;
}

/**
* Returns GPIO pin mode
* @return
*   -2 on unsupported mode
*   -1 on bad pin
*   0 on success
*/
GPIOMode PCAL9539::gpioMode(uint8_t pin) {
  GPIOMode ret = GPIOMode::UNINIT;
  if (pin < 16) { // Valid pin?
    const uint8_t conf_val = _get_shadow_value((pin < 8) ? PCAL9539RegId::CONFIG_PORT0  : PCAL9539RegId::CONFIG_PORT1);
    const uint8_t pe_val   = _get_shadow_value((pin < 8) ? PCAL9539RegId::PULL_ENABLE_0 : PCAL9539RegId::PULL_ENABLE_1);
    const uint8_t ps_val   = _get_shadow_value((pin < 8) ? PCAL9539RegId::PULL_SELECT_0 : PCAL9539RegId::PULL_SELECT_1);

    // Restrict to 7 bits
    if(pin >= 8){ pin = (pin - 0x08); }
    pin = pin & 0x07;

    const bool in = (conf_val & (0x01 << pin)); // Is pin input?
    const bool pe = (pe_val   & (0x01 << pin)); // Is pull enabled?
    const bool ps = (ps_val   & (0x01 << pin)); // Pull up or down?

    if (!in) {     ret = GPIOMode::OUTPUT; }
    else if(!pe) { ret = GPIOMode::INPUT;  }
    else if(ps) {  ret = GPIOMode::INPUT;  }
  }
  return ret;
}



/*******************************************************************************
* Hidden machinery
*******************************************************************************/
/**
* Internal function that safely wraps the callback invocation for pin states.
*
* @return 0 on success, -1 if there is no callback defined.
*/
int8_t PCAL9539::_invoke_pin_callback(uint8_t pin, bool value) {
  int8_t ret = -1;
  pin &= 0x0F;
  if (nullptr != _callbacks[pin]) {
    _callbacks[pin](pin, value?1:0);
    ret = 0;
  }
  return ret;
}


/**
* Setup the low-level pin details.
* Both pins are optional, so this can't fail if no GPIO pins are assigned to the
*   driver.
* If the reset pin is provided, this function will not assert it.
*
* @return 0 on success, -1 on pin setup failure.
*/
int8_t PCAL9539::_ll_pin_init() {
  int8_t ret = 0;
  if (!_pcal_flag(PCAL9539_FLAG_PINS_CONFD)) {
    if (255 != _IRQ_PIN) {
      ret = -1;
      if (0 <= pinMode(_IRQ_PIN, GPIOMode::INPUT_PULLUP)) {
        pcal9539_isr_fired[0] = !readPin(_IRQ_PIN);
        setPinFxn(_IRQ_PIN, IRQCondition::FALLING, pcal9539_isr_0);
        ret = 0;
      }
    }
    if ((0 == ret) & (255 != _RESET_PIN)) {
      ret = -1;
      if (0 <= pinMode(_RESET_PIN, GPIOMode::OUTPUT)) {
        ret = 0;
      }
    }
    _pcal_set_flag(PCAL9539_FLAG_PINS_CONFD, (0 == ret));
  }
  return ret;
}


/**
* Updates the shadow value and dispatches a write operation for the given
*   register and value.
*
* @param reg is the register to write.
* @param val is the new value.
* @return 0 on success, nonzero otherwise.
*/
int8_t PCAL9539::_write_register(PCAL9539RegId reg, uint8_t val) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    ret++;
    _set_shadow_value(reg, val);
    ret = _write_registers(reg, 1);
  }
  return ret;
}


/**
* Writes values in the register shadows to the hardware.
* Does not update the shadows.
*
* @param reg is the register to write.
* @param val is the number of registers to write.
* @return 0 on success, nonzero otherwise.
*/
int8_t PCAL9539::_write_registers(PCAL9539RegId reg, uint8_t len) {
  int8_t ret = -1;
  if (nullptr != _bus) {
    uint8_t reg_idx = (uint8_t) reg;
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = PCAL9539_REG_ADDR[reg_idx];
      op->setBuffer(&_shadows[reg_idx], len);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
*
* @param reg is the register to read.
* @param val is the number of registers to read.
* @return 0 on success
*        -1 on null bus assignment
*        -2 zero length
*        -3 BusOp allocation failure
*        -4 BusAdapter job rejection
*/
int8_t PCAL9539::_read_registers(PCAL9539RegId reg, uint8_t len) {
  int8_t ret = -1;
  if (nullptr != _bus) {
    ret--;
    uint8_t reg_idx = (uint8_t) reg;
    if (len > 0) {
      ret--;
      I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
      if (nullptr != op) {
        ret--;
        op->dev_addr = _dev_addr;
        op->sub_addr = PCAL9539_REG_ADDR[reg_idx];
        op->setBuffer(&_shadows[reg_idx], len);
        if (0 == queue_io_job(op)) {
          ret = 0;
        }
      }
    }
  }
  return ret;
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t PCAL9539::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t PCAL9539::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_ERROR;

  if (!op->hasFault()) {
    uint8_t* buf = op->buffer();
    uint32_t len = op->bufferLen();
    ret = BUSOP_CALLBACK_NOMINAL;
    if (!_pcal_flag(PCAL9539_FLAG_DEVICE_PRESENT)) {
      // With no ID register to check, we construe no failure on bus operation
      //   as the condition "device found".
      _pcal_set_flag(PCAL9539_FLAG_DEVICE_PRESENT);
    }
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint32_t i = 0; i < len; i++) {
          PCAL9539RegId reg   = _reg_id_from_addr(i + op->sub_addr);
          uint8_t     value = *(buf + i);
          switch (reg) {
            case PCAL9539RegId::OUTPUT_PORT0:
              _0_out_dat = value;
              _pcal_clear_flag(PCAL9539_FLAG_WRITE_IN_FLIGHT);
              break;
            case PCAL9539RegId::OUTPUT_PORT1:
              _1_out_dat = value;
              _pcal_clear_flag(PCAL9539_FLAG_WRITE_IN_FLIGHT);
              break;
            case PCAL9539RegId::POLAR_INV_PORT0:
            case PCAL9539RegId::POLAR_INV_PORT1:
            case PCAL9539RegId::CONFIG_PORT0:
            case PCAL9539RegId::CONFIG_PORT1:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_0:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_1:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_2:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_3:
            case PCAL9539RegId::INPUT_LATCH_0:
            case PCAL9539RegId::INPUT_LATCH_1:
            case PCAL9539RegId::PULL_ENABLE_0:
            case PCAL9539RegId::PULL_ENABLE_1:
            case PCAL9539RegId::PULL_SELECT_0:
            case PCAL9539RegId::PULL_SELECT_1:
            case PCAL9539RegId::IRQ_MASK_0:
            case PCAL9539RegId::IRQ_MASK_1:
            case PCAL9539RegId::IRQ_STATUS_0:
            case PCAL9539RegId::IRQ_STATUS_1:
              break;
            default:  // Anything else is invalid.
              break;
          }
        }
        break;

      case BusOpcode::RX:
        for (uint32_t i = 0; i < len; i++) {
          PCAL9539RegId reg   = _reg_id_from_addr(i + op->sub_addr);
          uint8_t     value = *(buf + i);
          switch (reg) {
            case PCAL9539RegId::INPUT_PORT0:
            case PCAL9539RegId::INPUT_PORT1:
              {
                uint8_t dir_val = _get_shadow_value((PCAL9539RegId)((int)reg + 0x06)); // Config registers are offset 6 bytes from the input registers (easy math)
                uint8_t* dat_val = (reg == PCAL9539RegId::INPUT_PORT0) ? &_0_in_dat : &_1_in_dat;
                uint8_t d = (*dat_val ^ value) & dir_val;  // Filter for changes in input values.
                uint8_t cb_base = (reg == PCAL9539RegId::INPUT_PORT0) ? 0 : 8;
                if (d) {
                  for (uint8_t i = 0; i < 8; i++) {
                    if ((d >> i) & 1) {
                      // We don't worry about initialized state here.
                      _invoke_pin_callback((i+cb_base), ((value >> i) & 1));
                    }
                  }
                  *dat_val = value;
                }
                _pcal_clear_flag(PCAL9539_FLAG_READ_IN_FLIGHT);
              }
              pcal9539_isr_fired[0] = (255 != _IRQ_PIN) ? !readPin(_IRQ_PIN) : true;
              break;
            case PCAL9539RegId::OUTPUT_PORT0:
              _0_out_dat = value;
              break;
            case PCAL9539RegId::OUTPUT_PORT1:
              _1_out_dat = value;
              break;

            case PCAL9539RegId::POLAR_INV_PORT0:
            case PCAL9539RegId::POLAR_INV_PORT1:
            case PCAL9539RegId::CONFIG_PORT0:
            case PCAL9539RegId::CONFIG_PORT1:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_0:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_1:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_2:
            case PCAL9539RegId::OUTPUT_DRIVE_STR_3:
            case PCAL9539RegId::INPUT_LATCH_0:
            case PCAL9539RegId::INPUT_LATCH_1:
            case PCAL9539RegId::PULL_ENABLE_0:
            case PCAL9539RegId::PULL_ENABLE_1:
            case PCAL9539RegId::PULL_SELECT_0:
            case PCAL9539RegId::PULL_SELECT_1:
            case PCAL9539RegId::IRQ_MASK_0:
            case PCAL9539RegId::IRQ_MASK_1:
            case PCAL9539RegId::IRQ_STATUS_0:
            case PCAL9539RegId::IRQ_STATUS_1:
              break;
            default:  // Anything else is invalid.
              break;
          }
        }
        break;

      default:
        break;
    }
  }
  return ret;
}



/*******************************************************************************
* Debugging fxns
*******************************************************************************/
/*
*
*/
void PCAL9539::printDebug(StringBuilder* output) {
  StringBuilder::styleHeader1(output, "PCAL9539");
  output->concatf("\tRESET Pin:   %u\n", _RESET_PIN);
  output->concatf("\tIRQ Pin:     %u\n", _IRQ_PIN);
  output->concatf("\tISR fired:   %c\n", pcal9539_isr_fired ? 'y' : 'n');
  output->concatf("\tDev found:   %c\n", devFound() ? 'y' : 'n');
  output->concatf("\tInitialized: %c\n", initialized() ? 'y' : 'n');
  output->concatf("\tPins setup:  %c\n", _pcal_flag(PCAL9539_FLAG_PINS_CONFD) ? 'y' : 'n');
  output->concatf("\tPreserve:    %c\n", preserveOnDestroy() ? 'y' : 'n');
  output->concatf("\t_x_dat:      0x%04x\n", _0_in_dat | ((uint16_t) _1_in_dat << 8));
}


/*
*  Print pin modes and values inside PCAL
*/
void PCAL9539::printPins(StringBuilder* output) {
  if (initialized()) {
    StringBuilder::styleHeader1(output, "PCAL9539 Pins");
    for (uint8_t i = 0; i < 16; i++) {
      output->concatf("\t%2d  %12s  %c\n", i, getPinModeStr(gpioMode(i)), digitalRead(i) ? '1' : '0');
    }
    output->concat("\n");
  }
  else {
    output->concat("PCAL9539 not initialized.\n");
  }
}


/*
* Print register values inside PCAL
*/
void PCAL9539::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < sizeof(_shadows); i++) {
    output->concatf("\t0x%02x:\t0x%02x\n", PCAL9539_REG_ADDR[i], _shadows[i]);
  }
}


/*******************************************************************************
* Console callback
*******************************************************************************/
/**
* @page console-handlers
* @section PCAL9539-tools PCAL9539 tools
*
* This is the console handler for using the PCAL9539 GPIO expander. Called without
*   arguments, this command will print the driver overview.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
* `mode`    | Render pin mode to the console, or set the pin mode. | <pin> <new-mode>
* `val`     | Renders pin details to the console, or set the value. | <pin> [new-value]
* `refresh` | Refresh the register shadows from the hardware. | None
* `regs`    | Prints the full list and contents of register shadows. | None
* `pins`    | Prints the mode and value of each pin | None
*/
int8_t PCAL9539::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd = args->position_trimmed(0);
  uint8_t arg0 = args->position_as_int(1);
  uint8_t arg1 = args->position_as_int(2);

  if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("PCAL9539.init() returns %d\n", init());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("PCAL9539.reset() returns %d\n", reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "mode")) {
    switch (args->count()) {
      case 3:
        switch ((GPIOMode) arg1) {
          case GPIOMode::INPUT:
          case GPIOMode::OUTPUT:
          case GPIOMode::INPUT_PULLUP:
          case GPIOMode::INPUT_PULLDOWN:
          case GPIOMode::OUTPUT_OD:
            text_return->concatf("gpioMode(%u, %s) Returns %d.\n", arg0, getPinModeStr((GPIOMode) arg1), gpioMode(arg0, (GPIOMode)arg1));
            break;
          default:
            text_return->concat("Invalid GPIO mode.\n");
            break;
        }
        break;
      default:
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::INPUT,           getPinModeStr(GPIOMode::INPUT));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::OUTPUT,          getPinModeStr(GPIOMode::OUTPUT));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::INPUT_PULLUP,    getPinModeStr(GPIOMode::INPUT_PULLUP));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::INPUT_PULLDOWN,  getPinModeStr(GPIOMode::INPUT_PULLDOWN));
        text_return->concatf("%u: %s\n", (uint8_t) GPIOMode::OUTPUT_OD,       getPinModeStr(GPIOMode::OUTPUT_OD));
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "val")) {
    text_return->concatf("GPIO %d ", arg0);
    switch (args->count()) {
      case 3:
        {
          int8_t ret0 = this->digitalWrite(arg0, (0 != arg1));
          text_return->concatf("set to %s. Returns %d.\n", (0 != arg1) ? "high" : "low", ret0);
        }
        break;
      default:
        {
          int8_t ret0 = this->digitalRead(arg0);
          text_return->concatf("reads %s.\n", ret0 ? "high" : "low");
        }
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
    text_return->concatf("PCAL9539.refresh() returns %d\n", refresh());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
    printRegs(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "pins")) {
    printPins(text_return);
  }
  else {
    printDebug(text_return);
  }
  return ret;
}
