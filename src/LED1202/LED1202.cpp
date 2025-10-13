/*
File:   LED1202.cpp
Author: J. Ian Lindsay
Date:   2025.04.28

*/

#include "LED1202.h"

// Readability defines...
#define LED1202_REG_COUNT  ((uint8_t) LED1202Register::INVALID)


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
static const float LED1202_MAX_CHAN_CURRENT[12] = {
  0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f,
  0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f, 0.0020f
};

/*
* These are the default register values for the part, with the exception of the
*   DEVID register, which we must read correctly to enable operation.
*/
static const uint8_t DEFAULT_REG_VALUES[LED1202_REG_COUNT] = {
  0x00, 0x00, 0xFF, 0x0F, 0x00, 0x0B, 0x00, 0x00, 0x00,
  0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27, 0x27,
  0x01,
  0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,

  // Pattern data registers.
  0x55, 0x05, 0x55, 0x05, 0x55, 0x05, 0x55, 0x05, 0x55, 0x05, 0x55, 0x05,
  0x55, 0x05, 0x55, 0x05, 0x55, 0x05, 0x55, 0x05, 0x55, 0x05, 0x55, 0x05,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,
  0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F, 0xFF, 0x0F,

  0x00, 0x00,   // <--- Deadzone
  0x00
};



static volatile LED1202* INSTANCE[LED1202_INTERRUPTER_COUNT] = {nullptr};

static float led1202_iled_to_amps(uint8_t val) {
  return (((float) val / 255.0f) * 0.02f);
}

static uint8_t led1202_iled_to_amps(float val) {
  return (12750.0 * val);
}


/*
* This is the ISR for the interrupt pin (if provided).
*/
void LED1202_ISR() {
  // TODO: Only the first device will be able to respond to interrupts.
  if (nullptr != INSTANCE[0]) {
    ((LED1202*) INSTANCE[0])->isr_fxn();
  }
}

// TODO: Bleh... I really dislike htis indirection...
void LED1202::isr_fxn() {
  if (initialized()) {
    _read_registers(LED1202Register::FAULT_STAT_IRQ, 2);
  }
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/
/*
* Constructor. Takes pin numbers as arguments. Uses hardware's maximum channel
*   current values as the safety ceiling.
*/
LED1202::LED1202(const uint8_t I2C_ADDR, const uint8_t IRQ_PIN, I2CAdapter* bus) :
  LED1202(
    I2C_ADDR, IRQ_PIN,
    LED1202_MAX_CHAN_CURRENT,
    bus
  ) {}


/*
* Constructor. Takes pin numbers as arguments. Uses hardware's maximum channel
*   current values as the safety ceiling.
*/
LED1202::LED1202(
  const uint8_t I2C_ADDR,
  const uint8_t IRQ_PIN,
  const float MAX_CURRENT[12],
  I2CAdapter* bus
) : I2CDevice(I2C_ADDR, bus), _IRQ_PIN(IRQ_PIN), _flags(0)
{
  _reset_register_values();
  for (unsigned int i = 0; i < LED1202_INTERRUPTER_COUNT; i++) {
    if (nullptr == INSTANCE[i]) {
      INSTANCE[i] = this;
      break;
    }
  }
  for (unsigned int i = 0; i < 12; i++) {  _MAX_CHAN_MILLIAMPS[i] = MAX_CURRENT[i];  }
}


/*
* Destructor.
*/
LED1202::~LED1202() {
}


int8_t LED1202::init(I2CAdapter* bus) {
  int8_t ret = -1;
  if (nullptr != bus) {
    _bus = bus;
  }
  if (nullptr != _bus) {
    ret--;
    _led1202_clear_flag(LED1202_FLAG_INIT_MASK);
    // Reset the device if it was already initialized.
    if (0 == _read_registers(LED1202Register::DEV_ID, 1)) {
      ret = 0;
    }
  }
  return ret;
}


/*
* Perform a software reset.
*/
int8_t LED1202::reset() {
  _reset_register_values();  // NOTE: This order is important since we don't
  return _send_device_reset();      // want to clobber our own register value in-flight.
}


int8_t LED1202::poll() {
  return 0;
}



bool LED1202::led_open(uint8_t chan) {
  const uint16_t REG_VAL = _get_shadow_value16(LED1202Register::OPEN_LED_L);
  return (REG_VAL & (1 << chan));
}


bool LED1202::led_enabled(uint8_t chan) {
  const uint16_t REG_VAL = _get_shadow_value16(LED1202Register::CHAN_ENABLE_L);
  return (REG_VAL & (1 << chan));
}


int8_t LED1202::led_enabled_mask(uint16_t mask, bool en) {
  const uint16_t MASK_VAL = mask;
  const uint16_t REG_VAL  = (_get_shadow_value16(LED1202Register::CHAN_ENABLE_L) & (~MASK_VAL));
  const uint16_t NEW_VAL  = (en ? (MASK_VAL | REG_VAL) : REG_VAL);
  _set_shadow_value16(LED1202Register::CHAN_ENABLE_L, NEW_VAL);
  if (0 == _write_registers(LED1202Register::CHAN_ENABLE_L, 1)) {
    return 0;
  }
  return -1;
}


int8_t LED1202::led_enabled(uint8_t chan, bool en) {
  const uint16_t MASK_VAL = (1 << chan);
  return led_enabled_mask(MASK_VAL, en);
}


int8_t LED1202::enabled(bool en) {
  _set_shadow_value16(LED1202Register::DEV_ENABLE, (en ? 1 : 0));
  if (0 == _write_registers(LED1202Register::DEV_ENABLE, 1)) {
    return 0;
  }
  return -1;
}

bool LED1202::enabled() {
  return (0 != _get_shadow_value16(LED1202Register::CHAN_ENABLE_L));
}


uint16_t* LED1202::led_pattern(uint8_t chan) {
  return nullptr;
}


float LED1202::led_max_current(uint8_t chan) {
  const LED1202Register REG = (LED1202Register) ((uint8_t) LED1202Register::CS0_CURRENT + chan);
  const uint8_t ILED_VAL = _get_shadow_value(REG);
  float ret = 0.0f;
  if (ILED_VAL < 0x0F) {
    ret = (((float) ILED_VAL / 255.0f) * 0.02f);
  }
  return ret;
}


int8_t LED1202::led_max_current(uint8_t chan, float val) {
  int8_t ret = -1;
  if (chan < 12) {
    ret--;
    if (val <= _MAX_CHAN_MILLIAMPS[chan]) {
      ret--;
      const LED1202Register REG = (LED1202Register) ((uint8_t) LED1202Register::CS0_CURRENT + chan);
      uint8_t iled_val = (12750.0 * val);
      // Shunt all undefined behavior to a zero value.
      iled_val = (0x0E > iled_val) ? 0 : iled_val;
      _chan_milliamps[chan] = iled_val;
      if (devFound()) {
        _set_shadow_value(REG, iled_val);
        if (0 == _write_registers(REG, 1)) {
          ret = 0;
        }
      }
      else {
        ret = 0;
      }
    }
    else {
      c3p_log(LOG_LEV_ERROR, "LED1202", "led_max_current(%u, %.4f) failed. 0.4f is the per-channel max.", chan, val, _MAX_CHAN_MILLIAMPS[chan]);
    }
  }
  else {
    c3p_log(LOG_LEV_ERROR, "LED1202", "led_max_current(%u, %.4f) failed. Channel must be in the range [0, 11].", chan, val);
  }
  return ret;
}


/*******************************************************************************
* Private functions
*******************************************************************************/

int8_t LED1202::_send_device_reset() {
  //if (!_io_in_flight()) {
    _set_shadow_value(LED1202Register::DEV_ENABLE, 0x80);
    if (0 == _write_registers(LED1202Register::DEV_ENABLE, 1)) {
      return 0;
    }
    return -2;
  //}
  return -1;
}


void LED1202::_reset_register_values() {
  for (unsigned int i = 0; i < sizeof(DEFAULT_REG_VALUES); i++) {
    _shadows[i] = DEFAULT_REG_VALUES[i];
  }
}


uint16_t LED1202::_get_shadow_value16(LED1202Register reg) {
  const uint8_t REG_BASE = ((uint8_t) reg);
  const uint8_t lsb = _shadows[REG_BASE + 0];
  const uint8_t msb = _shadows[REG_BASE + 1];
  return (((uint16_t) msb << 8) | lsb);
}


int8_t LED1202::_set_shadow_value16(LED1202Register reg, uint16_t val) {
  const uint8_t REG_BASE = ((uint8_t) reg);
  const uint8_t LSB = (val & 0x00FF);
  const uint8_t MSB = (val >> 8);
  _shadows[REG_BASE + 0] = LSB;
  _shadows[REG_BASE + 1] = MSB;
  return 0;
}



/**
* Reads the given register from the hardware.
*
* @param reg is the register address
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 I/O failure
*/
int8_t LED1202::_read_registers(LED1202Register reg, uint8_t len) {
  int8_t ret = -1;
  I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
  if (nullptr != op) {
    ret--;
    op->dev_addr = _dev_addr;
    op->sub_addr = (int16_t) reg;
    op->setBuffer(&_shadows[(uint8_t) reg], len);
    if (0 == queue_io_job(op)) {
      //_led1202_set_flag(LED1202_FLAG_IO_IN_FLIGHT);
      ret = 0;
    }
  }
  return ret;
}


/**
* Writes the given registers to the hardware.
*
* @param reg is the register address
* @param len is the number of values to write
* @return 0 on success
*        -1 if the device isn't yet found
*        -2 if there is already I/O in flight
*        -3 I/O failure
*/
int8_t LED1202::_write_registers(LED1202Register reg, uint8_t len) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    //if (!_io_in_flight()) {
      I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
      if (nullptr != op) {
        op->dev_addr = (LED1202Register::DEV_ENABLE == reg) ? 0x5C : _dev_addr;
        op->sub_addr = (int16_t) reg;
        op->setBuffer(&_shadows[(uint8_t) reg], len);
        if (0 == queue_io_job(op)) {
          //_led1202_set_flag(LED1202_FLAG_IO_IN_FLIGHT);
          ret = 0;
        }
      }
    //}
  }
  return ret;
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/
/* Transfers always permitted. */
int8_t LED1202::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t LED1202::io_op_callback(BusOp* _op) {
  I2CBusOp* op  = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint32_t len = op->bufferLen();
    const uint8_t SUB_ADDR = (uint8_t) op->sub_addr;
    if ((SUB_ADDR + len) < (uint32_t) LED1202Register::INVALID) {
      for (uint8_t reg_idx = SUB_ADDR; reg_idx < (SUB_ADDR+len); reg_idx++) {
        uint8_t reg_val = _get_shadow_value((LED1202Register) reg_idx);
        switch (op->get_opcode()) {
          case BusOpcode::TX:
          case BusOpcode::RX:
            _led1202_clear_flag(LED1202_FLAG_IO_IN_FLIGHT);
            switch ((LED1202Register) reg_idx) {
              case LED1202Register::DEV_ID:
                if (BusOpcode::RX == op->get_opcode()) {
                  const bool PRIOR_DEV_FOUND   = _led1202_flag(LED1202_FLAG_DEVICE_PRESENT);
                  const bool CURRENT_DEV_FOUND = (0x12 == reg_val);
                  if (PRIOR_DEV_FOUND ^ CURRENT_DEV_FOUND) {
                    _led1202_set_flag(LED1202_FLAG_DEVICE_PRESENT, CURRENT_DEV_FOUND);
                    if (CURRENT_DEV_FOUND) {
                      _set_shadow_value(LED1202Register::CLOCK_CONFIG, 0x02);
                      _write_registers(LED1202Register::CLOCK_CONFIG, 1);
                      _led1202_set_flag(LED1202_CONFIG_READ);




                  // If the clock is enabled, set channel current values, and
                  //   enable the channels that are non-zero.
                  uint16_t enable_mask = 0;
                  for (uint8_t reg_idx_offset = 0; reg_idx_offset < 12; reg_idx_offset++) {
                    const LED1202Register REG = (LED1202Register) ((uint8_t) LED1202Register::CS0_CURRENT + reg_idx_offset);
                    if (0 < _chan_milliamps[reg_idx_offset]) {
                      _set_shadow_value(REG, _chan_milliamps[reg_idx_offset]);
                      enable_mask |= (1 << reg_idx_offset);
                    }
                  }
                  _set_shadow_value16(LED1202Register::CHAN_ENABLE_L, enable_mask);
                  _write_registers(LED1202Register::CS0_CURRENT, 12);
                  _write_registers(LED1202Register::CHAN_ENABLE_L, 2);



                  _led1202_set_flag(LED1202_FLAG_PINS_CONFIGURED);
                  _set_shadow_value(LED1202Register::DEV_ENABLE, 0x01);
                  _write_registers(LED1202Register::DEV_ENABLE, 1);

                  _set_shadow_value(LED1202Register::PATTERN_SEQ_REP, 0xFF);
                  _write_registers(LED1202Register::PATTERN_SEQ_REP, 1);

                  _set_shadow_value(LED1202Register::CONFIG, 0x08);
                  _write_registers(LED1202Register::CONFIG, 1);


                    }
                  }
                }
                break;


              case LED1202Register::DEV_ENABLE:
                if (BusOpcode::TX == op->get_opcode()) {
                  if (reg_val & 1) {
                    _led1202_set_flag(LED1202_CONFIG_WRITTEN);
                  }
                }
                break;
              case LED1202Register::CHAN_ENABLE_L:
                break;
              case LED1202Register::CHAN_ENABLE_H:
                if (BusOpcode::TX == op->get_opcode()) {
                  // _led1202_set_flag(LED1202_FLAG_PINS_CONFIGURED);
                  // _set_shadow_value(LED1202Register::DEV_ENABLE, 0x01);
                  // _write_registers(LED1202Register::DEV_ENABLE, 1);
                }
                break;
              case LED1202Register::CONFIG:
              case LED1202Register::FAULT_STAT_MASK:
              case LED1202Register::FAULT_STAT_IRQ:
              case LED1202Register::OPEN_LED_L:
              case LED1202Register::OPEN_LED_H:
                break;
              case LED1202Register::CS0_CURRENT:
              case LED1202Register::CS1_CURRENT:
              case LED1202Register::CS2_CURRENT:
              case LED1202Register::CS3_CURRENT:
              case LED1202Register::CS4_CURRENT:
              case LED1202Register::CS5_CURRENT:
              case LED1202Register::CS6_CURRENT:
              case LED1202Register::CS7_CURRENT:
              case LED1202Register::CS8_CURRENT:
              case LED1202Register::CS9_CURRENT:
              case LED1202Register::CSA_CURRENT:
              case LED1202Register::CSB_CURRENT:
                break;
              case LED1202Register::PATTERN_SEQ_REP:
              case LED1202Register::PATTERN0_DURATION:
              case LED1202Register::PATTERN1_DURATION:
              case LED1202Register::PATTERN2_DURATION:
              case LED1202Register::PATTERN3_DURATION:
              case LED1202Register::PATTERN4_DURATION:
              case LED1202Register::PATTERN5_DURATION:
              case LED1202Register::PATTERN6_DURATION:
              case LED1202Register::PATTERN7_DURATION:
              case LED1202Register::PATTTERN0_CS0_L:
              case LED1202Register::PATTTERN0_CS0_H:
              case LED1202Register::PATTTERN0_CS1_L:
              case LED1202Register::PATTTERN0_CS1_H:
              case LED1202Register::PATTTERN0_CS2_L:
              case LED1202Register::PATTTERN0_CS2_H:
              case LED1202Register::PATTTERN0_CS3_L:
              case LED1202Register::PATTTERN0_CS3_H:
              case LED1202Register::PATTTERN0_CS4_L:
              case LED1202Register::PATTTERN0_CS4_H:
              case LED1202Register::PATTTERN0_CS5_L:
              case LED1202Register::PATTTERN0_CS5_H:
              case LED1202Register::PATTTERN0_CS6_L:
              case LED1202Register::PATTTERN0_CS6_H:
              case LED1202Register::PATTTERN0_CS7_L:
              case LED1202Register::PATTTERN0_CS7_H:
              case LED1202Register::PATTTERN0_CS8_L:
              case LED1202Register::PATTTERN0_CS8_H:
              case LED1202Register::PATTTERN0_CS9_L:
              case LED1202Register::PATTTERN0_CS9_H:
              case LED1202Register::PATTTERN0_CSA_L:
              case LED1202Register::PATTTERN0_CSA_H:
              case LED1202Register::PATTTERN0_CSB_L:
              case LED1202Register::PATTTERN0_CSB_H:
              case LED1202Register::PATTTERN1_CS0_L:
              case LED1202Register::PATTTERN1_CS0_H:
              case LED1202Register::PATTTERN1_CS1_L:
              case LED1202Register::PATTTERN1_CS1_H:
              case LED1202Register::PATTTERN1_CS2_L:
              case LED1202Register::PATTTERN1_CS2_H:
              case LED1202Register::PATTTERN1_CS3_L:
              case LED1202Register::PATTTERN1_CS3_H:
              case LED1202Register::PATTTERN1_CS4_L:
              case LED1202Register::PATTTERN1_CS4_H:
              case LED1202Register::PATTTERN1_CS5_L:
              case LED1202Register::PATTTERN1_CS5_H:
              case LED1202Register::PATTTERN1_CS6_L:
              case LED1202Register::PATTTERN1_CS6_H:
              case LED1202Register::PATTTERN1_CS7_L:
              case LED1202Register::PATTTERN1_CS7_H:
              case LED1202Register::PATTTERN1_CS8_L:
              case LED1202Register::PATTTERN1_CS8_H:
              case LED1202Register::PATTTERN1_CS9_L:
              case LED1202Register::PATTTERN1_CS9_H:
              case LED1202Register::PATTTERN1_CSA_L:
              case LED1202Register::PATTTERN1_CSA_H:
              case LED1202Register::PATTTERN1_CSB_L:
              case LED1202Register::PATTTERN1_CSB_H:
              case LED1202Register::PATTTERN2_CS0_L:
              case LED1202Register::PATTTERN2_CS0_H:
              case LED1202Register::PATTTERN2_CS1_L:
              case LED1202Register::PATTTERN2_CS1_H:
              case LED1202Register::PATTTERN2_CS2_L:
              case LED1202Register::PATTTERN2_CS2_H:
              case LED1202Register::PATTTERN2_CS3_L:
              case LED1202Register::PATTTERN2_CS3_H:
              case LED1202Register::PATTTERN2_CS4_L:
              case LED1202Register::PATTTERN2_CS4_H:
              case LED1202Register::PATTTERN2_CS5_L:
              case LED1202Register::PATTTERN2_CS5_H:
              case LED1202Register::PATTTERN2_CS6_L:
              case LED1202Register::PATTTERN2_CS6_H:
              case LED1202Register::PATTTERN2_CS7_L:
              case LED1202Register::PATTTERN2_CS7_H:
              case LED1202Register::PATTTERN2_CS8_L:
              case LED1202Register::PATTTERN2_CS8_H:
              case LED1202Register::PATTTERN2_CS9_L:
              case LED1202Register::PATTTERN2_CS9_H:
              case LED1202Register::PATTTERN2_CSA_L:
              case LED1202Register::PATTTERN2_CSA_H:
              case LED1202Register::PATTTERN2_CSB_L:
              case LED1202Register::PATTTERN2_CSB_H:
              case LED1202Register::PATTTERN3_CS0_L:
              case LED1202Register::PATTTERN3_CS0_H:
              case LED1202Register::PATTTERN3_CS1_L:
              case LED1202Register::PATTTERN3_CS1_H:
              case LED1202Register::PATTTERN3_CS2_L:
              case LED1202Register::PATTTERN3_CS2_H:
              case LED1202Register::PATTTERN3_CS3_L:
              case LED1202Register::PATTTERN3_CS3_H:
              case LED1202Register::PATTTERN3_CS4_L:
              case LED1202Register::PATTTERN3_CS4_H:
              case LED1202Register::PATTTERN3_CS5_L:
              case LED1202Register::PATTTERN3_CS5_H:
              case LED1202Register::PATTTERN3_CS6_L:
              case LED1202Register::PATTTERN3_CS6_H:
              case LED1202Register::PATTTERN3_CS7_L:
              case LED1202Register::PATTTERN3_CS7_H:
              case LED1202Register::PATTTERN3_CS8_L:
              case LED1202Register::PATTTERN3_CS8_H:
              case LED1202Register::PATTTERN3_CS9_L:
              case LED1202Register::PATTTERN3_CS9_H:
              case LED1202Register::PATTTERN3_CSA_L:
              case LED1202Register::PATTTERN3_CSA_H:
              case LED1202Register::PATTTERN3_CSB_L:
              case LED1202Register::PATTTERN3_CSB_H:
              case LED1202Register::PATTTERN4_CS0_L:
              case LED1202Register::PATTTERN4_CS0_H:
              case LED1202Register::PATTTERN4_CS1_L:
              case LED1202Register::PATTTERN4_CS1_H:
              case LED1202Register::PATTTERN4_CS2_L:
              case LED1202Register::PATTTERN4_CS2_H:
              case LED1202Register::PATTTERN4_CS3_L:
              case LED1202Register::PATTTERN4_CS3_H:
              case LED1202Register::PATTTERN4_CS4_L:
              case LED1202Register::PATTTERN4_CS4_H:
              case LED1202Register::PATTTERN4_CS5_L:
              case LED1202Register::PATTTERN4_CS5_H:
              case LED1202Register::PATTTERN4_CS6_L:
              case LED1202Register::PATTTERN4_CS6_H:
              case LED1202Register::PATTTERN4_CS7_L:
              case LED1202Register::PATTTERN4_CS7_H:
              case LED1202Register::PATTTERN4_CS8_L:
              case LED1202Register::PATTTERN4_CS8_H:
              case LED1202Register::PATTTERN4_CS9_L:
              case LED1202Register::PATTTERN4_CS9_H:
              case LED1202Register::PATTTERN4_CSA_L:
              case LED1202Register::PATTTERN4_CSA_H:
              case LED1202Register::PATTTERN4_CSB_L:
              case LED1202Register::PATTTERN4_CSB_H:
              case LED1202Register::PATTTERN5_CS0_L:
              case LED1202Register::PATTTERN5_CS0_H:
              case LED1202Register::PATTTERN5_CS1_L:
              case LED1202Register::PATTTERN5_CS1_H:
              case LED1202Register::PATTTERN5_CS2_L:
              case LED1202Register::PATTTERN5_CS2_H:
              case LED1202Register::PATTTERN5_CS3_L:
              case LED1202Register::PATTTERN5_CS3_H:
              case LED1202Register::PATTTERN5_CS4_L:
              case LED1202Register::PATTTERN5_CS4_H:
              case LED1202Register::PATTTERN5_CS5_L:
              case LED1202Register::PATTTERN5_CS5_H:
              case LED1202Register::PATTTERN5_CS6_L:
              case LED1202Register::PATTTERN5_CS6_H:
              case LED1202Register::PATTTERN5_CS7_L:
              case LED1202Register::PATTTERN5_CS7_H:
              case LED1202Register::PATTTERN5_CS8_L:
              case LED1202Register::PATTTERN5_CS8_H:
              case LED1202Register::PATTTERN5_CS9_L:
              case LED1202Register::PATTTERN5_CS9_H:
              case LED1202Register::PATTTERN5_CSA_L:
              case LED1202Register::PATTTERN5_CSA_H:
              case LED1202Register::PATTTERN5_CSB_L:
              case LED1202Register::PATTTERN5_CSB_H:
              case LED1202Register::PATTTERN6_CS0_L:
              case LED1202Register::PATTTERN6_CS0_H:
              case LED1202Register::PATTTERN6_CS1_L:
              case LED1202Register::PATTTERN6_CS1_H:
              case LED1202Register::PATTTERN6_CS2_L:
              case LED1202Register::PATTTERN6_CS2_H:
              case LED1202Register::PATTTERN6_CS3_L:
              case LED1202Register::PATTTERN6_CS3_H:
              case LED1202Register::PATTTERN6_CS4_L:
              case LED1202Register::PATTTERN6_CS4_H:
              case LED1202Register::PATTTERN6_CS5_L:
              case LED1202Register::PATTTERN6_CS5_H:
              case LED1202Register::PATTTERN6_CS6_L:
              case LED1202Register::PATTTERN6_CS6_H:
              case LED1202Register::PATTTERN6_CS7_L:
              case LED1202Register::PATTTERN6_CS7_H:
              case LED1202Register::PATTTERN6_CS8_L:
              case LED1202Register::PATTTERN6_CS8_H:
              case LED1202Register::PATTTERN6_CS9_L:
              case LED1202Register::PATTTERN6_CS9_H:
              case LED1202Register::PATTTERN6_CSA_L:
              case LED1202Register::PATTTERN6_CSA_H:
              case LED1202Register::PATTTERN6_CSB_L:
              case LED1202Register::PATTTERN6_CSB_H:
              case LED1202Register::PATTTERN7_CS0_L:
              case LED1202Register::PATTTERN7_CS0_H:
              case LED1202Register::PATTTERN7_CS1_L:
              case LED1202Register::PATTTERN7_CS1_H:
              case LED1202Register::PATTTERN7_CS2_L:
              case LED1202Register::PATTTERN7_CS2_H:
              case LED1202Register::PATTTERN7_CS3_L:
              case LED1202Register::PATTTERN7_CS3_H:
              case LED1202Register::PATTTERN7_CS4_L:
              case LED1202Register::PATTTERN7_CS4_H:
              case LED1202Register::PATTTERN7_CS5_L:
              case LED1202Register::PATTTERN7_CS5_H:
              case LED1202Register::PATTTERN7_CS6_L:
              case LED1202Register::PATTTERN7_CS6_H:
              case LED1202Register::PATTTERN7_CS7_L:
              case LED1202Register::PATTTERN7_CS7_H:
              case LED1202Register::PATTTERN7_CS8_L:
              case LED1202Register::PATTTERN7_CS8_H:
              case LED1202Register::PATTTERN7_CS9_L:
              case LED1202Register::PATTTERN7_CS9_H:
              case LED1202Register::PATTTERN7_CSA_L:
              case LED1202Register::PATTTERN7_CSA_H:
              case LED1202Register::PATTTERN7_CSB_L:
              case LED1202Register::PATTTERN7_CSB_H:
                break;
              case LED1202Register::CLOCK_CONFIG:
                // if (reg_val & 0x03) {
                //   // If the clock is enabled, set channel current values, and
                //   //   enable the channels that are non-zero.
                //   uint16_t enable_mask = 0;
                //   for (uint8_t reg_idx_offset = 0; reg_idx_offset < 12; reg_idx_offset++) {
                //     const LED1202Register REG = (LED1202Register) ((uint8_t) LED1202Register::CS0_CURRENT + reg_idx_offset);
                //     if (0 < _chan_milliamps[reg_idx_offset]) {
                //       _set_shadow_value(REG, _chan_milliamps[reg_idx_offset]);
                //       enable_mask |= (1 << reg_idx_offset);
                //     }
                //   }
                //   _set_shadow_value16(LED1202Register::CHAN_ENABLE_L, enable_mask);
                //   _write_registers(LED1202Register::CS0_CURRENT, 12);
                //   _write_registers(LED1202Register::CHAN_ENABLE_L, 2);
                // }
                break;
              default:
                break;
            }
            break;

          default: break;
        }
      }
    }
  }
  return ret;
}



/*******************************************************************************
* Console callback
* These are built-in handlers for using this instance via a console.
*******************************************************************************/

/*
* Dump this item to the log.
*/
void LED1202::printDebug(StringBuilder* output) {
  output->concatf("\tDev found:         %c\n", devFound() ? 'y' :'n');
  output->concatf("\tinit_complete:     %c\n", initialized() ? 'y' :'n');
  output->concatf("\tpins configured:   %c\n", _led1202_flag(LED1202_FLAG_PINS_CONFIGURED) ? 'y' :'n');
  output->concatf("\tconf read:         %c\n", _led1202_flag(LED1202_CONFIG_READ) ? 'y' :'n');
  output->concatf("\tconf written:      %c\n", _led1202_flag(LED1202_CONFIG_WRITTEN) ? 'y' :'n');
  if (255 != _IRQ_PIN) {  output->concatf("\tIRQ pin:           %d\n", _IRQ_PIN);  }
}



void LED1202::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < LED1202_REG_COUNT; i++) {
    const LED1202Register REG_ID = (LED1202Register) i;
    const uint8_t REG_VAL = _get_shadow_value(REG_ID);
    output->concatf(
      "\t[%3u]  0x%02x\n",
      i,
      REG_VAL
    );
  }
}


void LED1202::printChannelValues(StringBuilder* output, int8_t chan) {
  const int8_t CHAN_START = ((0 <= chan) & (12 > chan)) ? chan : 0;
  const int8_t CHAN_STOP  = ((0 <= chan) & (12 > chan)) ? (chan+1) : 12;
  output->concat("\tChan      OPN   EN   mA(setting)  mA(real)\n");
  output->concat("\t----------------------------------------\n");
  for (uint8_t i = CHAN_START; i < CHAN_STOP; i++) {
    const LED1202Register REG = (LED1202Register) ((uint8_t) LED1202Register::CS0_CURRENT + i);
    output->concatf("\t%2u:       %c    %c       %.2f     %.2f\n",
      i,
      (led_open(i) ? 'y' :'n'),
      (led_enabled(i) ? 'y' :'n'),
      1000 * led1202_iled_to_amps(_chan_milliamps[i]),
      1000 * led1202_iled_to_amps(_get_shadow_value(REG))
    );
  }
}


/**
* @page console-handlers
* @section led1202-tools LED1202 tools
*
* This is the console handler for using the LED1202 LED driver. If invoked without
*   arguments, it will print channel values, as are being observed by hardware.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
*/
int8_t LED1202::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int   ret  = 0;
  char* cmd  = args->position_trimmed(0);
  int   chan = (1 < args->count()) ? args->position_as_int(1) : -1;

  if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
    printRegs(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("LED1202 reset() returns %d.\n", reset());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("LED1202 init() returns %d.\n", init(_bus));
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "current")) {
    float arg1 = args->position_as_double(2);
    text_return->concatf("LED1202 led_max_current(%u, %.2f) returns %d.\n", chan, arg1, led_max_current(chan, arg1));
  }
  else {
    printChannelValues(text_return, chan);
  }

  return ret;
}
