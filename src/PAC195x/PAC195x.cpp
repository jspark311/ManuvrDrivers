/*
File:   PAC195x.cpp
Author: J. Ian Lindsay
Date:   2023.06.24
*/

#include "PAC195x.h"

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
/*
* Register parameterization. All arrays are indexed against the values encoded
*   by the PAC195xRegID enum.
* NOTE: Values are the native MSB where multibyte.
*/
static const uint8_t PAC195X_ADDR_MAP[PAC195X_REG_COUNT] = {  // Register addresses
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
  0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
  0x18, 0x19, 0x1A, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24,
  0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36,
  0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42,
  0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0xFD, 0xFE, 0xFF
};
static const uint8_t PAC195X_REG_WIDTHS[PAC195X_REG_COUNT] = {  // Widths (in bytes)
  0, 2, 4, 7, 7, 7, 7, 2, 2, 2, 2, 2,
  2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4,
  4, 4, 4, 1, 2, 0, 0, 1, 2, 2, 2, 2,
  1, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2,
  2, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2,
  2, 1, 1, 1, 1, 1, 3, 1, 1, 1, 1, 1
};
static const uint8_t PAC195X_SHADOW_OFFSETS[PAC195X_REG_COUNT] = {  // Shadow offsets
  0,   0,   2,   6,   13,  20,  27,  34,  36,  38,  40,  42,
  44,  46,  48,  50,  52,  54,  56,  58,  60,  62,  64,  66,
  70,  74,  78,  82,  83,  85,  85,  85,  86,  88,  90,  92,
  94,  95,  98,  101, 104, 106, 108, 110, 112, 114, 116, 118,
  120, 122, 125, 128, 131, 134, 136, 138, 140, 142, 144, 146,
  148, 150, 151, 152, 153, 154, 155, 158, 159, 160, 161, 162
};

static const char* const PAC195X_REG_NAME_MAP[PAC195X_REG_COUNT] = {  // Textual representations
  "REFRESH", "CTRL", "ACC_COUNT", "V_ACC_1", "V_ACC_2", "V_ACC_3",
  "V_ACC_4", "V_BUS_1", "V_BUS_2", "V_BUS_3", "V_BUS_4", "V_SENSE_0",
  "V_SENSE_1", "V_SENSE_2", "V_SENSE_3", "V_BUS_AVG_0", "V_BUS_AVG_1", "V_BUS_AVG_2",
  "V_BUS_AVG_3", "V_SENSE_AVG_0", "V_SENSE_AVG_1", "V_SENSE_AVG_2", "V_SENSE_AVG_3", "V_POWER_0",
  "V_POWER_1", "V_POWER_2", "V_POWER_3", "SMBUS_SETTINGS", "NEG_PWR_FSR", "REFRESH_G",
  "REFRESH_V", "SLOW", "CTRL_ACTIVE", "NEG_PWR_FSR_ACTIVE", "CTRL_LATCH", "NEG_PWR_FSR_LATCH",
  "ACCUM_CONFIG", "ALERT_STATUS", "SLOW_ALERT1", "GPIO_ALERT2", "ACC_FULLNESS_LIMITS", "OC_LIMIT_0",
  "OC_LIMIT_1", "OC_LIMIT_2", "OC_LIMIT_3", "UC_LIMIT_0", "UC_LIMIT_1", "UC_LIMIT_2",
  "UC_LIMIT_3", "OP_LIMIT_0", "OP_LIMIT_1", "OP_LIMIT_2", "OP_LIMIT_3", "OV_LIMIT_0",
  "OV_LIMIT_1", "OV_LIMIT_2", "OV_LIMIT_3", "UV_LIMIT_0", "UV_LIMIT_1", "UV_LIMIT_2",
  "UV_LIMIT_3", "OC_LIMIT_SAMPLES", "UC_LIMIT_SAMPLES", "OP_LIMIT_SAMPLES", "OV_LIMIT_SAMPLES", "UV_LIMIT_SAMPLES",
  "ALERT_ENABLE", "ACCUM_CONFIG_ACTIVE", "ACCUM_CONFIG_LATCH", "PROD_ID", "MANU_ID", "REVISION_ID"
};

const uint8_t PAC195x::_reg_addr(const PAC195xRegID id) {           return PAC195X_ADDR_MAP[(const uint8_t) id];        }
const uint8_t PAC195x::_reg_shadow_offset(const PAC195xRegID id) {  return PAC195X_SHADOW_OFFSETS[(const uint8_t) id];  }
const uint8_t PAC195x::_reg_width(const PAC195xRegID id) {          return PAC195X_REG_WIDTHS[(const uint8_t) id];      }
const char* const PAC195x::_reg_name_str(const PAC195xRegID id) {   return PAC195X_REG_NAME_MAP[(const uint8_t) id];    }
const PAC195xRegID PAC195x::_reg_id_from_addr(const uint8_t ADDR) {
  for (uint32_t i = 0; i < PAC195X_REG_COUNT; i++) {
    if (ADDR == PAC195X_ADDR_MAP[i]) return (PAC195xRegID) i;
  }
  return PAC195xRegID::INVALID;
}


/**
* Static function to convert enum to string.
*/
const char* PAC195x::stateStr(const PAC195xState e) {
  switch (e) {
    case PAC195xState::UNINIT:        return "UNINIT";
    case PAC195xState::PREINIT:       return "PREINIT";
    case PAC195xState::RESETTING:     return "RESETTING";
    case PAC195xState::DISCOVERY:     return "DISCOVERY";
    case PAC195xState::REGINIT:       return "REGINIT";
    case PAC195xState::USR_CONF:      return "USR_CONF";
    case PAC195xState::IDLE:          return "IDLE";
    case PAC195xState::READING:       return "READING";
    case PAC195xState::FAULT:         return "FAULT";
    default:   break;
  }
  return "INVALID";
}


// We can have up to two of these in a given system.
// TODO: AbstractPlatform needs a way to call specific objects from setPinFxn()
//   to avoid these sorts of pseudo-singleton patterns. ISR function scope.
#define PAC195X_MAX_INSTANCES    2
volatile static PAC195x* INSTANCES[PAC195X_MAX_INSTANCES] = {0, };

/**
* This is an ISR for ALTERT1 pins.
*/
void pac195x_isr0() {
  ((PAC195x*) INSTANCES[0])->alert1_irq = true;
}

/**
* This is an ISR for ALTERT2 pins.
*/
void pac195x_isr1() {
  ((PAC195x*) INSTANCES[0])->alert2_irq = true;
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
* Constructor specifying every setting.
*/
PAC195x::PAC195x(
  const PAC195xConfig* CONF,
  const uint8_t addr,
  const uint8_t pin_a1,
  const uint8_t pin_a2,
  const uint8_t pin_pwr_dwn
) :
  I2CDevice(addr),
  _ALERT1_PIN(pin_a1), _ALERT2_PIN(pin_a2), _PWR_DWN_PIN(pin_pwr_dwn),
  _desired_conf(CONF),
  _busop_irq_read(BusOpcode::RX, this),
  _busop_dat_read(BusOpcode::RX, this)
{
  bool unslotted = true;
  for (uint8_t i = 0; i < PAC195X_MAX_INSTANCES; i++) {
    if (unslotted) {
      if (nullptr == INSTANCES[i]) {
        //_slot_number = i;
        //INSTANCES[_slot_number] = this;
        unslotted = false;
      }
    }
  }
}


/**
* Destructor
*/
PAC195x::~PAC195x() {
  // TODO: This will almost certainly NEVER be called. Might be smart to wall
  //   it off with a preprocessor directive for pedantic destructors.
  if (255 != _ALERT1_PIN) {
    pinMode(_ALERT1_PIN, GPIOMode::INPUT_PULLUP);
    unsetPinFxn(_ALERT1_PIN);
  }
  if (255 != _ALERT2_PIN) {
    pinMode(_ALERT2_PIN, GPIOMode::INPUT_PULLUP);
    unsetPinFxn(_ALERT2_PIN);
  }
  if (255 != _PWR_DWN_PIN) {
    setPin(_PWR_DWN_PIN, 0);
  }
  //INSTANCES[_slot_number] = nullptr;
}



/*******************************************************************************
* High-level API functions
*******************************************************************************/

/**
* Clears the class, assigns the SPIAdapter, and allows the state machine
*   to march forward.
*
* @return -1 on failure, 0 on success.
*/
int8_t PAC195x::init() {
  int8_t ret = -1;
  if (nullptr != _bus) {
    _clear_registers();
    _busop_irq_read.dev_addr = _dev_addr;
    _busop_irq_read.sub_addr = 0x00;
    _busop_irq_read.setAdapter(_bus);
    _busop_irq_read.shouldReap(false);
    //_busop_irq_read.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) PAC195xRegID::IRQ], 1);

    _busop_dat_read.dev_addr = _dev_addr;
    _busop_dat_read.sub_addr = 0x00;
    _busop_dat_read.setAdapter(_bus);
    _busop_dat_read.shouldReap(false);
    //_busop_dat_read.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) PAC195xRegID::ADCDATA], 4);

    if (PAC195xState::UNINIT == _desired_state) {
      _desired_state = PAC195xState::READING;
    }
    _current_state = PAC195xState::PREINIT;
    _step_state_machine();
    ret = 0;
  }
  return ret;
}


/**
* Handles our configuration after reset.
* Unlocks the registers ahead of any other operation.
* NOTE: Presently sets IRQ pin to be push-pull. So multiple instances of this
*   driver will require independant IRQ pins.
*
* @return
*   -1 on failure to write a register.
*   0  on success.
*/
int8_t PAC195x::_post_reset_fxn() {
  int8_t ret = -1;
  //uint32_t c0_val = 0x00000080;

  // Enable register write.
  //ret = _write_register(PAC195xRegID::LOCK, 0x000000A5);
  //if (0 == ret) {
  //  // Enable fast command, disable IRQ on conversion start, IRQ pin is push-pull.
  //  ret = _write_register(PAC195xRegID::IRQ, 0x00000006);
  //  if (0 == ret) {
  //    ret = _write_register(PAC195xRegID::CONFIG0, c0_val);
  //    if (0 == ret) {
  //      // For simplicity, we select a 32-bit sign-extended data representation with
  //      //   channel identifiers.
  //      ret = _write_register(PAC195xRegID::CONFIG3, 0x000000F0);
  //    }
  //  }
  //}
  return ret;
}


/**
* Some applications of this driver might not want a continuous sample cycle.
*   For such cases, call this function to dispatch a single sample request.
* Applications that operate the sensor in continuous mode need never call this
*   function.
*
* @return 0 on successful triggering, or nonzero on failure.
*/
int8_t PAC195x::trigger() {
  int8_t ret = -1;
  if ((alert1_irq | alert2_irq) & _servicing_irqs()) {
    if (_busop_irq_read.hasFault()) {
      // If there was a bus fault, the BusOp might be left in an unqueuable state.
      // Try to reset the BusOp to satisfy the caller.
      _busop_irq_read.markForRequeue();
    }
    if (_busop_irq_read.isIdle()) {
      ret = _bus->queue_io_job(&_busop_irq_read);
      //isr_fired = false;
    }
  }
  if (scanComplete()) {
    ret = 2;
  }
  return ret;
}


/**
* Setup the low-level pin details. Execution is idempotent.
* No pins are required for this driver. So failure is only possible if one of
*   the provided modes is impossible on the platform for the given pin.
*
* @return 0 if the pin setup is complete. Nonzero otherwise.
*/
int8_t PAC195x::_ll_pin_init() {
  int8_t ret = 0;
  if (!_pac195x_flag(PAC195X_FLAG_PINS_CONFIGURED)) {
    if (255 != _ALERT1_PIN) {
      switch (_desired_conf.gpio1_mode) {
        case PAC195xGPIOMode::GPIO_OUTPUT_OD:
          if (0 == pinMode(_ALERT1_PIN, GPIOMode::INPUT_PULLUP)) {
            // Step 2: ???   But step 3 is "profit".
          }
          else { ret--; }
          break;
        case PAC195xGPIOMode::ALERT_OUTPUT:
          if (0 == pinMode(_ALERT1_PIN, GPIOMode::INPUT_PULLUP)) {
            setPinFxn(_ALERT1_PIN, IRQCondition::FALLING, pac195x_isr0);
          }
          else { ret--; }
          break;
        case PAC195xGPIOMode::GPIO_INPUT:
        case PAC195xGPIOMode::SLOW_INPUT:
          if (0 == pinMode(_ALERT1_PIN, GPIOMode::OUTPUT)) {
            setPin(_ALERT1_PIN, 0);
          }
          else { ret--; }
          break;
        default:
          ret--;
          break;
      }
    }
    if (255 != _ALERT2_PIN) {
      switch (_desired_conf.gpio2_mode) {
        case PAC195xGPIOMode::GPIO_OUTPUT_OD:
          if (0 == pinMode(_ALERT2_PIN, GPIOMode::INPUT_PULLUP)) {
            // Step 2: ???   But step 3 is "profit".
          }
          else { ret--; }
          break;
        case PAC195xGPIOMode::ALERT_OUTPUT:
          if (0 == pinMode(_ALERT2_PIN, GPIOMode::INPUT_PULLUP)) {
            setPinFxn(_ALERT2_PIN, IRQCondition::FALLING, pac195x_isr1);
          }
          else { ret--; }
          break;
        case PAC195xGPIOMode::GPIO_INPUT:
        case PAC195xGPIOMode::SLOW_INPUT:
          if (0 == pinMode(_ALERT2_PIN, GPIOMode::OUTPUT)) {
            setPin(_ALERT2_PIN, 0);
          }
          else { ret--; }
          break;
        default:
          ret--;
          break;
      }
    }
    if (255 != _PWR_DWN_PIN) {
      // If we have MCLK, we need to generate a squarewave on that pin.
      // Otherwise, we hope that the board has an XTAL attached.
      if (0 == pinMode(_PWR_DWN_PIN, GPIOMode::OUTPUT)) {
        setPin(_PWR_DWN_PIN, 1);
      }
      else { ret--; }
    }

    if (-1 == ret) {    _set_fault("_ll_pin_init() failed");    }
    else {  _pac195x_set_flag(PAC195X_FLAG_PINS_CONFIGURED);    }
  }
  return ret;
}


/**
* Causes a full refresh. Update our shadows with the state of the hardware
*   registers.
*
* @return
*    -2 on no bus adapter.
*    -1 on failure to read register.
*    0  on success.
*/
int8_t PAC195x::refresh() {
  int8_t  ret = -1;
  _pac195x_set_flag(PAC195X_FLAG_REFRESH_CYCLE);

  if (0 != ret) {
    _set_fault("Failed to refresh()");
  }
  return ret;
}


/**
* Do we have data for all the channels we asked for?
*
* @return true if so.
*/
bool PAC195x::scanComplete() {
  return false;
};


/**
* Some versions of this family are low-side sensors. At the time of this writing
*   that would be either a PAC1951-2 or a PAC1952-2. All other members of the
*   family are assumed to be high-side.
*
* @return true if the part is a -2 (low-side) variant.
*/
bool PAC195x::lowSideSensor() {
  const uint8_t REG_VAL = _get_shadow_value(PAC195xRegID::PROD_ID);
  return ((REG_VAL == 0x79) | (REG_VAL == 0x7B));
};



/*******************************************************************************
* Register abstraction functions
*******************************************************************************/

/**
* Returns the number of channels this part supports. Should be (1, 2, 3, 4). Any
*   other value is invalid and indicates a need for a register sync (if 0) or
*   the wrong device entirely.
*
* @return The known channel count of the device, based on its report.
*/
uint8_t PAC195x::_channel_count() {
  switch (_get_shadow_value(PAC195xRegID::PROD_ID)) {
    case 0x78:              // PAC1951-1
    case 0x79:   return 1;  // PAC1951-2
    case 0x7A:              // PAC1952-1
    case 0x7B:   return 2;  // PAC1952-2
    case 0x7C:   return 3;  // PAC1953-1
    case 0x7D:   return 4;  // PAC1954-1
  }
  return 0;
}

/**
* Resets the internal register shadows to their default values.
* Also resets internal second-order data to avoid corrupted results.
*
* @return 0 always
*/
void PAC195x::_clear_registers() {
  _flags = _flags & PAC195X_FLAG_RESET_MASK;  // Reset the flags.
  for (uint32_t i = 0; i < sizeof(_reg_shadows); i++) {
    // TODO: We decline to revert registers needed for device identity.
    _reg_shadows[i] = 0;
  }
  read_count       = 0;
  micros_last_read = 0;
}


/*
* Function stores register values as the ADC's native endianess (Big).
*/
int8_t PAC195x::_set_shadow_value(PAC195xRegID r, uint32_t val) {
  int8_t ret = -1;
  uint8_t register_size = _reg_width(r);
  uint8_t* buf_base = (uint8_t*) (_reg_shadows + _reg_shadow_offset(r));
  uint8_t i = 0;
  switch (register_size) {
    case 4:    *(buf_base + i++) = ((uint8_t) (val >> 24) & 0xFF);   // MSB-first
    case 3:    *(buf_base + i++) = ((uint8_t) (val >> 16) & 0xFF);
    case 2:    *(buf_base + i++) = ((uint8_t) (val >> 8)  & 0xFF);
    case 1:    *(buf_base + i++) = ((uint8_t) val & 0xFF);
      ret = 0;
      break;
    default:
      ret = -2;   // Error on unexpected width.
      break;
  }
  return ret;
}


uint32_t PAC195x::_get_shadow_value(PAC195xRegID r) {
  uint32_t ret = 0;
  uint8_t final_reg_val[4] = {0, 0, 0, 0};
  uint8_t register_size = _reg_width(r);
  uint8_t* buf_base = (uint8_t*) (_reg_shadows + _reg_shadow_offset(r));
  switch (register_size) {
    case 4:    final_reg_val[--register_size] = *(buf_base++);   // MSB-first
    case 3:    final_reg_val[--register_size] = *(buf_base++);
    case 2:    final_reg_val[--register_size] = *(buf_base++);
    case 1:    final_reg_val[--register_size] = *(buf_base++);
      ret = *((uint32_t*) final_reg_val);   // Now in native MCU endianness.
      break;
    default:
      break;
  }
  return ret;
}


uint64_t PAC195x::_get_shadow_value64(PAC195xRegID r) {
  uint64_t ret = 0;
  uint8_t final_reg_val[8] = {0, };
  uint8_t register_size = _reg_width(r);
  uint8_t* buf_base = (uint8_t*) (_reg_shadows + _reg_shadow_offset(r));
  switch (register_size) {
    case 7:    final_reg_val[--register_size] = *(buf_base++);   // MSB-first
    case 6:    final_reg_val[--register_size] = *(buf_base++);
    case 5:    final_reg_val[--register_size] = *(buf_base++);
    case 4:    final_reg_val[--register_size] = *(buf_base++);
    case 3:    final_reg_val[--register_size] = *(buf_base++);
    case 2:    final_reg_val[--register_size] = *(buf_base++);
    case 1:    final_reg_val[--register_size] = *(buf_base++);
      ret = *((uint64_t*) final_reg_val);   // Now in native MCU endianness.
      break;
    default:
      break;
  }
  return ret;
}


/**
* Internal function for writing registers.
* Does safety checks on the given value/register combo, updates the shadow, and
*   writes to the part.
*
* @param r The register to be written.
* @param val The value to be written to the register, in host endian representation.
* @return
*   0  on success
*   -1 on I/O failure
*   -2 if no bus adapter
*   -3 if illegal register
*/
int8_t PAC195x::_write_register(PAC195xRegID r, uint32_t val) {
  uint32_t safe_val = 0;
  int8_t ret = -2;
  if (nullptr != _bus) {
    switch (r) {
      // No safety required.
      case PAC195xRegID::REFRESH:
      case PAC195xRegID::CTRL:
      case PAC195xRegID::ACC_COUNT:
      case PAC195xRegID::V_ACC_1:
      case PAC195xRegID::V_ACC_2:
      case PAC195xRegID::V_ACC_3:
      case PAC195xRegID::V_ACC_4:
      case PAC195xRegID::V_BUS_1:
      case PAC195xRegID::V_BUS_2:
      case PAC195xRegID::V_BUS_3:
      case PAC195xRegID::V_BUS_4:
      case PAC195xRegID::V_SENSE_0:
      case PAC195xRegID::V_SENSE_1:
      case PAC195xRegID::V_SENSE_2:
      case PAC195xRegID::V_SENSE_3:
      case PAC195xRegID::V_BUS_AVG_0:
      case PAC195xRegID::V_BUS_AVG_1:
      case PAC195xRegID::V_BUS_AVG_2:
      case PAC195xRegID::V_BUS_AVG_3:
      case PAC195xRegID::V_SENSE_AVG_0:
      case PAC195xRegID::V_SENSE_AVG_1:
      case PAC195xRegID::V_SENSE_AVG_2:
      case PAC195xRegID::V_SENSE_AVG_3:
      case PAC195xRegID::V_POWER_0:
      case PAC195xRegID::V_POWER_1:
      case PAC195xRegID::V_POWER_2:
      case PAC195xRegID::V_POWER_3:
      case PAC195xRegID::SMBUS_SETTINGS:
      case PAC195xRegID::NEG_PWR_FSR:
      case PAC195xRegID::REFRESH_G:
      case PAC195xRegID::REFRESH_V:
      case PAC195xRegID::SLOW:
      case PAC195xRegID::CTRL_ACTIVE:
      case PAC195xRegID::NEG_PWR_FSR_ACTIVE:
      case PAC195xRegID::CTRL_LATCH:
      case PAC195xRegID::NEG_PWR_FSR_LATCH:
      case PAC195xRegID::ACCUM_CONFIG:
      case PAC195xRegID::ALERT_STATUS:
      case PAC195xRegID::SLOW_ALERT1:
      case PAC195xRegID::GPIO_ALERT2:
      case PAC195xRegID::ACC_FULLNESS_LIMITS:
      case PAC195xRegID::OC_LIMIT_0:
      case PAC195xRegID::OC_LIMIT_1:
      case PAC195xRegID::OC_LIMIT_2:
      case PAC195xRegID::OC_LIMIT_3:
      case PAC195xRegID::UC_LIMIT_0:
      case PAC195xRegID::UC_LIMIT_1:
      case PAC195xRegID::UC_LIMIT_2:
      case PAC195xRegID::UC_LIMIT_3:
      case PAC195xRegID::OP_LIMIT_0:
      case PAC195xRegID::OP_LIMIT_1:
      case PAC195xRegID::OP_LIMIT_2:
      case PAC195xRegID::OP_LIMIT_3:
      case PAC195xRegID::OV_LIMIT_0:
      case PAC195xRegID::OV_LIMIT_1:
      case PAC195xRegID::OV_LIMIT_2:
      case PAC195xRegID::OV_LIMIT_3:
      case PAC195xRegID::UV_LIMIT_0:
      case PAC195xRegID::UV_LIMIT_1:
      case PAC195xRegID::UV_LIMIT_2:
      case PAC195xRegID::UV_LIMIT_3:
      case PAC195xRegID::OC_LIMIT_SAMPLES:
      case PAC195xRegID::UC_LIMIT_SAMPLES:
      case PAC195xRegID::OP_LIMIT_SAMPLES:
      case PAC195xRegID::OV_LIMIT_SAMPLES:
      case PAC195xRegID::UV_LIMIT_SAMPLES:
        safe_val = val;
        break;

      // Filter out the unimplemented bits.
      case PAC195xRegID::ALERT_ENABLE:
        safe_val = val & 0x0000000F;
        break;

      // Anything else is not writable.
      default:   return -3;
    }
  }

  if (nullptr != _bus) {
    I2CBusOp* op = (I2CBusOp*) _bus->new_op(BusOpcode::TX, (BusOpCallback*) this);
    ret++;
    if (nullptr != op) {
      _set_shadow_value(r, safe_val);
      c3p_log(LOG_LEV_DEBUG, __PRETTY_FUNCTION__, "PAC195x::_write_register(%u) --> 0x%08x", (uint8_t) r, safe_val);
      op->sub_addr = _reg_addr(r);
      op->setBuffer(_get_shadow_address(r), _reg_width(r));
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Reads the given register.
*
* @param r The register to be read.
* @return
*   -2 on no bus
*   -1 on I/O failure
*   0 on success.
*/
int8_t PAC195x::_read_register(PAC195xRegID r) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    ret++;
    I2CBusOp* op = (I2CBusOp*) _bus->new_op(BusOpcode::RX, (BusOpCallback*) this);
    if (nullptr != op) {
      op->sub_addr = _reg_addr(r);
      op->setBuffer(_get_shadow_address(r), _reg_width(r));
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


uint8_t* PAC195x::_get_shadow_address(const PAC195xRegID REG) {
  return &_reg_shadows[_reg_shadow_offset(REG)];
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* Processes the consequences of a register write.
*
* @param r The register that was freshly written.
* @return BUSOP_CALLBACK_NOMINAL always
*/
int8_t PAC195x::_proc_reg_write(PAC195xRegID r) {
  uint32_t reg_val = _get_shadow_value(r);
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  bool usr_conf_check = false;
  c3p_log(LOG_LEV_DEBUG, __PRETTY_FUNCTION__, "PAC195x::_proc_reg_write(%s)  %u --> 0x%06x", stateStr(_current_state), (uint8_t) r, reg_val);

  switch (r) {
    case PAC195xRegID::REFRESH:                 break;
    case PAC195xRegID::CTRL:                    break;
    case PAC195xRegID::ACC_COUNT:               break;
    case PAC195xRegID::V_ACC_1:                 break;
    case PAC195xRegID::V_ACC_2:                 break;
    case PAC195xRegID::V_ACC_3:                 break;
    case PAC195xRegID::V_ACC_4:                 break;
    case PAC195xRegID::V_BUS_1:                 break;
    case PAC195xRegID::V_BUS_2:                 break;
    case PAC195xRegID::V_BUS_3:                 break;
    case PAC195xRegID::V_BUS_4:                 break;
    case PAC195xRegID::V_SENSE_0:               break;
    case PAC195xRegID::V_SENSE_1:               break;
    case PAC195xRegID::V_SENSE_2:               break;
    case PAC195xRegID::V_SENSE_3:               break;
    case PAC195xRegID::V_BUS_AVG_0:             break;
    case PAC195xRegID::V_BUS_AVG_1:             break;
    case PAC195xRegID::V_BUS_AVG_2:             break;
    case PAC195xRegID::V_BUS_AVG_3:             break;
    case PAC195xRegID::V_SENSE_AVG_0:           break;
    case PAC195xRegID::V_SENSE_AVG_1:           break;
    case PAC195xRegID::V_SENSE_AVG_2:           break;
    case PAC195xRegID::V_SENSE_AVG_3:           break;
    case PAC195xRegID::V_POWER_0:               break;
    case PAC195xRegID::V_POWER_1:               break;
    case PAC195xRegID::V_POWER_2:               break;
    case PAC195xRegID::V_POWER_3:               break;
    case PAC195xRegID::SMBUS_SETTINGS:          break;
    case PAC195xRegID::NEG_PWR_FSR:             break;
    case PAC195xRegID::REFRESH_G:               break;
    case PAC195xRegID::REFRESH_V:               break;
    case PAC195xRegID::SLOW:                    break;
    case PAC195xRegID::CTRL_ACTIVE:             break;
    case PAC195xRegID::NEG_PWR_FSR_ACTIVE:      break;
    case PAC195xRegID::CTRL_LATCH:              break;
    case PAC195xRegID::NEG_PWR_FSR_LATCH:       break;
    case PAC195xRegID::ACCUM_CONFIG:            break;
    case PAC195xRegID::ALERT_STATUS:            break;
    case PAC195xRegID::SLOW_ALERT1:             break;
    case PAC195xRegID::GPIO_ALERT2:             break;
    case PAC195xRegID::ACC_FULLNESS_LIMITS:     break;
    case PAC195xRegID::OC_LIMIT_0:              break;
    case PAC195xRegID::OC_LIMIT_1:              break;
    case PAC195xRegID::OC_LIMIT_2:              break;
    case PAC195xRegID::OC_LIMIT_3:              break;
    case PAC195xRegID::UC_LIMIT_0:              break;
    case PAC195xRegID::UC_LIMIT_1:              break;
    case PAC195xRegID::UC_LIMIT_2:              break;
    case PAC195xRegID::UC_LIMIT_3:              break;
    case PAC195xRegID::OP_LIMIT_0:              break;
    case PAC195xRegID::OP_LIMIT_1:              break;
    case PAC195xRegID::OP_LIMIT_2:              break;
    case PAC195xRegID::OP_LIMIT_3:              break;
    case PAC195xRegID::OV_LIMIT_0:              break;
    case PAC195xRegID::OV_LIMIT_1:              break;
    case PAC195xRegID::OV_LIMIT_2:              break;
    case PAC195xRegID::OV_LIMIT_3:              break;
    case PAC195xRegID::UV_LIMIT_0:              break;
    case PAC195xRegID::UV_LIMIT_1:              break;
    case PAC195xRegID::UV_LIMIT_2:              break;
    case PAC195xRegID::UV_LIMIT_3:              break;
    case PAC195xRegID::OC_LIMIT_SAMPLES:        break;
    case PAC195xRegID::UC_LIMIT_SAMPLES:        break;
    case PAC195xRegID::OP_LIMIT_SAMPLES:        break;
    case PAC195xRegID::OV_LIMIT_SAMPLES:        break;
    case PAC195xRegID::UV_LIMIT_SAMPLES:        break;
    case PAC195xRegID::ALERT_ENABLE:            break;
    case PAC195xRegID::ACCUM_CONFIG_ACTIVE:     break;
    case PAC195xRegID::ACCUM_CONFIG_LATCH:      break;
    case PAC195xRegID::PROD_ID:                 break;
    case PAC195xRegID::MANU_ID:                 break;
    case PAC195xRegID::REVISION_ID:             break;
    default:   // Anything else is an illegal target for write.
      break;
  }

  if (usr_conf_check) {
    if (0 == _apply_usr_config()) {
      // If the user's config is fully written, punch the FSM.
      _step_state_machine();
    }
  }
  return ret;
}


/**
* Processes the consequences of a register read.
*
* @param r The register that was freshly read.
* @return
*   BUSOP_CALLBACK_NOMINAL
*   BUSOP_CALLBACK_RECYCLE
*/
int8_t PAC195x::_proc_reg_read(PAC195xRegID r) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  //c3p_log(LOG_LEV_DEBUG, __PRETTY_FUNCTION__, "PAC195x::_proc_reg_read(%s)  %u --> 0x%02x", stateStr(_current_state), (uint8_t) r, reg_val);

  switch (r) {
    case PAC195xRegID::REFRESH:                 break;
    case PAC195xRegID::CTRL:                    break;
    case PAC195xRegID::ACC_COUNT:               break;
    case PAC195xRegID::V_ACC_1:                 break;
    case PAC195xRegID::V_ACC_2:                 break;
    case PAC195xRegID::V_ACC_3:                 break;
    case PAC195xRegID::V_ACC_4:                 break;
    case PAC195xRegID::V_BUS_1:                 break;
    case PAC195xRegID::V_BUS_2:                 break;
    case PAC195xRegID::V_BUS_3:                 break;
    case PAC195xRegID::V_BUS_4:                 break;
    case PAC195xRegID::V_SENSE_0:               break;
    case PAC195xRegID::V_SENSE_1:               break;
    case PAC195xRegID::V_SENSE_2:               break;
    case PAC195xRegID::V_SENSE_3:               break;
    case PAC195xRegID::V_BUS_AVG_0:             break;
    case PAC195xRegID::V_BUS_AVG_1:             break;
    case PAC195xRegID::V_BUS_AVG_2:             break;
    case PAC195xRegID::V_BUS_AVG_3:             break;
    case PAC195xRegID::V_SENSE_AVG_0:           break;
    case PAC195xRegID::V_SENSE_AVG_1:           break;
    case PAC195xRegID::V_SENSE_AVG_2:           break;
    case PAC195xRegID::V_SENSE_AVG_3:           break;
    case PAC195xRegID::V_POWER_0:               break;
    case PAC195xRegID::V_POWER_1:               break;
    case PAC195xRegID::V_POWER_2:               break;
    case PAC195xRegID::V_POWER_3:               break;
    case PAC195xRegID::SMBUS_SETTINGS:          break;
    case PAC195xRegID::NEG_PWR_FSR:             break;
    case PAC195xRegID::REFRESH_G:               break;
    case PAC195xRegID::REFRESH_V:               break;
    case PAC195xRegID::SLOW:                    break;
    case PAC195xRegID::CTRL_ACTIVE:             break;
    case PAC195xRegID::NEG_PWR_FSR_ACTIVE:      break;
    case PAC195xRegID::CTRL_LATCH:              break;
    case PAC195xRegID::NEG_PWR_FSR_LATCH:       break;
    case PAC195xRegID::ACCUM_CONFIG:            break;
    case PAC195xRegID::ALERT_STATUS:            break;
    case PAC195xRegID::SLOW_ALERT1:             break;
    case PAC195xRegID::GPIO_ALERT2:             break;
    case PAC195xRegID::ACC_FULLNESS_LIMITS:     break;
    case PAC195xRegID::OC_LIMIT_0:              break;
    case PAC195xRegID::OC_LIMIT_1:              break;
    case PAC195xRegID::OC_LIMIT_2:              break;
    case PAC195xRegID::OC_LIMIT_3:              break;
    case PAC195xRegID::UC_LIMIT_0:              break;
    case PAC195xRegID::UC_LIMIT_1:              break;
    case PAC195xRegID::UC_LIMIT_2:              break;
    case PAC195xRegID::UC_LIMIT_3:              break;
    case PAC195xRegID::OP_LIMIT_0:              break;
    case PAC195xRegID::OP_LIMIT_1:              break;
    case PAC195xRegID::OP_LIMIT_2:              break;
    case PAC195xRegID::OP_LIMIT_3:              break;
    case PAC195xRegID::OV_LIMIT_0:              break;
    case PAC195xRegID::OV_LIMIT_1:              break;
    case PAC195xRegID::OV_LIMIT_2:              break;
    case PAC195xRegID::OV_LIMIT_3:              break;
    case PAC195xRegID::UV_LIMIT_0:              break;
    case PAC195xRegID::UV_LIMIT_1:              break;
    case PAC195xRegID::UV_LIMIT_2:              break;
    case PAC195xRegID::UV_LIMIT_3:              break;
    case PAC195xRegID::OC_LIMIT_SAMPLES:        break;
    case PAC195xRegID::UC_LIMIT_SAMPLES:        break;
    case PAC195xRegID::OP_LIMIT_SAMPLES:        break;
    case PAC195xRegID::OV_LIMIT_SAMPLES:        break;
    case PAC195xRegID::UV_LIMIT_SAMPLES:        break;
    case PAC195xRegID::ALERT_ENABLE:            break;
    case PAC195xRegID::ACCUM_CONFIG_ACTIVE:     break;
    case PAC195xRegID::ACCUM_CONFIG_LATCH:      break;
    case PAC195xRegID::PROD_ID:                 break;
    case PAC195xRegID::MANU_ID:                 break;
    case PAC195xRegID::REVISION_ID:             break;
    default:   break;
  }
  return ret;
}


/**
* Called prior to the given bus operation beginning.
* Returning 0 will allow the operation to continue.
* Returning anything else will fail the operation with IO_RECALL.
*   Operations failed this way will have their callbacks invoked as normal.
*
* @param  _op  The bus operation that is about to run.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t PAC195x::io_op_callahead(BusOp* _op) {
  return 0;
}

/**
* When a bus operation completes, it is passed back to its issuing class.
* This driver never reads back from the device. Assume all ops are WRITEs.
* The initialization chain is carried forward in this function, so that we don't
*   swamp the bus queue. Display will enable at the end of the init chain.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t PAC195x::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (op == &_busop_irq_read) {
    // IRQ register read.
    if (alert1_irq | alert2_irq) {
      // If IRQ is still not disasserted, re-read the register.
      ret = BUSOP_CALLBACK_RECYCLE;
    }
  }
  else if (op == &_busop_dat_read) {
    // DATA register read.
    micros_last_read = micros();
    read_count++;
  }
  else {
    // This was register access.
    const PAC195xRegID REG_ID = _reg_id_from_addr(op->sub_addr);
    if (BusOpcode::TX == op->get_opcode()) {
      ret = _proc_reg_write(REG_ID);
    }
    else {
      if (_pac195x_flag(PAC195X_FLAG_REFRESH_CYCLE)) {
        // if (15 > reg_idx) {
        //   reg_idx++;
        //   op->sub_addr = _reg_addr(REG_ID);
        //   op->setBuffer((uint8_t*) &_reg_shadow_offset(REG_ID), _reg_width(REG_ID));
        //   ret = BUSOP_CALLBACK_RECYCLE;
        // }
        // else {   // This is the end of the refresh cycle.
        // }
        _pac195x_clear_flag(PAC195X_FLAG_REFRESH_CYCLE);
      }
      else {
        ret = _proc_reg_read(REG_ID);
      }
    }
  }
  return ret;
}


/**
* This is what is called when the class wants to conduct a transaction on the bus.
* Note that this is different from other class implementations, in that it checks for
*   callback population before clobbering it. This is because this class is also the
*   SPI driver. This might end up being reworked later.
*
* @param  _op  The bus operation to execute.
* @return 0 on success, or appropriate error code.
*/
int8_t PAC195x::queue_io_job(BusOp* _op) {
  // This is the choke-point whereby any parameters to the operation that are
  //   uniform for this driver can be set.
  I2CBusOp* op = (I2CBusOp*) _op;
  return _bus->queue_io_job(op);
}



/*******************************************************************************
* Register abstraction functions
*******************************************************************************/

/**
* Calling this function will cause the user's desired configuration to be
*   written to the registers. It may be necessary to call this several times
*   to achieve complete configuration. So call it until it returns 0.
*
* @return 1 on success with pending I/O
*         0 on success with no changes
*        -1 on failure.
*/
int8_t PAC195x::_apply_usr_config() {
  int8_t ret = -1;
  switch (ret) {
    case -1:   _set_fault("Failed to apply usr config.");    break;
    case 0:    _pac195x_set_flag(PAC195X_FLAG_USER_CONFIG);  break;
    default:   break;
  }
  return ret;
}


/*******************************************************************************
* State machine parts
*******************************************************************************/

/**
* This is NOT a polling loop. It doesn't check for valid conditions for
*   advancing to a given state. It only chooses and imparts the next state.
*
* @return
*   -1 on error
*   0 on nominal polling with stable state achieved
*   1 on unstable state with no advancement on this call
*   2 on advancement of current state toward desired state
*/
int8_t PAC195x::_step_state_machine() {
  int8_t ret = 0;
  if (!stateStable()) {
    // Check for globally-accessible desired states early so we don't repeat
    //   ourselves in several case blocks later on.
    bool continue_looping = true;
    switch (_desired_state) {
      case PAC195xState::RESETTING:
        break;
      default:
        break;
    }

    ret++;
    while (continue_looping) {
      continue_looping = false;   // Abort the loop by default.

      switch (_current_state) {
        case PAC195xState::PREINIT:
          // Regardless of where we are going, we only have one way out of here.
          // Check for memory allocation, pin control
          if (0 == _ll_pin_init()) {  // Configure the pins if they are not already.
            ret = 2;
          }
          else {
            ret = -1;
          }
          break;

        case PAC195xState::RESETTING:
          // If we were given one, check that the IRQ pin pulsed.
          break;

        case PAC195xState::DISCOVERY:
          if (devFound()) {
            if (0 == _post_reset_fxn()) {
              _set_state(PAC195xState::REGINIT);
              ret = 2;
            }
            else {
              _set_fault("_post_reset_fxn() failed");
              ret = -1;
            }
          }
          else {
            _set_fault("Failed to find PAC195x");
            ret = -1;
          }
          break;

        case PAC195xState::REGINIT:
          ret = 1;
          // If a re-init cycle happened after the clock and cal steps, jump
          //   right to reading.
          switch (_desired_state) {
            case PAC195xState::IDLE:     _set_state(PAC195xState::IDLE);     break;
            case PAC195xState::READING:  _set_state(PAC195xState::READING);  break;
            default:                     _set_state(PAC195xState::IDLE);     break;
          }
          continue_looping = true;
          break;

        case PAC195xState::USR_CONF:
          if (configured()) {
            switch (_desired_state) {
              case PAC195xState::IDLE:     _set_state(PAC195xState::IDLE);     break;
              case PAC195xState::READING:  _set_state(PAC195xState::READING);  break;
              default:                     _set_state(PAC195xState::IDLE);     break;
            }
            continue_looping = true;
            ret = 2;
          }
          break;

        case PAC195xState::IDLE:
          switch (_desired_state) {
            case PAC195xState::IDLE:
              break;
            case PAC195xState::READING:
              //if () {  // If the ADC is in one-shot mode, initiate a conversion cycle.
              //}
              //else {
              //}
              break;
            default:
              _set_fault("Illegal _desired_state");
              ret = -1;
              break;
          }
          break;
        case PAC195xState::READING:
          switch (_desired_state) {
            case PAC195xState::REGINIT:
              break;
            case PAC195xState::IDLE:
              break;
            case PAC195xState::READING:
              break;
            case PAC195xState::FAULT:
              _set_fault("Fault entered by outside caller.");
              ret = 2;
              break;
            default:
              _set_fault("Illegal _desired_state");
              ret = -1;
              break;
          }
          break;

        case PAC195xState::UNINIT:  // We can't step our way into this mess. We need to call init().
        case PAC195xState::FAULT:   // We can't step our way out of this mess. We need to be reset().
          break;
        default:
          _set_fault("Illegal _current_state");
          ret = -1;
          break;
      }
    }
  }
  return ret;
}


/**
* Only two cases should not set _current_state by calling this function.
*   1) set_fault(msg);
*   2) Exit from FAULT;
*
* @param e The state that should be stored in _current_state.
* @return 0 always
*/
void PAC195x::_set_state(PAC195xState e) {
  c3p_log(LOG_LEV_DEBUG, __PRETTY_FUNCTION__, "PAC195xState:  %s --> %s", stateStr(_current_state), stateStr(e));
  switch (e) {
    case PAC195xState::PREINIT:
    case PAC195xState::RESETTING:
    case PAC195xState::DISCOVERY:
    case PAC195xState::REGINIT:
    case PAC195xState::USR_CONF:
    case PAC195xState::IDLE:
    case PAC195xState::READING:
      _prior_state = _current_state;
      _current_state = e;
      break;
    case PAC195xState::FAULT:
      _set_fault("Fault entry by outside caller.");
      break;
    case PAC195xState::UNINIT:
    default:
      _set_fault("_set_state(): Illegal state");
      break;
  }
}


/**
* Put the driver into a FAULT state.
*
* @param msg is a debug string to be added to the log.
*/
void PAC195x::_set_fault(const char* msg) {
  c3p_log(LOG_LEV_WARN, __PRETTY_FUNCTION__, "PAC195x fault: %s", msg);
  _prior_state = _current_state;
  _current_state = PAC195xState::FAULT;
}


/*******************************************************************************
* Debugging output functions
*******************************************************************************/


void PAC195x::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < PAC195X_REG_COUNT; i++) {
    const PAC195xRegID REG_ID = (PAC195xRegID) i;
    output->concatf(
      "\t[%u] %19s (0x%02x) = %u\n", i,
      _reg_name_str(REG_ID),
      _reg_addr(REG_ID),
      _get_shadow_value(REG_ID)
    );
  }
}


void PAC195x::printPins(StringBuilder* output) {
  output->concatf("ALERT1:   %u\n", _ALERT1_PIN);
  output->concatf("ALERT2:   %u\n", _ALERT2_PIN);
  output->concatf("PWR_DWN:  %u\n", _PWR_DWN_PIN);
}



void PAC195x::printDebug(StringBuilder* output) {
  StringBuilder prod_str("PAC195");
  if (devFound()) {
    prod_str.concatf("%d-%c", (_channel_count() >> 1), (lowSideSensor() ? '2':'1'));
  }
  else prod_str.concat("x (not found)");

  StringBuilder::styleHeader2(output, (const char*) prod_str.string());
  if (stateStable()) {
    output->concatf("\tState stable:   %s\n", stateStr(_current_state));
  }
  else {
    output->concatf("\tCurrent State:\t%s\n", stateStr(_current_state));
    output->concatf("\tPrior State:  \t%s\n", stateStr(_prior_state));
    output->concatf("\tDesired State:\t%s\n", stateStr(_desired_state));
  }
  if (devFound()) {
    output->concatf("\tChannels:       %u\n", _channel_count());
    output->concatf("\tConfigured:     %c\n", (configured() ? 'y' : 'n'));
    output->concatf("\talert1_irq:     %c\n", (alert1_irq ? 'y' : 'n'));
    output->concatf("\talert2_irq:     %c\n", (alert2_irq ? 'y' : 'n'));
    output->concatf("\tRead count:     %u\n", read_count);
  }
}


/**
* Prints a single channel.
*/
void PAC195xChannel::printChannel(StringBuilder* output) {
  output->concatf(
    "\t%.5fV\t%.5fW\t%.5fWh\n",
    voltage(),
    power(),
    energy()
  );
}


/**
* Prints the values of all enabled channels.
*/
void PAC195x::printChannelValues(StringBuilder* output) {
  if (devFound()) {
    for (uint8_t chan = 1; chan <= _channel_count(); chan++) {
      switch (chan) {
        case 1:   chan_1.printChannel(output);   break;
        case 2:   chan_2.printChannel(output);   break;
        case 3:   chan_3.printChannel(output);   break;
        case 4:   chan_4.printChannel(output);   break;
        default:  break;
      }
    }
  }
  else {
    output->concat("PAC195x not found.\n");
  }
}


/*******************************************************************************
* Console callback
* These are built-in handlers for using this instance via a console.
*******************************************************************************/

/**
* @page console-handlers
* @section pac195x-tools PAC195x tools
*
* This is the console handler for using the PAC195x ADC. If invoked without
*   arguments, it will print channel values, as are being observed by hardware.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
* `irq`     | Force an ISR cycle. | None
* `refresh` | Refresh the register shadows from the hardware. | None
*/
int8_t PAC195x::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  if (0 < args->count()) {
    char* cmd = args->position_trimmed(0);

    if (0 == StringBuilder::strcasecmp(cmd, "info")) {
      printDebug(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
      printRegs(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "busops")) {
      _busop_irq_read.printDebug(text_return);
      _busop_dat_read.printDebug(text_return);
    }

    else if (0 == StringBuilder::strcasecmp(cmd, "pins")) {
      printPins(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
      text_return->concatf("PAC195x refresh() returns %d.\n", refresh());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("PAC195x init() returns %d.\n", init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "trigger")) {
      text_return->concatf("PAC195x trigger() returns %d\n", trigger());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "state")) {
      bool print_state_map = (2 > args->count());
      if (!print_state_map) {
        PAC195xState state = (PAC195xState) args->position_as_int(1);
        switch (state) {
          case PAC195xState::PREINIT:
          case PAC195xState::RESETTING:
          case PAC195xState::DISCOVERY:
          case PAC195xState::REGINIT:
          case PAC195xState::USR_CONF:
          case PAC195xState::IDLE:
          case PAC195xState::READING:
            text_return->concatf("PAC195x setDesiredState(%s)\n", PAC195x::stateStr(state));
            break;
          case PAC195xState::UNINIT:
          case PAC195xState::FAULT:
            text_return->concatf("PAC195x illegal desired state (%s).\n", PAC195x::stateStr(state));
            setDesiredState(state);
            break;
          default:
            print_state_map = true;
            break;
        }
      }

      if (print_state_map) {
        for (uint8_t i = 0; i < 11; i++) {
          PAC195xState ste = (PAC195xState) i;
          text_return->concatf("%u:\t%s", i, PAC195x::stateStr(ste));
          if (ste == _current_state) {
            text_return->concat("  <--- Current\n");
          }
          else if (ste == _prior_state) {
            text_return->concat("  <--- Prior\n");
          }
          else if (ste == _desired_state) {
            text_return->concat("  <--- Desired\n");
          }
          else {
            text_return->concat("\n");
          }
        }
      }
    }
  }
  else {
    printChannelValues(text_return);
  }

  return ret;
}