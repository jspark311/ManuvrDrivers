/*
* This file started out as a SparkFun driver. See header file for
*   my change list.
*                                                            ---J. Ian Lindsay
*/

/*
  This is a library written for the Panasonic Grid-EYE AMG88
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14568

  Written by Nick Poole @ SparkFun Electronics, January 11th, 2018

  The GridEYE from Panasonic is an 8 by 8 thermopile array capable
  of detecting temperature remotely at 64 discrete points.

  This library handles communication with the GridEYE and provides
  methods for manipulating temperature registers in Celsius,
  Fahrenheit and raw values.

  https://github.com/sparkfun/SparkFun_GridEYE_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AMG88xx.h"
#include <math.h>

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

/* The addresses of the registers named in the enum class AMG88XXRegID. */
static const uint8_t AMG88XX_ADDR_MAP[] = {
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08,
  0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x1F
};

/* Is a given AMG88XXRegID writable? */
static const bool AMG88XX_REG_WRITABLE_MAP[] = {
  true, true, true, true, false, true,  true,  true,
  true, true, true, true, true,  false, false, true
};

/* Is a given AMG88XXRegID readable? */
static const bool AMG88XX_REG_READABLE_MAP[] = {
  true, false, true, true, true, false, true, true,
  true, true,  true, true, true, true,  true, true
};


/* Converts a register address back into an enum. */
AMG88XXRegID GridEYE::_reg_id_from_addr(const uint8_t reg_addr) {
  switch (reg_addr & 0x3F) {
    case 0x00:   return AMG88XXRegID::POWER_CONTROL;
    case 0x01:   return AMG88XXRegID::RESET;
    case 0x02:   return AMG88XXRegID::FRAMERATE;
    case 0x03:   return AMG88XXRegID::INT_CONTROL;
    case 0x04:   return AMG88XXRegID::STATUS;
    case 0x05:   return AMG88XXRegID::STATUS_CLEAR;
    case 0x07:   return AMG88XXRegID::AVERAGE;
    case 0x08:   return AMG88XXRegID::INT_LEVEL_UPPER_LSB;
    case 0x09:   return AMG88XXRegID::INT_LEVEL_UPPER_MSB;
    case 0x0A:   return AMG88XXRegID::INT_LEVEL_LOWER_LSB;
    case 0x0B:   return AMG88XXRegID::INT_LEVEL_LOWER_MSB;
    case 0x0C:   return AMG88XXRegID::INT_LEVEL_HYST_LSB;
    case 0x0D:   return AMG88XXRegID::INT_LEVEL_HYST_MSB;
    case 0x0E:   return AMG88XXRegID::THERMISTOR_LSB;
    case 0x0F:   return AMG88XXRegID::THERMISTOR_MSB;
    case 0x1F:   return AMG88XXRegID::RESERVED_AVERAGE;
  }
  return AMG88XXRegID::INVALID;
}



volatile static bool amg_irq_fired = false;

/* ISR */
void amg_isr_fxn() {   amg_irq_fired = true;   }



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/*
* Constructor
*/
GridEYE::GridEYE(uint8_t addr, uint8_t irq) :
  I2CDevice(addr), _IRQ_PIN(irq),
  _frame_read(BusOpcode::RX, addr, TEMPERATURE_REGISTER_START, (uint8_t*) _frame, 128) {}

/*
* Destructor
*/
GridEYE::~GridEYE() {}


/*
* Init the sensor on the given bus.
* Successful init() means the sensor is running at 10FPS.
*/
int8_t GridEYE::init(I2CAdapter* b) {
  int8_t ret = -1;
  _ll_pin_init();  // Idempotent. Ok to call twice.
  _frame_read.shouldReap(false);
  _amg_clear_flag(GRIDEYE_FLAG_INITIALIZED);

  if (nullptr != b) {
    _bus = b;
    if (0 == setFramerate10FPS()) {
      if (0 == enabled(true)) {
        _amg_set_flag(GRIDEYE_FLAG_INITIALIZED);
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Performs a software reset.
*
* @return
*   0 on success
*   -1 on failure
*   -2 on no bus
*/
int8_t GridEYE::reset() {
  int8_t ret = -2;
  _flags = _flags & GRIDEYE_FLAG_RESET_MASK;
  if (nullptr != _bus) {
    for (uint8_t i = 0; i < 64; i++) {
      _frame[i] = 0;   // Zero the local framebuffer.
    }
    for (uint8_t i = 0; i < 16; i++) {
      _shadows[i] = 0; // Zero the register shadows.
    }
    ret = (0 == _write_register(AMG88XXRegID::RESET, 0x3F)) ? 0 : -1;
  }
  return ret;
}


/*
* Poll the class for updates.
*
* @return
*   -3 if not initialized and enabled.
*   -1 if the frame needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if a frame was read and is waiting.
*/
int8_t GridEYE::poll() {
  int8_t ret = -3;
  if (initialized() && enabled()) {
    ret = 0;
    if (255 != _IRQ_PIN) {
      if (amg_irq_fired) {
        // If we somehow fail to read the STATUS register, this ISR condition
        //   will try again next poll().
        amg_irq_fired = (0 != _read_registers(AMG88XXRegID::STATUS, 1));
      }
    }

    uint32_t now = millis();
    uint32_t r_interval = isFramerate10FPS() ? 100 : 1000;
    if ((now - _last_read) >= r_interval) {
      ret = (0 == _read_full_frame()) ? 1 : -1;
      // TODO: Optionally get the die temperature.
    }
  }
  return ret;
}


/**
*
*/
float GridEYE::getPixelTemperature(uint8_t pixel) {
  if (_amg_flag(GRIDEYE_FLAG_FRAME_UPDATED)) {
    _amg_clear_flag(GRIDEYE_FLAG_FRAME_UPDATED);
  }
  int16_t temperature = _dev_int16_to_float(getPixelRaw(pixel));
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
float GridEYE::getDeviceTemperature() {
  int16_t temperature = _dev_int16_to_float(getDeviceTemperatureRaw());
  return _normalize_units_returned(temperature * 0.0625);
}


/**
*
*/
int16_t GridEYE::getDeviceTemperatureRaw() {
  uint8_t reg_idx = (uint8_t) AMG88XXRegID::THERMISTOR_LSB;
  return (int16_t) (_shadows[reg_idx] + ((uint16_t) _shadows[reg_idx] << 8));
}


/**
*
*/
int8_t GridEYE::setFramerate1FPS() {
  return _write_register(AMG88XXRegID::FRAMERATE, 1);
}


/**
*
*/
int8_t GridEYE::setFramerate10FPS() {
  return _write_register(AMG88XXRegID::FRAMERATE, 0);
}


/**
*
*/
int8_t GridEYE::enabled(bool x) {
  return _write_register(AMG88XXRegID::POWER_CONTROL, x ? 0x00 : 0x10);
}



/**
*
*/
int8_t GridEYE::standby60seconds(){
  return _write_register(AMG88XXRegID::POWER_CONTROL, 0x20);
}


/**
*
*/
int8_t GridEYE::standby10seconds(){
  return _write_register(AMG88XXRegID::POWER_CONTROL, 0x21);
}


/**
*
*/
int8_t GridEYE::interruptPinEnable(){
  uint8_t ICRValue = _shadows[(uint8_t) AMG88XXRegID::INT_CONTROL];
  ICRValue |= (1 << 0);
  return _write_register(AMG88XXRegID::INT_CONTROL, ICRValue & 0xFF);
}


/**
*
*/
int8_t GridEYE::interruptPinDisable(){
  uint8_t ICRValue = _shadows[(uint8_t) AMG88XXRegID::INT_CONTROL];
  ICRValue &= ~(1 << 0);
  return _write_register(AMG88XXRegID::INT_CONTROL, ICRValue & 0xFF);
}


/**
*
*/
int8_t GridEYE::setInterruptModeAbsolute(){
  uint8_t ICRValue = _shadows[(uint8_t) AMG88XXRegID::INT_CONTROL];
  ICRValue |= (1 << 1);
  return _write_register(AMG88XXRegID::INT_CONTROL, ICRValue & 0xFF);
}


/**
*
*/
int8_t GridEYE::setInterruptModeDifference(){
  uint8_t ICRValue = _shadows[(uint8_t) AMG88XXRegID::INT_CONTROL];
  ICRValue &= ~(1 << 1);
  return _write_register(AMG88XXRegID::INT_CONTROL, ICRValue & 0xFF);
}


/**
*
*/
bool GridEYE::interruptPinEnabled(){
  uint8_t ICRValue = _shadows[(uint8_t) AMG88XXRegID::INT_CONTROL];
  return (ICRValue & (1 << 0));
}


/**
*
*/
bool GridEYE::interruptFlagSet(){
  uint8_t StatRegValue = _shadows[(uint8_t) AMG88XXRegID::STATUS];
  return (StatRegValue & (1 << 1));
}


/**
*
*/
bool GridEYE::pixelTemperatureOutputOK() {
  uint8_t StatRegValue = _shadows[(uint8_t) AMG88XXRegID::STATUS];
  return !(StatRegValue & (1 << 2));
}


/**
*
*/
bool GridEYE::deviceTemperatureOutputOK(){
  uint8_t StatRegValue = _shadows[(uint8_t) AMG88XXRegID::STATUS];
  return !(StatRegValue & (1 << 3));
}

/**
*
*/
int8_t GridEYE::clearPixelTemperatureOverflow(){
  return _write_register(AMG88XXRegID::STATUS_CLEAR, 0x04);
}


/**
*
*/
int8_t GridEYE::clearDeviceTemperatureOverflow(){
  return _write_register(AMG88XXRegID::STATUS_CLEAR, 0x08);
}


/**
*
*/
int8_t GridEYE::clearAllOverflow(){
  return _write_register(AMG88XXRegID::STATUS_CLEAR, 0x0C);
}


/**
*
*/
int8_t GridEYE::clearAllStatusFlags(){
  return _write_register(AMG88XXRegID::STATUS_CLEAR, 0x0E);
}


/**
*
*/
bool GridEYE::pixelInterruptSet(uint8_t pixel) {
  uint8_t interruptTableRegister = (pixel >> 3);
  uint8_t pixelPosition = (pixel & 0x07);
  int16_t interruptTableRow = _pixel_interrupts[interruptTableRegister];
  return (interruptTableRow & (1 << pixelPosition));
}


int8_t GridEYE::_read_pixel_int_states() {
  int8_t ret = -1;
  I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
  if (nullptr != op) {
    op->dev_addr = _dev_addr;
    op->sub_addr = INT_TABLE_REGISTER_INT0_START;
    op->setBuffer(_pixel_interrupts, 8);
    if (0 == queue_io_job(op)) {
      ret = 0;
    }
  }
  return ret;
}


/**
* TODO: Need to re-read the datasheet....
*/
int8_t GridEYE::movingAverage(bool x) {
  int8_t ret = -1;
  if (0 == _write_register(AMG88XXRegID::RESERVED_AVERAGE, 0x50)) {
    if (0 == _write_register(AMG88XXRegID::RESERVED_AVERAGE, 0x45)) {
      if (0 == _write_register(AMG88XXRegID::RESERVED_AVERAGE, 0x57)) {
        if (0 == _write_register(AMG88XXRegID::AVERAGE, x ? 0x20 : 0x00)) {
          if (0 == _write_register(AMG88XXRegID::RESERVED_AVERAGE, 0x00)) {
            ret = 0;
          }
        }
      }
    }
  }
  return ret;
}


/**
* Performs a refresh of the non-data registers.
*
* @return
*   0 on success
*   -1 on failure
*/
int8_t GridEYE::refresh() {
  int8_t ret = -1;
  if (0 == _read_registers(AMG88XXRegID::POWER_CONTROL, 1)) {
    if (0 == _read_registers(AMG88XXRegID::FRAMERATE, 3)) {
      if (0 == _read_registers(AMG88XXRegID::AVERAGE, 10)) {
        ret = 0;
      }
    }
  }
  return ret;
}



/**
*
*/
int8_t GridEYE::setUpperInterruptValue(float degrees){
  float DegreesC = _normalize_units_accepted(degrees);
  int16_t temperature = _native_float_to_dev_int16(DegreesC);
  return setUpperInterruptValueRaw(temperature);
}


/**
*
*/
int8_t GridEYE::setUpperInterruptValueRaw(int16_t regValue){
  return _write_registers(AMG88XXRegID::INT_LEVEL_UPPER_LSB, (uint16_t) regValue);
}


/**
*
*/
int8_t GridEYE::setLowerInterruptValue(float degrees){
  float DegreesC = _normalize_units_accepted(degrees);
  int16_t temperature = _native_float_to_dev_int16(DegreesC);
  return setLowerInterruptValueRaw(temperature);
}


/**
*
*/
int8_t GridEYE::setLowerInterruptValueRaw(int16_t regValue){
  return _write_registers(AMG88XXRegID::INT_LEVEL_LOWER_LSB, (uint16_t) regValue);
}


/**
*
*/
int8_t GridEYE::setInterruptHysteresis(float degrees){
  float DegreesC = _normalize_units_accepted(degrees);
  int16_t temperature = _native_float_to_dev_int16(DegreesC);
  return setInterruptHysteresisRaw(temperature);
}


/**
*
*/
int8_t GridEYE::setInterruptHysteresisRaw(int16_t regValue) {
  return _write_registers(AMG88XXRegID::INT_LEVEL_HYST_LSB, (uint16_t) regValue);
}


/**
*
*/
float GridEYE::getUpperInterruptValue() {
  int16_t temperature = _dev_int16_to_float(getUpperInterruptValueRaw());
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
int16_t GridEYE::getUpperInterruptValueRaw() {
  uint8_t reg_idx = (uint8_t) AMG88XXRegID::INT_LEVEL_UPPER_LSB;
  return (int16_t) (_shadows[reg_idx] + ((uint16_t) _shadows[reg_idx] << 8));
}


/**
*
*/
float GridEYE::getLowerInterruptValue() {
  int16_t temperature = _dev_int16_to_float(getLowerInterruptValueRaw());
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
int16_t GridEYE::getLowerInterruptValueRaw() {
  uint8_t reg_idx = (uint8_t) AMG88XXRegID::INT_LEVEL_LOWER_LSB;
  return (int16_t) (_shadows[reg_idx] + ((uint16_t) _shadows[reg_idx] << 8));
}


/**
*
*/
float GridEYE::getInterruptHysteresis() {
  int16_t temperature = _dev_int16_to_float(getInterruptHysteresisRaw());
  return _normalize_units_returned(temperature * 0.25);
}


/**
*
*/
int16_t GridEYE::getInterruptHysteresisRaw() {
  uint8_t reg_idx = (uint8_t) AMG88XXRegID::INT_LEVEL_HYST_LSB;
  return (int16_t) (_shadows[reg_idx] + ((uint16_t) _shadows[reg_idx] << 8));
}




/**
* Convenience fxn.
*
* @return
*   0 if register index is out of bounds.
*   The register value on success, which might also be 0.
*/
uint8_t GridEYE::_get_shadow_value(AMG88XXRegID reg) {
  uint8_t reg_idx = (uint8_t) reg;
  if (reg_idx < sizeof(_shadows)) {
    return _shadows[reg_idx];
  }
  return 0;
}


/**
* Convenience fxn.
*
* @return
*   0 on success
*   -1 if register index is out of bounds.
*/
int8_t GridEYE::_set_shadow_value(AMG88XXRegID reg, uint8_t val) {
  uint8_t reg_idx = (uint8_t) reg;
  if (reg_idx < sizeof(_shadows)) {
    _shadows[reg_idx] = val;
    return 0;
  }
  return -1;
}


/**
*
*/
int8_t GridEYE::_write_register(AMG88XXRegID reg, uint8_t val) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    uint8_t reg_idx = (uint8_t) reg;
    if (!AMG88XX_REG_WRITABLE_MAP[reg_idx]) {
      return -3;
    }
    ret++;
    _set_shadow_value(reg, val);
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = AMG88XX_ADDR_MAP[reg_idx];
      op->setBuffer(&_shadows[reg_idx], 1);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::_write_registers(AMG88XXRegID reg, uint16_t val) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    uint8_t reg_idx = (uint8_t) reg;
    if (!AMG88XX_REG_WRITABLE_MAP[reg_idx]) {
      return -3;
    }
    ret++;
    // Multibyte values are always little-endian in the hardware.
    _set_shadow_value((AMG88XXRegID) (reg_idx + 0), (uint8_t) (val & 0x00FF));
    _set_shadow_value((AMG88XXRegID) (reg_idx + 1), (uint8_t) (val >> 8));
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = AMG88XX_ADDR_MAP[reg_idx];
      op->setBuffer(&_shadows[reg_idx], 2);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
*
*/
int8_t GridEYE::_read_registers(AMG88XXRegID reg, uint8_t len) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    uint8_t reg_idx = (uint8_t) reg;
    if (len > 0) {
      for (uint8_t i = 0; i < len; i++) {
        if (!AMG88XX_REG_READABLE_MAP[reg_idx + i]) {
          return -3;
        }
      }
      ret++;
      I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
      if (nullptr != op) {
        op->dev_addr = _dev_addr;
        op->sub_addr = AMG88XX_ADDR_MAP[reg_idx];
        op->setBuffer(&_shadows[reg_idx], len);
        if (0 == queue_io_job(op)) {
          ret = 0;
        }
      }
    }
  }
  return ret;
}


/**
* Idempotently setup the low-level pin details. Since all non-bus pins on this
*   device are optional, failure isn't possible.
*
* @return 0 always
*/
int8_t GridEYE::_ll_pin_init() {
  if (!_amg_flag(GRIDEYE_FLAG_PINS_CONFIGURED)) {
    if (255 != _IRQ_PIN) {
      pinMode(_IRQ_PIN, GPIOMode::INPUT);
      setPinFxn(_IRQ_PIN, IRQCondition::FALLING, amg_isr_fxn);
      interruptPinEnable();
    }
    _amg_set_flag(GRIDEYE_FLAG_PINS_CONFIGURED);
  }
  return 0;
}


/**
* Read the entire data frame.
*
* @return
*   0 on successful dispatch of the read operation.
*   -1 if a read is in progress, and the driver needs to be patient.
*/
int8_t GridEYE::_read_full_frame() {
  int8_t ret = -1;
  if (_frame_read.isIdle()) {
    ret--;
    _frame_read.dev_addr = _dev_addr;
    if (0 == queue_io_job(&_frame_read)) {
      ret = 0;
    }
  }
  return ret;
}


/**
* Used to automatically convert from Fahrenheit if that is how the class is
*   configured to operate.
*/
float GridEYE::_normalize_units_accepted(float temperature) {
  if (unitsFahrenheit()) {
    temperature = (temperature - 32) / 1.8;
  }
  return temperature;
}


/**
* Used to automatically convert to Fahrenheit if that is how the class is
*   configured to operate.
*
* @return a unit-bound representation of temperature in C or F.
*/
float GridEYE::_normalize_units_returned(float temperature) {
  if (unitsFahrenheit()) {
    temperature = temperature * 1.8 + 32;
  }
  return temperature;
}


/**
* A conversion function to justify the device's idea of a signed 12-bit
*   against our own. Returns a proper sign-extended representation of the
*   temperature. Temperature is reported as 12-bit twos complement.
*
* @return a normalized 16-bit representation of the temperature.
*/
int16_t GridEYE::_dev_int16_to_float(int16_t temperature) {
  if (temperature & (1 << 11)) {    // Check if temperature is negative
    temperature &= ~(1 << 11);      // If temperature is negative, mask out the
    temperature = temperature * -1; // sign bit and make the float negative.
  }
  return temperature;
}


/**
* A conversion function to justify the device's idea of a signed 12-bit
*   against our own. Returns the device's preferred representation of the
*   temperature. Temperature is reported as 12-bit twos complement.
*
* @param Temperature in degrees-C
* @return The device's preferred representation of the temperature.
*/
int16_t GridEYE::_native_float_to_dev_int16(float DegreesC) {
  int16_t temperature = round(abs(DegreesC) * 4);
  if (DegreesC < 0) {
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  return temperature;
}




/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t GridEYE::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t GridEYE::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;


  if (!op->hasFault()) {
    if (!_amg_flag(GRIDEYE_FLAG_DEVICE_PRESENT)) {
      _amg_set_flag(GRIDEYE_FLAG_DEVICE_PRESENT);
    }
    if (op == &_frame_read) {   // Shortcut for our most common operation.
      // Process data from frame.
      _last_read = millis();
      _amg_set_flag(GRIDEYE_FLAG_FRAME_UPDATED);
      return ret;
    }
    else if (INT_TABLE_REGISTER_INT0_START == op->sub_addr) {
      // Another special case for pixel interrupts.
      // TODO: Handle this?
      return ret;
    }

    uint8_t* buf     = op->buffer();
    uint     len     = op->bufferLen();
    uint8_t  reg_idx = (uint8_t) _reg_id_from_addr(op->sub_addr);
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint i = 0; i < len; i++) {
          uint8_t value = *buf++;
          switch ((AMG88XXRegID) reg_idx) {
            case AMG88XXRegID::POWER_CONTROL:
              _amg_set_flag(GRIDEYE_FLAG_ENABLED, !(value & 0x10));
              break;
            case AMG88XXRegID::RESET:
              // We just reset.
              break;
            case AMG88XXRegID::FRAMERATE:
              _amg_set_flag(GRIDEYE_FLAG_10FPS, (0 == value));
              break;
            case AMG88XXRegID::INT_CONTROL:
              break;
            case AMG88XXRegID::STATUS_CLEAR:
              // After clearing the status flag, check if we are still interrupted.
              if (255 != _IRQ_PIN) {
                amg_irq_fired = !readPin(_IRQ_PIN);
              }
              break;
            case AMG88XXRegID::AVERAGE:
              _amg_set_flag(GRIDEYE_FLAG_HW_AVERAGING, (value & (1 << 5)));
              break;
            case AMG88XXRegID::INT_LEVEL_UPPER_LSB:
            case AMG88XXRegID::INT_LEVEL_UPPER_MSB:
            case AMG88XXRegID::INT_LEVEL_LOWER_LSB:
            case AMG88XXRegID::INT_LEVEL_LOWER_MSB:
            case AMG88XXRegID::INT_LEVEL_HYST_LSB:
            case AMG88XXRegID::INT_LEVEL_HYST_MSB:
            case AMG88XXRegID::RESERVED_AVERAGE:
              break;
            default:  // Anything else is invalid.
              break;
          }
          reg_idx++;
        }
        break;

      case BusOpcode::RX:
        for (uint i = 0; i < len; i++) {
          uint8_t value = *buf++;
          switch ((AMG88XXRegID) reg_idx) {
            case AMG88XXRegID::POWER_CONTROL:
            case AMG88XXRegID::FRAMERATE:
            case AMG88XXRegID::INT_CONTROL:
              break;
            case AMG88XXRegID::STATUS:
              if (value & 0x06) {   // Handle both OVF and THRESH.
                _read_pixel_int_states();
                _write_register(AMG88XXRegID::STATUS_CLEAR, 0x06);
              }
              break;
            case AMG88XXRegID::AVERAGE:
              _amg_set_flag(GRIDEYE_FLAG_HW_AVERAGING, (value & (1 << 5)));
              break;
            case AMG88XXRegID::INT_LEVEL_UPPER_LSB:
            case AMG88XXRegID::INT_LEVEL_UPPER_MSB:
            case AMG88XXRegID::INT_LEVEL_LOWER_LSB:
            case AMG88XXRegID::INT_LEVEL_LOWER_MSB:
            case AMG88XXRegID::INT_LEVEL_HYST_LSB:
            case AMG88XXRegID::INT_LEVEL_HYST_MSB:
            case AMG88XXRegID::THERMISTOR_LSB:
            case AMG88XXRegID::THERMISTOR_MSB:
            case AMG88XXRegID::RESERVED_AVERAGE:
              break;
            default:  // Anything else is invalid.
              break;
          }
          reg_idx++;
        }
        break;

      default:
        break;
    }
  }
  return ret;
}
