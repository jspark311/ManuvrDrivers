/*
File:   MCP23x17.cpp
Author: J. Ian Lindsay
Date:   2023.07.07
*/

#include "MCP23x17.h"

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

/** Register address sanitizer. */
const MCP23x17RegID _reg_id_from_addr(const uint8_t ADDR) {
  switch ((MCP23x17RegID) ADDR) {
    case MCP23x17RegID::IODIR:
    case MCP23x17RegID::IPOL:
    case MCP23x17RegID::GPINTEN:
    case MCP23x17RegID::GPPU:
    case MCP23x17RegID::GPIO:
    case MCP23x17RegID::OLAT:
      return (MCP23x17RegID) ADDR;
    default:  break;
  }
  return MCP23x17RegID::INVALID;
}


/**
* This is an ISR for ALTERT1 pins.
*/
void mcp23x17x_isr() {
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/* Delegate constructor. */
MCP23x17::MCP23x17(
  const uint8_t RESET_PIN,
  const uint8_t IRQ_PIN_A,
  const uint8_t IRQ_PIN_B
) : _RESET_PIN(RESET_PIN), _IRQ_PIN_A(IRQ_PIN_A), _IRQ_PIN_B(IRQ_PIN_B) {}



int8_t MCP23x17::init(I2CAdapter* b) {
  int8_t ret = -1;
  return ret;
}


int8_t MCP23x17::reset() {
  int8_t ret = -1;
  return ret;
}


/*
* Poll the driver for updates.
*/
int8_t MCP23x17::poll() {
  int8_t ret = -1;
  if (initialized()) {
  }
  return ret;
}


int8_t MCP23x17::refresh() {
  int8_t ret = -1;
  return ret;
}



int8_t MCP23x17::gpioMode(uint8_t pin, GPIOMode mode) {
  int8_t ret = -1;
  return ret;
}


GPIOMode MCP23x17::gpioMode(uint8_t pin) {
  GPIOMode ret = GPIOMode::INPUT;
  return ret;
}


int8_t MCP23x17::digitalWrite(uint8_t pin, bool value) {
  int8_t ret = -1;
  return ret;
}


uint8_t MCP23x17::digitalRead(uint8_t pin) {
  uint8_t ret = 0;
  return ret;
}


uint16_t MCP23x17::getPinValues() {
  uint16_t ret = 0;
  return ret;
}


int8_t MCP23x17::setPinValues(uint16_t) {
  int8_t ret = -1;
  return ret;
}


int8_t MCP23x17::attachInterrupt(uint8_t pin, PinCallback, IRQCondition condition) {
  int8_t ret = -1;
  return ret;
}


int8_t MCP23x17::detachInterrupt(uint8_t pin) {
  int8_t ret = -1;
  return ret;
}


void MCP23x17::printDebug(StringBuilder*) {
}


void MCP23x17::printPins(StringBuilder*) {
}


int8_t MCP23x17::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int8_t ret = -1;
  return ret;
}




/*******************************************************************************
* Members and logic specific to the i2c package
*******************************************************************************/
/*
* Bus-specific constructor. IRQ pins are pass-through to the parent class.
*/
MCP23017::MCP23017(
  uint8_t addr,
  const uint8_t RESET_PIN,
  const uint8_t IRQ_PIN_A,
  const uint8_t IRQ_PIN_B
) : MCP23x17(RESET_PIN, IRQ_PIN_A, IRQ_PIN_B), I2CDevice(addr),
  _busop_irq_read(BusOpcode::RX, this),
  _busop_pin_state(BusOpcode::RX, this) {}


/**
*
*/
int8_t MCP23017::_write_register(uint8_t addr, uint8_t* data) {
  int8_t ret = -2;
  if (devFound()) {
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = addr;
      op->setBuffer(data, 1);
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
int8_t MCP23017::_read_registers(uint8_t addr, uint8_t* data, uint8_t length) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = addr;
      op->setBuffer(data, length);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}



int8_t MCP23017::init() {
  int8_t ret = -1;
  if (nullptr != _bus) {
  }
  return ret;
}


/*
* Dump this item to the dev log.
*/
void MCP23017::printDebug(StringBuilder* output) {
  output->concatf("-- MCP23017 %sinitialized\n", (initialized() ? "" : "un"));
  I2CDevice::printDebug(output);
  output->concatf("\tFound:          %c\n", (devFound() ? 'y' : 'n'));
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t MCP23017::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t MCP23017::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    MCP23x17RegID reg = _reg_id_from_addr(op->sub_addr);
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        switch (reg) {
          case MCP23x17RegID::IODIR:
          case MCP23x17RegID::IPOL:
          case MCP23x17RegID::GPINTEN:
          case MCP23x17RegID::GPPU:
          case MCP23x17RegID::GPIO:
          case MCP23x17RegID::OLAT:
          default:
            break;
        }
        break;

      case BusOpcode::RX:
        switch (reg) {
          case MCP23x17RegID::IODIR:
          case MCP23x17RegID::IPOL:
          case MCP23x17RegID::GPINTEN:
          case MCP23x17RegID::GPPU:
          case MCP23x17RegID::GPIO:
          case MCP23x17RegID::OLAT:
          default:
            break;
        }
        break;

      default:
        break;
    }
  }
  return ret;
}
