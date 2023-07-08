/*
File:   MCP23x17.cpp
Author: J. Ian Lindsay
Date:   2023.07.07
*/


#include "MCP23x17.h"


/* Delegate constructor. */
MCP23x17::MCP23x17() {}



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



/*******************************************************************************
* Members and logic specific to the i2c package
*******************************************************************************/
MCP23017::MCP23017(uint8_t addr) :
  MCP23x17(), I2CDevice(addr) {}

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
    uint8_t* buf = op->buffer();
    uint8_t  r   = op->sub_addr;
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        break;

      case BusOpcode::RX:
        break;

      default:
        break;
    }
  }
  return ret;
}
