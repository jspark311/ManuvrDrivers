/*
File:   ADG2128.cpp
Author: J. Ian Lindsay
Date:   2014.03.10

Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


This driver's relationship to the base I2CDevice class is that the switch
  address is treated as the sub_addr, and switched on that basis.
*/

#include "ADG2128.h"
#include "StringBuilder.h"

static const uint8_t readback_addr[24] = {
  0x34, 0, 0x3c, 0, 0x74, 0, 0x7c, 0,
  0x35, 0, 0x3d, 0, 0x75, 0, 0x7d, 0,
  0x36, 0, 0x3e, 0, 0x76, 0, 0x7e, 0
};


/*
* Constructor. Takes the i2c address of this device as sole argument.
*/
ADG2128::ADG2128(const ADG2128Opts _o) : I2CDevice(_o.addr), _opts(&_o), _flags(0) {
  for (uint8_t i = 0; i < 12; i++) {   _values[i] = 0;  }
}


ADG2128::~ADG2128() {
  if (!preserveOnDestroy()) {
    //reset();  TODO: Without a reset pin, this will crash when the async i2c opts callback.
  }
}


/*
*
*/
ADG2128_ERROR ADG2128::init(I2CAdapter* b) {
  ADG2128_ERROR ret = ADG2128_ERROR::BUS;
  if (nullptr != b) {
    setAdapter(b);
  }
  _adg_clear_flag(ADG2128_FLAG_INITIALIZED | ADG2128_FLAG_IO_IN_FLIGHT | ADG2128_FLAG_NEED_REFRESH);
  if (0 == _ll_pin_init()) {
    if (nullptr != _bus) {
      ret = ADG2128_ERROR::NONE;
      ping_device();
    }
  }
  return ret;
}


ADG2128_ERROR ADG2128::enforce_cardinality(uint8_t col, uint8_t row) {
  if (col > 7)  return ADG2128_ERROR::BAD_COLUMN;
  if (row > 11) return ADG2128_ERROR::BAD_ROW;
  if (!_opts.many_c_per_r) {
    // Check that the given row isn't already attached to a different col.
  }
  if (!_opts.many_r_per_c) {
    // Check that the given col isn't already attached to a different row.
  }
  return ADG2128_ERROR::NONE;
}


ADG2128_ERROR ADG2128::compose_first_byte(uint8_t col, uint8_t row, bool set, uint8_t* result) {
  if (col > 7)  return ADG2128_ERROR::BAD_COLUMN;
  if (row > 11) return ADG2128_ERROR::BAD_ROW;
  uint8_t temp = (row < 6) ? row : (row+2);  // Dance around the reserved range in the middle.
  *result = (temp << 3) + col + (set ? 0x80 : 0x00);
  return ADG2128_ERROR::NONE;
}


ADG2128_ERROR ADG2128::setRoute(uint8_t col, uint8_t row, bool defer) {
  ADG2128_ERROR ret = enforce_cardinality(col, row);
  if (ADG2128_ERROR::NONE == ret) {
    ret = changeRoute(col, row, true, defer);
  }
  return ret;
}


ADG2128_ERROR ADG2128::unsetRoute(uint8_t col, uint8_t row, bool defer) {
  return changeRoute(col, row, false, defer);
}


ADG2128_ERROR ADG2128::changeRoute(uint8_t col, uint8_t row, bool sw_closed, bool defer) {
  ADG2128_ERROR ret = ADG2128_ERROR::ABSENT;
  if (devFound()) {
    uint8_t temp;
    ret = compose_first_byte(col, row, sw_closed, &temp);
    if (ADG2128_ERROR::NONE == ret) {
      ret = ADG2128_ERROR::NO_MEM;
      uint8_t* buf = (uint8_t*) malloc(2);  // The awfulness, part 1.
      if (nullptr != buf) {
        I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
        if (nullptr != op) {
          *(buf + 0) = temp;
          *(buf + 1) = defer ? 0 : 1;
          ret = ADG2128_ERROR::BUS;
          op->dev_addr = _dev_addr;
          op->sub_addr = -1;
          op->setBuffer(buf, 2);
          op->shouldFreeBuffer(true);
          if (0 == queue_io_job(op)) {
            _adg_set_flag(ADG2128_FLAG_IO_IN_FLIGHT, true);
            ret = ADG2128_ERROR::NONE;
          }
        }
        // Watch for leaks.
        if (ADG2128_ERROR::NONE != ret) {  free(buf);  }
      }
    }
  }
  return ret;
}


/*
* Opens all switches.
* Uses hardware reset if possible. Otherwise, will write each of the 96 switches
*   one by one. This will have a non-trivial heap load unless it is re-worked.
*/
ADG2128_ERROR ADG2128::reset() {
  ADG2128_ERROR ret = ADG2128_ERROR::ABSENT;

  if (255 != _opts.rst) {
    setPin(_opts.rst, false);
    sleep_us(2);   // Part is punchy. This delay might not even be needed.
    setPin(_opts.rst, true);  // Device will not respond to i2c while in reset state.
    sleep_us(2);   // Part is punchy. This delay might not even be needed.
    _adg_set_flag(ADG2128_FLAG_NEED_REFRESH, false);   // We know the state.
    for (uint8_t i = 0; i < 12; i++) {   _values[i] = 0;  }
    ret = ADG2128_ERROR::NONE;
  }
  else if (initialized()) {
    // If the driver is already initialized, a backup option exists if there is
    //   no reset pin. The driver will clobber the registers with zeros.
    for (int i = 0; i < 12; i++) {
      for (int j = 0; j < 8; j++) {
        // This will defer switch disconnect until the last write is completed.
        // So if reset fails, the part will be in an indeterminate state, but
        //   nothing will have changed in the switches.
        if (ADG2128_ERROR::NONE != unsetRoute(j, i, !((11 == i) && (7 == j)))) {
          return ADG2128_ERROR::BUS;
        }
      }
    }
  }
  return ret;
}


/*
* Readback on this part is organized by rows, with the return bits
* being the state of the switches to the corresponding column.
* The readback address table is hard-coded in the readback_addr array.
*
*
*/
ADG2128_ERROR ADG2128::readback(uint8_t row) {
  if (row > 11) return ADG2128_ERROR::BAD_ROW;

  ADG2128_ERROR ret = ADG2128_ERROR::ABSENT;
  if (devFound()) {
    ret = ADG2128_ERROR::BUS;
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = -1;
      op->setBuffer((uint8_t*) &readback_addr[row << 1], 2);
      op->shouldFreeBuffer(false);
      if (0 == queue_io_job(op)) {
        _adg_set_flag(ADG2128_FLAG_IO_IN_FLIGHT);
        ret = ADG2128_ERROR::NONE;
      }
    }
  }
  return ret;
}


uint8_t ADG2128::getCols(uint8_t row) {
  return ((row > 11) ? 0: (_values[row] >> 8));
}


/*
* Stores everything about the class in the provided buffer in this format...
*   Offset | Data
*   -------|----------------------
*   0      | Serializer version
*   1      | i2c address
*   2      | Reset pin
*   3      | Flags MSB
*   4      | Flags LSB
*   5-16   | Switch configuration
*
* Returns the number of bytes written to the buffer.
*/
uint8_t ADG2128::serialize(uint8_t* buf, unsigned int len) {
  uint8_t offset = 0;
  if (len >= ADG2128_SERIALIZE_SIZE) {
    if (initialized()) {
      uint16_t f = _flags & ADG2128_FLAG_SERIAL_MASK;
      *(buf + offset++) = ADG2128_SERIALIZE_VERSION;
      *(buf + offset++) = _opts.addr;
      *(buf + offset++) = _opts.rst;
      *(buf + offset++) = (uint8_t) 0xFF & (f >> 8);
      *(buf + offset++) = (uint8_t) 0xFF & f;
      for (uint8_t i = 0; i < 12; i++) {
        *(buf + offset++) = _values[i];
      }
    }
  }
  return offset;
}


int8_t ADG2128::unserialize(const uint8_t* buf, const unsigned int len) {
  uint8_t offset = 0;
  uint8_t expected_sz = 255;
  if (len >= ADG2128_SERIALIZE_SIZE) {
    uint16_t f = 0;
    uint8_t vals[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    switch (*(buf + offset++)) {
      case ADG2128_SERIALIZE_VERSION:
        expected_sz = ADG2128_SERIALIZE_SIZE;
        f = (*(buf + 3) << 8) | *(buf + 4);
        _flags = (_flags & ~ADG2128_FLAG_SERIAL_MASK) | (f & ADG2128_FLAG_SERIAL_MASK);
        offset += 4;  // Skip to the register offset.
        for (uint8_t i = 0; i < 12; i++) {
          vals[i] = *(buf + offset++);
        }
        break;
      default:  // Unhandled serializer version.
        return -1;
    }
    if (initialized()) {
      // If the device has already been initialized, we impart the new conf.
      for (uint8_t i = 0; i < 12; i++) {
        uint8_t row_val = vals[i];
        for (uint8_t j = 0; j < 8; j++) {
          // This will defer switch disconnect until the last write is completed.
          // So if reset fails, the part will be in an indeterminate state, but
          //   nothing will have changed in the switches.
          if (ADG2128_ERROR::NONE != changeRoute(j, i, ((row_val >> j) & 1), !((11 == i) && (7 == j)))) {
            return -2;
          }
        }
      }
    }
    else {
      _adg_set_flag(ADG2128_FLAG_FROM_BLOB);
      for (uint8_t i = 0; i < 12; i++) {
        _values[i] = vals[i];   // Save state for init()
      }
    }
  }
  return (expected_sz == offset) ? 0 : -1;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

int8_t ADG2128::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_ERROR;

  if (!op->hasFault()) {
    ret = BUSOP_CALLBACK_NOMINAL;
    switch (op->get_opcode()) {
      case BusOpcode::RX:
        // We just read back data from the switch. Propagate the readback for
        //   another bus cycle if needed.
        if (_need_refresh()) {
          // Calculate the index from the pointer addresses...
          const uint8_t VALUE_IDX = (op->buffer() - (uint8_t*) &_values[0]) >> 1;
          if (VALUE_IDX < 11) {
            op->set_opcode(BusOpcode::TX);
            op->sub_addr = -1;
            op->setBuffer((uint8_t*) &readback_addr[(VALUE_IDX+1) << 1], 2);
            op->shouldFreeBuffer(false);
            ret = BUSOP_CALLBACK_RECYCLE;
          }
          else {
            // If this was the last value to be read, end the readback cycle.
            _adg_set_flag(ADG2128_FLAG_NEED_REFRESH, false);
            _adg_set_flag(ADG2128_FLAG_INITIALIZED, true);
          }
        }
        break;

      case BusOpcode::TX:
        {
          const uint8_t byte0 = *(op->buffer()+0);
          const uint8_t s_col   = byte0 & 0x07;
          const bool    s_set   = (0 != (byte0 & 0x80));
          int8_t  s_row   = ((byte0 >> 3) & 0x0F);
          switch (byte0) {
            case 0x7e:  s_row++;
            case 0x76:  s_row++;
            case 0x3e:  s_row++;
            case 0x36:  s_row++;
            case 0x7d:  s_row++;
            case 0x75:  s_row++;
            case 0x3d:  s_row++;
            case 0x35:  s_row++;
            case 0x7c:  s_row++;
            case 0x74:  s_row++;
            case 0x3c:  s_row++;
            case 0x34:  s_row++;
              // We just wrote the readback address. Now we need to get two bytes.
              op->set_opcode(BusOpcode::RX);
              op->sub_addr = -1;
              op->setBuffer((uint8_t*) &_values[s_row], 2);
              op->shouldFreeBuffer(false);
              ret = BUSOP_CALLBACK_RECYCLE;
              break;

            default:
              // We just confirmed a write to the switch. Set the appropriate bit.
              // NOTE: TX ops of this class use the heap.
              if (s_row > 6) {  s_row -= 2;  }   // Dance around the reserved range in the middle.
              _values[s_row] = s_set ? (_values[s_row] | (1 << (s_col+8))) : (_values[s_row] & ~(1 << (s_col+8)));
              break;
          }
        }
        break;

      case BusOpcode::TX_CMD:
        // There is nothing like an ID register to read. So we construe a live
        //   ping of the given address to be evidence of presence.
        if (!devFound()) {
          _adg_set_flag(ADG2128_FLAG_DEV_FOUND, true);
          if (_need_refresh()) {
            readback(0);
          }
          else {
            // If the shadows are known, we are initialized.
            _adg_set_flag(ADG2128_FLAG_INITIALIZED, true);
          }
        }
        break;

      default:
        break;
    }
  }
  else {
    c3p_log(LOG_LEV_WARN, __PRETTY_FUNCTION__, "A BusOp requested by the ADG2128 failed");
  }
  _adg_set_flag(ADG2128_FLAG_IO_IN_FLIGHT, (BUSOP_CALLBACK_RECYCLE == ret));
  return ret;
}


/**
* Setup the low-level pin details. Idempotent.
*
* @return 0 on success (with or without action), or -1 if the platform rejects the pin settings.
*/
int8_t ADG2128::_ll_pin_init() {
  int8_t ret = 0;
  if (!_pins_confd()) {
    if (255 != _opts.rst) {
      // Some platforms benefit from priming the GPO value register ahead of
      //   setting its direction to avoid bouncing the logic level.
      // Read the existing GPIO state and prime the GPO registers. It shouldn't
      //   matter if the pin is I or O. C3P's API (if implemented correctly)
      //   should return the correct value regardlesss of pin direction.
      ret = -1;
      setPin(_opts.rst, readPin(_opts.rst));
      pinMode(_opts.rst, GPIOMode::OUTPUT);
      // TODO: preserveOnDestroy() does not imply setup conf-clobber.
      //   But it will do until someone cares.
      if (preserveOnDestroy()) {
        _adg_set_flag(ADG2128_FLAG_NEED_REFRESH, true);   // Signal for readback.
        ret = 0;
      }
      else if (ADG2128_ERROR::NONE == reset()) {
        ret = 0;  // Were we able to start the part in reset state?
      }
    }
    _pins_confd(0 == ret);
  }
  return ret;
}



/*******************************************************************************
* Optional console stuff
*******************************************************************************/
#if defined(ADG2128_CONSOLE)
/*
* Dump this item to the given buffer.
*/
void ADG2128::printDebug(StringBuilder* output) {
  StringBuilder::styleHeader2(output, "ADG2128 8x12 switch");
  I2CDevice::printDebug(output);
  if (initialized()) {
    StringBuilder txt_table("\t     0 1 2 3 4 5 6 7\n");
    txt_table.concat("\t   +----------------\n");
    for (int i = 0; i < 12; i++) {
      uint8_t tmp_cols = getCols(i);
      txt_table.concatf("\t%2u | ", i);
      for (int c = 0; c < 8; c++) {
        txt_table.concat((tmp_cols & 1) ? "X " : ". ");
        tmp_cols = (tmp_cols >> 1);
      }
      txt_table.concatf("    (0x%04x)\n", i, _values[i]);
    }
    txt_table.string();
    output->concatHandoff(&txt_table);
  }
  else {
    output->concat("\t Not initialized.\n");
    output->concatf("\t Pins confd:    %c\n", (_pins_confd()?'y':'n'));
    if (_pins_confd() & (255 != _opts.rst)) {
      output->concatf("\t reset:  %u  (State: %c)\n", _opts.rst, (readPin(_opts.rst) ? '1':'0'));
    }
    output->concatf("\t I/O in-flight: %c\n", (_io_in_flight()?'y':'n'));
    output->concatf("\t Found:         %c\n", (devFound()?'y':'n'));
  }
  output->concat("\n");
}


/**
* @page console-handlers
* @section ds1881-tools DS1881 tools
*
* This is the console handler for using the ADG2128 crosspoint switch. If invoked without
*   arguments, it will print its state.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `regs`    | Show the register shadows. | None
* `refresh` | Refresh the register shadows from the hardware. | None
*/
int8_t ADG2128::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  if (0 < args->count()) {
    char* cmd = args->position_trimmed(0);
    if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("init() returns %d.\n", init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "readback")) {
      uint8_t arg0 = (uint8_t) args->position_as_int(1);
      text_return->concatf("readback(%u) returns %d.\n", arg0, readback(arg0));
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "set")) {
      if (2 < args->count()) {
        uint8_t arg_x = (uint8_t) args->position_as_int(1);
        uint8_t arg_y = (uint8_t) args->position_as_int(2);
        text_return->concatf("setRoute(%u, %u) returns %d.\n", arg_x, arg_y, setRoute(arg_x, arg_y, false));
      }
      else {
        text_return->concatf("Usage: %s <column> <row>\n", cmd);
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "unset")) {
      if (2 < args->count()) {
        uint8_t arg_x = (uint8_t) args->position_as_int(1);
        uint8_t arg_y = (uint8_t) args->position_as_int(2);
        text_return->concatf("unsetRoute(%u, %u) returns %d.\n", arg_x, arg_y, unsetRoute(arg_x, arg_y, false));
      }
      else {
        text_return->concatf("Usage: %s <column> <row>\n", cmd);
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
      text_return->concatf("reset() returns %d.\n", reset());
    }
    else {
      ret = -1;
    }
  }
  else {
    printDebug(text_return);
  }
  return ret;
}

#endif  // ADG2128_CONSOLE
