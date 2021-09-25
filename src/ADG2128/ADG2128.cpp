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

static const uint8_t readback_addr[24] = {
  0x34, 0, 0x3c, 0, 0x74, 0, 0x7c, 0,
  0x35, 0, 0x3d, 0, 0x75, 0, 0x7d, 0,
  0x36, 0, 0x3e, 0, 0x76, 0, 0x7e, 0
};


/*
* Constructor. Takes the i2c address of this device as sole argument.
*/
ADG2128::ADG2128(const ADG2128Opts* _o) : I2CDevice(_o->addr), _opts(_o) {
  preserve_state_on_destroy = false;
  dev_init = false;
}

ADG2128::~ADG2128() {
  if (!preserve_state_on_destroy) {
    //reset();  TODO: This will crash when the async i2c opts callback.
  }
}


/*
*
*/
ADG2128_ERROR ADG2128::init() {
  for (int i = 0; i < 12; i++) {
    if (readback(i) != ADG2128_ERROR::NO_ERROR) {
      dev_init = false;
      #ifdef MANUVR_DEBUG
        //Kernel::log("Failed to init switch.\n");
      #endif
      return ADG2128_ERROR::BUS;
    }
  }
  return ADG2128_ERROR::NO_ERROR;
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
  return ADG2128_ERROR::NO_ERROR;
}


ADG2128_ERROR ADG2128::compose_first_byte(uint8_t col, uint8_t row, bool set, uint8_t* result) {
  if (col > 7)  return ADG2128_ERROR::BAD_COLUMN;
  if (row > 11) return ADG2128_ERROR::BAD_ROW;
  uint8_t temp = row;
  if (temp >= 6) temp = temp + 2; // Dance around the reserved range in the middle.
  *result = (temp << 3) + col + (set ? 0x80 : 0x00);
  return ADG2128_ERROR::NO_ERROR;
}


ADG2128_ERROR ADG2128::setRoute(uint8_t col, uint8_t row, bool defer) {
  ADG2128_ERROR return_value = enforce_cardinality(col, row);
  if (ADG2128_ERROR::NO_ERROR == return_value) {
    return_value = changeRoute(col, row, true, defer);
  }
  return return_value;
}


ADG2128_ERROR ADG2128::unsetRoute(uint8_t col, uint8_t row, bool defer) {
  return changeRoute(col, row, false, defer);
}


ADG2128_ERROR ADG2128::changeRoute(uint8_t col, uint8_t row, bool sw_closed, bool defer) {
  uint8_t temp;
  ADG2128_ERROR return_value = compose_first_byte(col, row, sw_closed, &temp);
  if (ADG2128_ERROR::NO_ERROR == return_value) {
    return_value = ADG2128_ERROR::NO_MEM;
    uint8_t* buf = (uint8_t*) malloc(2);  // The awfulness, part 1.
    if (nullptr != buf) {
      *(buf + 0) = temp;
      *(buf + 1) = defer ? 0 : 1;
      return_value = ADG2128_ERROR::NO_ERROR;
      if (!writeX(-1, 2, buf)) {
        return_value = ADG2128_ERROR::BUS;
        free(buf);
      }
    }
  }
  return return_value;
}


/*
* Opens all switches.
* Uses hardware reset if possible. Otherwise, will write each of the 96 switches
*   one by one. This will have a non-trivial heap load unless it is re-worked.
*/
ADG2128_ERROR ADG2128::reset() {
  if (255 != _opts.rst) {
    _opts.reset(false);
    _opts.reset(true);
  }
  else {
    for (int i = 0; i < 12; i++) {
      for (int j = 0; j < 8; j++) {
        // This will defer switch disconnect until the last write is completed.
        // So if reset fails, the part will be in an indeterminate state, but
        //   nothing will have changed in the switches.
        if (ADG2128_ERROR::NO_ERROR != unsetRoute(j, i, !((11 == i) && (7 == j)))) {
          return ADG2128_ERROR::BUS;
        }
      }
    }
  }
  return init();
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

  ADG2128_ERROR return_value = ADG2128_ERROR::BUS;
  if (writeX(-1, 2, (uint8_t*) &readback_addr[row << 1])) {
    return_value = ADG2128_ERROR::NO_ERROR;
  }
  return return_value;
}


uint8_t ADG2128::getValue(uint8_t row) {
  if (row > 11) return 0;
  return _values[row] >> 8;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

int8_t ADG2128::io_op_callback(BusOp* op) {
  I2CBusOp* completed = (I2CBusOp*) op;
  uint8_t byte0 = *(completed->buffer()+0);
  //uint8_t byte1 = *(completed->buffer()+1);

  switch (completed->get_opcode()) {
    case BusOpcode::RX:
      // We just read back data from the switch. Nothing needs to be done here.
      // Any read on one of our 16-bit subaddresses that is not a failure will
      //   be construed as evidence that the device exists.
      if (!completed->hasFault()) {
        dev_init = true;
      }
      break;
    case BusOpcode::TX:
      {
        uint8_t  s_col   = byte0 & 0x07;
        int8_t   s_row   = -1;
        bool     s_set   = (0 != (byte0 & 0x80));
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
            readX(-1, 2, (uint8_t*) &_values[s_row]);
            break;
          default:
            if (!completed->hasFault()) {
              // We just confirmed a write to the switch. Set the appropriate bit.
              dev_init = true;
              // TODO: TX ops of this class use the heap. This is terrible.
              free(completed->buffer());  // The awfulness, part 2.
              completed->setBuffer(nullptr, 0);
              s_row = ((byte0 >> 3) & 0x0F) % 12;  // Modulus is costly, but safer.
              _values[s_row] = s_set ? (_values[s_row] | (1 << (s_col+8))) : (_values[s_row] & ~(1 << (s_col+8)));
            }
            else {
              //Kernel::log("An i2c operation requested by the ADG2128 came back failed.\n");
            }
            break;
        }
      }
      break;
    default:
      break;
  }
  return 0;
}


/*
* Dump this item to the dev log.
*/
void ADG2128::printDebug(StringBuilder* output) {
  StringBuilder::styleHeader2(output, "ADG2128 8x12 switch");
  I2CDevice::printDebug(output);
  if (dev_init) {
    for (int i = 0; i < 12; i++) {
      output->concatf("\t Row %d: %u\n", i, _values[i]);
    }
  }
  else {
    output->concat("\t Not initialized.\n");
  }
  output->concat("\n");
}
