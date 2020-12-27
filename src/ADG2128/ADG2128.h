/*
File:   ADG2128.h
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

*/

#ifndef ADG2128_CROSSPOINT_H
#define ADG2128_CROSSPOINT_H

#include <AbstractPlatform.h>
#include <StringBuilder.h>
#include <I2CAdapter.h>


enum class ADG2128_ERROR : int8_t {
  NO_ERROR           = 0,   // There was no error.
  ABSENT             = -1,  // The ADG2128 appears to not be connected to the bus.
  BUS                = -2,  // Something went wrong with the i2c bus.
  BAD_COLUMN         = -3,  // Column was out-of-bounds.
  BAD_ROW            = -4,  // Row was out-of-bounds.
  NO_MEM             = -5,  // We needed a heap allocation and couldn't get it.
  CARD_VIOLATION_COL = -6,  // Command would violate col cardinality constraint.
  CARD_VIOLATION_ROW = -7   // Command would violate row cardinality constraint.
};


/*
* Options for the ADG2128
*/
class ADG2128Opts {
  public:
    const uint8_t addr;      // The device address on the i2c bus
    const uint8_t rst;       // ADG2128 reset pin
    const bool    many_c_per_r;  // Should 1<row>:many<cols> be allowed?
    const bool    many_r_per_c;  // Should 1<col>:many<rows> be allowed?

    ADG2128Opts(const ADG2128Opts* p) :
      addr(p->addr),
      rst(p->rst),
      many_c_per_r(p->many_c_per_r),
      many_r_per_c(p->many_r_per_c) {};

    ADG2128Opts(uint8_t _addr, uint8_t _rst, bool _mc_r, bool _mr_c) :
      addr(_addr), rst(_rst), many_c_per_r(_mc_r), many_r_per_c(_mr_c) {};

    inline bool reset(bool nu) const {
      if (255 != rst) setPin(rst, nu);
      return true;
    };

  private:
};


/*
* This class represents an Analog Devices ADG2128 8x12 analog cross-point switch. This switch is controlled via i2c.
* The 8-pin group are the columns, and the 12-pin group are rows.
*/
class ADG2128 : public I2CDevice {
  public:
    ADG2128(const ADG2128Opts*);
    virtual ~ADG2128();

    /* Overrides from I2CDevice... */
    int8_t io_op_callback(BusOp*);
    void printDebug(StringBuilder*);

    ADG2128_ERROR init();                                // Perform bus-related init tasks.
    ADG2128_ERROR reset();                               // Resets the entire device.

    /* Functions for manipulating individual switches. */

    ADG2128_ERROR changeRoute(uint8_t col, uint8_t row, bool sw_closed, bool defer);
    ADG2128_ERROR setRoute(uint8_t col, uint8_t row, bool defer = false);
    ADG2128_ERROR unsetRoute(uint8_t col, uint8_t row, bool defer = false);

    uint8_t getValue(uint8_t row);

    inline void preserveOnDestroy(bool x) {
      preserve_state_on_destroy = x;
    };


  private:
    const ADG2128Opts _opts;
    uint16_t _values[12];
    bool dev_init;
    bool preserve_state_on_destroy;

    ADG2128_ERROR readback(uint8_t row);
    ADG2128_ERROR compose_first_byte(uint8_t col, uint8_t row, bool set, uint8_t* result);
    ADG2128_ERROR enforce_cardinality(uint8_t col, uint8_t row);
};
#endif
