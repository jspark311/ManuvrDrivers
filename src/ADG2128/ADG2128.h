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

// TODO: Migrate
#define ADG2128_CONSOLE                1

class StringBuilder;
#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"

#define ADG2128_DEFAULT_I2C_ADDR    0x70
#define ADG2128_SERIALIZE_VERSION   0x01
#define ADG2128_SERIALIZE_SIZE        17

/* Class flags. */
#define ADG2128_FLAG_INITIALIZED    0x0001
#define ADG2128_FLAG_ALLOW_MR_TO_C  0x0002
#define ADG2128_FLAG_ALLOW_R_TO_MC  0x0004
#define ADG2128_FLAG_PRESERVE_STATE 0x0008
#define ADG2128_FLAG_PINS_CONFD     0x0010
#define ADG2128_FLAG_IO_IN_FLIGHT   0x0020
#define ADG2128_FLAG_NEED_REFRESH   0x0040
#define ADG2128_FLAG_DEV_FOUND      0x4000
#define ADG2128_FLAG_FROM_BLOB      0x8000


#define ADG2128_FLAG_SERIAL_MASK    0x000E  // Only these bits are serialized.


enum class ADG2128_ERROR : int8_t {
  NONE               = 0,   // There was no error.
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
};


/*
* This class represents an Analog Devices ADG2128 8x12 analog cross-point switch.
* This switch is controlled via i2c.
* The 8-pin group are the columns, and the 12-pin group are rows.
*/
class ADG2128 : public I2CDevice {
  public:
    ADG2128(const ADG2128Opts);
    ADG2128(const uint8_t* buf, const unsigned int len);
    virtual ~ADG2128();

    ADG2128_ERROR init(I2CAdapter* bus = nullptr);       // Perform bus-related init tasks.
    ADG2128_ERROR reset();                               // Resets the entire device.

    /* Functions for manipulating individual switches. */
    ADG2128_ERROR changeRoute(uint8_t col, uint8_t row, bool sw_closed, bool defer);
    ADG2128_ERROR setRoute(uint8_t col, uint8_t row, bool defer = false);
    ADG2128_ERROR unsetRoute(uint8_t col, uint8_t row, bool defer = false);
    uint8_t serialize(uint8_t* buf, unsigned int len);
    int8_t  unserialize(const uint8_t* buf, const unsigned int len);
    uint8_t  getCols(uint8_t row);
    uint16_t getRows(uint8_t col);

    inline bool initialized() {              return _adg_flag(ADG2128_FLAG_INITIALIZED);     };
    inline bool devFound() {                 return _adg_flag(ADG2128_FLAG_DEV_FOUND);       };
    inline void preserveOnDestroy(bool x) {  _adg_set_flag(ADG2128_FLAG_PRESERVE_STATE, x);  };
    inline bool preserveOnDestroy() {        return _adg_flag(ADG2128_FLAG_PRESERVE_STATE);  };

    /* Overrides from I2CDevice... */
    int8_t io_op_callback(BusOp*);

    #if defined(ADG2128_CONSOLE)
      /* Built-in per-instance console handler. */
      int8_t console_handler(StringBuilder* text_return, StringBuilder* args);
      void printDebug(StringBuilder*);
    #endif


  private:
    const ADG2128Opts _opts;
    uint16_t _values[12];
    uint16_t _flags;       // Class flags.

    ADG2128_ERROR readback(uint8_t row);
    ADG2128_ERROR compose_first_byte(uint8_t col, uint8_t row, bool set, uint8_t* result);
    ADG2128_ERROR enforce_cardinality(uint8_t col, uint8_t row);

    int8_t _ll_pin_init();

    inline void _pins_confd(bool x) {  return _adg_set_flag(ADG2128_FLAG_PINS_CONFD, x);  };
    inline bool _pins_confd() {        return _adg_flag(ADG2128_FLAG_PINS_CONFD);         };
    inline bool _need_refresh() {      return _adg_flag(ADG2128_FLAG_NEED_REFRESH);       };
    inline bool _io_in_flight() {      return _adg_flag(ADG2128_FLAG_IO_IN_FLIGHT);       };

    /* Flag manipulation inlines */
    inline uint16_t _adg_flags() {                return _flags;           };
    inline bool _adg_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _adg_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _adg_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _adg_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};
#endif
