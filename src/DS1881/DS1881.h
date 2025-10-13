/*
File:   DS1881.h
Author: J. Ian Lindsay
Date:   2019.07.13

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

#ifndef __DS1881_DRIVER_H__
#define __DS1881_DRIVER_H__

#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"

#define DS1881_BASE_I2C_ADDR        0x28
#define DS1881_SERIALIZE_VERSION    0x01
#define DS1881_SERIALIZE_SIZE          6

// TODO: Migrate
#define DS1881_CONSOLE                 1

/* Class flags. */
#define DS1881_FLAG_PRESERVE_STATE   0x01
#define DS1881_FLAG_ENABLED          0x02
#define DS1881_FLAG_MUTED            0x04
#define DS1881_FLAG_UNMUTE_ON_CHANGE 0x08
#define DS1881_FLAG_INITIALIZED      0x10
#define DS1881_FLAG_FROM_BLOB        0x20
#define DS1881_FLAG_IO_IN_FLIGHT     0x40
#define DS1881_FLAG_DEV_FOUND        0x80

#define DS1881_FLAG_SERIAL_MASK      0x0F  // Only these bits are serialized.


enum class DIGITALPOT_ERROR : int8_t {
  DEVICE_DISABLED = 3,   // A caller tried to set a wiper while the device is disabled. This may work...
  PEGGED_MAX      = 2,   // There was no error, but a call to change a wiper setting pegged the wiper at its highest position.
  PEGGED_MIN      = 1,   // There was no error, but a call to change a wiper setting pegged the wiper at its lowest position.
  NO_ERROR        = 0,   // There was no error.
  ABSENT          = -1,  // The pot appears to not be connected to the bus.
  BUS             = -2,  // Something went wrong with the i2c bus.
  ALREADY_AT_MAX  = -3,  // A caller tried to increase the value of the wiper beyond its maximum.
  ALREADY_AT_MIN  = -4,  // A caller tried to decrease the value of the wiper below its minimum.
  INVALID_POT     = -5   // There aren't that many potentiometers.
};


class DS1881 : public I2CDevice {
  public:
    DS1881(const uint8_t addr);
    DS1881(const uint8_t* buf, const unsigned int len);
    ~DS1881();

    /* Overrides from I2CDevice... */
    int8_t io_op_callback(BusOp*);

    #if defined(DS1881_CONSOLE)
      /* Built-in per-instance console handler. */
      int8_t console_handler(StringBuilder* text_return, StringBuilder* args);
      void printDebug(StringBuilder*);
    #endif

    inline bool devFound() {                 return _ds_flag(DS1881_FLAG_DEV_FOUND);              };
    inline bool preserveOnDestroy() {        return _ds_flag(DS1881_FLAG_PRESERVE_STATE);         };
    inline void preserveOnDestroy(bool x) {  return _ds_set_flag(DS1881_FLAG_PRESERVE_STATE, x);  };

    /*
    * Will take the device out of shutdown mode and set all the wipers
    *   to their minimum values.
    */
    inline DIGITALPOT_ERROR reset() {    return reset(0x00);  };

    inline uint8_t getValue(uint8_t pot) {
      return (pot > 1) ? 0 : 0x3F & registers[pot];
    };
    inline DIGITALPOT_ERROR init() {  return init(_bus);  };
    DIGITALPOT_ERROR init(I2CAdapter*);                   // Perform bus-related init tasks.
    DIGITALPOT_ERROR reset(uint8_t);    // Sets all volumes levels to current OTP state.
    DIGITALPOT_ERROR setValue(uint8_t val);               // Sets the value of both pots.
    DIGITALPOT_ERROR setValue(uint8_t pot, uint8_t val);  // Sets the value of the given pot.
    DIGITALPOT_ERROR enable(bool);                        //
    DIGITALPOT_ERROR refresh();                           // Forces a shadow refresh from hardware.

    uint8_t serialize(uint8_t* buf, unsigned int len);
    int8_t  unserialize(const uint8_t* buf, const unsigned int len);
    DIGITALPOT_ERROR storeWipers();
    DIGITALPOT_ERROR zerocrossWait(bool enable);
    inline bool zerocrossWait() {  return (registers[2] & 0x02);  };
    inline bool enabled() {        return _ds_flag(DS1881_FLAG_ENABLED);      };
    inline bool initialized() {    return _ds_flag(DS1881_FLAG_INITIALIZED);  };

    /* Returns the maximum value of any single potentiometer. */
    inline uint16_t getRange() {   return ((registers[2] & 0x01) ? 33 : 63);  };
    DIGITALPOT_ERROR setRange(uint8_t);

    static const char* const errorToStr(DIGITALPOT_ERROR);


  private:
    uint8_t  _flags;
    uint8_t registers[3];
    uint8_t alt_values[2];   // Ducking, mute, and other temporay values.

    inline bool _from_blob() {   return _ds_flag(DS1881_FLAG_FROM_BLOB);  };
    inline bool _ds_flag(uint8_t _flag) {       return (_flags & _flag);  };
    inline void _ds_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _ds_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _ds_set_flag(uint8_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };

    int8_t _write_register(uint8_t reg, uint8_t val);
};

#endif   // __DS1881_DRIVER_H__
