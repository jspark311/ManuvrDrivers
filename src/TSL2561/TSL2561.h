/*
* This file started out as an adafruit driver. I have mutated it.
*   ---J. Ian Lindsay
*/

/*!
 * @file Adafruit_TSL2561_U.h
 *
 * This is part of Adafruit's FXOS8700 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit FXOS8700 breakout:
 * https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <AbstractPlatform.h>
#include <I2CAdapter.h>


#ifndef __TSL2561_DRIVER_H_
#define __TSL2561_DRIVER_H_

/* Class flags */
#define TSL2561_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define TSL2561_FLAG_PINS_CONFIGURED  0x0002  // Low-level pin setup is complete.
#define TSL2561_FLAG_INITIALIZED      0x0004  // Registers are initialized.
#define TSL2561_FLAG_ENABLED          0x0008  // Device is measuring.
#define TSL2561_FLAG_AUTOGAIN         0x0010  // Class will adjust hardware gain automatically.
#define TSL2561_FLAG_GAIN_16X         0x0020  // Is the low-light setting on?
#define TSL2561_FLAG_INTEGRATION_MASK 0x00C0  // Integration time mask.

/* I2C address options */
#define TSL2561_ADDR_LOW          (0x29)    ///< Default address (pin pulled low)
#define TSL2561_ADDR_FLOAT        (0x39)    ///< Default address (pin left floating)
#define TSL2561_ADDR_HIGH         (0x49)    ///< Default address (pin pulled high)

// Lux calculations differ slightly for CS package
//#define TSL2561_PACKAGE_CS                ///< Chip scale package
#define TSL2561_PACKAGE_T_FN_CL             ///< Dual Flat No-Lead package

/* TSL2561 I2C Registers */
enum class TSL2561Reg : uint8_t {
  CONTROL          = 0x00, // Control/power register
  TIMING           = 0x01, // Set integration time register
  THRESHHOLDL_LOW  = 0x02, // Interrupt low threshold low-byte
  THRESHHOLDL_HIGH = 0x03, // Interrupt low threshold high-byte
  THRESHHOLDH_LOW  = 0x04, // Interrupt high threshold low-byte
  THRESHHOLDH_HIGH = 0x05, // Interrupt high threshold high-byte
  INTERRUPT        = 0x06, // Interrupt settings
  CRC              = 0x08, // Factory use only
  ID               = 0x0A, // TSL2561 identification setting
  CHAN0_LOW        = 0x0C, // Light data channel 0, low byte
  CHAN0_HIGH       = 0x0D, // Light data channel 0, high byte
  CHAN1_LOW        = 0x0E, // Light data channel 1, low byte
  CHAN1_HIGH       = 0x0F  // Light data channel 1, high byte
};

/* Time spent collecting and integrating data */
enum class TSLIntegrationTime : uint8_t {
  MS_13    = 0x00,    // 13.7ms
  MS_101   = 0x01,    // 101ms
  MS_402   = 0x02,    // 402ms
  INVALID  = 0x03     // INVALID
};



/*******************************************************************************
* Class definition
*******************************************************************************/
class TSL2561 : public I2CDevice {
  public:
    TSL2561(uint8_t addr, uint8_t irq_pin = 255);
    ~TSL2561();

    int8_t init();
    int8_t poll();
    int8_t enabled(bool);

    /* TSL2561 Functions */
    int8_t integrationTime(TSLIntegrationTime);
    inline TSLIntegrationTime integrationTime() {  return (TSLIntegrationTime) ((_flags >> 6) & 0x03);  };

    int8_t  highGain(bool);
    int8_t  getLuminosity();
    inline uint16_t getBroadband() {   return _broadband;  };
    inline uint16_t getIR() {          return _infrared;   };
    inline uint32_t getLux() {         return _lux;        };

    inline bool devFound() {        return _tsl_flag(TSL2561_FLAG_DEVICE_PRESENT);  };
    inline bool enabled() {         return _tsl_flag(TSL2561_FLAG_ENABLED);         };
    inline bool initialized() {     return _tsl_flag(TSL2561_FLAG_INITIALIZED);     };
    inline bool highGain() {        return _tsl_flag(TSL2561_FLAG_GAIN_16X);        };
    inline bool autogain() {        return _tsl_flag(TSL2561_FLAG_AUTOGAIN);        };
    inline void autogain(bool x) {  _tsl_set_flag(TSL2561_FLAG_AUTOGAIN, x);        };

    /* Overrides from the I2CDevice */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    const uint8_t _IRQ_PIN;
    uint8_t  _reg_devid  = 0;
    uint8_t  _reg_timing = 0;
    uint8_t  _reg_ctrl   = 0;
    uint16_t _flags      = 0;
    uint16_t _broadband  = 0;
    uint16_t _infrared   = 0;
    uint32_t _lux        = 0;
    uint32_t _last_read  = 0;
    uint8_t  _data_shadow[4] = {0, 0, 0, 0};

    int8_t  _calculate_lux();

    int8_t   _read_data_registers();
    int8_t   _ll_pin_init();
    int8_t   _write8(TSL2561Reg, uint8_t* val_ptr);
    int8_t   _read8(TSL2561Reg, uint8_t* val_ptr);

    /* Flag manipulation inlines */
    inline uint16_t _tsl_flags() {                return _flags;           };
    inline bool _tsl_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _tsl_flip_flag(uint16_t _flag) {  _flags ^= _flag;         };
    inline void _tsl_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _tsl_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _tsl_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};

#endif // __TSL2561_DRIVER_H_
