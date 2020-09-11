/*
* This file started out as a SparkFun driver. I have mutated it.
*   ---J. Ian Lindsay
*/

/******************************************************************************
SparkFunTMP102.h
SparkFunTMP102 Library Header File
Alex Wende @ SparkFun Electronics
Original Creation Date: April 29, 2016
https://github.com/sparkfun/Digital_Temperature_Sensor_Breakout_-_TMP102

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/
#include <Arduino.h>
#include <Wire.h>

#ifndef __TMP102_DRIVER_H_
#define __TMP102_DRIVER_H_

/* Class flags */
#define TMP102_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define TMP102_FLAG_PINS_CONFIGURED  0x0002  // Low-level pin setup is complete.
#define TMP102_FLAG_INITIALIZED      0x0004  // Registers are initialized.
#define TMP102_FLAG_ENABLED          0x0008  // Device is measuring.
#define TMP102_FLAG_EXTENDED_MODE    0x0010  // 13-bit temperature allows read out to 150C.
#define TMP102_FLAG_FREEDOM_UNITS    0x0020  // Units in Fahrenheit if true. Celcius if not.
#define TMP102_FLAG_DATA_RATE_MASK   0x00C0  // Hold 2-bit rate setting.
#define TMP102_FLAG_ALRT_ACTIVE_HIGH 0x0100  // Alert pin is active high.

enum class TMP102DataRate : uint8_t {
  RATE_0_25_HZ  = 0x00,    // 0 - 0.25 Hz
  RATE_1_HZ     = 0x01,    // 1 - 1 Hz
  RATE_4_HZ     = 0x02,    // 2 - 4 Hz (default)
  RATE_8_HZ     = 0x03     // 3 - 8 Hz
};



class TMP102 {
  public:
    TMP102(uint8_t addr, uint8_t alert_pin);
    ~TMP102();

    int8_t init(TwoWire* bus = &Wire);
    int8_t poll();

    inline bool  devFound() {         return _tmp_flag(TMP102_FLAG_DEVICE_PRESENT);  };
    inline bool  enabled() {          return _tmp_flag(TMP102_FLAG_ENABLED);         };
    inline bool  initialized() {      return _tmp_flag(TMP102_FLAG_INITIALIZED);     };
    inline bool  extendedMode() {     return _tmp_flag(TMP102_FLAG_EXTENDED_MODE);   };
    inline bool  unitsFahrenheit() {  return _tmp_flag(TMP102_FLAG_FREEDOM_UNITS);   };
    inline void  unitsFahrenheit(bool x) {   _tmp_set_flag(TMP102_FLAG_FREEDOM_UNITS, x); };
    inline float temperature() {      return _normalize_units_returned(_temp);       };

    bool   dataReady();        // Is data waiting for retrieval?
    int8_t enabled(bool);      // Sensor should be awake or asleep?
    bool   alert();            // Returns state of Alert register
    int8_t setLowTemp(float degrees);  // Sets T_LOW alert threshold
    int8_t setHighTemp(float degrees); // Sets T_HIGH alert threshold
    float  readLowTemp();      // Reads T_LOW register
    float  readHighTemp();     // Reads T_HIGH register

    int8_t conversionRate(TMP102DataRate);
    inline TMP102DataRate conversionRate() {
      return (TMP102DataRate)((_flags >> 6) & 0x03);
    };

    int8_t alertPolarity(bool);  // Set the polarity of Alert
    inline bool alertPolarity() {
      return _tmp_flag(TMP102_FLAG_ALRT_ACTIVE_HIGH);
    };

    // Enable or disable extended mode
    // 0 - disabled (-55C to +128C)
    // 1 - enabled  (-55C to +150C)
    int8_t extendedMode(bool mode);

    // Set the number of consecutive faults
    // 0 - 1 fault
    // 1 - 2 faults
    // 2 - 4 faults
    // 3 - 6 faults
    int8_t setFault(uint8_t faultSetting);

    // Set Alert type
    // 0 - Comparator Mode: Active from temp > T_HIGH until temp < T_LOW
    // 1 - Thermostat Mode: Active when temp > T_HIGH until any read operation occurs
    int8_t setAlertMode(bool mode);


  private:
    const uint8_t _ADDR;        // 0x48, 0x49, 0x4A, 0x4B
    const uint8_t _ALRT_PIN;
    uint16_t      _flags     = 0;
    TwoWire*      _bus       = nullptr;
    uint32_t      _last_read = 0;
    float         _temp      = 0.0;   // Stored as Celcius.

    int8_t  _open_ptr_register(uint8_t pointerReg); // Changes the pointer register
    uint8_t _read_register(bool registerNumber);  // reads 1 byte of from register
    int8_t  _read_temp();

    int8_t   _ll_pin_init();
    int8_t   _write_register(uint8_t reg, uint8_t val);
    int8_t   _read_registers(uint8_t reg, uint8_t len);
    float    _normalize_units_accepted(float deg);
    float    _normalize_units_returned(float deg);
    uint16_t _data_period_ms();

    /* Flag manipulation inlines */
    inline uint16_t _tmp_flags() {                return _flags;           };
    inline bool _tmp_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _tmp_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _tmp_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _tmp_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};


#endif  // __TMP102_DRIVER_H_
