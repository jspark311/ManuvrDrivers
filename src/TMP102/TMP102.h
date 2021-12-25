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
#include <AbstractPlatform.h>
#include <I2CAdapter.h>

#ifndef __TMP102_DRIVER_H_
#define __TMP102_DRIVER_H_

/* Class flags */
#define TMP102_FLAG_DEVICE_PRESENT     0x0001  // Part was found.
#define TMP102_FLAG_PINS_CONFIGURED    0x0002  // Low-level pin setup is complete.
#define TMP102_FLAG_INITIALIZED        0x0004  // Registers are initialized.
#define TMP102_FLAG_PTR_VALID          0x0008  // The shadow of the pointer register is correct.
#define TMP102_FLAG_IO_IN_FLIGHT       0x0010  // There is I/O happening.



enum class TMP102DataRate : uint8_t {
  RATE_0_25_HZ  = 0x00,    // 0 - 0.25 Hz
  RATE_1_HZ     = 0x01,    // 1 - 1 Hz
  RATE_4_HZ     = 0x02,    // 2 - 4 Hz (default)
  RATE_8_HZ     = 0x03,    // 3 - 8 Hz
  RATE_INVALID  = 0x04     // Invalid. Not repesented in hardware.
};


class TMP102 : public I2CDevice {
  public:
    TMP102(uint8_t addr, uint8_t alert_pin);
    ~TMP102();

    int8_t init(I2CAdapter* bus = nullptr);
    int8_t poll();

    void printDebug(StringBuilder*);

    inline bool  devFound() {         return _tmp_flag(TMP102_FLAG_DEVICE_PRESENT);  };
    inline bool  initialized() {      return _tmp_flag(TMP102_FLAG_INITIALIZED);     };
    inline float temperature() {      return _temp;  };

    bool   dataReady();        // Is data waiting for retrieval?
    int8_t enabled(bool);      // Sensor should be awake or asleep?
    bool   enabled();
    bool   alert();            // Returns state of Alert register
    int8_t setLowTemp(float degrees);  // Sets T_LOW alert threshold
    int8_t setHighTemp(float degrees); // Sets T_HIGH alert threshold
    float  readLowTemp();      // Reads T_LOW register
    float  readHighTemp();     // Reads T_HIGH register

    int8_t conversionRate(TMP102DataRate);
    TMP102DataRate conversionRate();

    int8_t alertActiveLow(bool);  // Set the polarity of Alert
    bool   alertActiveLow();

    // Enable or disable extended mode
    // 0 - disabled (-55C to +128C)
    // 1 - enabled  (-55C to +150C)
    int8_t extendedMode(bool mode);
    bool   extendedMode();

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

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    static const char* odrStr(const TMP102DataRate);


  private:
    const uint8_t _ALRT_PIN;
    uint8_t       _ptr_val    = 0;
    uint16_t      _flags      = 0;
    uint32_t      _last_read  = 0;
    float         _temp       = 0.0;   // Stored as Celcius.
    uint16_t      _shadows[4] = {0, 0, 0, 0};

    int8_t  _open_ptr_register(uint8_t pointerReg); // Changes the pointer register

    int8_t  _read_temp();

    int8_t   _ll_pin_init();
    uint16_t _get_shadow_value(uint8_t reg);
    int8_t   _read_register(uint8_t reg);
    int8_t   _write_register(uint8_t reg, uint16_t val);
    float    _normalize_units_accepted(float deg);
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
