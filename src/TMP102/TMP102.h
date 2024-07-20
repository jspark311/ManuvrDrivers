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
#ifndef __TMP102_DRIVER_H_
#define __TMP102_DRIVER_H_

#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"

/* Class flags */
#define TMP102_FLAG_DEVICE_PRESENT     0x0001  // Part was found.
#define TMP102_FLAG_PINS_CONFIGURED    0x0002  // Low-level pin setup is complete.
#define TMP102_FLAG_FRESH_VALUE        0x0004  // Set when the value is unread.
#define TMP102_FLAG_PTR_VALID          0x0008  // The shadow of the pointer register is correct.
#define TMP102_FLAG_IO_IN_FLIGHT       0x0010  // There is I/O happening.
#define TMP102_FLAG_REG_1_KNOWN        0x0020  // The value of this register is known.
#define TMP102_FLAG_REG_2_KNOWN        0x0040  // The value of this register is known.
#define TMP102_FLAG_REG_3_KNOWN        0x0080  // The value of this register is known.

#define TMP102_FLAG_INIT_MASK   (TMP102_FLAG_REG_1_KNOWN | TMP102_FLAG_REG_2_KNOWN | TMP102_FLAG_REG_3_KNOWN)


/* Enums for settings */
enum class TMP102DataRate : uint8_t {
  RATE_0_25_HZ  = 0x00,    // 0 - 0.25 Hz
  RATE_1_HZ     = 0x01,    // 1 - 1 Hz
  RATE_4_HZ     = 0x02,    // 2 - 4 Hz (default)
  RATE_8_HZ     = 0x03,    // 3 - 8 Hz
  RATE_INVALID  = 0x04     // Invalid. Not repesented in hardware.
};


class TMP102Opts {
  public:
    const uint8_t  ADDR;
    const uint8_t  ALRT_PIN;
    bool           extended_mode;
    bool           active_low_alert;
    TMP102DataRate rate;

    /** Copy constructor. */
    TMP102Opts(const TMP102Opts* o) :
      ADDR(o->ADDR),
      ALRT_PIN(o->ALRT_PIN),
      extended_mode(o->extended_mode),
      active_low_alert(o->active_low_alert),
      rate(o->rate) {};

    /**
    * Constructor that accepts desired configuration.
    *
    * @param i2c address
    * @param ALERT pin
    * @param Extended mode
    * @param Active-low alert pin
    * @param Conversion Rate
    */
    TMP102Opts(uint8_t addr, uint8_t a_pin, bool em, bool al, TMP102DataRate cr) :
      ADDR(addr),
      ALRT_PIN(a_pin),
      extended_mode(em),
      active_low_alert(al),
      rate(cr) {};

    uint16_t getConfValue() {
      uint16_t ret = (extended_mode << 4) | (!active_low_alert << 10);
      ret |= ((((uint16_t) rate) & 0x0003) << 6);
      return ret;
    }
};



class TMP102 : public I2CDevice {
  public:
    TMP102(const TMP102Opts*);
    ~TMP102();

    int8_t init(I2CAdapter* bus = nullptr);
    int8_t poll();

    void printDebug(StringBuilder*);

    inline bool  devFound() {         return _tmp_flag(TMP102_FLAG_DEVICE_PRESENT);  };
    inline bool  dataReady() {        return _tmp_flag(TMP102_FLAG_FRESH_VALUE);     };
    inline bool  initialized() {      return (TMP102_FLAG_INIT_MASK == (_flags & TMP102_FLAG_INIT_MASK));     };
    inline float temperature() {      _tmp_clear_flag(TMP102_FLAG_FRESH_VALUE);  return _temp;  };

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
    TMP102Opts    _opts;
    uint8_t       _ptr_val    = 0;
    uint16_t      _flags      = 0;
    uint32_t      _last_read  = 0;
    float         _temp       = 0.0;   // Stored as Celcius.
    uint16_t      _shadows[4] = {0, 0, 0, 0};

    int8_t  _open_ptr_register(uint8_t pointerReg); // Changes the pointer register

    bool    _need_to_read();        // Is data waiting for retrieval?
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
