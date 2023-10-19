/*
File:   LTC294x.h
Author: J. Ian Lindsay
Date:   2017.06.08

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

#ifndef __LTC294X_DRIVER_H__
#define __LTC294X_DRIVER_H__

#include <AbstractPlatform.h>
#include <FlagContainer.h>
#include "BusQueue/I2CAdapter.h"

#define LTC294X_I2CADDR        0x64
#define DEG_K_C_OFFSET      272.15f


/* State flags. */
#define LTC294X_FLAG_INIT_CTRL      0x0001  // Register init flags.
#define LTC294X_FLAG_INIT_AC        0x0002  // Register init flags.
#define LTC294X_FLAG_INIT_THRESH_T  0x0004  // Register init flags.
#define LTC294X_FLAG_INIT_THRESH_V  0x0008  // Register init flags.
#define LTC294X_FLAG_INIT_THRESH_CL 0x0010  // Register init flags.
#define LTC294X_FLAG_INIT_THRESH_CH 0x0020  // Register init flags.
#define LTC294X_FLAG_TRACKING_READY 0x0040  // Tracking data is available.
#define LTC294X_FLAG_BATTERY_GT55AM 0x0080  // The battery is bigger than 5.5AH.
#define LTC294X_FLAG_PINS_INITD     0x0100  // The pins have been setup.
#define LTC294X_FLAG_DATA_FRESH     0x0200  //

#define LTC294X_FLAG_MASK_INIT_CMPLT ( \
  LTC294X_FLAG_INIT_CTRL     | LTC294X_FLAG_INIT_THRESH_V | \
  LTC294X_FLAG_INIT_AC       | LTC294X_FLAG_INIT_THRESH_T | \
  LTC294X_FLAG_INIT_THRESH_CL | LTC294X_FLAG_INIT_THRESH_CH)


#define LTC294X_OPT_PIN_IS_CC   0x01  // Is the I/O pin to be treated as an output?
#define LTC294X_OPT_INTEG_SENSE 0x02  // This is a "-1" varient, and has an integrated sense resistor.
#define LTC294X_OPT_ADC_AUTO    0x04  // The converter should run automatically.


/* Registers that exist in hardware */
enum class LTC294xRegID : uint8_t {
  STATUS        = 0x00,
  CONTROL       = 0x01,
  ACC_CHARGE    = 0x02,  // 16-bit
  CHRG_THRESH_H = 0x04,  // 16-bit
  CHRG_THRESH_L = 0x06,  // 16-bit
  VOLTAGE       = 0x08,  // 16-bit
  V_THRESH      = 0x0A,  // 16-bit
  TEMP          = 0x0C,  // 16-bit
  TEMP_THRESH   = 0x0E,  // 16-bit
  INVALID       = 0x10
};


enum class LTC294xADCModes : uint8_t {
  SLEEP     = 0x00,
  MANUAL_T  = 0x40,
  MANUAL_V  = 0x80,
  AUTO      = 0xC0
};
#define LTC294X_OPT_MASK_ADC_MODE   0xC0


/**
* Pin defs for this module.
* Set pin def to 255 to mark it as unused.
*
* Datasheet imposes the following constraints:
*   Maximum battery capacity is 5500 mAh.
*/
class LTC294xOpts {
  public:
    const uint8_t pin;            // Which pin is bound to ~ALERT/CC?

    /** Copy constructor. */
    LTC294xOpts(const LTC294xOpts* o) :
      pin(o->pin),
      _flags(o->_flags) {};

    /**
    * Constructor.
    *
    * @param pin
    * @param Initial flags
    */
    LTC294xOpts(
      uint16_t _p,
      uint8_t _fi = 0
    ) :
      pin(_p),
      _flags(_fi) {};

    inline bool autostartReading() const {
      return (_flags & LTC294X_OPT_ADC_AUTO);
    };

    inline bool useAlertPin() const {
      return (255 != pin) & !(_flags & LTC294X_OPT_PIN_IS_CC);
    };

    inline bool useCCPin() const {
      return (255 != pin) & (_flags & LTC294X_OPT_PIN_IS_CC);
    };


  private:
    const uint8_t _flags;
};



/**
* Driver class for LTC294x.
*/
class LTC294x : public I2CDevice {
  public:
    LTC294x(const LTC294xOpts*, uint16_t batt_capacity);
    ~LTC294x();

    int8_t init();
    int8_t poll();

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    void printDebug(StringBuilder*);
    void printRegisters(StringBuilder*);

    /* Returns temperature in Celcius. */
    float temperature();
    float batteryVoltage();
    float batteryPercent();
    float batteryPercentVoltage();
    inline float batteryCurrent() {    return _chrg_dt;      };
    inline float minimumCurrent() {    return _chrg_min_dt;  };
    inline float maximumCurrent() {    return _chrg_max_dt;  };
    inline uint16_t batteryCharge() {  return _get_shadow_value(LTC294xRegID::ACC_CHARGE);  };
    inline int8_t batteryCharge(uint16_t x) {  return _set_charge_register(x);   };

    int8_t setChargeThresholds(uint16_t low, uint16_t high);
    int8_t setVoltageThreshold(float low, float high);
    int8_t setTemperatureThreshold(float low, float high);

    /**
    * @return true if the sensor has valid power-tracking data.
    */
    inline bool trackingReady() {   return _flags.value(LTC294X_FLAG_TRACKING_READY);   };

    /**
    * @return true if the sensor is initialized.
    */
    inline bool initComplete() {
      return (LTC294X_FLAG_MASK_INIT_CMPLT == (_flags.raw & LTC294X_FLAG_MASK_INIT_CMPLT));
    };

    inline bool asleep() {    return (1 == ((uint8_t) _get_shadow_value(LTC294xRegID::CONTROL) & 0x01));  };
    int8_t  sleep(bool);


    static LTC294x* INSTANCE;


  private:
    const LTC294xOpts _opts;
    const uint16_t _batt_volume; // Our idea about how many mA the battery should hold.
    uint16_t _thrsh_l_chrg = 0;  // cached threshold values
    uint16_t _thrsh_h_chrg = 0;  // cached threshold values
    FlagContainer16 _flags;
    uint8_t  _thrsh_l_temp = 0;  // cached threshold values
    uint8_t  _thrsh_h_temp = 0;  // cached threshold values
    uint8_t  _thrsh_l_volt = 0;  // cached threshold values
    uint8_t  _thrsh_h_volt = 0;  // cached threshold values
    uint8_t  shadows[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    /* Expected bounds of charge counter for given battery size. */
    uint16_t _charge_expected_min = 0;
    uint16_t _charge_expected_max = 0;

    /* Variables to help accurately track power usage. */
    uint32_t _sample_dt;          // The dt term, in ms.
    uint32_t _sample_time;        // The last sample time for charge, in ms.
    uint16_t _sample_count;       // How many readings have we taken?
    uint16_t _chrg_reading_0;     // Previous charge register value.
    float    _volt_reading_0;     // Previous voltage register value.
    float    _temp_reading_0;     // Previous temperature register value.
    float    _chrg_dt;            // dc/dt in columbs/millisec (mA).
    float    _volt_dt;            // dV/dt in volts/min.
    float    _temp_dt;            // dT/dt in deg-C/min.
    float    _chrg_min_dt;        // Minimum observed dI/dt (lowest drain).
    float    _volt_min_dt;        // Minimum observed dV/dt (lowest drain).
    float    _chrg_max_dt;        // Maximum observed dI/dt (peak drain).
    float    _volt_max_dt;        // Maximum observed dV/dt (peak drain).
    float    _temp_min;           // Minimum observed temperature.
    float    _temp_max;           // Maximum observed temperature.


    int8_t _set_charge_register(uint16_t x);
    int8_t _set_thresh_reg_charge(uint16_t l, uint16_t h);

    int8_t _write_control_reg(uint8_t v);
    int8_t _post_discovery_init();

    /**
    * @param The ADC mode.
    */
    int8_t  _adc_mode(LTC294xADCModes);

    /**
    * @return The ADC mode.
    */
    LTC294xADCModes _adc_mode() {
      return (LTC294xADCModes) ((uint8_t) _get_shadow_value(LTC294xRegID::CONTROL) & LTC294X_OPT_MASK_ADC_MODE);
    };

    /**
    * @return Should readings from the chip be taken seriously?
    */
    bool _is_monitoring() {
      return (initComplete() && (LTC294xADCModes::SLEEP != _adc_mode()));
    };

    /**
    * @return Is this device a 2942?
    */
    inline bool _is_2942() {   return (0 == ((uint8_t) _get_shadow_value(LTC294xRegID::STATUS) & 0x80));   };

    /**
    * @param Is power-tracking data valid?
    */
    inline void _tracking_ready(bool x) {
      _flags.set(LTC294X_FLAG_TRACKING_READY, x);
    };

    /**
    * @param Temperature threshold value.
    * @return Degrees Celcius
    */
    inline float convertT(uint16_t v) {    return (0.009155f * v);   };

    float _c_to_mA(uint16_t chrg);
    void _reset_tracking_data();
    void _update_tracking();
    void _proc_updated_status_reg(uint8_t);
    uint8_t _derive_prescaler();

    int8_t   _set_shadow_value(LTC294xRegID, uint16_t);
    uint16_t _get_shadow_value(LTC294xRegID);
    int8_t  _read_registers(LTC294xRegID, uint8_t);
    int8_t  _write_registers(LTC294xRegID, uint8_t);

    static LTC294xRegID _reg_id_from_addr(const uint8_t addr);
};


#endif  // __LTC294X_DRIVER_H__
