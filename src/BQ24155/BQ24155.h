/*
File:   BQ24155.h
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

#ifndef __BQ24155_DRIVER_H__
#define __BQ24155_DRIVER_H__

#include <AbstractPlatform.h>
#include <I2CAdapter.h>

/* Driver state flags. */
#define BQ24155_FLAG_INIT_CTRL      0x0001  // Register init flags.
#define BQ24155_FLAG_INIT_LIMITS    0x0002  // Register init flags.
#define BQ24155_FLAG_INIT_BATT_REG  0x0004  // Register init flags.
#define BQ24155_FLAG_INIT_FAST_CHRG 0x0008  // Register init flags.
#define BQ24155_FLAG_DISABLE_STAT   0x1000
#define BQ24155_FLAG_ISEL_HIGH      0x2000
#define BQ24155_FLAG_LIMIT_WRITING  0x4000  // There is write operation to the limit register pending.

#define BQ24155_FLAG_MASK_INIT_CMPLT ( \
  BQ24155_FLAG_INIT_CTRL     | BQ24155_FLAG_INIT_LIMITS | \
  BQ24155_FLAG_INIT_BATT_REG | BQ24155_FLAG_INIT_FAST_CHRG)

/* Registers that exist in hardware */
enum class BQ24155RegID : uint8_t {
  STATUS    = 0x00,
  LIMITS    = 0x01,
  BATT_REGU = 0x02,
  PART_REV  = 0x03,
  FAST_CHRG = 0x04,
  INVALID   = 0x05
};

enum class BQ24155Fault : uint8_t {
  NOMINAL      = 0x00,
  VBUS_OVP     = 0x01,
  SLEEP        = 0x02,
  VBUS_SAG     = 0x03,
  OUTPUT_OVP   = 0x04,
  THERMAL      = 0x05,
  TIMER        = 0x06,
  NO_BATTERY   = 0x07
};

enum class BQ24155State : uint8_t {
  READY    = 0x00,
  CHARGING = 0x01,
  CHARGED  = 0x02,
  FAULT    = 0x03
};

// oxo1
enum class BQ24155USBCurrent : uint8_t {
  LIMIT_100 = 0x00,
  LIMIT_500 = 0x01,
  LIMIT_800 = 0x02,
  NO_LIMIT  = 0x03
};


/* Offsets for various register manipulations. */
#define BQ24155_VOREGU_OFFSET  3.5f
#define BQ24155_VLOW_OFFSET    3.4f
#define BQ24155_VITERM_OFFSET  0.0034f
#define BQ24155_VIREGU_OFFSET  0.0374f

#define BQ24155_I2CADDR        0x6B


/*
* Pin defs and options for this module.
* Set pin def to 255 to mark it as unused.
* Pass no flags to accept default hardware assumptions.
*/
class BQ24155Opts {
  public:
    // If valid, will use a gpio operation to pull the SDA pin low to wake device.
    const uint16_t  sense_milliohms;
    const uint8_t   stat_pin;
    const uint8_t   isel_pin;
    const BQ24155USBCurrent  src_limit;
    const uint16_t _flgs_initial;

    /** Copy constructor. */
    BQ24155Opts(const BQ24155Opts* o) :
      sense_milliohms(o->sense_milliohms),
      stat_pin(o->stat_pin),
      isel_pin(o->isel_pin),
      src_limit(o->src_limit),
      _flgs_initial(o->_flgs_initial) {};

    /**
    * Constructor.
    *
    * @param sense_milliohms
    * @param stat_pin
    * @param isel_pin
    * @param src_limit
    * @param Initial flags
    */
    BQ24155Opts(
      uint16_t _smo,
      uint8_t  _stat_pin = 255,
      uint8_t  _isel_pin = 255,
      BQ24155USBCurrent _sl = BQ24155USBCurrent::LIMIT_500,
      uint16_t _fi = 0
    ) :
      sense_milliohms(_smo),
      stat_pin(_stat_pin),
      isel_pin(_isel_pin),
      src_limit(_sl),
      _flgs_initial(_fi)
    {};

    inline bool useStatPin() const {
      return (255 != stat_pin);
    };

    inline bool useISELPin() const {
      return (255 != isel_pin);
    };
};


/**
* Driver class for BQ24155.
*/
class BQ24155 : public I2CDevice {
  public:
    BQ24155(const BQ24155Opts*);
    ~BQ24155();

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    void printDebug(StringBuilder*);

    int8_t init();
    int8_t refresh();

    /**
    * @return true if the charger is initialized.
    */
    inline bool initComplete() {
      return (BQ24155_FLAG_MASK_INIT_CMPLT == (_flgs & BQ24155_FLAG_MASK_INIT_CMPLT));
    };

    /**
    * @return true if the charger is found on the bus.
    */
    inline bool devFound() {
      return (0x48 == (0xF8 & _get_shadow_value(BQ24155RegID::PART_REV)));
    };

    /**
    * @return true if the STAT pin is disabled.
    */
    inline bool disableSTATPin() {
      return (BQ24155_FLAG_DISABLE_STAT == (_flgs & BQ24155_FLAG_DISABLE_STAT));
    };


    int8_t   reset_charger_params();
    int8_t   punch_safety_timer();

    bool     charger_enabled();
    int8_t   charger_enabled(bool en);
    bool     hi_z_mode();
    int8_t   hi_z_mode(bool en);
    bool     charge_current_termination_enabled();
    int8_t   charge_current_termination_enabled(bool en);

    int16_t  charge_current_limit();
    int16_t  usb_current_limit();
    int8_t   usb_current_limit(int16_t milliamps);
    int8_t   usb_current_limit(BQ24155USBCurrent);

    float    batt_weak_voltage();
    int8_t   batt_weak_voltage(float);

    float    batt_reg_voltage();
    int8_t   batt_reg_voltage(float);

    BQ24155Fault getFault();
    BQ24155State getChargerState();


  private:
    const BQ24155Opts _opts;
    uint16_t _flgs      = 0;
    uint8_t  shadows[5] = {0, 0, 0, 0, 0};

    /**
    * @return ISEL pin state.
    */
    inline bool _isel_state() {     return (_flgs & BQ24155_FLAG_ISEL_HIGH);   };
    void _isel_state(bool x);

    /**
    * @return The chip's hardware revision.
    */
    inline uint8_t _part_revision() {   return (_get_shadow_value(BQ24155RegID::PART_REV) & 0x07);   };

    /**
    * Conversion fxn.
    * Given an optional step, returns the corresponding terminal-phase charging
    *   current (expressed in amps).
    * Compiler should optimize away the floating point div. TODO: verify this.
    *
    * @param integer for the step count.
    * @return The number of amps delivered in the termination phase.
    */
    inline float terminateChargeCurrent(uint8_t step = 0) {
      return (((step & 0x07) * (3.4f / _opts.sense_milliohms)) + BQ24155_VITERM_OFFSET);
    };

    /**
    * Conversion fxn.
    * Given an optional step, returns the corresponding bulk-phase charging
    *   current (expressed in amps).
    * Compiler should optimize away the floating point divs. TODO: verify this.
    *
    * @param integer for the step count.
    * @return The number of amps delivered in the bulk phase.
    */
    inline float bulkChargeCurrent(uint8_t step = 0) {
      return (((step & 0x07) * (6.8f / _opts.sense_milliohms)) + (BQ24155_VIREGU_OFFSET / _opts.sense_milliohms));
    };

    int8_t _post_discovery_init();

    int8_t  _set_shadow_value(BQ24155RegID, uint8_t);
    uint8_t _get_shadow_value(BQ24155RegID);
    int8_t  _read_registers(BQ24155RegID, uint8_t);
    int8_t  _write_registers(BQ24155RegID, uint8_t);

    static BQ24155RegID _reg_id_from_addr(const uint8_t addr);
};

#endif  // __LTC294X_DRIVER_H__
