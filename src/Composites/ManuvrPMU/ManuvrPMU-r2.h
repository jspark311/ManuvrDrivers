/*
File:   ManuvrPMU-r2.h
Author: J. Ian Lindsay
Date:   2017.06.31

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


Our goal is to encapsulate power-supply concerns to this class.
LiPo is the assumed chemistry.
*/

#include <inttypes.h>
#include <stdint.h>
#include <StringBuilder.h>
#include <Battery.h>
#include <FlagContainer.h>
#include <AbstractPlatform.h>
#include <BusQueue/I2CAdapter.h>

// Uses these two drivers from this same codebase
#include "../../BQ24155/BQ24155.h"
#include "../../LTC294x/LTC294x.h"

#ifndef __DIGITABULUM_PMU_DRIVER_H__
#define __DIGITABULUM_PMU_DRIVER_H__

#define DIGITAB_PMU_FLAG_PINS_CONFIGURED   0x01  // Low-level pin setup is complete.
#define DIGITAB_PMU_FLAG_V_25              0x02  // Aux regulator is set to 2.5v.
#define DIGITAB_PMU_FLAG_ENABLED           0x04  // Aux regulator is enabled.

/**
* Options for the PowerPlant.
*/
class ManuvrPMUOpts {
  public:
    const uint8_t vs_pin;  // Which pin is bound to aux voltage select?
    const uint8_t re_pin;  // Which pin is bound to aux regulator enable?
    const uint8_t flags;   // Flags that the class should start with.


    ManuvrPMUOpts(const ManuvrPMUOpts* o) :
      vs_pin(o->vs_pin),
      re_pin(o->re_pin),
      flags(o->flags) {};

    ManuvrPMUOpts(uint8_t _vspin, uint8_t _repin, uint8_t _f) :
      vs_pin(_vspin),
      re_pin(_repin),
      flags(_f) {};

    ManuvrPMUOpts(uint8_t _vspin, uint8_t _repin) :
      vs_pin(_vspin),
      re_pin(_repin),
      flags(0) {};

    ManuvrPMUOpts(uint8_t _f) :
      vs_pin(255),
      re_pin(255),
      flags(_f) {};

    inline bool useVSPin() const {
      return (255 != vs_pin);
    };

    inline bool useREPin() const {
      return (255 != re_pin);
    };
};


class ManuvrPMU {
  public:
    BQ24155 bq24155;
    LTC294x ltc294x;

    ManuvrPMU(const BQ24155Opts*, const LTC294xOpts*, const ManuvrPMUOpts*);
    ~ManuvrPMU();

    void printDebug(StringBuilder*);
    void printBattery(StringBuilder*);
    int console_handler(StringBuilder* text_return, StringBuilder* args);

    inline uint8_t logVerbosity() {           return _verbosity;    };
    inline void    logVerbosity(uint8_t x) {     _verbosity = x;    };

    int8_t init(I2CAdapter* bus = nullptr);
    int8_t poll();

    inline void attachCallback(BatteryStateCallback x) {  _callback = x;  };

    /* Is the aux regulator enabled? */
    inline bool auxRegEnabled() {   return (_flags.value(DIGITAB_PMU_FLAG_ENABLED));  };
    inline bool auxRegLowPower() {  return (_flags.value(DIGITAB_PMU_FLAG_V_25));     };
    inline ChargeState getChargeState() {   return _charge_state;   };

    /* Control over the auxilary regulator, if the hardware supports it. */
    int8_t auxRegEnabled(bool);
    int8_t auxRegLowPower(bool);

    inline float battVoltage() {    return ltc294x.batteryVoltage();   };


    /* Inlines for object-style usage of static functions... */
    inline const char* getChargeStateString() {   return BatteryOpts::batteryStateStr(_charge_state);  };


  private:
    const ManuvrPMUOpts  _opts;
    const BatteryOpts    _battery;

    uint32_t       _punch_timestamp = 0;
    uint32_t       _polling_period  = 2253;
    uint32_t       _last_meter_poll = 0;
    uint32_t       _last_charger_poll = 0;
    BatteryStateCallback _callback  = nullptr;
    ChargeState    _charge_state    = ChargeState::UNKNOWN;
    uint8_t        _verbosity       = 5;   // How chatty is the debug log?
    FlagContainer8 _flags;

    int8_t  _ll_pin_init();
    int8_t _invoke_batt_callback(ChargeState);
};

#endif //__DIGITABULUM_PMU_DRIVER_H__
