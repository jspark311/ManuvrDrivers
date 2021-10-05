/*
File:   ManuvrPMU-r2.cpp
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

*/

#include "ManuvrPMU-r2.h"


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
// volatile static unsigned long _stat1_change_time = 0;
// volatile static unsigned long _stat2_change_time = 0;
// volatile static unsigned int  _stat1_prior_delta = 0;
// volatile static unsigned int  _stat2_prior_delta = 0;

static ManuvrPMU* INSTANCE = nullptr;


/*
* Console breakouts for driver functions.
*/
int callback_pmu_tools(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char* cmd  = args->position_trimmed(0);
  int   arg1 = args->position_as_int(1);

  if (0 == StringBuilder::strcasecmp(cmd, "verbosity")) {
    switch (args->count()) {
      case 2:
        INSTANCE->logVerbosity(0x07 & (uint8_t) arg1);
        // NOTE: No break;
      case 1:
        text_return->concatf("PMU log verbosity is %u\n", INSTANCE->logVerbosity());
        break;
      default:
        text_return->concat("Usage:  verbosity [new_value]\n");
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
    text_return->concatf("Resetting charger parameters returns %d\n", INSTANCE->bq24155.reset_charger_fsm());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
    switch (arg1) {
      case 1:
        text_return->concatf("ltc294x.poll() returns %d.\n", INSTANCE->ltc294x.poll());
        break;
      case 2:
        text_return->concatf("bq24155.refresh() returns %d.\n", INSTANCE->bq24155.refresh());
        break;
      default:
        text_return->concat("Usage:  pmu refresh [1|2]\n");
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "regulation")) {
    if (2 == args->count()) {
      INSTANCE->bq24155.batt_reg_voltage(args->position_as_double(1));
    }
    text_return->concatf("bq24155 battery regulation voltage is %.2f.\n", INSTANCE->bq24155.batt_reg_voltage());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "weakness")) {
    if (2 == args->count()) {
      INSTANCE->bq24155.batt_weak_voltage(args->position_as_double(1));
    }
    text_return->concatf("bq24155 battery weakness voltage is %.2f.\n", INSTANCE->bq24155.batt_weak_voltage());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "punch")) {
    text_return->concatf("bq24155.punch_safety_timer() returns %d\n", INSTANCE->bq24155.punch_safety_timer());
  }

  else if (0 == StringBuilder::strcasecmp(cmd, "usb")) {
    if (2 == args->count()) {
      text_return->concatf("usb_current_limit(%u) returns %d\n", arg1, INSTANCE->bq24155.usb_current_limit((int16_t) arg1));
    }
    text_return->concatf("usb_current_limit = %d\n", INSTANCE->bq24155.usb_current_limit());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "termination")) {
    if (2 == args->count()) {
      INSTANCE->bq24155.charge_current_termination_enabled(1 == arg1);
    }
    text_return->concatf("termination is %sabled.\n", INSTANCE->bq24155.charge_current_termination_enabled() ? "en" : "dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "hiz")) {
    if (2 == args->count()) {
      INSTANCE->bq24155.hi_z_mode(1 == arg1);
    }
    text_return->concatf("hi_z_mode is %sabled.\n", INSTANCE->bq24155.hi_z_mode() ? "en" : "dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "charger")) {
    if (2 == args->count()) {
      INSTANCE->bq24155.charger_enabled(1 == arg1);
    }
    text_return->concatf("bq24155 is %sabled.\n", INSTANCE->bq24155.charger_enabled() ? "en" : "dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "termination")) {
    if (2 == args->count()) {
      INSTANCE->bq24155.charge_current_termination_enabled((1 == arg1), false);
    }
    text_return->concatf("bq24155 charge termination is %sabled.\n", INSTANCE->bq24155.charge_current_termination_enabled() ? "en" : "dis");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "sleep")) {
    if (2 == args->count()) {
      INSTANCE->ltc294x.sleep(1 == arg1);
    }
    text_return->concatf("ltc294x is %s.\n", INSTANCE->ltc294x.asleep() ? "asleep" : "awake");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "aux")) {
    if (2 == args->count()) {
      INSTANCE->auxRegEnabled(1 == arg1);
    }
    text_return->concatf("AUX regulator is %sabled.\n", INSTANCE->auxRegEnabled() ? "en" : "dis");
  }

  else if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    switch (arg1) {
      default:
      case 0:
        INSTANCE->printDebug(text_return);
        INSTANCE->printBattery(text_return);
        break;
      case 1:
        INSTANCE->ltc294x.printDebug(text_return);
        break;
      case 2:
        INSTANCE->bq24155.printDebug(text_return);
        break;
      case 3:
        INSTANCE->ltc294x.printRegisters(text_return);
        break;
      case 4:
        INSTANCE->bq24155.printRegisters(text_return);
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    switch (arg1) {
      default:
      case 0:
        text_return->concatf("PMU init() returns %d\n", INSTANCE->init());
        break;
      case 1:
        text_return->concatf("ltc294x.init() returns %d\n", INSTANCE->ltc294x.init());
        break;
      case 2:
        text_return->concatf("bq24155.init() returns %d\n", INSTANCE->bq24155.init());
        break;
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
    text_return->concatf("bq24155.refresh() returns %d\n", INSTANCE->bq24155.refresh());
  }
  else {
    ret = -1;
  }
  return ret;
}

const ConsoleCommand cmd00 = ConsoleCommand("pmu", 'p', ParsingConsole::tcodes_str_4, "PMU tools", "[info|punch|charging|aux|reset|init|refresh|verbosity]", 1, callback_pmu_tools);


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor. All params are required.
*/
ManuvrPMU::ManuvrPMU(const BQ24155Opts* charger_opts, const LTC294xOpts* fuel_gauge_opts, const ManuvrPMUOpts* o) :
  bq24155(charger_opts), ltc294x(fuel_gauge_opts, charger_opts->battery.capacity),
  _opts(o), _battery(&charger_opts->battery),
  _flags(_opts.flags) {
    INSTANCE = this;
}


ManuvrPMU::~ManuvrPMU() {
}


/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

/**
* Turns the regulator on or off.
*
* @param re True to enable the aux reg.
* @return non-zero on error.
*/
int8_t ManuvrPMU::auxRegEnabled(bool re) {
  if (_opts.useREPin()) {
    // Shutdown is achieved by pulling pin low.
    setPin(_opts.re_pin, re);
    _flags.set(DIGITAB_PMU_FLAG_ENABLED, re);
    return 0;
  }
  return -1;
}


/**
* Sets the regulator voltage to 2.5v or 3.3v.
*
* @param lpm True for low-power mode.
* @return non-zero on error.
*/
int8_t ManuvrPMU::auxRegLowPower(bool lpm) {
  if (_opts.useVSPin()) {
    // 2.5v mode is selected by pulling pin low.
    setPin(_opts.vs_pin, !lpm);
    _flags.set(DIGITAB_PMU_FLAG_V_25, lpm);
    return 0;
  }
  return -1;
}


/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void ManuvrPMU::printBattery(StringBuilder* output) {
  const uint8_t DBAR_WIDTH = 25;
  float v  = ltc294x.batteryVoltage();
  float vp = ltc294x.batteryPercentVoltage();
  char* bar_buf = (char*) alloca(DBAR_WIDTH+1);
  uint8_t mark = (vp/100.0f) * DBAR_WIDTH;
  for (uint8_t i = 0; i < DBAR_WIDTH; i++) {
    if (mark == i) {
      *(bar_buf + i) = '|';
    }
    else {
      *(bar_buf + i) = (i<mark) ? '=' : ' ';
    }
  }
  *(bar_buf + DBAR_WIDTH) = '\0';

  output->concatf("-- Battery (%.2fV)\n", v);
  output->concatf("\t[%s] %.2f%%\n", bar_buf, vp);
  output->concatf("\t%.2fV            %.2fV\n", _battery.voltage_min, _battery.voltage_max);
  output->concatf("\tCapacity %umAh\n", _battery.capacity);
  output->concatf("\tWeak     %.2fV\n", _battery.voltage_weak);
  output->concatf("\tFloat    %.2fV\n", _battery.voltage_float);
  output->concatf("\tMax      %.2fV\n", _battery.voltage_max);
}

/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void ManuvrPMU::printDebug(StringBuilder* output) {
  const char* aux_reg_state = auxRegLowPower() ? "2.5v" : "3.3v";
  StringBuilder::styleHeader1(output, "ManuvrPMU");
  output->concatf("\tVS/RE pins          %u/%u\n", _opts.vs_pin, _opts.re_pin);
  output->concatf("\tAuxiliary regulator %s\n", auxRegEnabled() ? aux_reg_state : "Disabled");
  output->concatf("\tCharge state        %s\n", getChargeStateString());
}


/*
* Initialize the driver
*/
int8_t ManuvrPMU::init(I2CAdapter* b) {
  int8_t ret = -1;
  if (0 == _ll_pin_init()) {   // Configure the pins if they are not already.
    ret--;
    if (nullptr != b) {
      ret--;
      ltc294x.assignBusInstance(b);
      bq24155.assignBusInstance(b);
      int8_t ret0 = ltc294x.init();
      int8_t ret1 = bq24155.init();
      if (0 == ret0) {
        // We want the gas guage to warn us if the voltage leaves the realm of safety.
        ltc294x.setVoltageThreshold(_battery.voltage_weak, _battery.voltage_max);
        ltc294x.sleep(false);
      }
      if ((0 == ret0) && (0 == ret1)) {
        ret = 0;
      }
    }
  }
  return ret;
}


/**
* Setup the low-level pin details. Execution is idempotent.
*
* @return
*   -1 if the pin setup is wrong. Class must halt.
*   0  if the pin setup is complete.
*/
int8_t ManuvrPMU::_ll_pin_init() {
  int8_t ret = 0;
  if (!_flags.value(DIGITAB_PMU_FLAG_PINS_CONFIGURED)) {
    // For safety's sake, this pin init order is important.
    // Flags are reliable at this point. So if the caller set flags indicating
    //   a given initial state for the auxililary regulator, it will be honored
    //   if possible.
    // If no flags are given, the default behavior is for the aux regulator
    //   to remain powered down, with 3.3v as the default power output when it is
    //   enabled.
    // If a flag is set a certain way, but no pin control is possible, the flags
    //   related to that feature effectively become constants, and we will trust
    //   that they reflect the state of the hardware.
    if (_opts.useVSPin()) {
      pinMode(_opts.vs_pin, GPIOMode::OUTPUT);
      setPin(_opts.vs_pin, !auxRegLowPower());
    }
    if (_opts.useREPin()) {
      pinMode(_opts.re_pin, GPIOMode::OUTPUT);
      setPin(_opts.re_pin, auxRegEnabled());
    }

    // TODO: Should be migrated to LTC294x driver.
    //if (fuel_gauge_opts->useAlertPin()) {
    //  // If we are going to use the alert feature, we will enable the pull-up and
    //  //   pitch an event.
    //  //setPinEvent(fuel_gauge_opts->pin, FALLING_PULL_UP, &_battery_alert_msg);
    //}
    _flags.set(DIGITAB_PMU_FLAG_PINS_CONFIGURED, (0 == ret));
  }
  return ret;
}



/*
* Periodically allocate time to the driver.
* TODO: This is on borrowed time.
*/
int8_t ManuvrPMU::poll() {
  uint32_t now = millis();
  int8_t ret = 0;
  if (ltc294x.initComplete()) {
    if (wrap_accounted_delta(_last_meter_poll, now) >= _polling_period) {
      if (2 == ltc294x.poll()) {
        ret += 1;
      }
      _last_meter_poll = now;
    }
  }
  if (bq24155.initComplete()) {
    if (wrap_accounted_delta(_punch_timestamp, now) >= 1740000) {
      // One every 32 minutes, the charger will stop.
      // We punch the safety timer every 29 minutes.
      bq24155.punch_safety_timer();
    }
    if (wrap_accounted_delta(_last_charger_poll, now) >= _polling_period) {
      bq24155.refresh_status();
      _last_charger_poll = now;
    }
  }
  return ret;
}


int8_t ManuvrPMU::_invoke_batt_callback(ChargeState e) {
  return (nullptr != _callback) ? _callback(e) : -1;
}


/*******************************************************************************
* Console
*******************************************************************************/

/**
* Insert our console commands into the given console.
*
* @param console
* @return 0 always
*/
int8_t ManuvrPMU::configureConsole(ParsingConsole* console) {
  console->defineCommand(&cmd00);
  return 0;
}


/**
* Allow the application to retreive the log.
*
* @param l is a reference to the buffer which should receive the log.
*/
void ManuvrPMU::fetchLog(StringBuilder* l) {
  ltc294x.fetchLog(l);
  //bq24155.fetchLog(l);
  if (_local_log.length() > 0) {
    if (nullptr != l) {
      _local_log.string();
      l->concatHandoff(&_local_log);
    }
  }
}


//     case 'L':
//     case 'l':
//       local_log.concatf(
//         "Setting auxiliary regulator to %.1fv... %s\n",
//         (*(str) == 'L' ? 2.5f : 3.3f),
//         (0 != auxRegLowPower(*(str) == 'L')) ? "failure" : "success"
//       );
//       break;
//
//     case 'u':   // USB host current limit.
//       if (temp_int) {
//         bq24155.usb_current_limit((unsigned int) temp_int);
//       }
//       local_log.concatf("USB current limit: %dmA\n", bq24155.usb_current_limit());
//       break;
