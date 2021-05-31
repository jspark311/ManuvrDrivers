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
  char* arg1 = args->position_trimmed(1);
  char* arg2 = args->position_trimmed(2);

  if (0 == StringBuilder::strcasecmp(cmd, "verbosity")) {
    switch (args->count()) {
      case 2:
        INSTANCE->logVerbosity(0x07 & args->position_as_int(1));
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
    text_return->concat("TODO: Unimplemented\n");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "info")) {
    INSTANCE->printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "battery")) {
    INSTANCE->printBattery(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("PMU init() returns %d\n", INSTANCE->init());
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
    text_return->concat("TODO: Unimplemented\n");
  }
  else {
    ret = -1;
  }
  return ret;
}

const ConsoleCommand cmd00 = ConsoleCommand("pmu", 'p', ParsingConsole::tcodes_str_4, "PMU tools", "[info|battery|punch|charging|aux|reset|init|refresh|verbosity]", 1, callback_pmu_tools);


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
ManuvrPMU::ManuvrPMU(const BQ24155Opts* charger_opts, const LTC294xOpts* fuel_gauge_opts, const ManuvrPMUOpts* o, const BatteryOpts* bo) :
  _opts(o), _battery(bo),
  _bq24155(charger_opts), _ltc294x(fuel_gauge_opts, bo->capacity), _flags(_opts.flags) {
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
  float v  = _ltc294x.batteryVoltage();
  float vp = _ltc294x.batteryPercentVoltage();
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
}

/**
* Debug support function.
*
* @param A pointer to a StringBuffer object to receive the output.
*/
void ManuvrPMU::printDebug(StringBuilder* output) {
  const char* aux_reg_state = auxRegLowPower() ? "2.5v" : "3.3v";
  output->concatf("-- VS/RE pins          %u/%u\n", _opts.vs_pin, _opts.re_pin);
  output->concatf("-- Auxiliary regulator %s\n", auxRegEnabled() ? aux_reg_state : "Disabled");
  output->concatf("-- Charge state        %s\n", getChargeStateString());
}


/*
* Initialize the driver
*/
int8_t ManuvrPMU::init(I2CAdapter* b) {
  int8_t ret = -1;
  // Now... we have battery details. So we derive some settings for the two
  //   I2C chips we are dealing with.
  _bq24155.batt_reg_voltage(_battery.voltage_max);
  _bq24155.batt_weak_voltage(_battery.voltage_weak);
  // We want the gas guage to warn us if the voltage leaves the realm of safety.
  _ltc294x.setVoltageThreshold(_battery.voltage_weak, _battery.voltage_max);
  if (0 == _ll_pin_init()) {   // Configure the pins if they are not already.
    if (nullptr != b) {
      _ltc294x.assignBusInstance(b);
      _bq24155.assignBusInstance(b);
      if (0 == _ltc294x.init()) {
      }
      if (0 == _bq24155.init()) {
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
  uint32_t ts = millis();
  int8_t ret = -1;
  //_ltc294x.readSensor();
  if (ts >= (_punch_timestamp + 1740000)) {
    // One every 32 minutes, the charger will stop.
    // We punch the safety timer every 29 minutes.
    _bq24155.punch_safety_timer();
  }
  else {
    _bq24155.refresh();
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
  if (_local_log.length() > 0) {
    if (nullptr != l) {
      _local_log.string();
      l->concatHandoff(&_local_log);
    }
  }
}


// void ManuvrPMU::consoleCmdProc(StringBuilder* input) {
//   // TODO: This function (and the open-scoping demands it makes on member classes)
//   //         is awful. It is hasty until I can learn enough from some other PMU
//   //         abstraction to do something more sensible.
//   const char* str = (char *) input->position(0);
//   char c    = *str;
//   int temp_int = 0;
//
//   if (input->count() > 1) {
//     // If there is a second token, we proceed on good-faith that it's an int.
//     temp_int = input->position_as_int(1);
//   }
//   else if (strlen(str) > 1) {
//     // We allow a short-hand for the sake of short commands that involve a single int.
//     temp_int = atoi(str + 1);
//   }
//
//   switch (c) {
//     case 'p':
//     case 'P':
//       // Start or stop the periodic sensor read.
//       if (temp_int) {
//         _periodic_pmu_read.alterSchedulePeriod(temp_int * 10);
//         local_log.concatf("_periodic_pmu_read set to %d ms period.\n", (temp_int * 10));
//       }
//       _periodic_pmu_read.enableSchedule(*(str) == 'P');
//       local_log.concatf("%s _periodic_pmu_read.\n", (*(str) == 'p' ? "Stopping" : "Starting"));
//       break;
//
//     case 'X':
//     case 'x':
//       local_log.concatf(
//         "%sabling auxiliary regulator... %s\n",
//         (*(str) == 'X' ? "En" : "Dis"),
//         (0 != auxRegEnabled(*(str) == 'X')) ? "failure" : "success"
//       );
//       break;
//
//     case 'L':
//     case 'l':
//       local_log.concatf(
//         "Setting auxiliary regulator to %.1fv... %s\n",
//         (*(str) == 'L' ? 2.5f : 3.3f),
//         (0 != auxRegLowPower(*(str) == 'L')) ? "failure" : "success"
//       );
//       break;
//
//     case 'i':
//       switch (temp_int) {
//         case 1:
//           _ltc294x.printDebug(&local_log);
//           break;
//         case 2:
//           _bq24155.printDebug(&local_log);
//           break;
//         case 5:
//           printBattery(&local_log);
//           break;
//
//         default:
//           printDebug(&local_log);
//       }
//       break;
//
//     case 'a':
//       switch (temp_int) {
//         case 1:
//           _ltc294x.init();
//           break;
//         case 2:
//           _bq24155.init();
//           break;
//         default:
//           local_log.concat("1: _ltc294x.init()\n");
//           local_log.concat("2: _bq24155.init()\n");
//           break;
//       }
//       break;
//
//     case 'd':
//       switch (temp_int) {
//         case 1:
//           local_log.concat("Refreshing _ltc294x.\n");
//           _ltc294x.readSensor();
//           break;
//         case 2:
//           local_log.concat("Refreshing _bq24155.\n");
//           _bq24155.refresh();
//           break;
//         case 3:
//           _ltc294x.printRegisters(&local_log);
//           break;
//         case 4:
//           _bq24155.printRegisters(&local_log);
//           break;
//         default:
//           local_log.concat("1: _ltc294x.readSensor()\n");
//           local_log.concat("2: _bq24155.refresh()\n");
//           break;
//       }
//       break;
//
//     ///////////////////////
//     // Gas guage
//     ///////////////////////
//     case 's':
//       switch (temp_int) {
//         case 1:
//           _ltc294x.setVoltageThreshold(3.3f, 4.4f);
//           break;
//         case 2:
//           _ltc294x.sleep(true);
//           break;
//         case 3:
//           _ltc294x.sleep(false);
//           break;
//       }
//       break;
//
//     ///////////////////////
//     // Charger
//     ///////////////////////
//     case 'E':
//     case 'e':
//       local_log.concatf("%sabling battery charge...\n", (*(str) == 'E' ? "En" : "Dis"));
//       _bq24155.charger_enabled(*(str) == 'E');
//       break;
//
//     case 'T':
//     case 't':
//       local_log.concatf("%sabling charge current termination...\n", (*(str) == 'T' ? "En" : "Dis"));
//       _bq24155.charger_enabled(*(str) == 'T');
//       break;
//
//     case '*':
//       local_log.concat("Punching safety timer...\n");
//       _bq24155.punch_safety_timer();
//       break;
//
//     case 'R':
//       local_log.concat("Resetting charger parameters...\n");
//       _bq24155.reset_charger_params();
//       _bq24155.batt_reg_voltage(_battery.voltage_max);
//       _bq24155.batt_weak_voltage(_battery.voltage_weak);
//       break;
//
//     case 'u':   // USB host current limit.
//       if (temp_int) {
//         _bq24155.usb_current_limit((unsigned int) temp_int);
//       }
//       local_log.concatf("USB current limit: %dmA\n", _bq24155.usb_current_limit());
//       break;
//
//     case 'v':   // Battery regulation voltage
//       if (temp_int) {
//         local_log.concatf("Setting battery regulation voltage to %.2fV\n", input->position_as_double(1));
//         _bq24155.batt_reg_voltage(input->position_as_double(1));
//       }
//       local_log.concatf("Batt reg voltage:  %.2fV\n", _bq24155.batt_reg_voltage());
//       break;
//
//     case 'w':   // Battery weakness voltage
//       if (temp_int) {
//         local_log.concatf("Setting battery weakness voltage to %.2fV\n", input->position_as_double(1));
//         _bq24155.batt_weak_voltage(input->position_as_double(1));
//       }
//       local_log.concatf("Batt weakness voltage:  %.2fV\n", _bq24155.batt_weak_voltage());
//       break;
//
//     default:
//       break;
//   }
//
//   flushLocalLog();
// }
