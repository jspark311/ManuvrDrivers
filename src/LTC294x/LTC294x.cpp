/*
File:   LTC294x.cpp
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


Support for the Linear Technology battery gas guage.

TODO: It should be noted that this class assumes an LTC2942-1. This should be
        expanded in the future.
*/

#include "LTC294x.h"

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

//const DatumDef datum_defs[] = {
//  {
//    .desc    = "Shunt temperature",
//    .units   = COMMON_UNITS_C,
//    .type_id = TCode::FLOAT,
//    .flgs    = SENSE_DATUM_FLAG_HARDWARE
//  },
//  {
//    .desc    = "Battery voltage",
//    .units   = COMMON_UNITS_VOLTS,
//    .type_id = TCode::FLOAT,
//    .flgs    = SENSE_DATUM_FLAG_HARDWARE
//  },
//  {
//    .desc    = "Battery charge",
//    .units   = COMMON_UNITS_AMP_HOURS,
//    .type_id = TCode::FLOAT,
//    .flgs    = SENSE_DATUM_FLAG_HARDWARE
//  },
//  {
//    .desc    = "Instantaneous current",
//    .units   = COMMON_UNITS_AMPS,
//    .type_id = TCode::FLOAT,
//    .flgs    = SENSE_DATUM_FLAG_HARDWARE
//  },
//  {
//    .desc    = "Battery percent",
//    .units   = COMMON_UNITS_PERCENT,
//    .type_id = TCode::FLOAT,
//    .flgs    = 0x00
//  }
//};


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor. Takes pin numbers as arguments.
*/
LTC294x::LTC294x(const LTC294xOpts* o, uint16_t bc) : I2CDevice(LTC294X_I2CADDR), _opts(o), _batt_volume(bc) {
  if (_opts.useAlertPin()) {
    // This is the chip's default configuration.
    pinMode(_opts.pin, GPIOMode::INPUT_PULLUP);
  }
  else if (_opts.useCCPin()) {
    // TODO: This requires testing before it is safe to enable.
    //pinMode(_opts.pin, GPIOMode::OUTPUT_OD);
  }

  //defineRegister(LTC294xRegID::STATUS,        (uint8_t)  0x00,   false, true,  false);
  //defineRegister(LTC294xRegID::CONTROL,       (uint8_t)  0x3C,   false, false, true);
  //defineRegister(LTC294xRegID::ACC_CHARGE,    (uint16_t) 0x7FFF, false, false, true);
  //defineRegister(LTC294xRegID::CHRG_THRESH_H, (uint16_t) 0xFFFF, false, false, true);
  //defineRegister(LTC294xRegID::CHRG_THRESH_L, (uint16_t) 0x0000, false, false, true);
  //defineRegister(LTC294xRegID::VOLTAGE,       (uint16_t) 0x0000, false, false, false);
  //defineRegister(LTC294xRegID::V_THRESH,      (uint16_t) 0xFF00, false, false, true);
  //defineRegister(LTC294xRegID::TEMP,          (uint16_t) 0x0000, false, false, false);
  //defineRegister(LTC294xRegID::TEMP_THRESH,   (uint16_t) 0xFF00, false, false, true);
  _reset_tracking_data();
}


/*
* Destructor.
*/
LTC294x::~LTC294x() {
}


/*
* Here, we will compare options and set them as defaults.
*/
int8_t LTC294x::init() {
  uint8_t val = (uint8_t) _get_shadow_value(LTC294xRegID::CONTROL);
  uint8_t dps = _derive_prescaler();
  uint8_t rewrite = (val & 0xC7) | (dps << 3);  // Set the prescalar if it isn't already...

  // Given the battery capacity, set the range parameters...
  //_batt_volume
  //_charge_expected_min
  //_charge_expected_max

  // Normalize hardware against pin selection...
  if (_opts.useAlertPin()) {    rewrite = (rewrite & 0xF9) | 0x04;  }
  else if (_opts.useCCPin()) {  rewrite = (rewrite & 0xF9) | 0x02;  }
  else {                        rewrite = (rewrite & 0xF9);         }

  if (_opts.autostartReading()) {
    // Set the ADC to sample automatically.
    rewrite = (rewrite & ~LTC294X_OPT_MASK_ADC_MODE) | (uint8_t) LTC294xADCModes::AUTO;
    rewrite = rewrite | 1;
  }
  _reset_tracking_data();

  _write_control_reg(rewrite);

  _flags |= LTC294X_FLAG_INIT_AC;         // TODO: Convenient lie until feature done.
  _flags |= LTC294X_FLAG_INIT_THRESH_CL;  // TODO: Convenient lie until feature done.
  _flags |= LTC294X_FLAG_INIT_THRESH_CH;  // TODO: Convenient lie until feature done.

  // This is a conservative range for the 'C' variant. The 'I' variant range is
  //   -65C to 85C.
  // NOTE: That's 'f' for float. These values are Celcius.
  int8_t ret = setTemperatureThreshold(0.0f, 70.0f);

  return ret;
}


int8_t LTC294x::poll() {
  int8_t ret = -1;
  if (initComplete()) {
    ret--;
    if (0 == _read_registers(LTC294xRegID::ACC_CHARGE, 2)) {
      if (0 == _read_registers(LTC294xRegID::VOLTAGE, 2)) {
        if (0 == _read_registers(LTC294xRegID::TEMP, 2)) {
          ret = 0;
        }
      }
    }
  }
  return ret;
}



/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

int8_t LTC294x::_post_discovery_init() {
  int8_t ret = -1;
  // Check for clean registers and mark them as init'd.
  //_set_shadow_value((uint8_t) BQ24155RegID::STATUS,    0x00);
  //_set_shadow_value((uint8_t) BQ24155RegID::LIMITS,    0x30);
  //_set_shadow_value((uint8_t) BQ24155RegID::BATT_REGU, 0x0A);
  //_set_shadow_value((uint8_t) BQ24155RegID::FAST_CHRG, 0x89);

  if (initComplete()) {
    ret = 0;
  }
  return ret;
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t LTC294x::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t LTC294x::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf     = op->buffer();
    uint     len     = op->bufferLen();
    uint8_t  reg_idx = (uint8_t) _reg_id_from_addr(op->sub_addr);
    bool run_post_init_fxn = false;
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint i = 0; i < len; i++) {
          switch ((LTC294xRegID) reg_idx) {
            case LTC294xRegID::CONTROL:
              _flags |= LTC294X_FLAG_INIT_CTRL;
              break;
            case LTC294xRegID::ACC_CHARGE:    // The accumulated charge register.
              _flags |= LTC294X_FLAG_INIT_AC;
              break;
            case LTC294xRegID::CHRG_THRESH_H:
              _flags |= LTC294X_FLAG_INIT_THRESH_CH;
              break;
            case LTC294xRegID::CHRG_THRESH_L:
              _flags |= LTC294X_FLAG_INIT_THRESH_CL;
              break;
            case LTC294xRegID::V_THRESH:
              _flags |= LTC294X_FLAG_INIT_THRESH_V;
              break;
            case LTC294xRegID::TEMP_THRESH:
              _flags |= LTC294X_FLAG_INIT_THRESH_T;
              break;
            default:  // Illegal. A bad mistake was made somewhere.
              break;
          }
          reg_idx++;
        }
        break;

      case BusOpcode::RX:
        for (uint i = 0; i < len; i++) {
          uint16_t val = _get_shadow_value((LTC294xRegID) reg_idx);
          switch ((LTC294xRegID) reg_idx) {
            case LTC294xRegID::STATUS:
              if (val & 0x01) {   // Undervoltage lockout.
                //Kernel::log("LTC294X: undervoltage.\n");
                _reset_tracking_data();
              }
              if (val & 0x02) {   // One of the battery voltage limits was exceeded.
                //Kernel::log("LTC294X: battery voltage.\n");
              }
              if (val & 0x04) {   // Charge alert low
                //Kernel::log("LTC294X: Charge alert low.\n");
              }
              if (val & 0x08) {   // Charge alert high
                //Kernel::log("LTC294X: Charge alert high.\n");
              }
              if (val & 0x10) {   // Temperature alert
                //Kernel::log("LTC294X: Temperature\n");
              }
              if (val & 0x20) {   // AccCharge over/underflow.
                //_reset_tracking_data();
                //Kernel::log("LTC294X: AccCharge over/underflow.\n");
              }
              break;
            case LTC294xRegID::CHRG_THRESH_H:
              // TODO: This will need auditing after the endianness filter in the superclass.
              _thrsh_h_chrg = val;
              break;
            case LTC294xRegID::CHRG_THRESH_L:
              // TODO: This will need auditing after the endianness filter in the superclass.
              _thrsh_l_chrg = val;
              break;
            case LTC294xRegID::V_THRESH:
              { // TODO: This will need auditing after the endianness filter in the superclass.
                _thrsh_l_volt = val & 0xFF;
                _thrsh_h_volt = (val >> 8) & 0xFF;
              }
              break;
            case LTC294xRegID::TEMP_THRESH:
              { // TODO: This will need auditing after the endianness filter in the superclass.
                _thrsh_l_temp = val & 0xFF;
                _thrsh_h_temp = (val >> 8) & 0xFF;
              }
              break;

            // These registers are data from the sensor.
            // We read in groups, but only note the time on LTC294xRegID::ACC_CHARGE,
            //   because it is first in the order, and therefore, closer to the actual
            //   timestamp of the measurement (neglecting bus latency).
            case LTC294xRegID::ACC_CHARGE:   // Note the time.
              if (_is_monitoring()) {
                uint32_t now = millis();
                _sample_dt = wrap_accounted_delta(now, _sample_time);
                _sample_time = now;
                _read_registers(LTC294xRegID::VOLTAGE, 2);
              }
              break;

            case LTC294xRegID::VOLTAGE:  // Nothing needs doing here.
              break;

            case LTC294xRegID::TEMP:  // Final data register. Refresh the tracking data.
              if (_is_monitoring()) {
                _update_tracking();
              }
              break;

            case LTC294xRegID::CONTROL:
            default:  // Illegal. A bad mistake was made somewhere.
              break;
          }
          reg_idx++;
        }
        if (run_post_init_fxn) {
          _post_discovery_init();
        }
        break;

      default:
        break;
    }
  }
  return ret;
}


/*
* Dump this item to the dev log.
*/
void LTC294x::printDebug(StringBuilder* output) {
  uint8_t dsp = _derive_prescaler();
  output->concatf("-- LTC294%c-1 %sinitialized\n", (_is_2942() ? '2' : '1'), (initComplete() ? "" : "un"));
  if (_opts.useAlertPin()) {
    output->concatf("\tALERT pin:         %u\n", _opts.pin);
  }
  else if (_opts.useCCPin()) {
    output->concatf("\tCC pin:            %d\n", _opts.pin);
  }
  output->concatf("\tPrescaler:         %u\n", dsp);
  output->concatf("\tDev/ADC state      A%s / ", asleep() ? "sleep":"wake");
  switch (_adc_mode()) {
    case LTC294xADCModes::SLEEP:
      output->concat("SLEEP");
      break;
    case LTC294xADCModes::MANUAL_T:
      output->concat("MANUAL_T");
      break;
    case LTC294xADCModes::MANUAL_V:
      output->concat("MANUAL_V");
      break;
    case LTC294xADCModes::AUTO:
      output->concat("AUTO");
      break;
  }
  output->concatf("\n\tSamples:           %u (dt: %ums)\n", _sample_count, _sample_dt);
  output->concatf("\tLast sample:       %u\n", _sample_time);
  output->concatf("\tBattery Voltage:   %.2f%%\n", (double) batteryPercentVoltage());

  output->concat("\n\tCurrent:     \t Thresholds:\n");
  output->concatf("\t  C:   %.2f%%\t %.2f%% / %.2f%%\n",
    (double) batteryPercent(),
    (double) (dsp * 0.06640625f * _thrsh_l_chrg),
    (double) (dsp * 0.06640625f * _thrsh_h_chrg)
  );
  output->concatf("\t  V:   %.2f \t %.2f / %.2f\n",
    (double) batteryVoltage(),
    (double) (0.0234368f * _thrsh_l_volt),
    (double) (0.0234368f * _thrsh_h_volt)
  );
  output->concatf("\t  T:   %.2f \t %.2f / %.2f\n",
    (double) temperature(),
    (double) convertT(_thrsh_l_temp << 8),
    (double) convertT(_thrsh_h_temp << 8)
  );

  if (trackingReady()) {
    output->concat("\tRates:        \t Min/Max:\n");
    switch (_sample_count) {
      // NOTE: Upside-down. No breaks at all.
      default:
        output->concatf("\t  I:   %.2f mA\t %.2f / %.2f\n",
          (double) batteryCurrent(),
          (double) minimumCurrent(),
          (double) maximumCurrent()
        );
        output->concatf("\t  V:   %.2f/sec\t %.2f / %.2f\n",
          (double) _volt_dt,
          (double) _volt_min_dt,
          (double) _volt_max_dt
        );
      case 2:
        output->concatf("\t  T:   %.2f/sec\t %.2f / %.2f\n",
          (double) _temp_dt,
          (double) _temp_min,
          (double) _temp_max
        );
      case 1:
      case 0:
        output->concat("\n");
    }
  }
}



/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/
/**
* Resets the variables that deal with data tracking.
*/
void LTC294x::_reset_tracking_data() {
  _tracking_ready(false);
  _sample_dt      = 0;
  _sample_time    = 0;
  _sample_count   = 0;
  _chrg_reading_0 = 0;
  _volt_reading_0 = 0.0f;
  _temp_reading_0 = 0.0f;
  _chrg_dt        = 0.0f;
  _volt_dt        = 0.0f;
  _temp_dt        = 0.0f;
  _chrg_min_dt    = 0.0f;
  _volt_min_dt    = 0.0f;
  _chrg_max_dt    = 0.0f;
  _volt_max_dt    = 0.0f;
  _temp_min       = 0.0f;
  _temp_max       = 0.0f;
}


/**
* Refresh our tracking data.
* Increments _sample_count.
*/
void LTC294x::_update_tracking() {
  uint16_t c = batteryCharge();
  float    v = batteryVoltage();
  float    t = temperature();
  _sample_count++;
  switch (_sample_count) {
    case 1:    // The first samples to arrive. Baseline 1st-order ranges.
      _temp_min = t;
      _temp_max = t;
      break;
    case 3:    // Baseline second-order ranges.
      _chrg_min_dt = _chrg_dt;
      _chrg_max_dt = _chrg_dt;
      _volt_max_dt = _volt_dt;
      _volt_min_dt = _volt_dt;
      _tracking_ready(true);   // Mark tracking data valid.
      // NOTE: No break;
    default:   // If we have two or more samples, we can take derivatives.
      _temp_min = strict_min(_temp_min, t);
      _temp_max = strict_max(_temp_max, t);
      if (_sample_dt) {
        // TODO: Not in proper units.
        _chrg_dt = ((_c_to_mA(c - _chrg_reading_0) * 1000) / ((float) _sample_dt));  // We want C/millisec (mA).
        _volt_dt = ((((uint32_t) v - _volt_reading_0) * 1000) / _sample_dt);  // We want V/sec.
        _temp_dt = ((((uint32_t) t - _temp_reading_0) * 1000) / _sample_dt);  // We want T/sec.
      }
      if (2 < _sample_count) {
        // If 3 or more samples, we can measure the range of 2nd-order data.
        _chrg_min_dt = strict_min(_chrg_min_dt, _chrg_dt);
        _chrg_max_dt = strict_max(_chrg_max_dt, _chrg_dt);
        _volt_min_dt = strict_min(_volt_min_dt, _volt_dt);
        _volt_max_dt = strict_max(_volt_max_dt, _volt_dt);
      }
      break;
  }

  _chrg_reading_0 = c;  // Shift the new values into place.
  _volt_reading_0 = v;
  _temp_reading_0 = t;
  //updateDatum(0, _temp_reading_0);
  //updateDatum(1, _volt_reading_0);
  //updateDatum(3, _chrg_dt);
  //updateDatum(4, batteryPercent());
}


int8_t LTC294x::_adc_mode(LTC294xADCModes m) {
  switch (m) {
    case LTC294xADCModes::SLEEP:
      _reset_tracking_data();
      break;
    case LTC294xADCModes::MANUAL_T:
    case LTC294xADCModes::MANUAL_V:
    case LTC294xADCModes::AUTO:
      break;
  }
  uint8_t val = (uint8_t) _get_shadow_value(LTC294xRegID::CONTROL);
  val = (val & ~LTC294X_OPT_MASK_ADC_MODE) | (uint8_t) m;
  return _write_control_reg(val);
}

int8_t LTC294x::sleep(bool x) {
  uint8_t val = (uint8_t) _get_shadow_value(LTC294xRegID::CONTROL);
  val = (val & 0xFE);
  if (x) {
    //_reset_tracking_data();
    val = val | 1;
  }
  return _write_control_reg(val);
}

/**
* Negative values returned by this function mean the battery is being drained.
*
* @return Instantaneous net current flow to/from the battery (in mA).
*/
float LTC294x::_c_to_mA(uint16_t chrg) {
  return (_derive_prescaler() * 0.006640625f * chrg);
}

/**
* @return Our best estimate of the battery's charge state, as a percentage.
*/
float LTC294x::batteryPercent() {
  // If the charge boundaries look sane, use charge register.
  return (_derive_prescaler() * 0.06640625f * _get_shadow_value(LTC294xRegID::ACC_CHARGE));
}

/**
* @return Our best estimate of the battery's charge state, as a percentage.
*/
float LTC294x::batteryPercentVoltage() {
  float return_value = 0.0f;
  float vl = (0.0234368f * _thrsh_l_volt);
  float vh = (0.0234368f * _thrsh_h_volt);
  float vc = batteryVoltage();
  if ((vc > vl) && (vh > vl)) {
    return_value = ((vc - vl) / (vh - vl)) * 100.0f;
  }
  return return_value;
}

/**
* @return The battery voltage.
*/
float LTC294x::batteryVoltage() {
  return (_get_shadow_value(LTC294xRegID::VOLTAGE) * 0.00009155f);
}

/**
* @return The chip's temperature in degrees Celcius.
*/
float LTC294x::temperature() {
  return ((_get_shadow_value(LTC294xRegID::TEMP) * 0.009155f) - DEG_K_C_OFFSET);
}

int8_t LTC294x::setChargeThresholds(uint16_t low, uint16_t high) {
  // TODO: Fill
  _thrsh_l_chrg = low;
  _thrsh_h_chrg = high;
  return _set_thresh_reg_charge(low, high);
}

int8_t LTC294x::setVoltageThreshold(float low, float high) {
  // NOTE: 0.0234368 == (0.00009155f >> 8)  // single byte register
  _thrsh_l_volt = (uint8_t) (low  / 0.0234368f);
  _thrsh_h_volt = (uint8_t) (high / 0.0234368f);
  uint16_t val = ((uint16_t) _thrsh_l_volt) | ((uint16_t) _thrsh_h_volt << 8);
  _set_shadow_value(LTC294xRegID::V_THRESH, val);
  return _write_registers(LTC294xRegID::V_THRESH, 2);
}

int8_t LTC294x::setTemperatureThreshold(float low, float high) {
  // NOTE: 2.34368 == (0.009155f >> 8)  // single byte register
  _thrsh_l_temp = (uint8_t) (low  / 2.34368f);
  _thrsh_h_temp = (uint8_t) (high / 2.34368f);
  uint16_t val = ((uint16_t) _thrsh_l_temp) | ((uint16_t) _thrsh_h_temp << 8);
  _set_shadow_value(LTC294xRegID::TEMP_THRESH, val);
  return _write_registers(LTC294xRegID::TEMP_THRESH, 2);
}

/*
* Set the accumulated charge register.
*/
int8_t LTC294x::_set_charge_register(uint16_t val) {
  if (initComplete() && !asleep()) {
    // We need to shut down the analog section before setting this register.
    return -1;
  }
  _set_shadow_value(LTC294xRegID::ACC_CHARGE, val);
  return _write_registers(LTC294xRegID::ACC_CHARGE, 2);
}


int8_t LTC294x::_set_thresh_reg_charge(uint16_t l, uint16_t h) {
  _set_shadow_value(LTC294xRegID::CHRG_THRESH_H, h);
  _set_shadow_value(LTC294xRegID::CHRG_THRESH_L, l);
  return _write_registers(LTC294xRegID::CHRG_THRESH_H, 4);
}


int8_t LTC294x::_write_control_reg(uint8_t v) {
  _set_shadow_value(LTC294xRegID::CONTROL, v);
  return _write_registers(LTC294xRegID::CONTROL, 1);
}


/**
* @return The minimum servicable prescaler value for the given battery capacity.
*/
uint8_t LTC294x::_derive_prescaler() {
  uint8_t res = _batt_volume * 0.022978f;
  uint8_t ret = 7;   // The max prescaler.
  if (res < 64) { ret--; }
  if (res < 32) { ret--; }
  if (res < 16) { ret--; }
  if (res < 8)  { ret--; }
  if (res < 4)  { ret--; }
  if (res < 2)  { ret--; }
  if (res < 1)  { ret--; }
  return ret;
}
