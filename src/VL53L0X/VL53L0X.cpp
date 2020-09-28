/*
I have hard-forked this driver in preparation for changing it.
The original library can be found here:
https://github.com/pololu/vl53l0x-arduino

Original license text is reproduced in the header file.
                                                ---J. Ian Lindsay
*/

// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include "VL53L0X.h"

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

static const uint8_t VL53L0X_REG_ADDRS[58] = {
  0x00, 0x01, 0x04, 0x09, 0x0A, 0x0B, 0x0C, 0x0E, 0x13, 0x14, 0x20, 0x27,
  0x28, 0x30, 0x30, 0x32, 0x33, 0x44, 0x46, 0x47, 0x48, 0x4E, 0x4F, 0x50,
  0x51, 0x52, 0x55, 0x56, 0x57, 0x60, 0x61, 0x62, 0x64, 0x67, 0x70, 0x71,
  0x72, 0x80, 0x81, 0x84, 0x89, 0x8A, 0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5,
  0xB6, 0xB6, 0xBC, 0xBF, 0xC0, 0xC0, 0xC2, 0xD0, 0xD4, 0xF8
};

static uint8_t _reg_addr_from_id(const VL53L0XRegID r) {
  if (((uint8_t) r) >= ((uint8_t) VL53L0XRegID::INVALID)) {
    return VL53L0X_REG_ADDRS[(uint8_t) r];
  }
  return 0xFF;
}

static VL53L0XRegID _reg_id_from_addr(const uint8_t) {
  // TODO
  return VL53L0XRegID::INVALID;
}

static const uint8_t _get_reg_len(uint8_t r) {
  switch (r) {
    case VL53L0X::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI:
    case VL53L0X::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI:
    case VL53L0X::OSC_CALIBRATE_VAL:
      return 2;
    case VL53L0X::SYSTEM_INTERMEASUREMENT_PERIOD:
      return 4;
    default:
      return 1;
  }
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/*
* Constructor.
*/
VL53L0X::VL53L0X(uint8_t addr) : I2CDevice(addr), io_timeout(0), did_timeout(false) {}


/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

void VL53L0X::setAddress(uint8_t new_addr) {
  // NOTE: This will cause this driver instance to be orphaned, as it won't change
  //   its own address. The calling function will need to tear down the driver
  //   and recreate it if it wants to keep using the hardware on this runtime.
  _write_register(I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool VL53L0X::init(bool io_2v8) {
  // check model ID register (value specified in datasheet)
  if (_read_registers(IDENTIFICATION_MODEL_ID) != 0xEE) { return false; }

  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8) {
    _write_register(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      _read_registers(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  _write_register(0x88, 0x00);

  _write_register(0x80, 0x01);
  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x00);
  stop_variable = _read_registers(0x91);
  _write_register(0x00, 0x01);
  _write_register(0xFF, 0x00);
  _write_register(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  _write_register(MSRC_CONFIG_CONTROL, _read_registers(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(0.25);

  _write_register(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  _read_buffer(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  _write_register(0xFF, 0x01);
  _write_register(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  _write_register(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  _write_register(0xFF, 0x00);
  _write_register(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++) {
    if (i < first_spad_to_enable || spads_enabled == spad_count) {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
      spads_enabled++;
    }
  }

  _write_buffer(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x00);

  _write_register(0xFF, 0x00);
  _write_register(0x09, 0x00);
  _write_register(0x10, 0x00);
  _write_register(0x11, 0x00);

  _write_register(0x24, 0x01);
  _write_register(0x25, 0xFF);
  _write_register(0x75, 0x00);

  _write_register(0xFF, 0x01);
  _write_register(0x4E, 0x2C);
  _write_register(0x48, 0x00);
  _write_register(0x30, 0x20);

  _write_register(0xFF, 0x00);
  _write_register(0x30, 0x09);
  _write_register(0x54, 0x00);
  _write_register(0x31, 0x04);
  _write_register(0x32, 0x03);
  _write_register(0x40, 0x83);
  _write_register(0x46, 0x25);
  _write_register(0x60, 0x00);
  _write_register(0x27, 0x00);
  _write_register(0x50, 0x06);
  _write_register(0x51, 0x00);
  _write_register(0x52, 0x96);
  _write_register(0x56, 0x08);
  _write_register(0x57, 0x30);
  _write_register(0x61, 0x00);
  _write_register(0x62, 0x00);
  _write_register(0x64, 0x00);
  _write_register(0x65, 0x00);
  _write_register(0x66, 0xA0);

  _write_register(0xFF, 0x01);
  _write_register(0x22, 0x32);
  _write_register(0x47, 0x14);
  _write_register(0x49, 0xFF);
  _write_register(0x4A, 0x00);

  _write_register(0xFF, 0x00);
  _write_register(0x7A, 0x0A);
  _write_register(0x7B, 0x00);
  _write_register(0x78, 0x21);

  _write_register(0xFF, 0x01);
  _write_register(0x23, 0x34);
  _write_register(0x42, 0x00);
  _write_register(0x44, 0xFF);
  _write_register(0x45, 0x26);
  _write_register(0x46, 0x05);
  _write_register(0x40, 0x40);
  _write_register(0x0E, 0x06);
  _write_register(0x20, 0x1A);
  _write_register(0x43, 0x40);

  _write_register(0xFF, 0x00);
  _write_register(0x34, 0x03);
  _write_register(0x35, 0x44);

  _write_register(0xFF, 0x01);
  _write_register(0x31, 0x04);
  _write_register(0x4B, 0x09);
  _write_register(0x4C, 0x05);
  _write_register(0x4D, 0x04);

  _write_register(0xFF, 0x00);
  _write_register(0x44, 0x00);
  _write_register(0x45, 0x20);
  _write_register(0x47, 0x08);
  _write_register(0x48, 0x28);
  _write_register(0x67, 0x00);
  _write_register(0x70, 0x04);
  _write_register(0x71, 0x01);
  _write_register(0x72, 0xFE);
  _write_register(0x76, 0x00);
  _write_register(0x77, 0x00);

  _write_register(0xFF, 0x01);
  _write_register(0x0D, 0x01);

  _write_register(0xFF, 0x00);
  _write_register(0x80, 0x01);
  _write_register(0x01, 0xF8);

  _write_register(0xFF, 0x01);
  _write_register(0x8E, 0x01);
  _write_register(0x00, 0x01);
  _write_register(0xFF, 0x00);
  _write_register(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  _write_register(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  _write_register(GPIO_HV_MUX_ACTIVE_HIGH, _read_registers(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  _write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  measurement_timing_budget_us = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  _write_register(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  _write_register(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  _write_register(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  _write_register(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool VL53L0X::setSignalRateLimit(float limit_Mcps) {
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  _write_register(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float VL53L0X::getSignalRateLimit() {
  return (float)_read_registers(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool VL53L0X::setMeasurementTimingBudget(uint32_t budget_us) {
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss) {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc) {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range) {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range) {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us) {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint32_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range) {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    _write_register(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t VL53L0X::getMeasurementTimingBudget() {
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc) {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss) {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc) {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range) {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range) {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool VL53L0X::setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks) {
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange) {
    // "Set phase check limits"
    switch (period_pclks) {
      case 12:
        _write_register(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        _write_register(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        _write_register(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        _write_register(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    _write_register(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    _write_register(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    _write_register(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    _write_register(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange) {
    switch (period_pclks) {
      case 8:
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        _write_register(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        _write_register(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        _write_register(0xFF, 0x01);
        _write_register(ALGO_PHASECAL_LIM, 0x30);
        _write_register(0xFF, 0x00);
        break;

      case 10:
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        _write_register(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        _write_register(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        _write_register(0xFF, 0x01);
        _write_register(ALGO_PHASECAL_LIM, 0x20);
        _write_register(0xFF, 0x00);
        break;

      case 12:
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        _write_register(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        _write_register(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        _write_register(0xFF, 0x01);
        _write_register(ALGO_PHASECAL_LIM, 0x20);
        _write_register(0xFF, 0x00);
        break;

      case 14:
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        _write_register(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        _write_register(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        _write_register(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        _write_register(0xFF, 0x01);
        _write_register(ALGO_PHASECAL_LIM, 0x20);
        _write_register(0xFF, 0x00);
        break;

      default:  // invalid period
        return false;
    }

    // apply new VCSEL period
    _write_register(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."
    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range) {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    _write_register(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"
  setMeasurementTimingBudget(measurement_timing_budget_us);
  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin
  uint8_t sequence_config = _read_registers(SYSTEM_SEQUENCE_CONFIG);
  _write_register(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  _write_register(SYSTEM_SEQUENCE_CONFIG, sequence_config);
  // VL53L0X_perform_phase_calibration() end
  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t VL53L0X::getVcselPulsePeriod(vcselPeriodType type) {
  if (type == VcselPeriodPreRange) {
    return decodeVcselPeriod(_read_registers(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange) {
    return decodeVcselPeriod(_read_registers(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  return 255;
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void VL53L0X::startContinuous(uint32_t period_ms) {
  _write_register(0x80, 0x01);
  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x00);
  _write_register(0x91, stop_variable);
  _write_register(0x00, 0x01);
  _write_register(0xFF, 0x00);
  _write_register(0x80, 0x00);

  if (period_ms != 0) {
    // continuous timed mode
    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
    uint16_t osc_calibrate_val = _read_registers(OSC_CALIBRATE_VAL);
    if (osc_calibrate_val != 0) {
      period_ms *= osc_calibrate_val;
    }
    _write_register(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    _write_register(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else {
    // continuous back-to-back mode
    _write_register(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void VL53L0X::stopContinuous() {
  _write_register(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x00);
  _write_register(0x91, 0x00);
  _write_register(0x00, 0x01);
  _write_register(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X::readRangeContinuousMillimeters() {
  startTimeout();
  while ((_read_registers(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpired()) {
      did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = _read_registers(RESULT_RANGE_STATUS + 10);
  _write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X::readRangeSingleMillimeters() {
  _write_register(0x80, 0x01);
  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x00);
  _write_register(0x91, stop_variable);
  _write_register(0x00, 0x01);
  _write_register(0xFF, 0x00);
  _write_register(0x80, 0x00);

  _write_register(SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout();
  while (_read_registers(SYSRANGE_START) & 0x01) {
    if (checkTimeoutExpired()) {
      did_timeout = true;
      return 65535;
    }
  }

  return readRangeContinuousMillimeters();
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool VL53L0X::timeoutOccurred() {
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool VL53L0X::getSpadInfo(uint8_t * count, bool * type_is_aperture) {
  uint8_t tmp;

  _write_register(0x80, 0x01);
  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x00);

  _write_register(0xFF, 0x06);
  _write_register(0x83, _read_registers(0x83) | 0x04);
  _write_register(0xFF, 0x07);
  _write_register(0x81, 0x01);

  _write_register(0x80, 0x01);

  _write_register(0x94, 0x6b);
  _write_register(0x83, 0x00);
  startTimeout();
  while (_read_registers(0x83) == 0x00) {
    if (checkTimeoutExpired()) { return false; }
  }
  _write_register(0x83, 0x01);
  tmp = _read_registers(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  _write_register(0x81, 0x00);
  _write_register(0xFF, 0x06);
  _write_register(0x83, _read_registers(0x83)  & ~0x04);
  _write_register(0xFF, 0x01);
  _write_register(0x00, 0x01);

  _write_register(0xFF, 0x00);
  _write_register(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void VL53L0X::getSequenceStepEnables(SequenceStepEnables * enables) {
  uint8_t sequence_config = _read_registers(SYSTEM_SEQUENCE_CONFIG);
  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void VL53L0X::getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts) {
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = _read_registers(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(_read_registers(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(_read_registers(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range) {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t VL53L0X::decodeTimeout(uint16_t reg_val) {
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
uint16_t VL53L0X::encodeTimeout(uint32_t timeout_mclks) {
  // format: "(LSByte * 2^MSByte) + 1"
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0) {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0) {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  return 0;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t VL53L0X::timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t VL53L0X::timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte) {
  _write_register(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  while ((_read_registers(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (checkTimeoutExpired()) { return false; }
  }

  _write_register(SYSTEM_INTERRUPT_CLEAR, 0x01);
  _write_register(SYSRANGE_START, 0x00);
  return true;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t VL53L0X::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t VL53L0X::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf     = op->buffer();
    uint     len     = op->bufferLen();
    uint8_t  r       = op->sub_addr;
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        switch (r) {
          default:   // All other registers are read-only.
            break;
        }
        break;

      case BusOpcode::RX:
        switch (r) {
          default:
            break;
        }
        break;

      default:
        break;
    }
  }
  return ret;
}



/*******************************************************************************
* Register access machinary
*******************************************************************************/

//TODO: This.
uint8_t* VL53L0X::_get_reg_ptr(uint8_t reg_addr) {
  return &shadows[0];
}

/**
* NOTE: Does not check for register writability.
*/
int8_t  VL53L0X::_set_shadow_value(uint8_t r, uint32_t val) {
  switch (r) {
    default:
      return -1;
  }
  uint8_t* ptr = _get_reg_ptr(r);   // Get register pointer.
  switch (_get_reg_len(r)) {  // Check register length and set shadow value.
    case 1:
      *(ptr + 0) = (uint8_t) (val & 0x000000FF);
      break;
    case 2:
      *(ptr + 0) = (uint8_t) ((val >> 8) & 0x000000FF);
      *(ptr + 1) = (uint8_t) (val & 0x000000FF);
      break;
    case 4:
      *(ptr + 0) = (uint8_t) ((val >> 24) & 0x000000FF);
      *(ptr + 1) = (uint8_t) ((val >> 16) & 0x000000FF);
      *(ptr + 2) = (uint8_t) ((val >> 8)  & 0x000000FF);
      *(ptr + 3) = (uint8_t) (val & 0x000000FF);
      break;
    default:
      return -2;
  }
  return 0;
}

/**
*
*/
uint VL53L0X::_get_shadow_value(uint8_t r) {
  uint ret = 0;
  switch (r) {
      break;
    default:
      // Illegal. A bad mistake was made somewhere.
      break;
  }
  return ret;
}

/**
*
*/
int8_t VL53L0X::_write_register(uint8_t r, uint32_t val) {
  int8_t ret = -1;
  uint8_t addr = r;   // TODO: Indirect register address from enum.
  // TODO: Check register writable.
  // TODO: Error-check register and value.
  uint8_t len  = _get_reg_len(r);
  uint8_t* ptr = _get_reg_ptr(r);   // Get register pointer.
  _set_shadow_value(r, (uint8_t) val);  // Set shadow value.
  ret = _write_buffer(r, ptr, len);
  return ret;
}

/**
*
*/
int8_t VL53L0X::_read_registers(uint8_t r, uint8_t reg_count) {
  int8_t ret = -1;
  if (nullptr != _bus) {
    ret--;
    if (true) {   // TODO: Are registers valid and contiguous?
      uint8_t addr = r;   // TODO: Indirect register address from enum.
      uint8_t* ptr = _get_reg_ptr(r);
      uint8_t len  = _get_reg_len(r);
      ret = _read_buffer(addr, ptr, len);
    }
  }
  return ret;
}

/**
*
*/
int8_t VL53L0X::_write_buffer(uint8_t addr, uint8_t* buf, uint8_t len) {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      ret--;
      op->dev_addr = _dev_addr;
      op->sub_addr = addr;
      op->setBuffer(buf, len);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}

/**
*
*/
int8_t VL53L0X::_read_buffer(uint8_t addr, uint8_t* buf, uint8_t len) {
  int8_t ret = -1;
  if (nullptr != _bus) {
    ret--;
    I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
    if (nullptr != op) {
      ret--;
      op->dev_addr = _dev_addr;
      op->sub_addr = addr;
      op->setBuffer(buf, len);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}
