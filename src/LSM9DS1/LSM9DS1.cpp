/*
File:   LSM9DS1.cpp
Author: J. Ian Lindsay
Date:   2014.03.27

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

#include "LSM9DS1.h"


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

/* The addresses of the registers named in the enum class LSM9DS1RegID. */
static const uint8_t LSM9DS1_ADDR_MAP[] = {
  0x05, 0x07, 0x09,  // M: 16-bit offset registers
  0x0f, 0x20,
  0x21, 0x22, 0x23, 0x24, 0x27,
  0x28, 0x2a, 0x2c,  // M: 16-bit data registers
  0x30, 0x31,
  0x32,              // M: 16-bit threshold register
  // This is where the AG registers start.
  0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b,
  0x0c, 0x0d, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x14,
  0x15,              // I: 16-bit temperature register (11-bit)
  0x17,
  0x18, 0x1a, 0x1c,  // G: 16-bit gyro data registers
  0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24,
  0x26, 0x27,
  0x28, 0x2a, 0x2c,  // A: 16-bit acc data registers
  0x2e, 0x2f, 0x30,
  0x31, 0x33, 0x35,  // G: 16-bit threshold registers
  0x37
};

/* The addresses of the registers named in the enum class LSM9DS1RegID. */
static const uint8_t LSM9DS1_SHADOW_OFFSETS[] = {
   0,  2,  4,  6,  7,  8,  9,  10, 11, 12, 13, 15, 17, 19, 20, 21,
   23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
   39, 41, 42, 44, 46, 48, 50, 51, 52, 53, 54, 55, 56, 57, 58, 60,
   62, 64, 65, 66, 67, 69, 71, 72
};

/*
* The default values of the registers named in the enum class LSM9DS1RegID.
* Some of these values should be construed as being 16-bit, but the size of
*   these arrays must remain the same.
*/
const uint8_t LSM9DS1::_imu_reg_defaults[] = {
  0x00, 0x00, 0x00,  // M: 16-bit offset registers
  0x3d, 0x40,
  0x00, 0x03, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,  // M: 16-bit data registers
  0x08, 0x00,
  0x00,              // M: 16-bit threshold register
  // This is where the AG registers start.
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00,              // I: 16-bit temperature register (11-bit)
  0x00,
  0x00, 0x00, 0x00,  // G: 16-bit gyro data registers
  0x38, 0x38, 0x00, 0x00, 0x04, 0x00, 0x00,
  0x00, 0x00,
  0x00, 0x00, 0x00,  // A: 16-bit acc data registers
  0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,  // G: 16-bit threshold registers
  0x00
};


/* The widths of the registers named in the enum class LSM9DS1RegID. */
const uint8_t _imu_register_width_map[] = {
  2, 2, 2,  // M: 16-bit offset registers
  1, 1,
  1, 1, 1, 1, 1,
  2, 2, 2,  // M: 16-bit data registers
  1, 1,
  2,        // M: 16-bit threshold register
  // This is where the AG registers start.
  1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1,
  2,        // I: 16-bit temperature register (11-bit)
  1,
  2, 2, 2,  // G: 16-bit gyro data registers
  1, 1, 1, 1, 1, 1, 1,
  1, 1,
  2, 2, 2,  // A: 16-bit acc data registers
  1, 1, 1,
  2, 2, 2,  // G: 16-bit threshold registers
  1
};


/* The widths of the registers named in the enum class LSM9DS1RegID. */
const bool _imu_register_writable_map[] = {
  true,  true,  true,   // M: 16-bit offset registers
  false, true,
  true,  true,  true,  true,  false,
  false, false, false,  // M: 16-bit data registers
  true,  false,
  true,                 // M: 16-bit threshold register
  // This is where the AG registers start.
  true,  true,  true,  true,  true,  true,  true,  true,
  true,  true,  false, true,  true,  true,  true,  false,
  false,                // I: 16-bit temperature register (11-bit)
  false,
  false, false, false,  // G: 16-bit gyro data registers
  true,  true,  true,  true,  true,  true,  true,
  false, false,
  false, false, false,  // A: 16-bit acc data registers
  true,  false, true,
  true,  true,  true,   // G: 16-bit threshold registers
  true
};


/* The string representations of the registers named in the enum class LSM9DS1RegID. */
const char* _imu_register_names[] = {
  "M_OFFSET_X", "M_OFFSET_Y", "M_OFFSET_Z",
  "M_WHO_AM_I", "M_CTRL_REG1",
  "M_CTRL_REG2", "M_CTRL_REG3", "M_CTRL_REG4", "M_CTRL_REG5", "M_STATUS_REG",
  "M_DATA_X", "M_DATA_Y", "M_DATA_Z",
  "M_INT_CFG", "M_INT_SRC",
  "M_INT_TSH",
  // This is where the AG registers start.
  "AG_ACT_THS", "AG_ACT_DUR", "A_INT_GEN_CFG", "A_INT_GEN_THS_X",
  "A_INT_GEN_THS_Y", "A_INT_GEN_THS_Z", "A_INT_GEN_DURATION", "G_REFERENCE",
  "AG_INT1_CTRL", "AG_INT2_CTRL", "AG_WHO_AM_I", "G_CTRL_REG1",
  "G_CTRL_REG2", "G_CTRL_REG3", "G_ORIENT_CFG", "G_INT_GEN_SRC",
  "AG_DATA_TEMP",
  "AG_STATUS_REG",
  "G_DATA_X", "G_DATA_Y", "G_DATA_Z",
  "AG_CTRL_REG4", "A_CTRL_REG5", "A_CTRL_REG6", "A_CTRL_REG7",
  "AG_CTRL_REG8", "AG_CTRL_REG9", "AG_CTRL_REG10",
  "A_INT_GEN_SRC", "AG_STATUS_REG_ALT",
  "A_DATA_X", "A_DATA_Y", "A_DATA_Z",
  "AG_FIFO_CTRL", "AG_FIFO_SRC", "G_INT_GEN_CFG",
  "G_INT_GEN_THS_X", "G_INT_GEN_THS_Y", "G_INT_GEN_THS_Z",
  "G_INT_GEN_DURATION"
};


// TODO: Incurring two branches might be cheaper than a const map. Investigate.
LSM9DS1RegID LSM9DS1::_reg_id_from_addr_mag(const uint8_t reg_addr) {
  switch (reg_addr & 0x3F) {
    case 0x05: return LSM9DS1RegID::M_OFFSET_X;
    case 0x07: return LSM9DS1RegID::M_OFFSET_Y;
    case 0x09: return LSM9DS1RegID::M_OFFSET_Z;
    case 0x0f: return LSM9DS1RegID::M_WHO_AM_I;
    case 0x20: return LSM9DS1RegID::M_CTRL_REG1;
    case 0x21: return LSM9DS1RegID::M_CTRL_REG2;
    case 0x22: return LSM9DS1RegID::M_CTRL_REG3;
    case 0x23: return LSM9DS1RegID::M_CTRL_REG4;
    case 0x24: return LSM9DS1RegID::M_CTRL_REG5;
    case 0x27: return LSM9DS1RegID::M_STATUS_REG;
    case 0x28: return LSM9DS1RegID::M_DATA_X;
    case 0x2a: return LSM9DS1RegID::M_DATA_Y;
    case 0x2c: return LSM9DS1RegID::M_DATA_Z;
    case 0x30: return LSM9DS1RegID::M_INT_CFG;
    case 0x31: return LSM9DS1RegID::M_INT_SRC;
    case 0x32: return LSM9DS1RegID::M_INT_TSH;
  }
  return LSM9DS1RegID::INVALID;
}


// TODO: Incurring two branches might be cheaper than a const map. Investigate.
LSM9DS1RegID LSM9DS1::_reg_id_from_addr_imu(const uint8_t reg_addr) {
  switch (reg_addr & 0x7F) {
    case 0x04: return LSM9DS1RegID::AG_ACT_THS;
    case 0x05: return LSM9DS1RegID::AG_ACT_DUR;
    case 0x06: return LSM9DS1RegID::A_INT_GEN_CFG;
    case 0x07: return LSM9DS1RegID::A_INT_GEN_THS_X;
    case 0x08: return LSM9DS1RegID::A_INT_GEN_THS_Y;
    case 0x09: return LSM9DS1RegID::A_INT_GEN_THS_Z;
    case 0x0a: return LSM9DS1RegID::A_INT_GEN_DURATION;
    case 0x0b: return LSM9DS1RegID::G_REFERENCE;
    case 0x0c: return LSM9DS1RegID::AG_INT1_CTRL;
    case 0x0d: return LSM9DS1RegID::AG_INT2_CTRL;
    case 0x0f: return LSM9DS1RegID::AG_WHO_AM_I;
    case 0x10: return LSM9DS1RegID::G_CTRL_REG1;
    case 0x11: return LSM9DS1RegID::G_CTRL_REG2;
    case 0x12: return LSM9DS1RegID::G_CTRL_REG3;
    case 0x13: return LSM9DS1RegID::G_ORIENT_CFG;
    case 0x14: return LSM9DS1RegID::G_INT_GEN_SRC;
    case 0x15: return LSM9DS1RegID::AG_DATA_TEMP;
    case 0x17: return LSM9DS1RegID::AG_STATUS_REG;
    case 0x18: return LSM9DS1RegID::G_DATA_X;
    case 0x1a: return LSM9DS1RegID::G_DATA_Y;
    case 0x1c: return LSM9DS1RegID::G_DATA_Z;
    case 0x1e: return LSM9DS1RegID::AG_CTRL_REG4;
    case 0x1f: return LSM9DS1RegID::A_CTRL_REG5;
    case 0x20: return LSM9DS1RegID::A_CTRL_REG6;
    case 0x21: return LSM9DS1RegID::A_CTRL_REG7;
    case 0x22: return LSM9DS1RegID::AG_CTRL_REG8;
    case 0x23: return LSM9DS1RegID::AG_CTRL_REG9;
    case 0x24: return LSM9DS1RegID::AG_CTRL_REG10;
    case 0x26: return LSM9DS1RegID::A_INT_GEN_SRC;
    case 0x27: return LSM9DS1RegID::AG_STATUS_REG_ALT;
    case 0x28: return LSM9DS1RegID::A_DATA_X;
    case 0x2a: return LSM9DS1RegID::A_DATA_Y;
    case 0x2c: return LSM9DS1RegID::A_DATA_Z;
    case 0x2e: return LSM9DS1RegID::AG_FIFO_CTRL;
    case 0x2f: return LSM9DS1RegID::AG_FIFO_SRC;
    case 0x30: return LSM9DS1RegID::G_INT_GEN_CFG;
    case 0x31: return LSM9DS1RegID::G_INT_GEN_THS_X;
    case 0x33: return LSM9DS1RegID::G_INT_GEN_THS_Y;
    case 0x35: return LSM9DS1RegID::G_INT_GEN_THS_Z;
    case 0x37: return LSM9DS1RegID::G_INT_GEN_DURATION;
  }
  return LSM9DS1RegID::AG_DATA_TEMP;  // Obviously wrong, but will fail a write operation.
}


/**
* Print the IMU register name.
*
* @return const char*
*/
const char* LSM9DS1::_reg_name_str(const LSM9DS1RegID id) {
  return _imu_register_names[(const uint8_t) id];
}

const uint8_t LSM9DS1::_reg_addr(const LSM9DS1RegID id) {
  return LSM9DS1_ADDR_MAP[(const uint8_t) id];
}

const uint8_t LSM9DS1::_get_shadow_offset(const LSM9DS1RegID id) {
  return LSM9DS1_SHADOW_OFFSETS[(const uint8_t) id];
}

const uint8_t LSM9DS1::_reg_width(const LSM9DS1RegID id) {
  return _imu_register_width_map[(const uint8_t) id];
}

const bool LSM9DS1::_reg_writable(const LSM9DS1RegID id) {
  return _imu_register_writable_map[(const uint8_t) id];
}


const bool LSM9DS1::_reg_is_for_mag(const LSM9DS1RegID id) {
  switch (id) {
    case LSM9DS1RegID::M_INT_TSH:
    case LSM9DS1RegID::M_OFFSET_X:
    case LSM9DS1RegID::M_OFFSET_Y:
    case LSM9DS1RegID::M_OFFSET_Z:
    case LSM9DS1RegID::M_WHO_AM_I:
    case LSM9DS1RegID::M_CTRL_REG1:
    case LSM9DS1RegID::M_CTRL_REG2:
    case LSM9DS1RegID::M_CTRL_REG3:
    case LSM9DS1RegID::M_CTRL_REG4:
    case LSM9DS1RegID::M_CTRL_REG5:
    case LSM9DS1RegID::M_STATUS_REG:
    case LSM9DS1RegID::M_DATA_X:
    case LSM9DS1RegID::M_DATA_Y:
    case LSM9DS1RegID::M_DATA_Z:
    case LSM9DS1RegID::M_INT_CFG:
    case LSM9DS1RegID::M_INT_SRC:
      return true;
    default:
      break;
  }
  return false;
}


///*
//* Given an indexed register definition and a length, return true if the
//*   corresponding address range is continuous.
//*/
//static bool _reg_range_continuous(LSM9DS1RegID r, uint8_t len) {
//  const uint8_t r_int = (uint8_t) r;
//  if ((r_int + len) < sizeof(LSM9DS1_ADDR_MAP)) {  // Valid length?
//    if (len > 1) {
//      uint8_t r_last = LSM9DS1_ADDR_MAP[r_int];
//      for (uint8_t i = 1; i < len; i++) {
//        uint8_t r_this = r_int + i;
//        if (LSM9DS1_ADDR_MAP[r_this] != (r_last + 1)) {  // Check for discontinuity.
//          return false;
//        }
//        r_last = LSM9DS1_ADDR_MAP[r_this];
//      }
//    }
//  }
//  return (len != 0);
//}


static const IMUState _state_indicies[] = {
  IMUState::STAGE_0,   // Undiscovered. Maybe absent.
  IMUState::STAGE_1,   // Discovered, but not init'd.
  IMUState::STAGE_2,   // Discovered and initiallized, but unknown register values.
  IMUState::STAGE_3,   // Fully initialized and sync'd. Un-calibrated.
  IMUState::STAGE_4,   // Calibrated and idle.
  IMUState::STAGE_5,   // Calibrated and reading.
  IMUState::FAULT,     // Fault.
  IMUState::UNDEF      // Not a state-machine value. A return code to simplifiy error-checks.
};


/*
* These are tables of frequencies versus periods (in micros). Lookup is faster
*   than calculation, typically.
*/
const UpdateRate2Hertz LSM9DS1::rate_settings_i[MAXIMUM_RATE_INDEX_AG] = {
  {0.0,  0.0f},
  {14.9, (1/14.9f)},
  {59.5, (1/59.5f)},
  {119,  (1/119.0f)},
  {238,  (1/238.0f)},
  {476,  (1/476.0f)},
  {952,  (1/952.0f)}
};


const UpdateRate2Hertz LSM9DS1::rate_settings_m[MAXIMUM_RATE_INDEX_MAG] = {
  {0.625, (1/0.625f)},
  {1.25,  (1/1.25f)},
  {2.5,   (1/2.5f)},
  {5.0,   (1/5.0f)},
  {10.0,  (1/10.0f)},
  {20.0,  (1/20.0f)},
  {40.0,  (1/40.0f)},
  {80.0,  (1/80.0f)}
};


/*
* These are generic table of scales versus unit-per-bit for 16-bit types.
* TODO: These need to be validated against actual datasheet values.
*/
const GainErrorMap LSM9DS1::error_map_acc[MAXIMUM_GAIN_INDEX_ACC] = {
  {2,  (2/32768.0f),  0.000030},
  {4,  (4/32768.0f),  0.000061},
  {6,  (6/32768.0f),  0.000092},
  {8,  (8/32768.0f),  0.000122},
  {16, (16/32768.0f), 0.000244}
};

const GainErrorMap LSM9DS1::error_map_gyr[MAXIMUM_GAIN_INDEX_GYR] = {
  {245,   (245/32768.0f),  0.00437 * 0.0174532777778},
  {500,   (500/32768.0f),  0.00875 * 0.0174532777778},
  {2000,  (2000/32768.0f), 0.03500 * 0.0174532777778}
};

const GainErrorMap LSM9DS1::error_map_mag[MAXIMUM_GAIN_INDEX_MAG] = {
  {4,  (4/32768.0f),  0.000061},
  {8,  (8/32768.0f),  0.000122},
  {12, (12/32768.0f), 0.00032},
  {16, (16/32768.0f), 0.000244}
};


const float LSM9DS1::max_range_vect_acc  = 16.0;
const float LSM9DS1::max_range_vect_gyr  = 2000.0;
const float LSM9DS1::max_range_vect_mag  = 16.0;


/**
* Return an enumerator given the state index.
*
* @return enum State
*/
IMUState LSM9DS1::getStateByIndex(uint8_t state_idx) {
  if (state_idx < sizeof(_state_indicies)) {
    return _state_indicies[state_idx];
  }
  return IMUState::UNDEF;
}


/**
* Print a human-readable representation of the IMU fault condition.
*
* @return const char*
*/
const char* LSM9DS1::getErrorString(IMUFault fault_code) {
  switch (fault_code) {
    case IMUFault::NO_ERROR               :  return "NO_ERROR";
    case IMUFault::WRONG_IDENTITY         :  return "WRONG_IDENTITY";
    case IMUFault::INVALID_PARAM          :  return "INVALID_PARAM";
    case IMUFault::NOT_CALIBRATED         :  return "NOT_CALIBRATED";
    case IMUFault::NOT_WRITABLE           :  return "NOT_WRITABLE";
    case IMUFault::DATA_EXHAUSTED         :  return "DATA_EXHAUSTED";
    case IMUFault::NOT_INITIALIZED        :  return "NOT_INITIALIZED";
    case IMUFault::BUS_INSERTION_FAILED   :  return "INSERTION_FAILED";
    case IMUFault::BUS_OPERATION_FAILED_R :  return "OPERATION_FAILED_R";
    case IMUFault::BUS_OPERATION_FAILED_W :  return "OPERATION_FAILED_W";
  }
  return "<UNKNOWN>";
}


/**
* Print a human-readable representation of the IMU state.
*
* @return const char*
*/
const char* LSM9DS1::getStateString(IMUState state) {
  switch (state) {
    case IMUState::UNDEF:    return "UNDEF";
    case IMUState::FAULT:    return "FAULT";
    case IMUState::STAGE_0:  return "STAGE_0";
    case IMUState::STAGE_1:  return "STAGE_1";
    case IMUState::STAGE_2:  return "STAGE_2";
    case IMUState::STAGE_3:  return "STAGE_3";
    case IMUState::STAGE_4:  return "STAGE_4";
    case IMUState::STAGE_5:  return "STAGE_5";
  }
  return "<UNKNOWN>";
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
* Constructor for i2c-specific driver.
*/
LSM9DS1_I2C::LSM9DS1_I2C(uint8_t a_imu, uint8_t a_mag, uint8_t irq0, uint8_t irq1, uint8_t irq2, uint8_t irq3)
  : LSM9DS1(a_imu, a_mag, irq0, irq1, irq2, irq3), I2CDevice(a_imu) {   // TODO: Sketchy...
  _fifo_read.set_opcode(BusOpcode::RX);
  _fifo_read.shouldReap(false);
  _fifo_read.callback = this;
  _fifo_read.dev_addr = _ADDR_IMU;
  _fifo_read.sub_addr = _reg_addr(LSM9DS1RegID::A_DATA_X) | 0x80;
  _fifo_read.setBuffer(&shadows[_get_shadow_offset(LSM9DS1RegID::A_DATA_X)], 10);
}


/*
* Constructor for general driver.
*/
LSM9DS1::LSM9DS1(uint8_t a_imu, uint8_t a_mag, uint8_t irq0, uint8_t irq1, uint8_t irq2, uint8_t irq3)
  : _ADDR_IMU(a_imu), _ADDR_MAG(a_mag), _IRQ_0_PIN(irq0), _IRQ_1_PIN(irq1), _IRQ_2_PIN(irq2), _IRQ_3_PIN(irq3)
{
  for (uint8_t i = 0; i < sizeof(shadows); i++) {
    shadows[i] = 0;
  }
}



IMUFault LSM9DS1::lastRead(IMUSense sense, float* x, float* y, float* z) {
  IMUFault ret = IMUFault::NO_ERROR;
  switch (sense) {
    case IMUSense::ACC:
      break;
    case IMUSense::GYR:
      break;
    case IMUSense::MAG:
      break;
    case IMUSense::THERM:
      break;
  }
  return ret;
}


int8_t LSM9DS1::pendingSamples(IMUSense sense) {
  int8_t ret = 0;
  switch (sense) {
    case IMUSense::ACC:
      break;
    case IMUSense::GYR:
      break;
    case IMUSense::MAG:
      break;
    case IMUSense::THERM:
      break;
  }
  return ret;
}


/**
* Called to init the common boilerplate for this sensor.
*
* @return  IMUFault::NO_ERROR or appropriate failure code.
*/
IMUFault LSM9DS1::init() {
  IMUFault ret = IMUFault::BUS_OPERATION_FAILED_R;
  _ll_pin_init();

  if (devFound()) {
    // Setup our default config.
    _set_shadow_value(LSM9DS1RegID::AG_CTRL_REG8, 0x01);
    _set_shadow_value(LSM9DS1RegID::M_CTRL_REG2, 0x04);
    _write_registers(LSM9DS1RegID::AG_CTRL_REG8, 1);
    if (0 == _configure_sensor()) {
      ret = IMUFault::NO_ERROR;
    }
  }
  else {
    for (uint8_t i = 0; i < sizeof(shadows); i++) {
      shadows[i] = 0;
    }
    if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::M_WHO_AM_I, 1)) {
      if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::AG_WHO_AM_I, 1)) {
        ret = IMUFault::NO_ERROR;
      }
    }
  }
  return ret;
}


/**
* Calling this function will reset the common class elements to their
*   default states.
*/
void LSM9DS1::reset() {
  scale_mag           = 0;
  scale_acc           = 0;
  scale_gyr           = 0;
  update_rate_m       = 0;
  update_rate_i       = 0;
  discards_remain_m   = 0;
  discards_total_m    = 0;
  discards_remain_i   = 0;
  discards_total_i    = 0;

  // Force our states back to reset.
  imu_state          = IMUState::STAGE_0;
  desired_state      = IMUState::STAGE_3;
  error_condition    = IMUFault::NO_ERROR;
  for (uint8_t i = 0; i < sizeof(shadows); i++) {
    shadows[i] = 0;  // mark_it_zero();
  }
  _set_shadow_value(LSM9DS1RegID::AG_CTRL_REG8, 0x01);
  _set_shadow_value(LSM9DS1RegID::M_CTRL_REG2, 0x04);
  _write_registers(LSM9DS1RegID::AG_CTRL_REG8, 1);
  _write_registers(LSM9DS1RegID::M_CTRL_REG2, 1);
}


/**
* This fxn refreshes all shadows except for the WHO_AM_I registers.
*   default states.
*/
int8_t LSM9DS1::refresh() {
  int8_t ret = -1;
  if (devFound()) {
    ret--;
    if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::AG_ACT_THS, 10)) {
      ret--;
      if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::G_CTRL_REG1, 21)) {
        ret--;
        if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::AG_STATUS_REG_ALT, 18)) {
          ret--;
          if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::M_OFFSET_X, 6)) {
            ret--;
            if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::M_CTRL_REG1, 5)) {
              ret--;
              if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::M_DATA_X, 7)) {
                ret--;
                if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::M_INT_CFG, 4)) {
                  ret = 0;
                }
              }
            }
          }
        }
      }
    }
  }
  else if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::M_WHO_AM_I, 1)) {
    if (IMUFault::NO_ERROR == _read_registers(LSM9DS1RegID::AG_WHO_AM_I, 1)) {
      _class_set_flag(LSM9DS1_FLAG_READING_ID);
      ret = 0;
    }
  }
  return ret;
}


/**
* Calling this will cause us to generate two random bytes and write them to two
*   separate registers chosen by the extending class. Those writes (if successful)
*   should cause an automatic re-read of those same registers to check that the bytes
*   were written.
*/
void LSM9DS1::write_test_bytes() {
  io_test_val_0 = (uint8_t) randomUInt32();
  io_test_val_1 = (uint8_t) randomUInt32();
  _set_shadow_value(LSM9DS1RegID::G_INT_GEN_THS_Y, io_test_val_0);
  _set_shadow_value(LSM9DS1RegID::M_OFFSET_Z, io_test_val_1);
  _write_registers(LSM9DS1RegID::G_INT_GEN_THS_Y, 1);
  _write_registers(LSM9DS1RegID::M_OFFSET_Z, 1);
}


/*
* For now:
* 0: Off  (Implies other register activity)
* 1: Lowest rate while still collecting data.
* 2: Low-accuracy rate. ~100Hz from each FIFO'd sensor.
* 3: Moderate rate.
* 4: Highest rate supported.
*/
void LSM9DS1::setSampleRateProfile(uint8_t profile_idx) {
  switch (profile_idx) {
    case 0:
      //delta_t = 0.0f;
      set_sample_rate_acc(0);
      set_sample_rate_gyr(0);
      set_sample_rate_mag(0);
      break;
    case 1:
      //delta_t = 0.0f;
      set_sample_rate_acc(1);
      set_sample_rate_gyr(1);
      set_sample_rate_mag(1);
      break;
    case 2:
      //delta_t = 0.0f;
      set_sample_rate_acc(6);
      set_sample_rate_gyr(2);
      set_sample_rate_mag(5);
      break;
    case 3:
      //delta_t = 0.0f;
      set_sample_rate_acc(8);
      set_sample_rate_gyr(3);
      set_sample_rate_mag(5);
      break;
    case 4:
      //delta_t = 0.0f;
      set_sample_rate_acc(10);
      set_sample_rate_gyr(4);
      set_sample_rate_mag(6);
      break;
    default:
      break;
  }
}


/*
*
*/
bool LSM9DS1::is_setup_completed() {
  return (devFound() & initialized());
}


/**
* Our purpose here is to verify that our test value comes back on a read. This is a
*   bus-integrity test that must be passed prior to entering INIT-3.
*
* @return true if the test passes. False otherwise.
*/
bool LSM9DS1::integrity_check() {
  if (!devFound()) return false;

  // If we are ain a state where we are reading the init values back, look for our test
  // values, and fail the init if they are not found.
  if (io_test_val_0 == _get_shadow_value(LSM9DS1RegID::G_INT_GEN_THS_Y)) {
    if (io_test_val_1 == _get_shadow_value(LSM9DS1RegID::M_OFFSET_Z)) {
        // We will call this successful init.
        if (getVerbosity() > 5) {
          StringBuilder local_log;
          local_log.concat("Successful readback!");
          Kernel::log(&local_log);
        }
        // Rewrite valid values to those registers if necessary.
        //writeDirtyRegisters();
        _class_set_flag(IMU_COMMON_FLAG_HW_WRITABLE);
        return true;
    }
    else {
      if (getVerbosity() > 2) {
        StringBuilder local_log;
        local_log.concatf("Failed integrity check (M_OFFSET_Z). Found 0x%02x. Expected 0x%02x.\n", _get_shadow_value(LSM9DS1RegID::M_OFFSET_Z), io_test_val_1);
        Kernel::log(&local_log);
      }
    }
  }
  else {
    if (getVerbosity() > 2) {
      StringBuilder local_log;
      local_log.concatf("Failed integrity check (G_INT_GEN_THS_Y). Found 0x%02x. Expected 0x%02x.\n", _get_shadow_value(LSM9DS1RegID::G_INT_GEN_THS_Y), io_test_val_0);
      Kernel::log(&local_log);
    }
  }

  error_condition = IMUFault::NOT_WRITABLE;
  return false;
}



/**
* Call to set the desired IMU state. The class should then take whatever action is needed
*   to make reality match. This will be asynchronously done, so the caller will probably need
*   to be informed of the change after it has been completed, or can go no further.
*
* @param  uint8_t The new state desired by the firmware (the Legend).
* @return non-zero on error.
*/
IMUFault LSM9DS1::setDesiredState(IMUState nu) {
  if (devFound() && (nu < IMUState::STAGE_1)) {
    // If we already know the sensor is there, why go back further than this?
    Kernel::log("LSM9DS1::setDesiredState(): Trying to move to a state lower than allowed.\n");
    return IMUFault::INVALID_PARAM;
  }

  if (desired_state != nu) {
    if (!desired_state_attained()) {
      // TODO
      // The IMU is not at equilibrium. It may be ok to change the desired stage as long as we don't have
      //   bus operations pending.
      if (getVerbosity() > 2) {
        //local_log.concatf("%s tried to move to state %s while the IMU is off-balance (%s --> %s). Rejecting request.\n", imu_type(), getStateString(nu), getStateString(), getStateString(desired_state));
        StringBuilder local_log;
        local_log.concatf("Tried to move to state %s while the IMU is off-balance (%s --> %s). We will allow this for now.\n", getStateString(nu), getStateString(), getStateString(desired_state));
        Kernel::log(&local_log);
      }
      //return -2;
    }

    desired_state = nu;
    //step_state();
  }
  return IMUFault::NO_ERROR;
}


/**
* Assumes that the operation prior set the state to whatever is current.
* Formerly known as   "bool step_states();"
* TODO: The cyclomatic complexity of this fxn hurts my brain. Rework it.
*
* @return true if the state is stable, and the integrator should be notified.
*/
int8_t LSM9DS1::poll() {
  int8_t ret = 0;
  if (!desired_state_attained()) {
    if (IMUFault::NO_ERROR != error_condition) {
      // We shouldn't be changing states if there is an error condition.
      // Reset is the only way to exit the condition at present.
      if (getVerbosity() > 2) {
        StringBuilder local_log;
        local_log.concatf("Step_state() was called while we are in an error condition: %s\n",  getErrorString());
        Kernel::log(&local_log);
      }
      return 0;
    }

    switch (getState()) {
      case IMUState::STAGE_0:  // We think the IMU might be physicaly absent.
        if (devFound()) {
          set_state(IMUState::STAGE_1);
        }
        else {
          // We lost the IMU, perhaps...
          return 0;
        }
        // NOTE: no break.
      case IMUState::STAGE_1:  // We are sure the IMU is present, but we haven't done anything with it.
        //configure_sensor();
        break;

      case IMUState::STAGE_2:  // Discovered and initiallized, but unknown register values.
        if (is_setup_completed()) {
          refresh();
        }
        else {
          set_state(IMUState::STAGE_1);
          return 0;
        }
        break;

      case IMUState::STAGE_3:  // Fully initialized and sync'd. Un-calibrated.
        sb_next_write = 0;
        _read_fifo();
        // TODO: Stop skipping calibrate().
        break;

      case IMUState::STAGE_4:  // Calibrated and idle.
        if (desiredState() == IMUState::STAGE_5) {
          // Enable the chained reads, and start the process rolling.
          _read_fifo();
        }
        else {
          // Downgrading to init state 3 (recalibrate).
          set_state(IMUState::STAGE_3);
          sb_next_write = 0;
          //_read_fifo();  Should already be reading.
          return 0;
        }
        break;

      case IMUState::STAGE_5:  // Calibrated and reading.
        switch (desiredState()) {
          case IMUState::STAGE_4:   // Stop reads.
            set_state(IMUState::STAGE_4);
            return -3;   /// Note the slight break from convention... Careful...

          case IMUState::STAGE_3:  // Downgrading to init state 3 (recalibrate).
            set_state(IMUState::STAGE_3);
            sb_next_write = 0;
            //_read_fifo();  Should already be reading.
            return 0;
          case IMUState::STAGE_5:  // Keep reading.
            return 0;
          default:
            break;
        }
        break;

      default:
        break;
    }
    ret = -2;
  }
  return ret;
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
unsigned int LSM9DS1::_get_shadow_value(LSM9DS1RegID idx) {
  if (((uint8_t) idx) < sizeof(shadows)) {
    const uint8_t* ptr = &shadows[_get_shadow_offset(idx)];
    switch (_reg_width(idx)) {
      // These are the only two widths in the sensor.
      case 1: return *(ptr);
      case 2: return (((uint16_t) *(ptr)) << 8) + *(ptr+1);
    }
  }
  return 0;
}


/*
* Convenience fxn. Returns 0 if register index is out of bounds.
*/
int8_t LSM9DS1::_set_shadow_value(LSM9DS1RegID reg, unsigned int val) {
  if (((uint8_t) reg) < sizeof(shadows)) {
    uint8_t* ptr = &shadows[_get_shadow_offset(reg)];
    switch (_reg_width(reg)) {
      // These are the only two widths in the sensor.
      case 1:
        *(ptr) = 0xFF & val;
        return 0;
      case 2:
        *(ptr + 0) = 0xFF & val;
        *(ptr + 1) = 0xFF & (val >> 8);
        return 0;
      default:
        break;
    }
  }
  return -1;
}



/**
* Dump the contents of this device to the logger.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void LSM9DS1::printDebug(StringBuilder* output) {
  output->concatf("\n-------------------------------------------------------\n--- IMU  %s ==> %s \n-------------------------------------------------------\n", getStateString(imu_state), (desired_state_attained() ? "STABLE" : getStateString(desired_state)));
  output->concatf("--- pending_samples     %d\n\n", _get_shadow_value(LSM9DS1RegID::AG_FIFO_SRC) & 0x1F);
  if (getVerbosity() > 1) {
    output->concatf("--- calibration smpls   %d\n", sb_next_write);
    output->concatf("--- Base filter param   %d\n", base_filter_param);
  }
  output->concatf("--- Error condition     %s\n", getErrorString(error_condition));

  if (getVerbosity() > 1) {
    output->concatf("---\n--- update_rate_m       %3.0f Hz\n", (double) rate_settings_m[update_rate_m].hertz);
    output->concatf("--- update_rate_i       %3.0f Hz\n", (double) rate_settings_i[update_rate_i].hertz);
    if (getVerbosity() > 2) {
      output->concatf("--- scale_mag           +/-%d gauss\n", error_map_mag[scale_mag].scale);
      output->concatf("--- scale_acc           +/-%d m/s\n", error_map_acc[scale_acc].scale);
      output->concatf("--- scale_gyr           +/-%d deg/s\n", error_map_gyr[scale_gyr].scale);
      output->concatf("--- autoscale_mag       %s\n", (autoscale_mag() ? "yes" : "no"));
      output->concatf("--- autoscale_acc       %s\n", (autoscale_acc() ? "yes" : "no"));
      output->concatf("--- autoscale_gyr       %s\n", (autoscale_gyr() ? "yes" : "no"));
    }
  }

  output->concat("--\n-- Register           Addr  Value\n   -------------------------------\n");
  for (int i = 0; i < 56; i++) {
    const LSM9DS1RegID id = (const LSM9DS1RegID) i;
    switch (_reg_width(id)) {
      // These are the only two widths in the sensor.
      case 2: output->concatf("   %18s 0x%02x  0x%04x  0x%02x\n", _reg_name_str(id), _reg_addr(id), _get_shadow_value(id), _get_shadow_offset(id));
        break;
      case 1: output->concatf("   %18s 0x%02x  0x%02x    0x%02x\n", _reg_name_str(id), _reg_addr(id), _get_shadow_value(id), _get_shadow_offset(id));
        break;
    }
  }
}



/*******************************************************************************
*  __  __                        _                       _
* |  \/  |                      | |                     | |
* | \  / | __ _  __ _ _ __   ___| |_ ___  _ __ ___   ___| |_ ___ _ __
* | |\/| |/ _` |/ _` | '_ \ / _ \ __/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
* | |  | | (_| | (_| | | | |  __/ || (_) | | | | | |  __/ ||  __/ |
* |_|  |_|\__,_|\__, |_| |_|\___|\__\___/|_| |_| |_|\___|\__\___|_|
*                __/ |
*               |___/
*******************************************************************************/

/**
* Call to rescale the sensor.
*/
IMUFault LSM9DS1::request_rescale_mag(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_MAG) {
    if (scale_mag != nu_scale_idx) {
      //if (getVerbosity() > 2) Kernel::log("request_rescale_mag():\tRescaling magnetometer.\n");
      _set_shadow_value(LSM9DS1RegID::M_CTRL_REG2, (nu_scale_idx << 5));
      return _write_registers(LSM9DS1RegID::M_CTRL_REG2, 1);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM;
}


/**
* Call to alter sample rate.
*/
IMUFault LSM9DS1::set_sample_rate_mag(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_MAG) {
    if (update_rate_m != nu_srate_idx) {
      uint8_t temp8 = _get_shadow_value(LSM9DS1RegID::M_CTRL_REG1);
      if (0 == nu_srate_idx) {
        // Power the sensor down.
      }
      else {
        //if (getVerbosity() > 2) Kernel::log("set_sample_rate_mag():\tMagnetometer sample rate change.\n");
        temp8 =  (temp8 & ~0x1C) | ((nu_srate_idx-1) << 2);
      }
      update_rate_m = nu_srate_idx;
      _set_shadow_value(LSM9DS1RegID::M_CTRL_REG1, temp8);
      return _write_registers(LSM9DS1RegID::M_CTRL_REG1, 1);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM;
}



/*
* The purpose here is to figure out why one of our interrupt lines is going off.
* Because this sensor is so awesome, it has configurable interrupt sources that
*   are bindable to two separate pins.
* Since everything is async, we will process this task in stages. We need to
*   keep in mind that there will be two other devices sharing IRQ lines with
*   this one, and we need to give them a chance to read their registers before
*   their interrupts go stale.
// TODO: staggered priorities if separate bus access? Might help find the cause faster...

* Stage 0) Read all relevant registers. Return.
* Stage 1) Process register data and service interrupt if applicable in io_op_callback().
*
* Return codes:
*   Success, with IRQ service
*   Success, no IRQ service
*   Failure
*/
IMUFault LSM9DS1::irq_2() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_2()\n");
  return IMUFault::NO_ERROR;
}

IMUFault LSM9DS1::irq_1() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_1()\n");
  return IMUFault::NO_ERROR;
}

IMUFault LSM9DS1::irq_drdy() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_drdy()\n");
  return IMUFault::NO_ERROR;
}

IMUFault LSM9DS1::irq_m() {
  if (getVerbosity() > 3) Kernel::log("LSM9DS1::irq_m()\n");
  return IMUFault::NO_ERROR;
}



/*******************************************************************************
*                       _                               _
*     /\               | |                             | |
*    /  \   ___ ___ ___| | ___ _ __ ___  _ __ ___   ___| |_ ___ _ __
*   / /\ \ / __/ __/ _ \ |/ _ \ '__/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
*  / ____ \ (_| (_|  __/ |  __/ | | (_) | | | | | |  __/ ||  __/ |
* /_/    \_\___\___\___|_|\___|_|  \___/|_| |_| |_|\___|\__\___|_|
*
*******************************************************************************/


/**
* Call to rescale the sensor.
*/
IMUFault LSM9DS1::request_rescale_acc(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      uint8_t temp8 = _get_shadow_value(LSM9DS1RegID::A_CTRL_REG6);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      _set_shadow_value(LSM9DS1RegID::A_CTRL_REG6, temp8);
      return _write_registers(LSM9DS1RegID::A_CTRL_REG6, 1);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM;
}


/**
* Call to alter sample rate.
*/
IMUFault LSM9DS1::set_sample_rate_acc(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_i != nu_srate_idx) {
      uint8_t temp8 = _get_shadow_value(LSM9DS1RegID::A_CTRL_REG6);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      update_rate_i = nu_srate_idx;
      _set_shadow_value(LSM9DS1RegID::A_CTRL_REG6, temp8);
      return _write_registers(LSM9DS1RegID::A_CTRL_REG6, 1);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM;
}


// Call to change the bandwidth of the AA filter.
IMUFault LSM9DS1::set_base_filter_param_acc(uint8_t nu_bw_idx) {
  if ((nu_bw_idx < 4) && (nu_bw_idx != base_filter_param)) {
    base_filter_param = nu_bw_idx;
    uint8_t temp8 = _get_shadow_value(LSM9DS1RegID::A_CTRL_REG6);
    temp8 =  (temp8 & 0xFC) | (nu_bw_idx & 0x03);
    _set_shadow_value(LSM9DS1RegID::A_CTRL_REG6, temp8);
    return _write_registers(LSM9DS1RegID::A_CTRL_REG6, 1);
  }
  return IMUFault::INVALID_PARAM;
}


/*
* Convenience function that intiates a chain of register I/O to drain the FIFO.
*/
IMUFault LSM9DS1::_read_fifo() {
  return _read_registers(LSM9DS1RegID::AG_FIFO_SRC, 1);
}


/*
* Default interrupt arrangement...
*/
int8_t LSM9DS1::_configure_sensor() {
  int8_t ret = -1;
  uint8_t a_int_gen_val = 0;
  uint8_t int1_ctrl_val = 0;
  uint8_t int2_ctrl_val = 0;
  // If we have interrupt pins, we setup interrupt conditions.

  _set_shadow_value(LSM9DS1RegID::A_INT_GEN_CFG, a_int_gen_val);
  _set_shadow_value(LSM9DS1RegID::AG_INT1_CTRL, int1_ctrl_val);
  _set_shadow_value(LSM9DS1RegID::AG_INT2_CTRL, int2_ctrl_val);
  if (IMUFault::NO_ERROR == _write_registers(LSM9DS1RegID::A_INT_GEN_CFG, 1)) {

  }
  return ret;
}


/*******************************************************************************
*   _____
*  / ____|
* | |  __ _   _ _ __ ___
* | | |_ | | | | '__/ _ \
* | |__| | |_| | | | (_) |
*  \_____|\__, |_|  \___/
*          __/ |
*         |___/
*******************************************************************************/

/**
* Call to rescale the sensor.
*/
IMUFault LSM9DS1::request_rescale_gyr(uint8_t nu_scale_idx) {
  if (nu_scale_idx < MAXIMUM_GAIN_INDEX_ACC) {
    if (scale_acc != nu_scale_idx) {
      if (getVerbosity() > 2) Kernel::log("request_rescale_gyr():\tRescaling Gyro.\n");
      uint8_t temp8 = _get_shadow_value(LSM9DS1RegID::G_CTRL_REG1);
      temp8 =  (temp8 & 0xE7) | (nu_scale_idx << 3);
      _set_shadow_value(LSM9DS1RegID::G_CTRL_REG1, temp8);
      return _write_registers(LSM9DS1RegID::G_CTRL_REG1, 1);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM;
}


/**
* Call to alter sample rate.
*/
IMUFault LSM9DS1::set_sample_rate_gyr(uint8_t nu_srate_idx) {
  if (nu_srate_idx < MAXIMUM_RATE_INDEX_AG) {
    if (update_rate_i != nu_srate_idx) {
      if (getVerbosity() > 2) Kernel::log("set_sample_rate_gyr():\t\n");
      uint8_t temp8 = _get_shadow_value(LSM9DS1RegID::G_CTRL_REG1);
      temp8 =  (temp8 & 0x1F) | (nu_srate_idx << 5);
      //update_rate_gyr = nu_srate_idx;
      _set_shadow_value(LSM9DS1RegID::G_CTRL_REG1, temp8);
      return _write_registers(LSM9DS1RegID::G_CTRL_REG1, 1);
    }
    return IMUFault::NO_ERROR;
  }
  return IMUFault::INVALID_PARAM;
}



// Call to change the cutoff of the gyro's base filter.
IMUFault LSM9DS1::set_base_filter_param_gyr(uint8_t nu_bw_idx) {
  return IMUFault::INVALID_PARAM;
}




/*
* Setup the platform pins for interrupt signals.
*/
int8_t LSM9DS1::_ll_pin_init() {
  int8_t ret = -1;
  if (_class_flag(LSM9DS1_FLAG_PINS_CONFIGURED)) {
    ret = 0;
  }
  else {
    ret = 0; //_setup_bus_pin();
    if (255 != _IRQ_0_PIN) {
      pinMode(_IRQ_0_PIN, GPIOMode::INPUT);
      //detachInterrupt(_PINS.IRQ, imu_isr_0, FALLING);
    }
    if (255 != _IRQ_1_PIN) {
      pinMode(_IRQ_1_PIN, GPIOMode::INPUT);
      //detachInterrupt(_PINS.IRQ, imu_isr_1, FALLING);
    }
    if (255 != _IRQ_2_PIN) {
      pinMode(_IRQ_2_PIN, GPIOMode::INPUT);
      //detachInterrupt(_PINS.IRQ, imu_isr_2, FALLING);
    }
    if (255 != _IRQ_3_PIN) {
      pinMode(_IRQ_3_PIN, GPIOMode::INPUT);
      //detachInterrupt(_PINS.IRQ, imu_isr_3, FALLING);
    }

    if (0 == ret) {
      _class_set_flag(LSM9DS1_FLAG_PINS_CONFIGURED);
    }
  }
  return ret;
}


/*
* Private bus-abstracted register I/O callback for magnetometer.
*/
int8_t LSM9DS1::_io_op_callback_mag(const LSM9DS1RegID reg, BusOp* op) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  unsigned int value = _get_shadow_value(reg);
  StringBuilder local_log;
  if (getVerbosity() > 5) local_log.concatf("\t (%c) LSM9DS1RegID::%s: 0x%02x\n", (op->get_opcode() == BusOpcode::TX) ? "W":"R", _reg_name_str(reg), (uint8_t) value);
  switch (op->get_opcode()) {
    case BusOpcode::TX:
      switch (reg) {
        case LSM9DS1RegID::M_OFFSET_X:
        case LSM9DS1RegID::M_OFFSET_Y:
        case LSM9DS1RegID::M_OFFSET_Z:
          break;
        case LSM9DS1RegID::M_CTRL_REG1:
          if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_m = ((value >> 6) & 0x03)+1;
          break;
        case LSM9DS1RegID::M_CTRL_REG2:
          if (((value >> 4) & 0x03) < MAXIMUM_GAIN_INDEX_MAG)  scale_mag = (value >> 5) & 0x03;
          if (value & 0x04) { // Did we write here to reset?
            if (!devFound()) {
              //integrator->init();
            }
          }
          break;
        case LSM9DS1RegID::M_CTRL_REG3:
          power_to_mag(value & 0x02);
          break;
        case LSM9DS1RegID::M_CTRL_REG4:
        case LSM9DS1RegID::M_CTRL_REG5:
          break;
        case LSM9DS1RegID::M_INT_CFG:
        case LSM9DS1RegID::M_INT_TSH:  // TODO: Datasheet claims this is not writable. Suspect typo. Test.
        default:  // Anything else is invalid.
          break;
      }
      break;


    case BusOpcode::RX:
      switch (reg) {
        case LSM9DS1RegID::M_OFFSET_X:
        case LSM9DS1RegID::M_OFFSET_Y:
        case LSM9DS1RegID::M_OFFSET_Z:
          break;
        case LSM9DS1RegID::M_WHO_AM_I:
          if ((IMUState::STAGE_0 == getState()) && devFound()) {
            _class_clear_flag(LSM9DS1_FLAG_READING_ID);
            init();
          }
          break;
        case LSM9DS1RegID::M_CTRL_REG1:
          if (((value >> 2) & 0x07) < MAXIMUM_RATE_INDEX_MAG)  update_rate_m = ((value >> 6) & 0x03)+1;
          break;
        case LSM9DS1RegID::M_CTRL_REG2:
          if (((value >> 4) & 0x03) < MAXIMUM_GAIN_INDEX_MAG)  scale_mag = (value >> 5) & 0x03;
          break;
        case LSM9DS1RegID::M_CTRL_REG3:
          power_to_mag(value & 0x02);
          break;
        case LSM9DS1RegID::M_CTRL_REG4:
        case LSM9DS1RegID::M_CTRL_REG5:
        case LSM9DS1RegID::M_STATUS_REG:
          break;
        case LSM9DS1RegID::M_DATA_X:  // The data registers are always read in a 6-byte block.
        case LSM9DS1RegID::M_DATA_Y:  // The data registers are always read in a 6-byte block.
        case LSM9DS1RegID::M_DATA_Z:  // The data registers are always read in a 6-byte block.
          break;
        case LSM9DS1RegID::M_INT_CFG:
        case LSM9DS1RegID::M_INT_SRC:
        case LSM9DS1RegID::M_INT_TSH:
        default:  // Anything else is invalid.
          break;
      }
      break;

    default:
      break;
  }
  if (local_log.length() > 0) Kernel::log(&local_log);
  return ret;
}


/*
* Private bus-abstracted register I/O callback for IMU.
*/
int8_t LSM9DS1::_io_op_callback_imu(const LSM9DS1RegID reg, BusOp* op) {
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  unsigned int value = _get_shadow_value(reg);
  StringBuilder local_log;
  if (getVerbosity() > 5) local_log.concatf("\t (%c) LSM9DS1RegID::%s: 0x%02x\n", (op->get_opcode() == BusOpcode::TX) ? "W":"R", _reg_name_str(reg), (uint8_t) value);
  switch (op->get_opcode()) {
    case BusOpcode::TX:
      switch (reg) {
        case LSM9DS1RegID::AG_ACT_THS:
        case LSM9DS1RegID::AG_ACT_DUR:
        case LSM9DS1RegID::A_INT_GEN_CFG:
        case LSM9DS1RegID::A_INT_GEN_THS_X:
        case LSM9DS1RegID::A_INT_GEN_THS_Y:
        case LSM9DS1RegID::A_INT_GEN_THS_Z:
        case LSM9DS1RegID::A_INT_GEN_DURATION:
        case LSM9DS1RegID::G_REFERENCE:
        case LSM9DS1RegID::AG_INT1_CTRL:
        case LSM9DS1RegID::AG_INT2_CTRL:
        case LSM9DS1RegID::G_CTRL_REG1:
        case LSM9DS1RegID::G_CTRL_REG2:
        case LSM9DS1RegID::G_CTRL_REG3:
        case LSM9DS1RegID::G_ORIENT_CFG:
        case LSM9DS1RegID::AG_CTRL_REG4:
        case LSM9DS1RegID::A_CTRL_REG5:
        case LSM9DS1RegID::A_CTRL_REG6:
        case LSM9DS1RegID::A_CTRL_REG7:
        case LSM9DS1RegID::AG_CTRL_REG8:
        case LSM9DS1RegID::AG_CTRL_REG9:
        case LSM9DS1RegID::AG_CTRL_REG10:
        case LSM9DS1RegID::AG_FIFO_CTRL:
        case LSM9DS1RegID::G_INT_GEN_CFG:
        case LSM9DS1RegID::G_INT_GEN_THS_X:
        case LSM9DS1RegID::G_INT_GEN_THS_Y:
        case LSM9DS1RegID::G_INT_GEN_THS_Z:
        case LSM9DS1RegID::G_INT_GEN_DURATION:
        default:
          break;
      }
      break;


    case BusOpcode::RX:
      switch (reg) {
        case LSM9DS1RegID::AG_ACT_THS:
        case LSM9DS1RegID::AG_ACT_DUR:
        case LSM9DS1RegID::A_INT_GEN_CFG:
        case LSM9DS1RegID::A_INT_GEN_THS_X:
        case LSM9DS1RegID::A_INT_GEN_THS_Y:
        case LSM9DS1RegID::A_INT_GEN_THS_Z:
        case LSM9DS1RegID::A_INT_GEN_DURATION:
        case LSM9DS1RegID::G_REFERENCE:
        case LSM9DS1RegID::AG_INT1_CTRL:
        case LSM9DS1RegID::AG_INT2_CTRL:
          break;
        case LSM9DS1RegID::AG_WHO_AM_I:
          if ((IMUState::STAGE_0 == getState()) && devFound()) {
            _class_clear_flag(LSM9DS1_FLAG_READING_ID);
            init();
          }
          break;
        case LSM9DS1RegID::G_CTRL_REG1:
          if ((value >> 4) < MAXIMUM_RATE_INDEX_AG)  update_rate_i = (value >> 4) & 0x0F;
          break;
        case LSM9DS1RegID::G_CTRL_REG2:
        case LSM9DS1RegID::G_CTRL_REG3:
        case LSM9DS1RegID::G_ORIENT_CFG:
          break;
        case LSM9DS1RegID::G_INT_GEN_SRC:
          if (value & 0x01) {                 // An interrupt was seen because we crossed a threshold we set.
            if (value & 0xE0) {               // Did we exceed our set threshold?
              if (autoscale_gyr()) request_rescale_gyr(scale_gyr+1);
            }
            else if (value & 0x1C) {          // Did we drop below our set threshold?
              if (autoscale_gyr()) request_rescale_gyr(scale_gyr-1);
            }
          }
          else if (value & 0x02) {            // We had a range overflow. Means we need to autoscale...
            if (autoscale_gyr()) request_rescale_gyr(scale_gyr+1);
          }
          break;
        case LSM9DS1RegID::AG_DATA_TEMP:
          _temperature = 1.0 * (int16_t) _get_shadow_value(LSM9DS1RegID::AG_DATA_TEMP);
          break;
        case LSM9DS1RegID::AG_STATUS_REG:    /* Status of the gyr data registers on the sensor. */
          break;
        case LSM9DS1RegID::G_DATA_X:
        case LSM9DS1RegID::G_DATA_Y:
        case LSM9DS1RegID::G_DATA_Z:
          break;
        case LSM9DS1RegID::AG_CTRL_REG4:
          if (((value >> 3) & 0x07) < MAXIMUM_GAIN_INDEX_ACC)  scale_acc = (value >> 3) & 0x07;
          base_filter_param = (value >> 6) & 0x03;
          break;
        case LSM9DS1RegID::A_CTRL_REG5:
          break;
        case LSM9DS1RegID::A_CTRL_REG6:
          if (((value >> 3) & 0x03) < MAXIMUM_GAIN_INDEX_GYR)  scale_gyr = (value >> 3) & 0x03;
          break;
        case LSM9DS1RegID::A_CTRL_REG7:
        case LSM9DS1RegID::AG_CTRL_REG8:
        case LSM9DS1RegID::AG_CTRL_REG9:
        case LSM9DS1RegID::AG_CTRL_REG10:
        case LSM9DS1RegID::A_INT_GEN_SRC:
        case LSM9DS1RegID::AG_STATUS_REG_ALT:
          break;
        case LSM9DS1RegID::A_DATA_X:  // The data registers are always read in a 6-byte block.
        case LSM9DS1RegID::A_DATA_Y:  // The data registers are always read in a 6-byte block.
        case LSM9DS1RegID::A_DATA_Z:  // The data registers are always read in a 6-byte block.
          break;
        case LSM9DS1RegID::AG_FIFO_CTRL:
          break;
        case LSM9DS1RegID::AG_FIFO_SRC:
          _fifo_remaining = 0x1F & _get_shadow_value(LSM9DS1RegID::AG_FIFO_SRC);
          if (_fifo_remaining) {
            if (initialized()) {
              _read_registers(LSM9DS1RegID::G_DATA_X, 6);
              _read_registers(LSM9DS1RegID::A_DATA_X, 6);
            }
          }
          break;
        case LSM9DS1RegID::G_INT_GEN_CFG:
        case LSM9DS1RegID::G_INT_GEN_THS_X:
        case LSM9DS1RegID::G_INT_GEN_THS_Y:
        case LSM9DS1RegID::G_INT_GEN_THS_Z:
        case LSM9DS1RegID::G_INT_GEN_DURATION:
          break;
        default:
          break;
      }
      break;

    default:
      break;
  }
  if (local_log.length() > 0) Kernel::log(&local_log);
  return ret;
}



/*******************************************************************************
* I2C functions
*******************************************************************************/

IMUFault LSM9DS1_I2C::_write_registers(LSM9DS1RegID idx, uint8_t len) {
  if (len > 0) {
    uint8_t* ptr = &shadows[_get_shadow_offset(idx)];
    uint8_t byte_len = 0;
    for (uint8_t i = 0; i < len; i++) {
      byte_len +=_reg_width(idx);
      if (!_reg_writable(idx)) {
        return IMUFault::NOT_WRITABLE;
      }
    }
    uint8_t d_addr = _ADDR_IMU;
    uint8_t s_addr = _reg_addr(idx);
    if (_reg_is_for_mag(idx)) {
      d_addr  = _ADDR_MAG;
      //s_addr |= 0x40;
    }

    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = d_addr;
      op->sub_addr = s_addr;
      op->setBuffer(ptr, byte_len);
      if (0 == _bus->queue_io_job(op)) {
        return IMUFault::NO_ERROR;
      }
    }
    return IMUFault::BUS_OPERATION_FAILED_W;
  }
  return IMUFault::INVALID_PARAM;  // TODO: Crossed ontological wires... Fix.
}


IMUFault LSM9DS1_I2C::_read_registers(LSM9DS1RegID idx, uint8_t len) {
  if (len > 0) {
    uint8_t* ptr = &shadows[_get_shadow_offset(idx)];
    uint8_t byte_len = 0;
    for (uint8_t i = 0; i < len; i++) {
      byte_len +=_reg_width(idx);
    }
    uint8_t d_addr = _ADDR_IMU;
    uint8_t s_addr = _reg_addr(idx) | 0x80;
    if (_reg_is_for_mag(idx)) {
      d_addr  = _ADDR_MAG;
      //s_addr |= 0x40;
    }

    I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
    if (nullptr != op) {
      op->dev_addr = d_addr;
      op->sub_addr = s_addr;
      op->setBuffer(ptr, byte_len);
      if (0 == _bus->queue_io_job(op)) {
        return IMUFault::NO_ERROR;
      }
    }
    return IMUFault::BUS_OPERATION_FAILED_W;
  }
  return IMUFault::INVALID_PARAM;  // TODO: Crossed ontological wires... Fix.
}


/* Used to setup any bus-related nuances between SPI and I2C. */
int8_t LSM9DS1_I2C::_setup_bus_pin() {
  return 0;   // I2C doesn't need a CS pin.
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t LSM9DS1_I2C::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t LSM9DS1_I2C::io_op_callback(BusOp* _op) {
  I2CBusOp* completed = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;
  uint8_t r_addr = completed->sub_addr;

  if (!completed->hasFault()) {
    if (completed->dev_addr == _ADDR_IMU) {
      ret = _io_op_callback_imu(_reg_id_from_addr_imu(r_addr), _op);
    }
    else if (completed->dev_addr == _ADDR_MAG) {
      ret = _io_op_callback_mag(_reg_id_from_addr_mag(r_addr), _op);
    }
  }
  return ret;
}
