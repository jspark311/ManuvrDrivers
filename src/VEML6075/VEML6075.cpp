/*
* This file started out as a SparkFun driver. See header file for
*   my change list.
*                                                            ---J. Ian Lindsay
*/

/*
  This is a library written for the VEML6075 UVA/UVB/UV index Sensopr
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14748
  Written by Jim Lindblom @ SparkFun Electronics, May 23, 2018

  The VEML6075 senses UVA and UVB light, which allows for a calculation
  of the UV index.

  This library handles the initialization, configuration and monitoring of the
  UVA and UVB intensity, and calculation of the UV index.

  https://github.com/sparkfunX/SparkFun_VEML6075_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.5
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "VEML6075.h"

#define VEML6075_ADDRESS   0x10

#define NUM_INTEGRATION_TIMES 5

#define VEML6075_DEVICE_ID 0x26

#define VEML6075_UV_IT_MASK  0x70
#define VEML6075_UV_IT_SHIFT 4

#define VEML6075_HD_MASK  0x08
#define VEML6075_HD_SHIFT 3

#define VEML6075_TRIG_MASK  0x04
#define VEML6075_TRIG_SHIFT 2

#define VEML6075_AF_MASK  0x02
#define VEML6075_AF_SHIFT 1

const float HD_SCALAR = 2.0;

// Calibration constants:
// Four gain calibration constants -- alpha, beta, gamma, delta -- can be used to correct the output in
// reference to a GOLDEN sample. The golden sample should be calibrated under a solar simulator.
// Setting these to 1.0 essentialy eliminates the "golden"-sample calibration
const float UV_ALPHA   = 1.0;   // UVA / UVAgolden
const float UV_BETA    = 1.0;   // UVB / UVBgolden
const float UV_GAMMA   = 1.0;   // UVcomp1 / UVcomp1golden
const float UV_DELTA   = 1.0;   // UVcomp2 / UVcomp2golden

const float UVA_A_COEF = 2.22;  // Visible
const float UVA_B_COEF = 1.33;  // IR
const float UVA_C_COEF = 2.95;  // Visible
const float UVA_D_COEF = 1.75;  // IR


/*
* Responsivity converts a raw 16-bit UVA/UVB reading to a relative irradiance (W/m^2).
* These values will need to be adjusted as either integration time or dynamic settings are modififed.
* These values are recommended by the "Designing the VEML6075 into an application" app note for 100ms IT
*/
#define UVA_RESPONSIVITY_100MS_UNCOVERED    0.001111
#define UVB_RESPONSIVITY_100MS_UNCOVERED    0.00125

const float UVA_RESPONSIVITY[NUM_INTEGRATION_TIMES] = {
  UVA_RESPONSIVITY_100MS_UNCOVERED / 0.5016286645, // 50ms
  UVA_RESPONSIVITY_100MS_UNCOVERED,                // 100ms
  UVA_RESPONSIVITY_100MS_UNCOVERED / 2.039087948,  // 200ms
  UVA_RESPONSIVITY_100MS_UNCOVERED / 3.781758958,  // 400ms
  UVA_RESPONSIVITY_100MS_UNCOVERED / 7.371335505   // 800ms
};

const float UVB_RESPONSIVITY[NUM_INTEGRATION_TIMES] = {
  UVB_RESPONSIVITY_100MS_UNCOVERED / 0.5016286645, // 50ms
  UVB_RESPONSIVITY_100MS_UNCOVERED,                // 100ms
  UVB_RESPONSIVITY_100MS_UNCOVERED / 2.039087948,  // 200ms
  UVB_RESPONSIVITY_100MS_UNCOVERED / 3.781758958,  // 400ms
  UVB_RESPONSIVITY_100MS_UNCOVERED / 7.371335505   // 800ms
};



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

/* Converts a register address back into an enum. */
VEML6075RegId VEML6075::_reg_id_from_addr(const uint8_t reg_addr) {
  switch (reg_addr & 0x0F) {
    case 0x00:   return VEML6075RegId::UV_CONF;
    case 0x07:   return VEML6075RegId::UVA_DATA;
    case 0x09:   return VEML6075RegId::UVB_DATA;
    case 0x0A:   return VEML6075RegId::UVCOMP1_DATA;
    case 0x0B:   return VEML6075RegId::UVCOMP2_DATA;
    case 0x0C:   return VEML6075RegId::ID;
  }
  return VEML6075RegId::INVALID;
}


/* Converts an enum into a register address. */
uint8_t VEML6075::_reg_addr_from_id(const VEML6075RegId r) {
  switch (r) {
    case VEML6075RegId::UV_CONF:       return 0x00;
    case VEML6075RegId::UVA_DATA:      return 0x07;
    case VEML6075RegId::UVB_DATA:      return 0x09;
    case VEML6075RegId::UVCOMP1_DATA:  return 0x0A;
    case VEML6075RegId::UVCOMP2_DATA:  return 0x0B;
    case VEML6075RegId::ID:            return 0x0C;
    default:
      break;
  }
  return 0;
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

VEML6075::VEML6075() : I2CDevice(VEML6075_ADDRESS) {}


int8_t VEML6075::init() {
  int8_t ret = -1;
  if (nullptr != _bus) {
    ret--;
    _veml_clear_flag(VEML6075_FLAG_INITIALIZED);
    if (0 == _read_registers(VEML6075RegId::ID, 2)) {
      ret = 0;
    }
  }
  return ret;
}


/*
* Poll the class for updates.
* Returns...
*   -3 if not initialized and enabled.
*   -1 if data needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if data was read and is fresh.
*/
int8_t VEML6075::poll() {
  int8_t ret = -3;
  if (initialized() && enabled()) {
    ret = 0;
    if (wrap_accounted_delta(millis(), _last_read) >= _integrationTime) {
      if (!_veml_flag(VEML6075_FLAG_READ_IN_FLIGHT)) {
        ret = (VEML6075Err::SUCCESS == _read_data()) ? 0 : -1;
      }
    }
    if (_veml_flag(VEML6075_FLAG_DATA_FRESH)) {
      _veml_clear_flag(VEML6075_FLAG_DATA_FRESH);
      ret = 1;
    }
  }
  return ret;
}


/*******************************************************************************
* Functions specific to this class....                                         *
*******************************************************************************/

int8_t VEML6075::_post_discovery_init() {
  int8_t ret = -1;
  *((uint8_t*) shadows) = 0x10;
  if (0 == _write_registers(VEML6075RegId::UV_CONF, 2)) {
    ret = 0;
  }
  //if (0 == _read_registers(VEML6075RegId::UV_CONF, 2)) {
  //}
  return ret;
}


VEML6075Err VEML6075::setIntegrationTime(VEML6075IntTime it) {
  uint8_t conf = *((uint8_t*) shadows);  // Valid conf is the first byte of the first 16-bit index.
  switch (it) {
    case VEML6075IntTime::IT_50MS:
    case VEML6075IntTime::IT_100MS:
    case VEML6075IntTime::IT_200MS:
    case VEML6075IntTime::IT_400MS:
    case VEML6075IntTime::IT_800MS:
      conf &= ~(VEML6075_UV_IT_MASK);
      conf |= (((uint8_t) it) << VEML6075_UV_IT_SHIFT);
      *((uint8_t*) shadows) = conf;
      if (0 != _write_registers(VEML6075RegId::UV_CONF, 2)) {
        return VEML6075Err::WRITE;
      }
      break;
    default:
      return VEML6075Err::UNDEFINED;
  }
  return VEML6075Err::SUCCESS;
}


VEML6075Err VEML6075::setHighDynamic(VEML6075DynamicMode hd) {
  uint8_t conf = *((uint8_t*) shadows);  // Valid conf is the first byte of the first 16-bit index.
  switch (hd) {
    case VEML6075DynamicMode::DYNAMIC_NORMAL:
    case VEML6075DynamicMode::DYNAMIC_HIGH:
      conf &= ~(VEML6075_HD_MASK);
      conf |= (((uint8_t) hd) << VEML6075_HD_SHIFT);
      *((uint8_t*) shadows) = conf;
      if (0 != _write_registers(VEML6075RegId::UV_CONF, 2)) {
        return VEML6075Err::WRITE;
      }
      break;
    default:
      return VEML6075Err::UNDEFINED;
  }
  return VEML6075Err::SUCCESS;
}


VEML6075Err VEML6075::setTrigger(VEML6075Trigger trig) {
  uint8_t conf = *((uint8_t*) shadows);  // Valid conf is the first byte of the first 16-bit index.
  switch (trig) {
    case VEML6075Trigger::NO_TRIGGER:
    case VEML6075Trigger::TRIGGER_ONE_OR_UV_TRIG:
      conf &= ~(VEML6075_TRIG_MASK);
      conf |= (((uint8_t) trig) << VEML6075_TRIG_SHIFT);
      *((uint8_t*) shadows) = conf;
      if (0 != _write_registers(VEML6075RegId::UV_CONF, 2)) {
        return VEML6075Err::WRITE;
      }
      break;
    default:
      return VEML6075Err::UNDEFINED;
  }
  return VEML6075Err::SUCCESS;
}


VEML6075Err VEML6075::setAutoForce(veml6075ActiveForce af) {
  uint8_t conf = *((uint8_t*) shadows);  // Valid conf is the first byte of the first 16-bit index.
  switch (af) {
    case veml6075ActiveForce::AF_DISABLE:
    case veml6075ActiveForce::AF_ENABLE:
      conf &= ~(VEML6075_AF_MASK);
      conf |= (((uint8_t) af) << VEML6075_AF_SHIFT);
      *((uint8_t*) shadows) = conf;
      if (0 != _write_registers(VEML6075RegId::UV_CONF, 2)) {
        return VEML6075Err::WRITE;
      }
      break;
    default:
      return VEML6075Err::UNDEFINED;
  }
  return VEML6075Err::SUCCESS;
}


VEML6075Err VEML6075::enabled(bool en) {
  uint8_t conf = *((uint8_t*) shadows);  // Valid conf is the first byte of the first 16-bit index.
  if (en != _veml_flag(VEML6075_FLAG_ENABLED)) {
    conf &= 0xFE; // Clear shutdown bit
    if (!en) {
      conf |= 1;
    }
    if (0 != _write_registers(VEML6075RegId::UV_CONF, 2)) {
      return VEML6075Err::WRITE;
    }
  }
  return VEML6075Err::SUCCESS;
}


/*
* Dump this item to the dev log.
*/
void VEML6075::printDebug(StringBuilder* output) {
  output->concatf("-- VEML6075 %sinitialized\n", (initialized() ? "" : "un"));
  I2CDevice::printDebug(output);
  output->concatf("\tFound:          %c\n", (devFound() ? 'y' : 'n'));
  output->concatf("\tEnabled:        %c\n", (enabled() ? 'y' : 'n'));
  output->concatf("\tAF enabled:     %c\n", (_veml_flag(VEML6075_FLAG_AF_ENABLED) ? 'y' : 'n'));
  output->concatf("\tTrig enabled:   %c\n", (_veml_flag(VEML6075_FLAG_TRIGGER_ENABLED) ? 'y' : 'n'));
  output->concatf("\tDynamic mode:   %s\n", (_veml_flag(VEML6075_FLAG_DYNAMIC_HIGH) ? "HIGH" : "NORM"));
  output->concatf("\tIntgration time: %ums\n", _integrationTime);
  output->concatf("\t_last_read:     %u\n", _last_read);
  output->concatf("\tCONF:           0x%04x\n", shadows[0]);
  output->concatf("\t_lastCOMP1:     0x%04x\n", _lastCOMP1);
  output->concatf("\t_lastCOMP2:     0x%04x\n", _lastCOMP2);
  output->concatf("\t_lastUVA:       %.3f\n", _lastUVA);
  output->concatf("\t_lastUVB:       %.3f\n", _lastUVB);
  output->concatf("\t_lastIndex:     %.3f\n", _lastIndex);
}


VEML6075Err VEML6075::trigger() {
  return setTrigger(VEML6075Trigger::TRIGGER_ONE_OR_UV_TRIG);
}


float VEML6075::index() {
  float uvia = _lastUVA * (1.0 / UV_ALPHA) * _aResponsivity;
  float uvib = _lastUVB * (1.0 / UV_BETA)  * _bResponsivity;
  _lastIndex = (uvia + uvib) / 2.0;
  if (_veml_flag(VEML6075_FLAG_DYNAMIC_HIGH)) {
    _lastIndex *= HD_SCALAR;
  }
  return _lastIndex;
}


/*
* Encompasses UVA, UVB, UVCOMP1, and UVCOMP2. Stores raw values in shadows.
*/
VEML6075Err VEML6075::_read_data() {
  VEML6075Err err = VEML6075Err::UNDEFINED;
  if (initialized() && enabled()) {
    err = VEML6075Err::READ;
    if (0 == _read_registers(VEML6075RegId::UVA_DATA, 2)) {
      if (0 == _read_registers(VEML6075RegId::UVB_DATA, 2)) {
        if (0 == _read_registers(VEML6075RegId::UVCOMP1_DATA, 2)) {
          if (0 == _read_registers(VEML6075RegId::UVCOMP2_DATA, 2)) {
            _veml_set_flag(VEML6075_FLAG_READ_IN_FLIGHT);
            err = VEML6075Err::SUCCESS;
          }
        }
      }
    }
  }
  return err;
}


int8_t  VEML6075::_read_registers(VEML6075RegId r, uint8_t len) {
  int8_t ret = -1;
  switch (r) {
    case VEML6075RegId::UV_CONF:
    case VEML6075RegId::UVA_DATA:
    case VEML6075RegId::UVB_DATA:
    case VEML6075RegId::UVCOMP1_DATA:
    case VEML6075RegId::UVCOMP2_DATA:
    case VEML6075RegId::ID:
      break;
    default:
      return ret;
  }
  uint8_t* ptr = (uint8_t*) &shadows[(uint8_t) r];
  // TODO: Write LSB first. Ensure this on big-endian platforms.
  I2CBusOp* op = _bus->new_op(BusOpcode::RX, this);
  if (nullptr != op) {
    ret--;
    op->dev_addr = _dev_addr;
    op->sub_addr = _reg_addr_from_id(r);
    op->setBuffer(ptr, len);
    if (0 == queue_io_job(op)) {
      ret = 0;
    }
  }
  return ret;
}


int8_t  VEML6075::_write_registers(VEML6075RegId r, uint8_t len) {
  int8_t ret = -1;
  if (VEML6075RegId::UV_CONF == r) {   // The only writable register.
    ret--;
    uint8_t* ptr = (uint8_t*) &shadows[(uint8_t) r];
    // TODO: Write LSB first. Ensure this on big-endian platforms.
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      ret--;
      op->dev_addr = _dev_addr;
      op->sub_addr = _reg_addr_from_id(r);
      op->setBuffer(ptr, len);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}



int8_t VEML6075::_process_new_config(uint8_t new_conf) {
  VEML6075IntTime it = (VEML6075IntTime) ((new_conf & VEML6075_UV_IT_MASK) >> VEML6075_UV_IT_SHIFT);
  VEML6075DynamicMode hd = (VEML6075DynamicMode) ((new_conf & VEML6075_HD_MASK) >> VEML6075_HD_SHIFT);
  VEML6075Trigger trig = (VEML6075Trigger) ((new_conf & VEML6075_TRIG_MASK) >> VEML6075_TRIG_SHIFT);
  veml6075ActiveForce af = (veml6075ActiveForce) ((new_conf & VEML6075_AF_MASK) >> VEML6075_AF_SHIFT);

  _aResponsivity = UVA_RESPONSIVITY[(uint8_t) it];
  _bResponsivity = UVB_RESPONSIVITY[(uint8_t) it];
  switch (it) {
    case VEML6075IntTime::IT_50MS:   _integrationTime = 50;     break;
    case VEML6075IntTime::IT_100MS:  _integrationTime = 100;    break;
    case VEML6075IntTime::IT_200MS:  _integrationTime = 200;    break;
    case VEML6075IntTime::IT_400MS:  _integrationTime = 400;    break;
    case VEML6075IntTime::IT_800MS:  _integrationTime = 800;    break;
    default:                         _integrationTime = 0;      break;
  }

  _veml_set_flag(VEML6075_FLAG_DYNAMIC_HIGH, (hd == VEML6075DynamicMode::DYNAMIC_HIGH));
  _veml_set_flag(VEML6075_FLAG_TRIGGER_ENABLED, (trig == VEML6075Trigger::TRIGGER_ONE_OR_UV_TRIG));
  _veml_set_flag(VEML6075_FLAG_AF_ENABLED, (af == veml6075ActiveForce::AF_ENABLE));
  _veml_set_flag(VEML6075_FLAG_ENABLED, (0 == (new_conf & 0x01)));
  if (!initialized()) {
    _veml_set_flag(VEML6075_FLAG_INITIALIZED);
  }
  return 0;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t VEML6075::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t VEML6075::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf    = op->buffer();
    uint     len    = op->bufferLen();
    VEML6075RegId r = _reg_id_from_addr(op->sub_addr);
    uint8_t value   = *buf;

    switch (op->get_opcode()) {
      case BusOpcode::TX:
        switch (r) {
          case VEML6075RegId::UV_CONF:
            _process_new_config(value);
            break;
          default:  // Anything else is an illegal write target.
            break;
        }
        break;

      case BusOpcode::RX:
        switch (r) {
          case VEML6075RegId::UV_CONF:
            _process_new_config(value);
            break;
          case VEML6075RegId::UVA_DATA:  // The first three data registers are
          case VEML6075RegId::UVB_DATA:  //   observed on the fouth's callback.
          case VEML6075RegId::UVCOMP1_DATA:
            //break;
          case VEML6075RegId::UVCOMP2_DATA:
            {
              uint8_t* uva_ptr = ((uint8_t*) &shadows[1]);
              uint16_t new_uva = ((uint16_t) *(uva_ptr + 0)) | ((uint16_t) *(uva_ptr + 1) << 8);
              uint16_t new_uvb = ((uint16_t) *(uva_ptr + 2)) | ((uint16_t) *(uva_ptr + 3) << 8);
              _lastCOMP1 = ((uint16_t) *(uva_ptr + 4)) | ((uint16_t) *(uva_ptr + 5) << 8);
              _lastCOMP2 = ((uint16_t) *(uva_ptr + 6)) | ((uint16_t) *(uva_ptr + 7) << 8);
              _lastUVA = ((float) new_uva) - ((UVA_A_COEF * UV_ALPHA * _lastCOMP1) / UV_GAMMA) - ((UVA_B_COEF * UV_ALPHA * _lastCOMP2) / UV_DELTA);
              _lastUVB = ((float) new_uvb) - ((UVA_C_COEF * UV_BETA  * _lastCOMP1) / UV_GAMMA) - ((UVA_D_COEF * UV_BETA  * _lastCOMP2) / UV_DELTA);
              _last_read = millis();
              _veml_clear_flag(VEML6075_FLAG_READ_IN_FLIGHT);
              _veml_set_flag(VEML6075_FLAG_DATA_FRESH);
            }
            break;
          case VEML6075RegId::ID:
            {
              StringBuilder output;
              op->printDebug(&output);
              Serial.println((char*) output.string());
            }
            _veml_set_flag(VEML6075_FLAG_DEVICE_PRESENT, (VEML6075_DEVICE_ID == value));
            if (!initialized()) {
              _post_discovery_init();
            }
            break;
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
