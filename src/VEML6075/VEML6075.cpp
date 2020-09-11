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

#define VEML6075_REGISTER_LENGTH 2   // 2 bytes per register
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

#define VEML6075_MASK(reg, mask, shift) ((reg & mask) >> shift)

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


VEML6075::VEML6075() : I2CDevice(VEML6075_ADDRESS) {}


VEML6075_error_t VEML6075::init() {
  VEML6075_error_t err = VEML6075_ERROR_UNDEFINED;
  if (nullptr != _bus) {
    if (VEML6075_ERROR_SUCCESS == _connected()) {
      if (VEML6075_ERROR_SUCCESS == enabled(true)) {   // Power on
        err = setIntegrationTime(IT_100MS);      // Set intergration time to 100ms
        if (err == VEML6075_ERROR_SUCCESS) {
          err = setHighDynamic(DYNAMIC_NORMAL);  // High-dynamic mode off
          if (err == VEML6075_ERROR_SUCCESS) {
            err = setAutoForce(AF_DISABLE);      // Disable auto-force mode
          }
        }
      }
    }
  }
  _veml_set_flag(VEML6075_FLAG_INITIALIZED, (VEML6075_ERROR_SUCCESS == err));
  return err;
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
    if ((_last_read + _integrationTime) <= millis()) {
      ret = (VEML6075_ERROR_SUCCESS == _read_data()) ? 1 : -1;
    }
  }
  return ret;
}


VEML6075_error_t VEML6075::setIntegrationTime(VEML6075::veml6075_uv_it_t it) {
  veml6075_t conf;
  if (it >= IT_RESERVED_0) {
    return VEML6075_ERROR_UNDEFINED;
  }

  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (err != VEML6075_ERROR_SUCCESS) {
    return err;
  }

  conf &= ~(VEML6075_UV_IT_MASK);
  conf |= (it<<VEML6075_UV_IT_SHIFT);
  err = writeI2CRegister(conf, VEML6075::REG_UV_CONF);
  if (err != VEML6075_ERROR_SUCCESS) {
    return err;
  }

  _aResponsivity = UVA_RESPONSIVITY[(uint8_t)it];
  _bResponsivity = UVB_RESPONSIVITY[(uint8_t)it];

  switch (it) {
    case IT_50MS:   _integrationTime = 50;     break;
    case IT_100MS:  _integrationTime = 100;    break;
    case IT_200MS:  _integrationTime = 200;    break;
    case IT_400MS:  _integrationTime = 400;    break;
    case IT_800MS:  _integrationTime = 800;    break;
    default:        _integrationTime = 0;      break;
  }
  return err;
}


VEML6075::veml6075_uv_it_t VEML6075::getIntegrationTime() {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (err != VEML6075_ERROR_SUCCESS) {
    return IT_INVALID;
  }
  return static_cast<VEML6075::veml6075_uv_it_t>((conf & VEML6075_UV_IT_MASK) >> VEML6075_UV_IT_SHIFT);
}


VEML6075_error_t VEML6075::setHighDynamic(VEML6075::veml6075_hd_t hd) {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (err == VEML6075_ERROR_SUCCESS) {
    conf &= ~(VEML6075_HD_MASK);
    conf |= (hd << VEML6075_HD_SHIFT);
    err = writeI2CRegister(conf, VEML6075::REG_UV_CONF);
    if (err == VEML6075_ERROR_SUCCESS) {
      _veml_set_flag(VEML6075_FLAG_DYNAMIC_HIGH, (hd == DYNAMIC_HIGH));
    }
  }
  return err;
}


VEML6075::veml6075_hd_t VEML6075::getHighDynamic() {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (err != VEML6075_ERROR_SUCCESS) {
    return HD_INVALID;
  }
  return static_cast<VEML6075::veml6075_hd_t>((conf & VEML6075_HD_MASK) >> VEML6075_HD_SHIFT);
}


VEML6075_error_t VEML6075::setTrigger(VEML6075::veml6075_uv_trig_t trig) {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (VEML6075_ERROR_SUCCESS == err) {
    conf &= ~(VEML6075_TRIG_MASK);
    conf |= (trig << VEML6075_TRIG_SHIFT);
    err = writeI2CRegister(conf, VEML6075::REG_UV_CONF);
    if (VEML6075_ERROR_SUCCESS == err) {
      _veml_set_flag(VEML6075_FLAG_TRIGGER_ENABLED, (trig == TRIGGER_ONE_OR_UV_TRIG));
    }
  }
  return err;
}


VEML6075::veml6075_uv_trig_t VEML6075::getTrigger() {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (err != VEML6075_ERROR_SUCCESS) {
    return TRIGGER_INVALID;
  }
  return static_cast<VEML6075::veml6075_uv_trig_t>((conf & VEML6075_TRIG_MASK) >> VEML6075_TRIG_SHIFT);
}


VEML6075_error_t VEML6075::setAutoForce(VEML6075::veml6075_af_t af) {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (VEML6075_ERROR_SUCCESS == err) {
    conf &= ~(VEML6075_AF_MASK);
    conf |= (af << VEML6075_AF_SHIFT);
    err = writeI2CRegister(conf, VEML6075::REG_UV_CONF);
    if (VEML6075_ERROR_SUCCESS == err) {
      _veml_set_flag(VEML6075_FLAG_AF_ENABLED);
    }
  }
  return err;
}


VEML6075::veml6075_af_t VEML6075::getAutoForce() {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (err != VEML6075_ERROR_SUCCESS) {
    return AF_INVALID;
  }
  return static_cast<VEML6075::veml6075_af_t>((conf & VEML6075_AF_MASK) >> VEML6075_AF_SHIFT);
}


VEML6075_error_t VEML6075::enabled(bool en) {
  veml6075_t conf;
  VEML6075_error_t err = readI2CRegister(&conf, VEML6075::REG_UV_CONF);
  if (VEML6075_ERROR_SUCCESS == err) {
    conf &= 0xFE; // Clear shutdown bit
    if (!en) {
      conf |= 1;
    }
    err = writeI2CRegister(conf, VEML6075::REG_UV_CONF);
    if (VEML6075_ERROR_SUCCESS == err) {
      _veml_set_flag(VEML6075_FLAG_ENABLED, en);
    }
  }
  return err;
}


VEML6075_error_t VEML6075::trigger() {
  return setTrigger(TRIGGER_ONE_OR_UV_TRIG);
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
VEML6075_error_t VEML6075::_read_data() {
  VEML6075_error_t err = VEML6075_ERROR_UNDEFINED;
  if (initialized() && enabled()) {
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    err = readI2CBuffer(&buffer[0], VEML6075::REG_UVA_DATA, 2);
    if (VEML6075_ERROR_SUCCESS == err) {
      err = readI2CBuffer(&buffer[2], VEML6075::REG_UVB_DATA, 2);
      if (VEML6075_ERROR_SUCCESS == err) {
        err = readI2CBuffer(&buffer[4], VEML6075::REG_UVCOMP1_DATA, 2);
        if (VEML6075_ERROR_SUCCESS == err) {
          err = readI2CBuffer(&buffer[6], VEML6075::REG_UVCOMP2_DATA, 2);
          if (VEML6075_ERROR_SUCCESS == err) {
            uint16_t new_uva = (buffer[0] & 0x00FF) | ((buffer[1] & 0x00FF) << 8);
            uint16_t new_uvb = (buffer[2] & 0x00FF) | ((buffer[3] & 0x00FF) << 8);
            _lastCOMP1 = (buffer[4] & 0x00FF) | ((buffer[5] & 0x00FF) << 8);
            _lastCOMP2 = (buffer[6] & 0x00FF) | ((buffer[7] & 0x00FF) << 8);
            _last_read = millis();
            _lastUVA = ((float) new_uva) - ((UVA_A_COEF * UV_ALPHA * _lastCOMP1) / UV_GAMMA) - ((UVA_B_COEF * UV_ALPHA * _lastCOMP2) / UV_DELTA);
            _lastUVB = ((float) new_uvb) - ((UVA_C_COEF * UV_BETA  * _lastCOMP1) / UV_GAMMA) - ((UVA_D_COEF * UV_BETA  * _lastCOMP2) / UV_DELTA);
          }
        }
      }
    }
  }
  return err;
}


VEML6075_error_t VEML6075::_connected() {
  veml6075_t devID;
  VEML6075_error_t err = VEML6075_ERROR_INVALID_ADDRESS;
  if (VEML6075_ERROR_SUCCESS == readI2CRegister(&devID, VEML6075::REG_ID)) {
    if (VEML6075_DEVICE_ID == (devID & 0x00FF)) {
      err = VEML6075_ERROR_SUCCESS;
    }
  }
  _veml_set_flag(VEML6075_FLAG_DEVICE_PRESENT, (VEML6075_ERROR_SUCCESS == err));
  return err;
}


VEML6075_error_t VEML6075::readI2CBuffer(uint8_t* dest, VEML6075_REGISTER_t startRegister, uint16_t len) {
  //VEML6075_DEBUGLN((STORAGE("(readI2CBuffer): read ") + String(len) + STORAGE(" @ 0x") + String(startRegister, HEX)));
  _bus->beginTransmission((uint8_t) VEML6075_ADDRESS);
  _bus->write(startRegister);
  if (_bus->endTransmission(false) != 0) {
    //VEML6075_DEBUGLN(STORAGE("    ERR (readI2CBuffer): End transmission"));
    return VEML6075_ERROR_READ;
  }

  _bus->requestFrom((uint8_t) VEML6075_ADDRESS, (uint8_t)len);
  for (uint16_t i = 0; i < len; i++) {
    dest[i] = _bus->read();
    //VEML6075_DEBUGLN((STORAGE("    ") + String(i) + STORAGE(": 0x") + String(dest[i], HEX)));
  }
  return VEML6075_ERROR_SUCCESS;
}


VEML6075_error_t VEML6075::writeI2CBuffer(uint8_t* src, VEML6075_REGISTER_t startRegister, uint16_t len) {
  _bus->beginTransmission((uint8_t) VEML6075_ADDRESS);
  _bus->write(startRegister);
  for (uint16_t i = 0; i < len; i++) {
    _bus->write(src[i]);
  }
  return (_bus->endTransmission(true) == 0) ? VEML6075_ERROR_SUCCESS : VEML6075_ERROR_WRITE;
}


VEML6075_error_t VEML6075::readI2CRegister(veml6075_t* dest, VEML6075_REGISTER_t registerAddress) {
  uint8_t tempDest[2];
  VEML6075_error_t err = readI2CBuffer(tempDest, registerAddress, VEML6075_REGISTER_LENGTH);
  if (err == VEML6075_ERROR_SUCCESS) {
    *dest = (tempDest[0]) | ((veml6075_t) tempDest[1] << 8);
  }
  return err;
}


VEML6075_error_t VEML6075::writeI2CRegister(veml6075_t data, VEML6075_REGISTER_t registerAddress) {
  uint8_t d[2] = {   // Write LSB first
    (uint8_t) (data & 0x00FF),
    (uint8_t) ((data & 0xFF00) >> 8)
  };
  return writeI2CBuffer(d, registerAddress, VEML6075_REGISTER_LENGTH);
}
