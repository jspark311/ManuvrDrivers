/*
* This file started out as a SparkFun driver. I have mutated it.
*
* Refactor log:
* Added flag member to track various boolean conditions in the class and the
*   hardware, rather than waste time in I/O for requests we ought to be able to
*   answer immediately.
* Error returns from functions were added, as well as a function for supplying
*   a TwoWire pointer. Data operations are now guarded by hardware error checks.
* Encapsulated data is now condensed and aligned.
* Added a temporal read marker so that polling-generated I/O can be minimized..
*                                                  ---J. Ian Lindsay  2020.02.03
* Converted typedefs to enum classes. Converted to async via CppPotpourri.
* TODO: This driver still makes the assumption of the host being little-endian.
*                                                  ---J. Ian Lindsay  2020.09.26
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

#include <AbstractPlatform.h>
#include <I2CAdapter.h>

#ifndef __VEML6075_DRIVER_H_
#define __VEML6075_DRIVER_H_

/* Class flags */
#define VEML6075_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define VEML6075_FLAG_PINS_CONFIGURED  0x0002  // Low-level pin setup is complete.
#define VEML6075_FLAG_INITIALIZED      0x0004  // Registers are initialized.
#define VEML6075_FLAG_ENABLED          0x0008  // Device is measuring.
#define VEML6075_FLAG_AF_ENABLED       0x0010  //
#define VEML6075_FLAG_TRIGGER_ENABLED  0x0020  //
#define VEML6075_FLAG_DYNAMIC_HIGH     0x0040  //


/*
* Responsivity converts a raw 16-bit UVA/UVB reading to a relative irradiance (W/m^2).
* These values will need to be adjusted as either integration time or dynamic settings are modififed.
* These values are recommended by the "Designing the VEML6075 into an application" app note for 100ms IT
*/
#define UVA_RESPONSIVITY_100MS_UNCOVERED    0.001111
#define UVB_RESPONSIVITY_100MS_UNCOVERED    0.00125


typedef uint16_t veml6075_t;

/* VEML6075 error code returns */
enum class VEML6075Err : int8_t {
  READ            = -4,
  WRITE           = -3,
  INVALID_ADDRESS = -2,
  UNDEFINED       = -1,
  SUCCESS         = 0
};


enum class VEML6075IntTime : uint8_t {
  IT_50MS         = 0,
  IT_100MS        = 1,
  IT_200MS        = 2,
  IT_400MS        = 3,
  IT_800MS        = 4,
  IT_RESERVED_0   = 5,
  IT_RESERVED_1   = 6,
  IT_RESERVED_2   = 7,
  IT_INVALID      = 8
};

enum class veml6075_hd_t : uint8_t {
  DYNAMIC_NORMAL  = 0,
  DYNAMIC_HIGH    = 1,
  HD_INVALID      = 2
};

enum class veml6075_uv_trig_t : uint8_t {
  NO_TRIGGER,
  TRIGGER_ONE_OR_UV_TRIG,
  TRIGGER_INVALID
};

enum class veml6075_af_t : uint8_t {
  AF_DISABLE,
  AF_ENABLE,
  AF_INVALID
};

/* VEML6075 registers. These are indicies, and not addresses. */
enum class VEML6075RegId : uint8_t {
  UV_CONF      = 0x00,
  UVA_DATA     = 0x01,
  UVB_DATA     = 0x02,
  UVCOMP1_DATA = 0x03,
  UVCOMP2_DATA = 0x04,
  ID           = 0x05,
  INVALID      = 0x06
};


/*******************************************************************************
* Class definition
*******************************************************************************/
class VEML6075  : public I2CDevice {
  public:
    VEML6075();
    ~VEML6075() {};

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    int8_t init();
    int8_t poll();

    inline bool devFound() {        return _veml_flag(VEML6075_FLAG_DEVICE_PRESENT);  };
    inline bool initialized() {     return _veml_flag(VEML6075_FLAG_INITIALIZED);     };
    inline bool enabled() {         return _veml_flag(VEML6075_FLAG_ENABLED);         };
    VEML6075Err enabled(bool);


    // Configuration controls
    VEML6075Err setIntegrationTime(VEML6075IntTime it);
    inline uint16_t getIntegrationTime() {
      return _integrationTime;
    };

    VEML6075Err setHighDynamic(veml6075_hd_t hd);
    inline veml6075_hd_t getHighDynamic() {
      return _veml_flag(VEML6075_FLAG_DYNAMIC_HIGH) ? veml6075_hd_t::DYNAMIC_HIGH : veml6075_hd_t::DYNAMIC_NORMAL;
    };

    VEML6075Err setTrigger(veml6075_uv_trig_t trig);
    inline veml6075_uv_trig_t getTrigger() {
      return _veml_flag(VEML6075_FLAG_TRIGGER_ENABLED) ? veml6075_uv_trig_t::TRIGGER_ONE_OR_UV_TRIG : veml6075_uv_trig_t::NO_TRIGGER;
    };
    VEML6075Err trigger();

    VEML6075Err setAutoForce(veml6075_af_t af);
    inline veml6075_af_t getAutoForce() {
      return _veml_flag(VEML6075_FLAG_AF_ENABLED) ? veml6075_af_t::AF_ENABLE : veml6075_af_t::AF_DISABLE;
    };

    inline float uva() {    return _lastUVA;     };
    inline float uvb() {    return _lastUVB;     };
    inline uint16_t visibleCompensation() {   return _lastCOMP1;   };
    inline uint16_t irCompensation() {        return _lastCOMP2;   };
    float index();


  private:
    uint16_t  _flags           = 0;
    uint16_t  _integrationTime = 0;
    uint16_t  _lastCOMP1       = 0;
    uint16_t  _lastCOMP2       = 0;
    uint16_t  shadows[6]       = {0, 0, 0, 0, 0, 0};
    uint32_t  _last_read       = 0;
    float     _lastUVA         = 0.0;
    float     _lastUVB         = 0.0;
    float     _lastIndex       = 0.0;
    float     _aResponsivity   = UVA_RESPONSIVITY_100MS_UNCOVERED;
    float     _bResponsivity   = UVB_RESPONSIVITY_100MS_UNCOVERED;

    VEML6075Err _read_data();
    int8_t _post_discovery_init();
    int8_t _process_new_config(uint8_t);

    int8_t  _read_registers(VEML6075RegId, uint8_t);
    int8_t  _write_registers(VEML6075RegId, uint8_t);

    /* Flag manipulation inlines */
    inline uint16_t _veml_flags() {                return _flags;           };
    inline bool _veml_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _veml_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _veml_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _veml_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };

    static VEML6075RegId _reg_id_from_addr(const uint8_t addr);
    static uint8_t      _reg_addr_from_id(const VEML6075RegId);
};


#endif  // __VEML6075_DRIVER_H_
