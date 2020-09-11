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
typedef enum {
    VEML6075_ERROR_READ            = -4,
    VEML6075_ERROR_WRITE           = -3,
    VEML6075_ERROR_INVALID_ADDRESS = -2,
    VEML6075_ERROR_UNDEFINED       = -1,
    VEML6075_ERROR_SUCCESS         = 1
} VEML6075_error_t;


/*******************************************************************************
* Class definition
*******************************************************************************/
class VEML6075  : public I2CDevice {
  public:
    typedef enum {
        IT_50MS,
        IT_100MS,
        IT_200MS,
        IT_400MS,
        IT_800MS,
        IT_RESERVED_0,
        IT_RESERVED_1,
        IT_RESERVED_2,
        IT_INVALID
    } veml6075_uv_it_t;

    typedef enum {
        DYNAMIC_NORMAL,
        DYNAMIC_HIGH,
        HD_INVALID
    } veml6075_hd_t;

    typedef enum {
        NO_TRIGGER,
        TRIGGER_ONE_OR_UV_TRIG,
        TRIGGER_INVALID
    } veml6075_uv_trig_t;

    typedef enum {
        AF_DISABLE,
        AF_ENABLE,
        AF_INVALID
    } veml6075_af_t;

    VEML6075();

    VEML6075_error_t init();
    int8_t poll();

    inline bool devFound() {        return _veml_flag(VEML6075_FLAG_DEVICE_PRESENT);  };
    inline bool initialized() {     return _veml_flag(VEML6075_FLAG_INITIALIZED);     };
    inline bool enabled() {         return _veml_flag(VEML6075_FLAG_ENABLED);         };
    VEML6075_error_t enabled(bool);


    // Configuration controls
    VEML6075_error_t setIntegrationTime(veml6075_uv_it_t it);
    veml6075_uv_it_t getIntegrationTime();

    VEML6075_error_t setHighDynamic(veml6075_hd_t hd);
    veml6075_hd_t getHighDynamic();

    VEML6075_error_t setTrigger(veml6075_uv_trig_t trig);
    veml6075_uv_trig_t getTrigger();
    VEML6075_error_t trigger();

    VEML6075_error_t setAutoForce(veml6075_af_t af);
    veml6075_af_t getAutoForce();

    inline float uva() {    return _lastUVA;     };
    inline float uvb() {    return _lastUVB;     };
    inline uint16_t visibleCompensation() {   return _lastCOMP1;   };
    inline uint16_t irCompensation() {        return _lastCOMP2;   };
    float index();


  private:
    uint16_t  _flags           = 0;
    uint16_t  _integrationTime = 0;
    uint32_t  _last_read       = 0;
    uint16_t  _lastCOMP1       = 0;
    uint16_t  _lastCOMP2       = 0;
    float     _lastUVA         = 0.0;
    float     _lastUVB         = 0.0;
    float     _lastIndex       = 0.0;
    float     _aResponsivity   = UVA_RESPONSIVITY_100MS_UNCOVERED;
    float     _bResponsivity   = UVB_RESPONSIVITY_100MS_UNCOVERED;
    // VEML6075 registers:
    typedef enum {
        REG_UV_CONF = 0x00,
        REG_UVA_DATA = 0x07,
        REG_UVB_DATA = 0x09,
        REG_UVCOMP1_DATA = 0x0A,
        REG_UVCOMP2_DATA = 0x0B,
        REG_ID = 0x0C
    } VEML6075_REGISTER_t;

    VEML6075_error_t _read_data();
    VEML6075_error_t _connected();

    // I2C Read/Write
    VEML6075_error_t readI2CBuffer(uint8_t* dest, VEML6075_REGISTER_t startRegister, uint16_t len);
    VEML6075_error_t writeI2CBuffer(uint8_t* src, VEML6075_REGISTER_t startRegister, uint16_t len);
    VEML6075_error_t readI2CRegister(veml6075_t* dest, VEML6075_REGISTER_t registerAddress);
    VEML6075_error_t writeI2CRegister(veml6075_t data, VEML6075_REGISTER_t registerAddress);

    /* Flag manipulation inlines */
    inline uint16_t _veml_flags() {                return _flags;           };
    inline bool _veml_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _veml_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _veml_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _veml_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};


#endif  // __VEML6075_DRIVER_H_
