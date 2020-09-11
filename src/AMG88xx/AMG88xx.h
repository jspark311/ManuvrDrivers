/*
* This file started out as a SparkFun driver. I have mutated it.
*
* Refactor log:
* Before:   914 lines
*           Frame read took 43ms.
*           Sketch uses 152304 bytes of program storage space
*           Global variables use 171616 bytes of dynamic memory
*
* After:    596 lines
*           Frame read took 15ms.
*           Sketch uses 152960 bytes of program storage space
*           Global variables use 172256 bytes of dynamic memory
*
* Culled much useless whitespace and replicated code.
* Unit conversion is now done with a flag, and determines units for ALL
*   interchange with the class.
* Added flag member to track various boolean conditions in the class and the
*   hardware, rather than waste time in I/O for requests we ought to be able to
*   answer immediately.
* Error returns from functions were added, as well as a function for supplying
*   a TwoWire pointer. Data operations are now guarded by hardware error checks.
* Implemented handling of the IRQ pin.
* Added a function to read an entire frame in a more efficient manner. This
*   demanded the addition of a local shadow for the frame. But if we can't spare
*   128 bytes, we have deeper problems.
* Encapsulated data is now condensed and aligned.
* Added a temporal read marker so that polling-generated I/O can be minimized
*   if the IRQ pin isn't available.
* Unified member name convention. Encapsulated members now start with an
*   underscore.
*                                                  ---J. Ian Lindsay  2020.02.03
* Refactoring for I2CAdapter and async. This removes the local concern over
*   buffer sizes. Added register shadows. Storage requirement increased a bit.
* Added software reset.
* Register I/O is no longer implied for state accessors.
* Register I/O is no longer replicated across functions that do the same thing.
* Scattered commentary improvement.
* Finished refresh().
* Improved the support for pixel interrupts. Not finished yet.
*                                                  ---J. Ian Lindsay  2020.06.27
*/

/*
  This is a library written for the Panasonic Grid-EYE AMG88
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14568

  Written by Nick Poole @ SparkFun Electronics, January 11th, 2018

  The GridEYE from Panasonic is an 8 by 8 thermopile array capable
  of detecting temperature remotely at 64 discrete points.

  This library handles communication with the GridEYE and provides
  methods for manipulating temperature registers in Celsius,
  Fahrenheit and raw values.

  https://github.com/sparkfun/SparkFun_GridEYE_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <AbstractPlatform.h>
#include <I2CAdapter.h>

#ifndef __AMG88XX_DRIVER_H_
#define __AMG88XX_DRIVER_H_

/* Class flags */
#define GRIDEYE_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define GRIDEYE_FLAG_PINS_CONFIGURED  0x0002  // Low-level pin setup is complete.
#define GRIDEYE_FLAG_INITIALIZED      0x0004  // Registers are initialized.
#define GRIDEYE_FLAG_ENABLED          0x0008  // Device is measuring.
#define GRIDEYE_FLAG_10FPS            0x0010  // 10Hz update rate if true. 1Hz if not.
#define GRIDEYE_FLAG_FREEDOM_UNITS    0x0020  // Units in Fahrenheit if true. Celcius if not.
#define GRIDEYE_FLAG_HW_AVERAGING     0x0040  // Use the sensor's hardware averaging?
#define GRIDEYE_FLAG_FRAME_UPDATED    0x0080  // The frame has been refreshed.

#define GRIDEYE_FLAG_RESET_MASK       (GRIDEYE_FLAG_DEVICE_PRESENT | GRIDEYE_FLAG_PINS_CONFIGURED)

/*
* Registers
* NOTE: These are indicies. Not real register addresses. Only order is reflected.
*/
enum class AMG88XXRegID : uint8_t {
  POWER_CONTROL       = 0x00,
  RESET               = 0x01,
  FRAMERATE           = 0x02,
  INT_CONTROL         = 0x03,
  STATUS              = 0x04,
  STATUS_CLEAR        = 0x05,
  AVERAGE             = 0x06,
  INT_LEVEL_UPPER_LSB = 0x07,
  INT_LEVEL_UPPER_MSB = 0x08,
  INT_LEVEL_LOWER_LSB = 0x09,
  INT_LEVEL_LOWER_MSB = 0x0A,
  INT_LEVEL_HYST_LSB  = 0x0B,
  INT_LEVEL_HYST_MSB  = 0x0C,
  THERMISTOR_LSB      = 0x0D,
  THERMISTOR_MSB      = 0x0E,
  RESERVED_AVERAGE    = 0x0F,
  INVALID             = 0xFF
};


#define INT_TABLE_REGISTER_INT0_START 0x10
#define TEMPERATURE_REGISTER_START    0x80


/*******************************************************************************
* Class definition
* Class defaults on construction:
*   - All temperatures in Celcius
*******************************************************************************/
class GridEYE : public I2CDevice {
  public:
    GridEYE(uint8_t addr = 0x69, uint8_t irq_pin = 255);
    ~GridEYE();

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    int8_t init(I2CAdapter* bus);
    int8_t poll();
    int8_t reset();
    int8_t refresh();

    inline bool devFound() {         return _amg_flag(GRIDEYE_FLAG_DEVICE_PRESENT);  };
    inline bool enabled() {          return _amg_flag(GRIDEYE_FLAG_ENABLED);         };
    inline bool initialized() {      return _amg_flag(GRIDEYE_FLAG_INITIALIZED);     };
    inline bool frameReady() {       return _amg_flag(GRIDEYE_FLAG_FRAME_UPDATED);   };
    inline bool unitsFahrenheit() {  return _amg_flag(GRIDEYE_FLAG_FREEDOM_UNITS);   };
    inline void unitsFahrenheit(bool x) {  _amg_set_flag(GRIDEYE_FLAG_FREEDOM_UNITS, x); };

    float   getPixelTemperature(uint8_t pixel);
    inline int16_t getPixelRaw(uint8_t pixel) {   return (pixel < 64) ? _frame[pixel] : 0;  };

    float   getDeviceTemperature();
    int16_t getDeviceTemperatureRaw();

    int8_t setFramerate1FPS();
    int8_t setFramerate10FPS();
    inline bool isFramerate10FPS() {   return _amg_flag(GRIDEYE_FLAG_10FPS);  };

    int8_t enabled(bool);
    int8_t standby60seconds();
    int8_t standby10seconds();

    int8_t interruptPinEnable();
    int8_t interruptPinDisable();
    int8_t setInterruptModeAbsolute();
    int8_t setInterruptModeDifference();
    bool   interruptPinEnabled();

    bool   interruptFlagSet();
    bool   pixelTemperatureOutputOK();
    bool   deviceTemperatureOutputOK();
    int8_t clearPixelTemperatureOverflow();
    int8_t clearDeviceTemperatureOverflow();
    int8_t clearAllOverflow();
    int8_t clearAllStatusFlags();
    bool   pixelInterruptSet(uint8_t pixel);

    int8_t movingAverage(bool);
    inline bool movingAverage() {  return _amg_flag(GRIDEYE_FLAG_HW_AVERAGING);   };

    int8_t setUpperInterruptValue(float degrees);
    int8_t setUpperInterruptValueRaw(int16_t regValue);
    int8_t setLowerInterruptValue(float degrees);
    int8_t setLowerInterruptValueRaw(int16_t regValue);
    int8_t setInterruptHysteresis(float degrees);
    int8_t setInterruptHysteresisRaw(int16_t regValue);

    float   getUpperInterruptValue();
    int16_t getUpperInterruptValueRaw();
    float   getLowerInterruptValue();
    int16_t getLowerInterruptValueRaw();
    float   getInterruptHysteresis();
    int16_t getInterruptHysteresisRaw();


  private:
    const uint8_t _IRQ_PIN;
    uint16_t      _flags     = 0;
    uint32_t      _last_read = 0;
    int16_t       _frame[64];
    uint8_t       _shadows[16];
    uint8_t       _pixel_interrupts[8];
    I2CBusOp      _frame_read;

    int8_t  _ll_pin_init();

    /* Basal register access and utility fxn's */
    int8_t  _set_shadow_value(AMG88XXRegID, uint8_t val);
    uint8_t _get_shadow_value(AMG88XXRegID);
    int8_t  _write_register(AMG88XXRegID, uint8_t val);
    int8_t  _write_registers(AMG88XXRegID, uint16_t val);
    int8_t  _read_registers(AMG88XXRegID, uint8_t len);
    int8_t  _read_full_frame();
    int8_t _read_pixel_int_states();

    /* Conversion of units and data */
    float   _normalize_units_accepted(float deg);
    float   _normalize_units_returned(float deg);
    int16_t _dev_int16_to_float(int16_t temperature);
    int16_t _native_float_to_dev_int16(float temperature);

    /* Flag manipulation inlines */
    inline uint16_t _amg_flags() {                return _flags;           };
    inline bool _amg_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _amg_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _amg_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _amg_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };

    static AMG88XXRegID _reg_id_from_addr(const uint8_t reg_addr);
};

#endif   // __AMG88XX_DRIVER_H_
