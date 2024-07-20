/*
* This file started out as a driver by Tyler Glenn with additions of others
*   noted in his comment block below.
*
* Refactor log:
* Culled much useless whitespace, complexity, and implementations for SPI and
*   bit-banged i2c. But I left the door wide open for their re-addition when/if
*   needed. After doing this, all files were condensed into a nice neat
*   pair.
* Base class now has a protected constructor that is fully delegated.
* Fixed a bug that cause altitude reports to be in feet when meters was
*   requested.
* Error returns from functions were added, as well as a function for supplying
*   a TwoWire pointer. Data operations are now guarded by hardware error checks.
* Encapsulated data is now (mostly) condensed and aligned.
*                                                  ---J. Ian Lindsay  2020.02.04
*
* Added read timing member and poll() fxn. Timing value is still hard-coded. But
*   at least it takes the complexity out of the application.
* Data is now always read uniformly, and is cached in the class.
* Some style enforcement changes.
* Unit settings are now fully encapsulated, but there is no way to set them.
* Initialization and enabled flags are now being properly set and observed.
*                                                  ---J. Ian Lindsay  2020.02.06
*/

/*
BME280.h

This code records data from the BME280 sensor and provides an API.
This file is part of the Arduino BME280 library.
Copyright (C) 2016  Tyler Glenn

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Written: Dec 30 2015.
Last Updated: Oct 07 2017.

This code is licensed under the GNU LGPL and is open for ditrbution
and copying in accordance with the license.
This header must be included in any derived code or copies of the code.

Based on the data sheet provided by Bosch for the Bme280 environmental sensor,
calibration code based on algorithms providedBosch, some unit conversations courtesy
of www.endmemo.com, altitude equation courtesy of NOAA, and dew point equation
courtesy of Brian McNoldy at http://andrew.rsmas.miami.edu.
*/

#ifndef TG_BME_280_H
#define TG_BME_280_H

#include "AbstractPlatform.h"
#include "EnumeratedTypeCodes.h"
#include "FlagContainer.h"
#include "BusQueue/I2CAdapter.h"

/* Class flags */
#define BME280_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define BME280_FLAG_HAS_HUMIDITY     0x0002  // Low-level pin setup is complete.
#define BME280_FLAG_INITIALIZED      0x0004  // Registers are initialized.
#define BME280_FLAG_ENABLED          0x0008  // Device is measuring.
#define BME280_FLAG_USE_SPI          0x0010  // Enable the SPI interface.
#define BME280_FLAG_CAL_DATA_READ    0x0020  // Calibration data read.
#define BME280_FLAG_READ_IN_FLIGHT   0x0040  // Presently waiting on read.
#define BME280_FLAG_DATA_FRESH       0x0080  // Data is fresh.

enum class TempUnit : uint8_t {
  Celsius,
  Fahrenheit
};

enum class PresUnit : uint8_t {
  Pa,
  hPa,
  inHg,
  atm,
  bar,
  torr,
  psi
};

enum class LengthUnit : uint8_t {
  Meters,
  Feet
};

enum class BME280OSR : uint8_t {
  X1  = 1,
  X2  = 2,
  X4  = 3,
  X8  = 4,
  X16 = 5
};

enum class BME280Mode : uint8_t {
  Sleep  = 0,
  Forced = 1,
  Normal = 3
};

enum class BME280StandbyTime : uint8_t {
  StandbyTime_500us   = 0,
  StandbyTime_62500us = 1,
  StandbyTime_125ms   = 2,
  StandbyTime_250ms   = 3,
  StandbyTime_50ms    = 4,
  StandbyTime_1000ms  = 5,
  StandbyTime_10ms    = 6,
  StandbyTime_20ms    = 7
};

enum class BME280Filter : uint8_t {
  Off  = 0,
  X2   = 1,
  X4   = 2,
  X8   = 3,
  X16  = 4
};


struct BME280Settings {
  BME280Settings(
    uint8_t _bb,  // i2c addr, or CS pin
    BME280OSR _tosr       = BME280OSR::X1,
    BME280OSR _hosr       = BME280OSR::X1,
    BME280OSR _posr       = BME280OSR::X1,
    BME280Mode _mode      = BME280Mode::Forced,
    BME280StandbyTime _st = BME280StandbyTime::StandbyTime_1000ms,
    BME280Filter _filter  = BME280Filter::Off
  ):  BUS_BYTE(_bb),
      tempOSR(_tosr),
      humOSR(_hosr),
      presOSR(_posr),
      mode(_mode),
      standbyTime(_st),
      filter(_filter) {}

  uint8_t           BUS_BYTE;
  BME280OSR         tempOSR;
  BME280OSR         humOSR;
  BME280OSR         presOSR;
  BME280Mode        mode;
  BME280StandbyTime standbyTime;
  BME280Filter      filter;
  uint8_t padding;   // TODO: This can be dropped once the fields above are consolidated.
};



/// BME280 - Driver class for Bosch Bme280 sensor
/// Based on the data sheet provided by Bosch for
/// the Bme280 environmental sensor.
class BME280 {
  public:
    inline bool  devFound() {       return _flags.value(BME280_FLAG_DEVICE_PRESENT);  };
    inline bool  enabled() {        return _flags.value(BME280_FLAG_ENABLED);         };
    inline bool  initialized() {    return _flags.value(BME280_FLAG_INITIALIZED);     };
    inline bool  hasHumidity() {    return _flags.value(BME280_FLAG_HAS_HUMIDITY);    };
    inline bool  calibrated() {     return _flags.value(BME280_FLAG_CAL_DATA_READ);   };


    int8_t poll();
    int8_t refresh();

    inline float temp() {  return _air_temp;  };  // Temperature in real units.
    inline float pres() {  return _pressure;  };  // Pressure in real units.
    inline float hum() {   return _humidity;  };  // Humidity as a percentage.

    inline bool sleeping() {  return (0 == (_shadow_ctrl_mea & 0x03));  };


    // Calculate the altitude based on the pressure with the
    // specified units.
    float Altitude(float pressure, float seaLevelPressure = PRESSURE_AT_SEALEVEL); // Pressure given in Pa.

    // Convert current pressure to sea-level pressure, returns
    // Altitude (in meters), temperature in Celsius
    // return the equivalent pressure at sea level.
    // @deprecated
    float SealevelAlitude(float alitude, float temp, float pres);  // A: current altitude (meters).

    // Convert current pressure to equivalent sea-level pressure.
    // @param altitude in meters.
    // @param temp in Celsius.
    // @return the equivalent pressure at sea level.
    float EquivalentSeaLevelPressure(float altitude, float temp, float pressure);

    // Calculate the dew point based on the temperature and
    // humidity with the specified units.
    float DewPoint(float temp, float hum);

    const BME280Settings& getSettings() const;


  protected:
    BME280Settings m_settings;   // Main grouping of operational settings.
    uint8_t    m_dig[32];
    uint8_t    _shadow_sdat[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t    _shadow_id       = 0;
    uint8_t    _shadow_ctrl_hum = 0;
    uint8_t    _shadow_ctrl_mea = 0;
    uint8_t    _shadow_ctrl     = 0;

    float      _air_temp    = 0.0;
    float      _pressure    = 0.0;
    float      _humidity    = 0.0;
    float      _altitude    = 0.0;
    float      _dew_point   = 0.0;
    float      _esl_pres    = 0.0;
    uint32_t   _last_read   = 0;
    LengthUnit _unit_length = LengthUnit::Meters;
    TempUnit   _unit_temp   = TempUnit::Celsius;
    PresUnit   _unit_pres   = PresUnit::Pa;
    FlagContainer16 _flags;

    /* This constructor is only a delegate to an extending class. */
    BME280(const BME280Settings& settings);

    /* Mandatory overrides for register access. */
    virtual int8_t _write_register(uint8_t addr, uint8_t* data) =0;
    virtual int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length) =0;

    bool _priv_init();  // Write configuration to BME280, return true if successful.

    inline bool _useSPI() {     return _flags.value(BME280_FLAG_USE_SPI);     };


    // Read the data from the BME280 in the specified unit.
    bool _refresh_data();

    // Write the settings to the chip.
    bool WriteSettings();

    // Read the the trim data from the BME280, return true if
    // successful.
    bool ReadTrim();

    /* Convert the raw data and the trim into real units, return a float. */
    float CalculateTemperature(int32_t raw, int32_t& t_fine);
    float CalculateHumidity(int32_t raw, int32_t t_fine);
    float CalculatePressure(int32_t raw, int32_t t_fine);
};


//////////////////////////////////////////////////////////////////
/// BME280I2C - I2C Implementation of BME280.
class BME280I2C: public BME280, public I2CDevice {
  public:
    // Constructor used to create the class. All parameters have
    // default values.
    BME280I2C(const BME280Settings& settings);

    int8_t init();     // Method used to initialize the class.
    void printDebug(StringBuilder*);

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    // Write values to BME280 registers.
    virtual int8_t _write_register(uint8_t addr, uint8_t* data);
    // Read values from BME280 registers.
    virtual int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length);
};

#endif // TG_BME_280_H
