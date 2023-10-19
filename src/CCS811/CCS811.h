/*
* This file started out as a SparkFun driver. The original license is preserved
*   below.
* https://github.com/adafruit/Adafruit_CCS811
*
*
*
*
*
*
* Refactor log:
*   Initial import from AF driver. Broken. Won't compile. Header is merged.
*                                                  ---J. Ian Lindsay  2021.09.03
*/

/*
The MIT License (MIT)

Copyright (c) 2017 Adafruit Industries

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef LIB_ADAFRUIT_CCS811_H
#define LIB_ADAFRUIT_CCS811_H

#include <AbstractPlatform.h>
#include <FlagContainer.h>
#include "BusQueue/I2CAdapter.h"

/* Class flags */
#define CCS811_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define CCS811_FLAG_INITIALIZED      0x0004  // Registers are initialized.
#define CCS811_FLAG_ENABLED          0x0008  // Device is measuring.
#define CCS811_FLAG_CAL_DATA_READ    0x0020  // Calibration data read.
#define CCS811_FLAG_READ_IN_FLIGHT   0x0040  // Presently waiting on read.
#define CCS811_FLAG_DATA_FRESH       0x0080  // Data is fresh.


#define CCS811_ADDRESS          0x5A
#define CCS811_HW_ID_CODE       0x81
#define CCS811_REF_RESISTOR   100000

/* Registers */
enum class CCS811Reg : uint8_t {
  STATUS          = 0x00,
  MEAS_MODE       = 0x01,
  ALG_RESULT_DATA = 0x02,
  RAW_DATA        = 0x03,
  ENV_DATA        = 0x05,
  NTC             = 0x06,
  THRESHOLDS      = 0x10,
  BASELINE        = 0x11,
  HW_ID           = 0x20,
  HW_VERSION      = 0x21,
  FW_BOOT_VERSION = 0x23,
  FW_APP_VERSION  = 0x24,
  ERROR_ID        = 0xE0,
  SW_RESET        = 0xFF
};

/* Bootloader registers */
enum class CCS811BootReg : uint8_t {
  APP_ERASE  = 0xF1,
  APP_DATA   = 0xF2,
  APP_VERIFY = 0xF3,
  APP_START  = 0xF4
};

enum class CCS811DriveMode : uint8_t {
  DM_IDLE  = 0x00,
  DM_1SEC  = 0x01,
  DM_10SEC = 0x02,
  DM_60SEC = 0x03,
  DM_250MS = 0x04,
};


class CCS811 : public I2CDevice {
  public:
    CCS811(const uint8_t ADDR) : I2CDevice(ADDR) {};
    ~CCS811();

    bool init();
    int8_t poll();
    int8_t refresh();

    void printDebug(StringBuilder*);

    inline bool  devFound() {       return _flags.value(CCS811_FLAG_DEVICE_PRESENT);  };
    inline bool  enabled() {        return _flags.value(CCS811_FLAG_ENABLED);         };
    inline bool  initialized() {    return _flags.value(CCS811_FLAG_INITIALIZED);     };
    inline bool  calibrated() {     return _flags.value(CCS811_FLAG_CAL_DATA_READ);   };

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    void setEnvironmentalData(float humidity, float temperature);
    uint16_t getBaseline();
    void setBaseline(uint16_t baseline);

    // calculate temperature based on the NTC register
    double calculateTemperature();

    void setThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis = 50);

    void SWReset();

    void setDriveMode(uint8_t mode);
    void enableInterrupt();
    void disableInterrupt();

    /**************************************************************************/
    /*!
        @brief  returns the stored total volatile organic compounds measurement.
       This does does not read the sensor. To do so, call readData()
        @returns TVOC measurement as 16 bit integer
    */
    /**************************************************************************/
    uint16_t getTVOC() { return _TVOC; }

    /**************************************************************************/
    /*!
        @brief  returns the stored estimated carbon dioxide measurement. This does
       does not read the sensor. To do so, call readData()
        @returns eCO2 measurement as 16 bit integer
    */
    /**************************************************************************/
    uint16_t geteCO2() { return _eCO2; }

    /**************************************************************************/
    /*!
        @brief  returns the "Current Selected" in uA.
       This does does not read the sensor. To do so, call readData()
        @returns "Current Selected" in uA as 16 bit integer
    */
    /**************************************************************************/
    uint16_t getCurrentSelected() { return _currentSelected; }

    /**************************************************************************/
    /*!
        @brief  returns the raw ADC reading. This does
       does not read the sensor. To do so, call readData()
        @returns raw ADC reading as 16 bit integer
    */
    /**************************************************************************/
    uint16_t getRawADCreading() { return _rawADCreading; }

    /**************************************************************************/
    /*!
        @brief  set the temperature compensation offset for the device. This is
       needed to offset errors in NTC measurements.
        @param offset the offset to be added to temperature measurements.
    */
    /**************************************************************************/
    void setTempOffset(float offset) { _tempOffset = offset; }

    // check if data is available to be read
    bool available();
    uint8_t readData();

    bool checkError();


  private:
    float    _tempOffset      = 0.0;
    uint32_t _last_read       = 0;
    uint16_t _TVOC            = 0;
    uint16_t _eCO2            = 0;
    uint16_t _currentSelected = 0;
    uint16_t _rawADCreading   = 0;
    FlagContainer16 _flags;

    void write8(byte reg, byte value);
    void write16(byte reg, uint16_t value);
    uint8_t read8(byte reg);

    void read(uint8_t reg, uint8_t *buf, uint8_t num);
    void write(uint8_t reg, uint8_t *buf, uint8_t num);

    /*=========================================================================
            REGISTER BITFIELDS
        -----------------------------------------------------------------------*/
    // The status register
    struct status {

      /* 0: no error
       *  1: error has occurred
       */
      uint8_t ERROR : 1;

      // reserved : 2

      /* 0: no samples are ready
       *  1: samples are ready
       */
      uint8_t DATA_READY : 1;
      uint8_t APP_VALID : 1;

      // reserved : 2

      /* 0: boot mode, new firmware can be loaded
       *  1: application mode, can take measurements
       */
      uint8_t FW_MODE : 1;

      void set(uint8_t data) {
        ERROR = data & 0x01;
        DATA_READY = (data >> 3) & 0x01;
        APP_VALID = (data >> 4) & 0x01;
        FW_MODE = (data >> 7) & 0x01;
      }
    };
    status _status;

    // measurement and conditions register
    struct meas_mode {
      // reserved : 2

      /* 0: interrupt mode operates normally
  *  1: Interrupt mode (if enabled) only asserts the nINT signal (driven low) if
  the new ALG_RESULT_DATA crosses one of the thresholds set in the THRESHOLDS
  register by more than the hysteresis value (also in the THRESHOLDS register)
  */
      uint8_t INT_THRESH : 1;

      /* 0: int disabled
  *  1: The nINT signal is asserted (driven low) when a new sample is ready in
                      ALG_RESULT_DATA. The nINT signal will stop being driven low
  when ALG_RESULT_DATA is read on the I²C interface.
  */
      uint8_t INT_DATARDY : 1;

      uint8_t DRIVE_MODE : 3;

      uint8_t get() {
        return (INT_THRESH << 2) | (INT_DATARDY << 3) | (DRIVE_MODE << 4);
      }
    };
    meas_mode _meas_mode;

    struct error_id {
      /* The CCS811 received an I²C write request addressed to this station but
         with invalid register address ID */
      uint8_t WRITE_REG_INVALID : 1;

      /* The CCS811 received an I²C read request to a mailbox ID that is invalid
       */
      uint8_t READ_REG_INVALID : 1;

      /* The CCS811 received an I²C request to write an unsupported mode to
              MEAS_MODE */
      uint8_t MEASMODE_INVALID : 1;

      /* The sensor resistance measurement has reached or exceeded the maximum
              range */
      uint8_t MAX_RESISTANCE : 1;

      /* The Heater current in the CCS811 is not in range */
      uint8_t HEATER_FAULT : 1;

      /*  The Heater voltage is not being applied correctly */
      uint8_t HEATER_SUPPLY : 1;

      void set(uint8_t data) {
        WRITE_REG_INVALID = data & 0x01;
        READ_REG_INVALID = (data & 0x02) >> 1;
        MEASMODE_INVALID = (data & 0x04) >> 2;
        MAX_RESISTANCE = (data & 0x08) >> 3;
        HEATER_FAULT = (data & 0x10) >> 4;
        HEATER_SUPPLY = (data & 0x20) >> 5;
      }
    };
    error_id _error_id;
};

#endif    // LIB_ADAFRUIT_CCS811_H
