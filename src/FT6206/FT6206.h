/*
* This file started out as an adafruit driver. I have mutated it.
*
*
*
*
*
*
* Refactor log:
*   Initial import. Broken. Won't compile. Header is merged.
*                                                  ---J. Ian Lindsay  2021.09.09
*/

#include <AbstractPlatform.h>
#include <FlagContainer.h>
#include <I2CAdapter.h>

#ifndef ADAFRUIT_FT6206_LIBRARY
#define ADAFRUIT_FT6206_LIBRARY

#define FT62XX_ADDR 0x38           //!< I2C address
#define FT62XX_G_FT5201ID 0xA8     //!< FocalTech's panel ID
#define FT62XX_REG_NUMTOUCHES 0x02 //!< Number of touch points

#define FT62XX_NUM_X 0x33 //!< Touch X position
#define FT62XX_NUM_Y 0x34 //!< Touch Y position

#define FT62XX_REG_MODE 0x00        //!< Device mode, either WORKING or FACTORY
#define FT62XX_REG_CALIBRATE 0x02   //!< Calibrate mode
#define FT62XX_REG_WORKMODE 0x00    //!< Work mode
#define FT62XX_REG_FACTORYMODE 0x40 //!< Factory mode
#define FT62XX_REG_THRESHHOLD 0x80  //!< Threshold for touch detection
#define FT62XX_REG_POINTRATE 0x88   //!< Point rate
#define FT62XX_REG_FIRMVERS 0xA6    //!< Firmware version
#define FT62XX_REG_CHIPID 0xA3      //!< Chip selecting
#define FT62XX_REG_VENDID 0xA8      //!< FocalTech's panel ID

#define FT62XX_VENDID 0x11  //!< FocalTech's panel ID
#define FT6206_CHIPID 0x06  //!< Chip selecting
#define FT6236_CHIPID 0x36  //!< Chip selecting
#define FT6236U_CHIPID 0x64 //!< Chip selecting

// calibrated for Adafruit 2.8" ctp screen
#define FT62XX_DEFAULT_THRESHOLD 128 //!< Default threshold for touch detection

/**************************************************************************/
/*!
@brief  Helper class that stores a TouchScreen Point with x, y, and z
coordinates, for easy math/comparison
*/
/**************************************************************************/
class TS_Point {
public:
  TS_Point();
  TS_Point(int16_t x, int16_t y, int16_t z);

  bool operator==(TS_Point);
  bool operator!=(TS_Point);

  int16_t x; /*!< X coordinate */
  int16_t y; /*!< Y coordinate */
  int16_t z; /*!< Z coordinate (often used for pressure) */
};

/**************************************************************************/
/*!
@brief  Class that stores state and functions for interacting with FT6206
capacitive touch chips
*/
/**************************************************************************/
class FT6206 : public I2CDevice {
  public:
    FT6206() {};
    ~FT6206() {};

    int8_t init(uint8_t thresh = FT62XX_DEFAULT_THRESHOLD);
    uint8_t touched();
    TS_Point getPoint(uint8_t n = 0);
    // void autoCalibrate();

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    void printDebug(StringBuilder*);


  private:
    uint8_t touches = 0;
    uint16_t touchX[2];
    uint16_t touchY[2];
    uint16_t touchID[2];

    void writeRegister8(uint8_t reg, uint8_t val);
    uint8_t readRegister8(uint8_t reg);
    void readData();
};

#endif // ADAFRUIT_FT6206_LIBRARY
