/*
*/

#include <stdint.h>
#include <AbstractPlatform.h>

#ifndef __LSM9DS1_TYPES_H__
#define __LSM9DS1_TYPES_H__


/* We use this struct to map between update rates and timestamp deltas. */
typedef struct {
  const float hertz;      // Frequency
  const float ts_delta;   // Period (in seconds)
} UpdateRate2Hertz;


/* We use the struct to map between scales and error-rates. */
typedef struct {
  const uint16_t scale;     // This is the maximum magnatude of the sensor reading at the given gain.
  const float    per_lsb;   // Each LSB counts for this many of whatever unit.
  const float    error;     // Given in the sensor's native unit. Each reading at this scale has this error.
} GainErrorMap;


#define MAXIMUM_GAIN_INDEX_MAG  4
#define MAXIMUM_GAIN_INDEX_ACC  5
#define MAXIMUM_GAIN_INDEX_GYR  3
#define MAXIMUM_RATE_INDEX_MAG  8
#define MAXIMUM_RATE_INDEX_AG   7


/*
* Internally-used register IDs. We need this abstraction because the addresses
*   within a given package are not disjoint sets.
* The order corrosponds to the order of the named register's occurance in the device.
* 16-bit registers are merged into a single ID.
*
* NOTE: These values are NOT register addresses, they are indicies.
*         Magnetometer registers occur first.
* NOTE: In order to get away with using the restricted list, the sensors MUST be
*   configured to allow multiple sequential register access. The means for doing
*   this vary between mag/ag.
*/
enum class LSM9DS1RegID : uint8_t {
  M_OFFSET_X         =  0,  // 16-bit offset registers
  M_OFFSET_Y         =  1,  // 16-bit offset registers  TODO: Condense into X?
  M_OFFSET_Z         =  2,  // 16-bit offset registers  TODO: Condense into X?
  M_WHO_AM_I         =  3,
  M_CTRL_REG1        =  4,
  M_CTRL_REG2        =  5,
  M_CTRL_REG3        =  6,
  M_CTRL_REG4        =  7,
  M_CTRL_REG5        =  8,
  M_STATUS_REG       =  9,
  M_DATA_X           = 10,  // 16-bit data registers
  M_DATA_Y           = 11,  // 16-bit data registers  TODO: Condense into X?
  M_DATA_Z           = 12,  // 16-bit data registers  TODO: Condense into X?
  M_INT_CFG          = 13,
  M_INT_SRC          = 14,
  M_INT_TSH          = 15,  // 16-bit threshold register
  AG_ACT_THS         = 16,
  AG_ACT_DUR         = 17,
  A_INT_GEN_CFG      = 18,
  A_INT_GEN_THS_X    = 19,  // 8-bit threshold registers
  A_INT_GEN_THS_Y    = 20,  // 8-bit threshold registers  TODO: Condense into X?
  A_INT_GEN_THS_Z    = 21,  // 8-bit threshold registers  TODO: Condense into X?
  A_INT_GEN_DURATION = 22,
  G_REFERENCE        = 23,
  AG_INT1_CTRL       = 24,
  AG_INT2_CTRL       = 25,
  AG_WHO_AM_I        = 26,
  G_CTRL_REG1        = 27,
  G_CTRL_REG2        = 28,
  G_CTRL_REG3        = 29,
  G_ORIENT_CFG       = 30,
  G_INT_GEN_SRC      = 31,
  AG_DATA_TEMP       = 32,  // 16-bit temperature register (11-bit)
  AG_STATUS_REG      = 33,
  G_DATA_X           = 34,  // 16-bit gyro data registers
  G_DATA_Y           = 35,  // 16-bit gyro data registers  TODO: Condense into X?
  G_DATA_Z           = 36,  // 16-bit gyro data registers  TODO: Condense into X?
  AG_CTRL_REG4       = 37,
  A_CTRL_REG5        = 38,
  A_CTRL_REG6        = 39,
  A_CTRL_REG7        = 40,
  AG_CTRL_REG8       = 41,
  AG_CTRL_REG9       = 42,
  AG_CTRL_REG10      = 43,
  A_INT_GEN_SRC      = 44,
  AG_STATUS_REG_ALT  = 45,
  A_DATA_X           = 46,  // 16-bit accelerometer data registers
  A_DATA_Y           = 47,  // 16-bit accelerometer data registers  TODO: Condense into X?
  A_DATA_Z           = 48,  // 16-bit accelerometer data registers  TODO: Condense into X?
  AG_FIFO_CTRL       = 49,
  AG_FIFO_SRC        = 50,
  G_INT_GEN_CFG      = 51,
  G_INT_GEN_THS_X    = 52,  // 16-bit threshold registers
  G_INT_GEN_THS_Y    = 53,  // 16-bit threshold registers  TODO: Condense into X?
  G_INT_GEN_THS_Z    = 54,  // 16-bit threshold registers  TODO: Condense into X?
  G_INT_GEN_DURATION = 55,
  INVALID            = 255
};


#endif // __LSM9DS1_TYPES_H__
