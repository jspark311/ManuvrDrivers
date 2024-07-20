/*
File:   ManuvrDrivers.h
Author: J. Ian Lindsay

This header file is meant to work around Arduino's strange inclusion behavior.
Other build systems may sensibly use this header to include the entire driver
  library that is verified working. But will probably get better build times
  by including only the drivers being used by the project.

NOTE: This file does not have header-guards. Each driver is responsible for
  implementing inclusion protections as it sees fit. No driver in this repo
  should ever include this file.
*/

/*** Commodity hardware *******************************************************/
#include "ShiftRegister/ShiftRegister.h"   // Drivers for SPI-connected shift-registers.
#include "I2CEEPROM/I2CEEPROM.h"           // Common driver for many small I2C EEPROMS.
#include "GPIOPatterns/GPIButton/GPIButton.h"  // Push-buttons connected to GPIO pins.


/*** Specific hardware drivers ************************************************/
/* Vishay semiconductors */
#include "VEML6075/VEML6075.h"  // UVA/UVB light sensor (I2C)

/* AMS/OSRAM */
#include "TSL2561/TSL2561.h"    // IR corrected LUX sensor (I2C)

/* Linear Technology */
#include "LTC294x/LTC294x.h"    // Battery gas guage (I2C)

/* Bosch Sensortech */
#include "BME280/BME280.h"      // BMx280 humidity and pressure sensor (I2C and SPI)

/* Texas Instruments */
#include "TMP102/TMP102.h"      // A low-power temperature sensor (I2C)
#include "BQ24155/BQ24155.h"    // Lithium battery charge controller (I2C)

/* Analog Devices */
#include "ADG2128/ADG2128.h"    // 8x12 split-rail analog switch (I2C)

/* Solomon Systech */
#include "SSD13xx/SSD13xx.h"    // SSD13xx family of OLED displays (I2C and SPI)

/* Panasonic */
#include "AMG88xx/AMG88xx.h"    // GridEYE family of thermopile sensors (I2C)

/* Semtech */
#include "SX127x/SX127x.h"      // LORA radio module (SPI)
#include "SX1503/SX1503.h"      // GPIO expander with PLD feature (I2C)
#include "SX8634/SX8634.h"      // Touch sensor with GPIO and NVM (I2C)

/* Dallas/Maxim */
#include "DS1881/DS1881.h"      // Digital volume control with NVM (I2C)

/* Microchip */
#include "MCP356x/MCP356x.h"    // 24-bit multi-channel ADC (SPI)
#include "MCP4728/MCP4728.h"    // 12-bit 4-channel DAC with NVM storage (I2C)
#include "MCP23x17/MCP23x17.h"  // 16-bit GPIO expanders (I2C and SPI)
#include "PAC195x/PAC195x.h"    // A flexible multi-channel power monitor

/* ST Microelectronics */
#include "LSM6DSOX/LSM6DSOX.h"  // 6-DoF IMU (SPI)
#include "VL53L0X/VL53L0X.h"    // ToF distance sensor with 2m range (I2C)


/*** High-level composite hardware ********************************************/
#include "AudemeMOVI/AudemeMOVI.h"  // A manager driver for Audeme's speech board


/*** Drivers that are broken, unported, unfinished, etc. **********************/
//#include "MAX7219/MAX7219.h"
