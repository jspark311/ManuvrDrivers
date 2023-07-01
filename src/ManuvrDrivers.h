/*
File:   ManuvrDrivers.h
Author: J. Ian Lindsay

This header file is meant to work around Arduino's strange inclusion behavior.
Other build systems may sensibly use this header to include the entire driver
  library that is verified working. But will probably get better build times
  by including only the drivers being used by the project.
*/

#include "SSD13xx/SSD13xx.h"
#include "I2CEEPROM/I2CEEPROM.h"
#include "ShiftRegister/ShiftRegister.h"
#include "VEML6075/VEML6075.h"
#include "BME280/BME280.h"
#include "AMG88xx/AMG88xx.h"
#include "TSL2561/TSL2561.h"
#include "MCP356x/MCP356x.h"
#include "MCP4728/MCP4728.h"
#include "SX127x/SX127x.h"
#include "SX1503/SX1503.h"
#include "SX8634/SX8634.h"
#include "DS1881/DS1881.h"
#include "ADG2128/ADG2128.h"
#include "TMP102/TMP102.h"
#include "VL53L0X/VL53L0X.h"
#include "BQ24155/BQ24155.h"
#include "LTC294x/LTC294x.h"
#include "PAC195x/PAC195x.h"
#include "AudemeMOVI/AudemeMOVI.h"

// TODO: These drivers were broken, unported, unfinished, etc.
//#include "MAX7219/MAX7219.h"
