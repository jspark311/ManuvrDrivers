/*
* This file started out as a driver by Tyler Glenn. See header file for
*   my change list.
*                                                            ---J. Ian Lindsay
*/

/*
BME280.cpp

This code records data from the BME280 sensor and provides an API.
This file is part of the Arduino BME280 library.
Copyright (C) 2016   Tyler Glenn

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.   See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.   If not, see <http://www.gnu.org/licenses/>.

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

#include <math.h>
#include "BME280.h"

#define CTRL_HUM_ADDR          0xF2
#define CTRL_MEAS_ADDR         0xF4
#define CONFIG_ADDR            0xF5
#define PRESS_ADDR             0xF7
#define TEMP_ADDR              0xFA
#define HUM_ADDR               0xFD
#define TEMP_DIG_ADDR          0x88
#define PRESS_DIG_ADDR         0x8E
#define HUM_DIG_ADDR1          0xA1
#define HUM_DIG_ADDR2          0xE1
#define ID_ADDR                0xD0

#define TEMP_DIG_LENGTH        6
#define PRESS_DIG_LENGTH       18
#define HUM_DIG_ADDR1_LENGTH   1
#define HUM_DIG_ADDR2_LENGTH   7
#define DIG_LENGTH             32
#define SENSOR_DATA_LENGTH     8


/* Delegate constructor. */
BME280::BME280(const BME280Settings& settings) : m_settings(settings) {}


bool BME280::_priv_init() {
  bool ret = false;
  if (0 == _read_chip_id()) {
    if (ReadTrim()) {
      ret = WriteSettings();
      _baro_set_flag(BME280_FLAG_INITIALIZED, ret);
      _baro_set_flag(BME280_FLAG_ENABLED, ret);
    }
  }
  return ret;
}


/*
* Poll the class for updates.
* Returns...
*   -3 if not initialized and enabled.
*   -1 if data needed to be read, but doing so failed.
*   0  if nothing needs doing.
*   1  if data was read and is waiting.
*/
int8_t BME280::poll() {
  int8_t ret = -3;
  if (initialized() && enabled()) {
    uint32_t now = millis();
    uint32_t r_interval = 100;
    if ((now - _last_read) >= r_interval) {
      ret = _refresh_data() ? 1 : -1;
    }
    else {
      ret = 0;
    }
  }
  return ret;
}


int8_t BME280::_read_chip_id() {
  int8_t ret = -1;
  _baro_clear_flag(BME280_FLAG_HAS_HUMIDITY);
  uint8_t id = 0;
  _read_registers(ID_ADDR, &id, 1);
  switch (id) {  // No breaks
    case 0x60:  _baro_set_flag(BME280_FLAG_HAS_HUMIDITY);  // BME280 (With humidity)
    case 0x58:  ret = 0;      // BMP280 (No humidity)
    default:    break;
  }
  _baro_set_flag(BME280_FLAG_DEVICE_PRESENT, (0 == ret));
  return ret;
}


bool BME280::WriteSettings() {
  bool ret = false;
  uint8_t ctrlHum, ctrlMeas, config;

  CalculateRegisters(ctrlHum, ctrlMeas, config);
  if (_write_register(CTRL_HUM_ADDR, ctrlHum)) {
    if (_write_register(CTRL_MEAS_ADDR, ctrlMeas)) {
      if (_write_register(CONFIG_ADDR, config)) {
        ret = true;
      }
    }
  }
  return ret;
}


bool BME280::setSettings(const BME280Settings& settings) {
  m_settings = settings;
  return WriteSettings();
}


const BME280Settings& BME280::getSettings() const {
  return m_settings;
}


void BME280::CalculateRegisters(uint8_t& ctrlHum, uint8_t& ctrlMeas, uint8_t& config) {
  // ctrl_hum register. (ctrl_hum[2:0] = Humidity oversampling rate.)
  ctrlHum = (uint8_t)m_settings.humOSR;
  // ctrl_meas register. (ctrl_meas[7:5] = temperature oversampling rate, ctrl_meas[4:2] = pressure oversampling rate, ctrl_meas[1:0] = mode.)
  ctrlMeas = ((uint8_t)m_settings.tempOSR << 5) | ((uint8_t)m_settings.presOSR << 2) | (uint8_t)m_settings.mode;
  // config register. (config[7:5] = standby time, config[4:2] = filter, ctrl_meas[0] = spi enable.)
  config = ((uint8_t)m_settings.standbyTime << 5) | ((uint8_t)m_settings.filter << 2) | _useSPI() ? 1 : 0;
}


bool BME280::ReadTrim() {
   uint8_t ord(0);
   bool success = true;

   // Temp. Dig
   success &= _read_registers(TEMP_DIG_ADDR, &m_dig[ord], TEMP_DIG_LENGTH);
   ord += TEMP_DIG_LENGTH;

   // Pressure Dig
   success &= _read_registers(PRESS_DIG_ADDR, &m_dig[ord], PRESS_DIG_LENGTH);
   ord += PRESS_DIG_LENGTH;

   // Humidity Dig 1
   success &= _read_registers(HUM_DIG_ADDR1, &m_dig[ord], HUM_DIG_ADDR1_LENGTH);
   ord += HUM_DIG_ADDR1_LENGTH;

   // Humidity Dig 2
   success &= _read_registers(HUM_DIG_ADDR2, &m_dig[ord], HUM_DIG_ADDR2_LENGTH);
   ord += HUM_DIG_ADDR2_LENGTH;

   return success && ord == DIG_LENGTH;
}


float BME280::CalculateTemperature(int32_t raw, int32_t& t_fine) {
  // Code based on calibration algorthim provided by Bosch.
  int32_t var1, var2, final;
  uint16_t dig_T1 = (m_dig[1] << 8) | m_dig[0];
  int16_t   dig_T2 = (m_dig[3] << 8) | m_dig[2];
  int16_t   dig_T3 = (m_dig[5] << 8) | m_dig[4];
  var1 = ((((raw >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((raw >> 4) - ((int32_t)dig_T1)) * ((raw >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  final = (t_fine * 5 + 128) >> 8;
  return _unit_temp == TempUnit::Celsius ? final/100.0 : final/100.0*9.0/5.0 + 32.0;
}


float BME280::CalculateHumidity(int32_t raw, int32_t t_fine) {
   // Code based on calibration algorthim provided by Bosch.
   int32_t var1;
   uint8_t   dig_H1 =   m_dig[24];
   int16_t dig_H2 = (m_dig[26] << 8) | m_dig[25];
   uint8_t   dig_H3 =   m_dig[27];
   int16_t dig_H4 = (m_dig[28] << 4) | (0x0F & m_dig[29]);
   int16_t dig_H5 = (m_dig[30] << 4) | ((m_dig[29] >> 4) & 0x0F);
   int8_t   dig_H6 =   m_dig[31];

   var1 = (t_fine - ((int32_t)76800));
   var1 = (((((raw << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * var1)) +
   ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)dig_H6)) >> 10) * (((var1 *
   ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
   ((int32_t)dig_H2) + 8192) >> 14));
   var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
   var1 = (var1 < 0 ? 0 : var1);
   var1 = (var1 > 419430400 ? 419430400 : var1);
   return ((uint32_t)(var1 >> 12))/1024.0;
}


float BME280::CalculatePressure(int32_t raw, int32_t t_fine) {
   // Code based on calibration algorthim provided by Bosch.
   int64_t var1, var2, pressure;
   float final;

   uint16_t dig_P1 = (m_dig[7]   << 8) | m_dig[6];
   int16_t   dig_P2 = (m_dig[9]   << 8) | m_dig[8];
   int16_t   dig_P3 = (m_dig[11] << 8) | m_dig[10];
   int16_t   dig_P4 = (m_dig[13] << 8) | m_dig[12];
   int16_t   dig_P5 = (m_dig[15] << 8) | m_dig[14];
   int16_t   dig_P6 = (m_dig[17] << 8) | m_dig[16];
   int16_t   dig_P7 = (m_dig[19] << 8) | m_dig[18];
   int16_t   dig_P8 = (m_dig[21] << 8) | m_dig[20];
   int16_t   dig_P9 = (m_dig[23] << 8) | m_dig[22];

   var1 = (int64_t)t_fine - 128000;
   var2 = var1 * var1 * (int64_t)dig_P6;
   var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
   var2 = var2 + (((int64_t)dig_P4) << 35);
   var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
   var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
   if (var1 == 0) { return NAN; }                                                         // Don't divide by zero.
   pressure   = 1048576 - raw;
   pressure = (((pressure << 31) - var2) * 3125)/var1;
   var1 = (((int64_t)dig_P9) * (pressure >> 13) * (pressure >> 13)) >> 25;
   var2 = (((int64_t)dig_P8) * pressure) >> 19;
   pressure = ((pressure + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
   final = ((uint32_t)pressure)/256.0;

   // Conversion units courtesy of www.endmemo.com.
   switch (_unit_pres) {
      case PresUnit::hPa: /* hPa */
         final /= 100.0;
         break;
      case PresUnit::inHg: /* inHg */
         final /= 3386.3752577878;          /* final pa * 1inHg/3386.3752577878Pa */
         break;
      case PresUnit::atm: /* atm */
         final /= 101324.99766353; /* final pa * 1 atm/101324.99766353Pa */
         break;
      case PresUnit::bar: /* bar */
         final /= 100000.0;               /* final pa * 1 bar/100kPa */
         break;
      case PresUnit::torr: /* torr */
         final /= 133.32236534674;            /* final pa * 1 torr/133.32236534674Pa */
         break;
      case PresUnit::psi: /* psi */
         final /= 6894.744825494;   /* final pa * 1psi/6894.744825494Pa */
         break;
      default: /* Pa (case: 0) */
         break;
   }
   return final;
}


bool BME280::_refresh_data() {
  int32_t data[8];
  uint8_t buffer[SENSOR_DATA_LENGTH];
  int32_t t_fine;
  // For forced mode we need to write the mode to BME280 register before reading
  bool ret = (m_settings.mode == BME280Mode::Forced) ? WriteSettings() : true;

  // Registers are in order. So we can start at the pressure register and read 8 bytes.
  if (ret) {
    ret = _read_registers(PRESS_ADDR, buffer, SENSOR_DATA_LENGTH);
    for(int i = 0; i < SENSOR_DATA_LENGTH; ++i) {
      data[i] = static_cast<int32_t>(buffer[i]);
    }
    if (ret) {
      _last_read = millis();
      uint32_t rawPressure = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
      uint32_t rawTemp     = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
      uint32_t rawHumidity = (data[6] << 8)  | data[7];
      _air_temp = CalculateTemperature(rawTemp,  t_fine);
      _pressure = CalculatePressure(rawPressure, t_fine);
      _humidity = CalculateHumidity(rawHumidity, t_fine);
    }
  }
  return ret;
}



/****************************************************************/


float BME280::Altitude(float pressure, float seaLevelPressure) {
  // Equations courtesy of NOAA;
  float altitude = NAN;
  if (!isnan(pressure) && !isnan(seaLevelPressure)){
    altitude = 1000.0 * (seaLevelPressure - pressure) / 3386.3752577878;
  }
  return (LengthUnit::Meters == _unit_length) ? altitude : altitude * 0.3048;
}


float BME280::SealevelAlitude(float A, float T, float P) {
  return (P / pow(1-((0.0065 *A) / (T + (0.0065 *A) + 273.15)),5.257));
}


float BME280::EquivalentSeaLevelPressure(float altitude, float temp, float pres) {
  return (pres / pow(1-((0.0065 *altitude) / (temp + (0.0065 *altitude) + 273.15)),5.257));
}


/*
* Equations courtesy of Brian McNoldy from http://andrew.rsmas.miami.edu;
*/
float BME280::DewPoint(float temp, float hum) {
  float dewPoint = NAN;
  if (!isnan(temp) && !isnan(hum)) {
    if (TempUnit::Celsius == _unit_temp) {
      dewPoint = 243.04 * (log(hum/100.0) + ((17.625 * temp)/(243.04 + temp))) /(17.625 - log(hum/100.0) - ((17.625 * temp)/(243.04 + temp)));
    }
    else {
      float ctemp = (temp - 32.0) / 1.8;
      dewPoint = 243.04 * (log(hum/100.0) + ((17.625 * ctemp)/(243.04 + ctemp))) /(17.625 - log(hum/100.0) - ((17.625 * ctemp)/(243.04 + ctemp)));
      dewPoint = dewPoint * 1.8 + 32.0;
    }
  }
  return dewPoint;
}


/*******************************************************************************
* Members and logic specific to the i2c package
*******************************************************************************/
BME280I2C::BME280I2C(const BME280Settings& settings) : BME280(settings), I2CDevice(settings.BUS_BYTE) {}

/**
*
*/
int8_t BME280I2C::_write_register(uint8_t addr, uint8_t data) {
  int8_t ret = -2;
  if (devFound()) {
    //I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    //if (nullptr != op) {
    //  op->dev_addr = _dev_addr;
    //  op->sub_addr = addr;
    //  op->setBuffer(&_shadows[reg_idx], 2);
    //  if (0 == queue_io_job(op)) {
    //    ret = 0;
    //  }
    //}
  }
  return ret;
}


/**
*
*/
int8_t BME280I2C::_read_registers(uint8_t addr, uint8_t* data, uint8_t length) {
  int8_t ret = -2;
  if (nullptr != _bus) {
    I2CBusOp* op = _bus->new_op(BusOpcode::TX, this);
    if (nullptr != op) {
      op->dev_addr = _dev_addr;
      op->sub_addr = addr;
      op->setBuffer(data, length);
      if (0 == queue_io_job(op)) {
        ret = 0;
      }
    }
  }
  return ret;
}



int8_t BME280I2C::init() {
  int8_t ret = -1;
  if (nullptr != _bus) {
    _baro_clear_flag(BME280_FLAG_USE_SPI);
    ret = _priv_init() ? 0 : -2;
  }
  return ret;
}



/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t BME280I2C::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
int8_t BME280I2C::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf     = op->buffer();
    uint     len     = op->bufferLen();
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        break;

      case BusOpcode::RX:
        break;

      default:
        break;
    }
  }
  return ret;
}
