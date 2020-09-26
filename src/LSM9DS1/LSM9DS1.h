/*
File:   LSM9DSx.h
Author: J. Ian Lindsay
Date:   2014.03.27

Copyright 2016 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#ifndef __LSM9DS1_MERGED_H__
#define __LSM9DS1_MERGED_H__

#include <AbstractPlatform.h>
#include <I2CAdapter.h>
#include "LSM9DS1Types.h"


/*
* These are possible error states for the IMU state-machine.
*/
enum class IMUFault : int8_t {
  NO_ERROR               =  0,
  WRONG_IDENTITY         = -1,
  INVALID_PARAM          = -2,
  NOT_CALIBRATED         = -3,
  NOT_WRITABLE           = -4,  // TODO: Cut. Should no longer be possible.
  DATA_EXHAUSTED         = -5,
  NOT_INITIALIZED        = -6,
  BUS_INSERTION_FAILED   = -7,   // TODO: Cut. Should no longer be possible.
  BUS_OPERATION_FAILED_R = -8,   // TODO: Cut. Should no longer be possible.
  BUS_OPERATION_FAILED_W = -9    // TODO: Cut. Should no longer be possible.
};

/*
* These are the class states.
*/
enum class IMUState : uint8_t {
  STAGE_0 = 0,  // Undiscovered. Maybe absent.
  STAGE_1,      // Discovered, but not init'd.
  STAGE_2,      // Discovered and initiallized, but unknown register values.
  STAGE_3,      // Fully initialized and sync'd. Un-calibrated.
  STAGE_4,      // Calibrated and idle.
  STAGE_5,      // Calibrated and reading.
  FAULT,        // Fault.
  UNDEF         // Not a state-machine value. A return code to simplifiy error-checks.
};


/* Different integrated sensors this class supports. */
enum class IMUSense : uint8_t {
  ACC = 0,  // Accelerometer
  GYR,      // Gyroscope
  MAG,      // Magnetometer
  THERM     // Temperature
};


/* Sensor capability flags */
#define IMU_CAP_FLAG_HAVE_ACC_X   0x0001
#define IMU_CAP_FLAG_HAVE_ACC_Y   0x0002
#define IMU_CAP_FLAG_HAVE_ACC_Z   0x0004
#define IMU_CAP_FLAG_HAVE_GYR_X   0x0008
#define IMU_CAP_FLAG_HAVE_GYR_Y   0x0010
#define IMU_CAP_FLAG_HAVE_GYR_Z   0x0020
#define IMU_CAP_FLAG_HAVE_MAG_X   0x0040
#define IMU_CAP_FLAG_HAVE_MAG_Y   0x0080
#define IMU_CAP_FLAG_HAVE_MAG_Z   0x0100
#define IMU_CAP_FLAG_HAVE_THERM   0x0200  // Has a temperature sensor.

#define IMU_CAP_FLAG_3AXIS_ACC    (IMU_CAP_FLAG_HAVE_ACC_X | IMU_CAP_FLAG_HAVE_ACC_Y | IMU_CAP_FLAG_HAVE_ACC_Z)
#define IMU_CAP_FLAG_3AXIS_GYR    (IMU_CAP_FLAG_HAVE_GYR_X | IMU_CAP_FLAG_HAVE_GYR_Y | IMU_CAP_FLAG_HAVE_GYR_Z)
#define IMU_CAP_FLAG_3AXIS_MAG    (IMU_CAP_FLAG_HAVE_MAG_X | IMU_CAP_FLAG_HAVE_MAG_Y | IMU_CAP_FLAG_HAVE_MAG_Z)
#define IMU_CAP_FLAG_9DOF         (IMU_CAP_FLAG_3AXIS_ACC | IMU_CAP_FLAG_3AXIS_GYR | IMU_CAP_FLAG_3AXIS_MAG)


class AbstractIMU {
  public:
    virtual int8_t   poll() =0;

    virtual uint16_t capabilities() =0;
    virtual IMUFault lastRead(IMUSense, float*, float*, float*) =0;
    virtual int8_t   pendingSamples(IMUSense) =0;
    virtual bool     devFound() =0;
    virtual bool     initialized() =0;
    virtual bool     calibrated() =0;

  protected:
};
// TODO: Everything above this line should probably be migrated to a separate header.

#define IMU_COMMON_FLAG_VERBOSITY_MASK  0x0007
#define IMU_COMMON_FLAG_HW_WRITABLE     0x0008
#define IMU_COMMON_FLAG_PROFILING       0x0010
#define IMU_COMMON_FLAG_CANCEL_ERROR    0x0020
#define IMU_COMMON_FLAG_AUTOSCALE_0     0x0040
#define IMU_COMMON_FLAG_AUTOSCALE_1     0x0080
#define LSM9DS1_FLAG_PINS_CONFIGURED    0x0800
#define LSM9DS1_FLAG_READING_ID         0x1000
#define IMU_COMMON_FLAG_MAG_POWERED     0x2000
#define IMU_COMMON_FLAG_GYR_POWERED     0x4000
#define IMU_COMMON_FLAG_ACC_POWERED     0x8000

/*
* In addition to the natural ADC error, the sensor accuracy worsens at these rates for
* every degree C deviant of 25C...
*/
const float MAG_TEMPERATURE_DERATE = 0.03f;
const float ACC_TEMPERATURE_DERATE = 0.015f;
const float GYR_TEMPERATURE_DERATE = 0.02f;


/*******************************************************************************
* The abstract LSM9DS1 class. Acc, Gyr, and Mag are all handled here.
*******************************************************************************/
class LSM9DS1 : public AbstractIMU {
  public:
    LSM9DS1(uint8_t a_imu, uint8_t a_mag, uint8_t irq0, uint8_t irq1, uint8_t irq2, uint8_t irq3);
    ~LSM9DS1() {};

    /* Overrides from AbstractIMU */
    uint16_t capabilities() {  return (IMU_CAP_FLAG_9DOF | IMU_CAP_FLAG_HAVE_THERM);  };
    int8_t   poll();
    IMUFault lastRead(IMUSense, float*, float*, float*);
    int8_t   pendingSamples(IMUSense);
    bool devFound() {      return ((0x3D == _get_shadow_value(LSM9DS1RegID::M_WHO_AM_I)) && (0x68 == _get_shadow_value(LSM9DS1RegID::AG_WHO_AM_I)));  };
    bool initialized() {   return (IMUState::STAGE_3 <= getState());   };
    bool calibrated() {    return (IMUState::STAGE_4 <= getState());   };


    void setSampleRateProfile(uint8_t);
    IMUFault setDesiredState(IMUState);   // Used to set the state the OS wants the IMU class to acheive.
    void     write_test_bytes();

    IMUFault init();
    void     reset();           // Reset our state without causing a re-init.
    int8_t   refresh();

    /* Debug stuff... */
    void printDebug(StringBuilder*);

    /* State-check functions. Inlined where practical. */
    inline IMUState getState() {            return imu_state;         };
    inline IMUState desiredState() {        return desired_state;     };

    inline bool initPending() {             return ((IMUState::STAGE_1 == getState()) || (IMUState::STAGE_2 == getState()));  }
    inline bool initReadback() {            return (IMUState::STAGE_2 == getState());        }
    inline bool idle() {                    return (IMUState::STAGE_4 == getState());        }
    inline bool reading() {                 return (IMUState::STAGE_5 == getState());        }
    inline bool desired_state_attained() {  return (getState() == desiredState());        }

    inline bool profile() {             return _class_flag(IMU_COMMON_FLAG_PROFILING);     };
    inline bool cancel_error() {        return _class_flag(IMU_COMMON_FLAG_CANCEL_ERROR);  };
    inline void profile(bool x) {       _class_set_flag(IMU_COMMON_FLAG_PROFILING, x);     };
    inline void cancel_error(bool x) {  _class_set_flag(IMU_COMMON_FLAG_CANCEL_ERROR, x);  };

    inline bool autoscale_mag() {        return _class_flag(IMU_COMMON_FLAG_AUTOSCALE_0);  };
    inline void autoscale_mag(bool x) {  _class_set_flag(IMU_COMMON_FLAG_AUTOSCALE_0, x);  };
    inline bool autoscale_acc() {        return _class_flag(IMU_COMMON_FLAG_AUTOSCALE_0);  };
    inline void autoscale_acc(bool x) {  _class_set_flag(IMU_COMMON_FLAG_AUTOSCALE_0, x);  };
    inline bool autoscale_gyr() {        return _class_flag(IMU_COMMON_FLAG_AUTOSCALE_1);  };
    inline void autoscale_gyr(bool x) {  _class_set_flag(IMU_COMMON_FLAG_AUTOSCALE_1, x);  };

    inline float scaleA() {  return error_map_acc[scale_acc].per_lsb;  };
    inline float scaleG() {  return error_map_gyr[scale_gyr].per_lsb;  };
    inline float scaleM() {  return error_map_mag[scale_mag].per_lsb;  };

    inline float deltaT_I() {  return rate_settings_i[update_rate_i].ts_delta;  };
    inline float deltaT_M() {  return rate_settings_m[update_rate_m].ts_delta;  };


    IMUFault request_rescale_mag(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    IMUFault set_sample_rate_mag(uint8_t nu_srate_idx);     // Call to alter sample rate.
    IMUFault set_base_filter_param_mag(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    IMUFault request_rescale_acc(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    IMUFault set_sample_rate_acc(uint8_t nu_srate_idx);     // Call to alter sample rate.
    IMUFault set_base_filter_param_acc(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    IMUFault request_rescale_gyr(uint8_t nu_scale_idx);     // Call to rescale the sensor.
    IMUFault set_sample_rate_gyr(uint8_t nu_srate_idx);     // Call to alter sample rate.
    IMUFault set_base_filter_param_gyr(uint8_t nu_bw_idx);  // Call to change the bandwidth of the AA filter.

    IMUFault irq_drdy(); // When an IRQ signal fires, find the cause and service it.
    IMUFault irq_m();    // When an IRQ signal fires, find the cause and service it.
    IMUFault irq_1();    // When an IRQ signal fires, find the cause and service it.
    IMUFault irq_2();    // When an IRQ signal fires, find the cause and service it.


    /* Inlines for the specialized flag duty of get/set class verbosity. */
    inline uint8_t getVerbosity() {
      return (_imu_flags & IMU_COMMON_FLAG_VERBOSITY_MASK);
    };
    inline void  setVerbosity(uint8_t nu) {
      _imu_flags = (nu & IMU_COMMON_FLAG_VERBOSITY_MASK) | (_imu_flags & ~IMU_COMMON_FLAG_VERBOSITY_MASK);
    };

    inline const char* getStateString() {    return getStateString(imu_state);        }
    inline const char* getErrorString() {    return getErrorString(error_condition);  }

    static IMUState getStateByIndex(uint8_t state_idx);
    static const char* getStateString(IMUState);
    static const char* getErrorString(IMUFault);


    static const float max_range_vect_mag;
    static const float max_range_vect_acc;
    static const float max_range_vect_gyr;

    static const GainErrorMap error_map_mag[];
    static const GainErrorMap error_map_acc[];
    static const GainErrorMap error_map_gyr[];

    static const UpdateRate2Hertz rate_settings_m[];
    static const UpdateRate2Hertz rate_settings_i[];



  protected:
    const uint8_t _ADDR_IMU;
    const uint8_t _ADDR_MAG;
    const uint8_t _IRQ_0_PIN;
    const uint8_t _IRQ_1_PIN;
    const uint8_t _IRQ_2_PIN;
    const uint8_t _IRQ_3_PIN;
    uint16_t  _imu_flags          = 7;
    uint32_t  _last_read_imu      = 0;
    uint32_t  _last_read_mag      = 0;
    uint32_t  _last_read_temp     = 0;
    uint32_t  discards_total_i    = 0;     // Track how many discards we've ASKED for.
    uint32_t  discards_total_m    = 0;     // Track how many discards we've ASKED for.
    uint16_t  discards_remain_i   = 0;     // If we know we need to discard samples...
    uint16_t  discards_remain_m   = 0;     // If we know we need to discard samples...
    float     _temperature        = 0.0;   // In Celcius.
    uint8_t   _fifo_remaining     = 0;     // How much data is left in the FIFO?
    uint8_t   sb_next_read        = 0;
    uint8_t   sb_next_write       = 0;
    IMUFault  error_condition     = IMUFault::NO_ERROR;
    IMUState  imu_state           = IMUState::STAGE_0;
    IMUState  desired_state       = IMUState::STAGE_0;
    uint8_t   scale_mag           = 0;     // TODO: Strike. Index to the scale array.
    uint8_t   scale_acc           = 0;     // TODO: Strike. Index to the scale array.
    uint8_t   scale_gyr           = 0;     // TODO: Strike. Index to the scale array.
    uint8_t   update_rate_i       = 0;     // TODO: Strike. Index to the update-rate array.
    uint8_t   update_rate_m       = 0;     // TODO: Strike. Index to the update-rate array.
    uint8_t   io_test_val_0       = 0;     // TODO: Strike
    uint8_t   io_test_val_1       = 0;     // TODO: Strike
    int8_t    base_filter_param   = 0;
    uint8_t   shadows[73];     // Shadow registers.

    /* Basal register access and utility fxn's */
    int8_t   _clear_registers();
    int8_t   _check_register_value_for_write(LSM9DS1RegID, unsigned int val);
    int8_t   _set_shadow_value(LSM9DS1RegID, unsigned int val);
    unsigned int _get_shadow_value(LSM9DS1RegID);
    IMUFault _read_fifo();
    int8_t   _configure_sensor();

    /* These must be provided by the bus-specific child class. */
    virtual IMUFault _write_registers(LSM9DS1RegID reg, uint8_t len) =0;
    virtual IMUFault _read_registers(LSM9DS1RegID reg, uint8_t len)  =0;
    virtual int8_t   _setup_bus_pin() =0;


    int8_t _ll_pin_init();
    bool integrity_check();
    bool is_setup_completed();

    /**
    * Sets the current position of the IMU state machine.
    */
    inline void set_state(IMUState nu) {     imu_state = nu;   }

    /* Flag manipulation inlines */
    inline uint16_t _class_flags() {                return _imu_flags;           };
    inline bool _class_flag(uint16_t _flag) {       return (_imu_flags & _flag); };
    inline void _class_clear_flag(uint16_t _flag) { _imu_flags &= ~_flag;        };
    inline void _class_set_flag(uint16_t _flag) {   _imu_flags |= _flag;         };
    inline void _class_set_flag(uint16_t _flag, bool nu) {
      if (nu) _imu_flags |= _flag;
      else    _imu_flags &= ~_flag;
    };


    inline bool power_to_mag() {   return _class_flag(IMU_COMMON_FLAG_MAG_POWERED);       };
    inline void power_to_mag(bool x) {  _class_set_flag(IMU_COMMON_FLAG_MAG_POWERED, x);  };
    inline bool power_to_acc() {   return _class_flag(IMU_COMMON_FLAG_ACC_POWERED);       };
    inline void power_to_acc(bool x) {  _class_set_flag(IMU_COMMON_FLAG_ACC_POWERED, x);  };
    inline bool power_to_gyr() {   return _class_flag(IMU_COMMON_FLAG_GYR_POWERED);       };
    inline void power_to_gyr(bool x) {  _class_set_flag(IMU_COMMON_FLAG_GYR_POWERED, x);  };


    int8_t _io_op_callback_mag(const LSM9DS1RegID, BusOp*);
    int8_t _io_op_callback_imu(const LSM9DS1RegID, BusOp*);


    static const uint8_t _imu_reg_defaults[];
    static const char*   _reg_name_str(const LSM9DS1RegID);
    static const uint8_t _reg_addr(const LSM9DS1RegID);
    static const uint8_t _get_shadow_offset(const LSM9DS1RegID);
    static const uint8_t _reg_width(const LSM9DS1RegID);
    static const bool    _reg_writable(const LSM9DS1RegID);
    static const bool    _reg_is_for_mag(const LSM9DS1RegID);
    static LSM9DS1RegID  _reg_id_from_addr_mag(const uint8_t reg_addr);
    static LSM9DS1RegID  _reg_id_from_addr_imu(const uint8_t reg_addr);
};



/*******************************************************************************
* The instantiable i2c-specific child class.
*******************************************************************************/
class LSM9DS1_I2C : public LSM9DS1, public I2CDevice {
  public:
    LSM9DS1_I2C(uint8_t a_imu, uint8_t a_mag, uint8_t irq0, uint8_t irq1, uint8_t irq2, uint8_t irq3);
    ~LSM9DS1_I2C() {};

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

  protected:
    IMUFault _write_registers(LSM9DS1RegID reg, uint8_t len);
    IMUFault _read_registers(LSM9DS1RegID reg, uint8_t len);
    int8_t   _setup_bus_pin();

  private:
    I2CBusOp  _fifo_read;
};

#endif // __LSM9DS1_MERGED_H__
