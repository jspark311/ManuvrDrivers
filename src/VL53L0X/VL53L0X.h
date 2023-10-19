/*
I have hard-forked this driver in preparation for changing it.
The original library can be found here:
https://github.com/pololu/vl53l0x-arduino

A crowd-sources register manifest was developed, which helped organize this
driver. It is located here:
https://github.com/GrimbiXcode/VL53L0X-Register-Map

Original license text is reproduced below.
                                                ---J. Ian Lindsay
-----------------------------------------------------------------

Copyright (c) 2017 Pololu Corporation.  For more information, see

https://www.pololu.com/
https://forum.pololu.com/

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

=================================================================

Most of the functionality of this library is based on the VL53L0X
API provided by ST (STSW-IMG005), and some of the explanatory
comments are quoted or paraphrased from the API source code, API
user manual (UM2039), and the VL53L0X datasheet.

The following applies to source code reproduced or derived from
the API:

-----------------------------------------------------------------

Copyright © 2016, STMicroelectronics International N.V.  All
rights reserved.

Redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following
conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.
* Neither the name of STMicroelectronics nor the
names of its contributors may be used to endorse or promote
products derived from this software without specific prior
written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/

#include <AbstractPlatform.h>
#include "BusQueue/I2CAdapter.h"


#ifndef __VL53L0X_DRIVER_H_
#define __VL53L0X_DRIVER_H_

#define ADDRESS_DEFAULT 0x29


/* Registers that exist in hardware */
enum class VL53L0XRegID : uint8_t {
  SYSRANGE_START                              = 0x00, // 0x00
  SYSTEM_SEQUENCE_CONFIG                      = 0x01, // 0x01
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x02, // 0x04
  SYSTEM_RANGE_CONFIG                         = 0x03, // 0x09
  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x04, // 0x0A
  SYSTEM_INTERRUPT_CLEAR                      = 0x05, // 0x0B
  SYSTEM_THRESH_HIGH                          = 0x06, // 0x0C
  SYSTEM_THRESH_LOW                           = 0x07, // 0x0E
  RESULT_INTERRUPT_STATUS                     = 0x08, // 0x13
  RESULT_RANGE_STATUS                         = 0x09, // 0x14
  RESULT_RANGE                                = 0x0A, // 0x1E
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x0B, // 0x20
  PRE_RANGE_CONFIG_MIN_SNR                    = 0x0C, // 0x27
  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x0D, // 0x28
  ALGO_PHASECAL_LIM                           = 0x0E, // 0x30  // These two have the same address (0x30)?
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x0F, // 0x30  // These two have the same address (0x30)?
  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x10, // 0x32
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x11, // 0x33
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x12, // 0x44
  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x13, // 0x46
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x14, // 0x47
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x15, // 0x48
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x16, // 0x4E
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x17, // 0x4F
  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x18, // 0x50
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x19, // 0x51
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x1A, // 0x52
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x1B, // 0x55
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x1C, // 0x56
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x1D, // 0x57
  MSRC_CONFIG_CONTROL                         = 0x1E, // 0x60
  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x1F, // 0x61
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x20, // 0x62
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x21, // 0x64
  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x22, // 0x67
  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x23, // 0x70
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x24, // 0x71
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x25, // 0x72
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x26, // 0x80
  SYSTEM_HISTOGRAM_BIN                        = 0x27, // 0x81
  UNDOCUMENTED_1                              = 0x28, // 0x83
  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x29, // 0x84
  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x2A, // 0x89
  I2C_SLAVE_DEVICE_ADDRESS                    = 0x2B, // 0x8A
  INTERNAL_TUNING_1                           = 0x2C, // 0x91
  UNDOCUMENTED_2                              = 0x2D, // 0x92
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0x2E, // 0xB0
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0x2F, // 0xB1
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0x30, // 0xB2
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0x31, // 0xB3
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0x32, // 0xB4
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0x33, // 0xB5
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0x34, // 0xB6  // These two have the same address (0xB6)?
  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0x35, // 0xB6  // These two have the same address (0xB6)?
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0x36, // 0xBC
  SOFT_RESET_GO2_SOFT_RESET_N                 = 0x37, // 0xBF
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0x38, // 0xC0  // These two have the same address (0xC0)?
  IDENTIFICATION_MODEL_ID                     = 0x39, // 0xC0  // These two have the same address (0xC0)?
  IDENTIFICATION_REVISION_ID                  = 0x3A, // 0xC2
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0x3B, // 0xD0
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0x3C, // 0xD4
  OSC_CALIBRATE_VAL                           = 0x3D, // 0xF8
  INTERNAL_TUNING_2                           = 0x3E, // 0xFF
  INVALID                                     = 0x3F
};


class VL53L0X : public I2CDevice {
  public:
    enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };
    uint8_t last_status; // status of last I2C transmission

    VL53L0X(uint8_t addr = ADDRESS_DEFAULT);
    ~VL53L0X() {};

    inline bool devFound() {        return true;  };   // TODO

    void setAddress(uint8_t new_addr);
    inline uint8_t getAddress() { return _dev_addr; }

    bool init(bool io_2v8 = true);

    bool setSignalRateLimit(float limit_Mcps);
    float getSignalRateLimit();

    bool setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
    uint8_t getVcselPulsePeriod(vcselPeriodType type);

    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous();
    uint16_t readRangeContinuousMillimeters();
    uint16_t readRangeSingleMillimeters();

    inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    inline uint16_t getTimeout() { return io_timeout; }
    bool timeoutOccurred();

    /* Overrides from the I2CDevice */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    struct SequenceStepEnables {
      bool tcc;          // TCC: Target CentreCheck
      bool msrc;         // MSRC: Minimum Signal Rate Check
      bool dss;          // DSS: Dynamic Spad Selection
      bool pre_range;
      bool final_range;
    };

    struct SequenceStepTimeouts {
      uint16_t pre_range_vcsel_period_pclks;
      uint16_t final_range_vcsel_period_pclks;
      uint16_t msrc_dss_tcc_mclks;
      uint16_t pre_range_mclks;
      uint16_t final_range_mclks;
      uint32_t msrc_dss_tcc_us;
      uint32_t pre_range_us;
      uint32_t final_range_us;
    };

    uint32_t measurement_timing_budget_us;
    uint16_t io_timeout;
    uint16_t _last_range = 0;
    uint16_t timeout_start_ms;
    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint8_t shadows[68] = {0, };
    uint8_t byte_buffer = 0;
    bool did_timeout;
    //bool _operation_running;

    bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

    void getSequenceStepEnables(SequenceStepEnables * enables);
    void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

    bool performSingleRefCalibration(uint8_t vhv_init_byte);

    inline void startTimeout() {   timeout_start_ms = millis();    };

    inline bool checkTimeoutExpired() {
      return (io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms) > io_timeout));
    };

    // Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
    // from register value
    // based on VL53L0X_decode_vcsel_period()
    inline uint8_t decodeVcselPeriod(uint8_t reg_val) {
      return (((reg_val) + 1) << 1);
    };

    // Encode VCSEL pulse period register value from period in PCLKs
    // based on VL53L0X_encode_vcsel_period()
    inline uint8_t encodeVcselPeriod(uint8_t period_pclks) {
      return (((period_pclks) >> 1) - 1);
    };

    /* Shadow value manipulation functions. */
    uint     _get_shadow_value(VL53L0XRegID);
    uint8_t* _get_reg_ptr(VL53L0XRegID);

    /* Lowest-level register manipulation functions before I/O. */
    int8_t _read_registers(VL53L0XRegID, uint8_t reg_count = 1);
    int8_t _write_register32(VL53L0XRegID, uint32_t val);
    int8_t _write_register16(VL53L0XRegID, uint16_t val);
    int8_t _write_register(VL53L0XRegID,   uint8_t val);
    int8_t _write_register(uint8_t, uint8_t val);

    /* All I/O is concentrated into these two functions. */
    int8_t _read_buffer(uint8_t, uint8_t* buf, uint8_t len);
    int8_t _write_buffer(uint8_t, uint8_t* buf, uint8_t len);


    // Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
    // based on VL53L0X_calc_macro_period_ps()
    // PLL_period_ps = 1655; macro_period_vclks = 2304
    static inline uint32_t calcMacroPeriod(uint16_t vcsel_period_pclks) {
      return ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
    };


    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint32_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
};

#endif   // __VL53L0X_DRIVER_H_
