/*
File:   PAC195x.h
Author: J. Ian Lindsay
Date:   2023.06.24
*/

#ifndef __PAC195x_H__
#define __PAC195x_H__

#include <inttypes.h>
#include <stdint.h>
#include <stdarg.h>
#include "StringBuilder.h"
#include "CppPotpourri.h"
#include "StopWatch.h"
#include "I2CAdapter.h"
#include "FlagContainer.h"
#include "EnumWrapper.h"
#include "FiniteStateMachine.h"

class PAC195x;

/* These enum values indicate order only. Not addresses. */
enum class PAC195xRegID : uint8_t {
  /*                    Idx    Addr   Sz   Notes
  ----------------------------------------------------- */
  REFRESH             = 0,   // 0x00  0    W (empty)
  CTRL                = 1,   // 0x01  2    RW
  ACC_COUNT           = 2,   // 0x02  4    R
  V_ACC_1             = 3,   // 0x03  7    R
  V_ACC_2             = 4,   // 0x04  7    R
  V_ACC_3             = 5,   // 0x05  7    R
  V_ACC_4             = 6,   // 0x06  7    R
  V_BUS_1             = 7,   // 0x07  2    R
  V_BUS_2             = 8,   // 0x08  2    R
  V_BUS_3             = 9,   // 0x09  2    R
  V_BUS_4             = 10,  // 0x0A  2    R
  V_SENSE_0           = 11,  // 0x0B  2    R
  V_SENSE_1           = 12,  // 0x0C  2    R
  V_SENSE_2           = 13,  // 0x0D  2    R
  V_SENSE_3           = 14,  // 0x0E  2    R
  V_BUS_AVG_0         = 15,  // 0x0F  2    R
  V_BUS_AVG_1         = 16,  // 0x10  2    R
  V_BUS_AVG_2         = 17,  // 0x11  2    R
  V_BUS_AVG_3         = 18,  // 0x12  2    R
  V_SENSE_AVG_0       = 19,  // 0x13  2    R
  V_SENSE_AVG_1       = 20,  // 0x14  2    R
  V_SENSE_AVG_2       = 21,  // 0x15  2    R
  V_SENSE_AVG_3       = 22,  // 0x16  2    R
  V_POWER_0           = 23,  // 0x17  4    R
  V_POWER_1           = 24,  // 0x18  4    R
  V_POWER_2           = 25,  // 0x19  4    R
  V_POWER_3           = 26,  // 0x1A  4    R
  SMBUS_SETTINGS      = 27,  // 0x1C  1    RW  (Discontinuity)
  NEG_PWR_FSR         = 28,  // 0x1D  2    RW
  REFRESH_G           = 29,  // 0x1E  0    W (empty)
  REFRESH_V           = 30,  // 0x1F  0    W (empty)
  SLOW                = 31,  // 0x20  1    RW
  CTRL_ACTIVE         = 32,  // 0x21  2    R
  NEG_PWR_FSR_ACTIVE  = 33,  // 0x22  2    R
  CTRL_LATCH          = 34,  // 0x23  2    R
  NEG_PWR_FSR_LATCH   = 35,  // 0x24  2    R
  ACCUM_CONFIG        = 36,  // 0x25  1    RW
  ALERT_STATUS        = 37,  // 0x26  3    RW  (write to clear status)
  SLOW_ALERT1         = 38,  // 0x27  3    RW
  GPIO_ALERT2         = 39,  // 0x28  3    RW
  ACC_FULLNESS_LIMITS = 40,  // 0x29  2    RW
  OC_LIMIT_0          = 41,  // 0x30  2    RW  (Discontinuity)
  OC_LIMIT_1          = 42,  // 0x31  2    RW
  OC_LIMIT_2          = 43,  // 0x32  2    RW
  OC_LIMIT_3          = 44,  // 0x33  2    RW
  UC_LIMIT_0          = 45,  // 0x34  2    RW
  UC_LIMIT_1          = 46,  // 0x35  2    RW
  UC_LIMIT_2          = 47,  // 0x36  2    RW
  UC_LIMIT_3          = 48,  // 0x37  2    RW
  OP_LIMIT_0          = 49,  // 0x38  3    RW
  OP_LIMIT_1          = 50,  // 0x39  3    RW
  OP_LIMIT_2          = 51,  // 0x3A  3    RW
  OP_LIMIT_3          = 52,  // 0x3B  3    RW
  OV_LIMIT_0          = 53,  // 0x3C  2    RW
  OV_LIMIT_1          = 54,  // 0x3D  2    RW
  OV_LIMIT_2          = 55,  // 0x3E  2    RW
  OV_LIMIT_3          = 56,  // 0x3F  2    RW
  UV_LIMIT_0          = 57,  // 0x40  2    RW
  UV_LIMIT_1          = 58,  // 0x41  2    RW
  UV_LIMIT_2          = 59,  // 0x42  2    RW
  UV_LIMIT_3          = 60,  // 0x43  2    RW
  OC_LIMIT_SAMPLES    = 61,  // 0x44  1    RW
  UC_LIMIT_SAMPLES    = 62,  // 0x45  1    RW
  OP_LIMIT_SAMPLES    = 63,  // 0x46  1    RW
  OV_LIMIT_SAMPLES    = 64,  // 0x47  1    RW
  UV_LIMIT_SAMPLES    = 65,  // 0x48  1    RW
  ALERT_ENABLE        = 66,  // 0x49  3    RW
  ACCUM_CONFIG_ACTIVE = 67,  // 0x4A  1    R
  ACCUM_CONFIG_LATCH  = 68,  // 0x4B  1    R
  PROD_ID             = 69,  // 0xFD  1    R  (Discontinuity)
  MANU_ID             = 70,  // 0xFE  1    R
  REVISION_ID         = 71,  // 0xFF  1    R
  INVALID             = 72   // End-of-enum. There are 72 registers.
};
#define PAC195X_REG_COUNT   ((uint8_t) PAC195xRegID::INVALID)


/*
* Conversion modes
* NOTE: These enum values translate directly to register values.
*/
enum class PAC195xMode : uint8_t {
  SPS_ADAPTIVE_1024 = 0x00,
  SPS_ADAPTIVE_256  = 0x01,
  SPS_ADAPTIVE_64   = 0x02,
  SPS_ADAPTIVE_8    = 0x03,
  SPS_1024          = 0x04,
  SPS_256           = 0x05,
  SPS_64            = 0x06,
  SPS_8             = 0x07,
  SINGLE            = 0x08,
  SINGLE_8X         = 0x09,
  FAST              = 0x0A,
  BURST             = 0x0B,
  SLEEP             = 0x0F
};

/*
* This family has two configurable GPIO pins, which defaults to input.
*/
enum class PAC195xGPIOMode : uint8_t {
  GPIO_INPUT     = 0,  // Default on reset for ALERT2.
  GPIO_OUTPUT_OD = 1,
  ALERT_OUTPUT   = 2,
  SLOW_INPUT     = 3   // Default on reset for ALERT1.
};

/*
* For the purposes of alerts when programmed limits are exceeded, we can specify
*   sample counts for which the condition of excess must persist before an
*   interrupt is triggered.
* NOTE: These enum values translate directly to register values.
*/
enum class PAC195xAlertHysterisis : uint8_t {
  SAMPLES_1  = 0,  // Default is no alert hysteresis.
  SAMPLES_4  = 1,
  SAMPLES_8  = 2,
  SAMPLES_16 = 3
};

/*
* Channel topology.
*/
enum class PAC195xChanTopology : uint8_t {
  UNIPOLAR   = 0,
  FSR_OVER_2 = 1,
  BIPOLAR    = 2
};

/*
* Channel accumulation targets.
*/
enum class PAC195xAccumTarget : uint8_t {
  V_POWER  = 0,   // Watt meter
  V_SENSE  = 1,   // Coulomb meter
  V_BUS    = 2    // Voltage averaging
};



enum class PAC195xState : uint8_t {
  UNINIT = 0,  // init() has never been called.
  PREINIT,     // Pin control is being established.
  RESETTING,   // Driver is resetting the sensor.
  DISCOVERY,   // Driver is probing for the sensor.
  USR_CONF,    // User config is being written.
  IDLE,        // Powered up and calibrated, but not reading.
  READING,     // Everything running, data collection proceeding.
  FAULT,       // State machine encountered something it couldn't cope with.
  INVALID      // Catch-all for illegal states.
};


/*
* Class flags.
* NOTE: PAC195X_FLAG_USE_INTERNAL_CLK takes priority over
*   PAC195X_FLAG_GENERATE_MCLK to avoid potential pin contention. If both flags
*   are set, the MCLK pin (if given) will be configured as an input, and the
*   flag directing the class to generate a clock on that pin will be ignored.
*/
#define PAC195X_FLAG_DEVICE_PRESENT   0x00000001  // Part is likely an PAC195x.
#define PAC195X_FLAG_PINS_CONFIGURED  0x00000002  // Low-level pin setup is complete.
#define PAC195X_FLAG_USER_CONFIG      0x00000004  // Registers are initialized with the user's values.
#define PAC195X_FLAG_REFRESH_CYCLE    0x00000008  // We are undergoing a full register refresh.
#define PAC195X_FLAG_PENDING_POR      0x00002000  // We are waiting for the POR bit to clear.
#define PAC195X_FLAG_SERVICING_IRQS   0x00010000  // The class will respond to IRQ signals.

// Bits to preserve through reset.
#define PAC195X_FLAG_RESET_MASK  (PAC195X_FLAG_DEVICE_PRESENT | PAC195X_FLAG_PINS_CONFIGURED)


// TODO: might replace these with higher-level data selections. Don't code too
//   deeply against this yet.
#define PAC195X_CHAN_MASK_0   0x01  //
#define PAC195X_CHAN_MASK_1   0x02  //
#define PAC195X_CHAN_MASK_2   0x04  //
#define PAC195X_CHAN_MASK_3   0x08  //
#define PAC195X_CHAN_VOLTAGE  0x10  //


/*
* A class to hold enum'd config for the sensor.
*/
class PAC195xConfig {
  public:
    uint8_t          scan;        // Bitmask of channels that should be reporting.
    PAC195xMode      mode;        // The sensor sampling mode.
    PAC195xGPIOMode  gpio1_mode;  //
    PAC195xGPIOMode  gpio2_mode;  //

    /* Trivial constructor. */
    PAC195xConfig() : scan(0),
        mode(PAC195xMode::SINGLE),
        gpio1_mode(PAC195xGPIOMode::SLOW_INPUT),
        gpio2_mode(PAC195xGPIOMode::GPIO_INPUT) {};

    PAC195xConfig(
      const uint8_t SCAN = 0,
      const PAC195xMode MODE = PAC195xMode::SINGLE,
      const PAC195xGPIOMode P1_MODE = PAC195xGPIOMode::SLOW_INPUT,
      const PAC195xGPIOMode P2_MODE = PAC195xGPIOMode::GPIO_INPUT
    ) : scan(SCAN),
        mode(MODE),
        gpio1_mode(P1_MODE),
        gpio2_mode(P2_MODE) {};

    /* Copy constructor. */
    PAC195xConfig(const PAC195xConfig* CFG) :
      scan(CFG->scan),
      mode(CFG->mode),
      gpio1_mode(CFG->gpio1_mode),
      gpio2_mode(CFG->gpio2_mode) {};

    ~PAC195xConfig() {};


  private:
};


/*
* A representation of a sensor channel. Four of these are composed into the
*   the top-level sensor driver.
*/
class PAC195xChannel {
  public:
    // Data accessors.
    inline float  voltage() {   return _voltage;  };
    inline float  power() {     return _power;    };
    inline double energy() {    return _energy;   };
    inline bool   fresh() {     return (!_conf_dirty & _fresh);    };

    // Wrapped channel operations. Calling these allows simpler application
    //   logic, and generates I/O from the driver.
    int8_t resetAccumulator();
    int8_t enabled(bool);
    bool   enabled();

    int8_t setVoltageLimits(float under_volt, float over_volt, PAC195xAlertHysterisis hyst);
    int8_t setCurrentLimits(float under_amp,  float over_amp,  PAC195xAlertHysterisis hyst);
    int8_t setPowerLimit(float over_wattage, PAC195xAlertHysterisis hyst);

    void printChannel(StringBuilder*);


  private:
    friend PAC195x;

    PAC195xChannel(PAC195x*, const float OHMS, const uint8_t C_NUM);
    ~PAC195xChannel() {};

    PAC195x* _SENSOR_PTR;
    const float    _SENSE_OHMS;  // Channel config. Value (in Ohm's) of the sense resistor.
    const uint8_t  _CHAN_NUM;    // The channel number.
    PAC195xChanTopology _topo;
    PAC195xAccumTarget  _acc_target;
    bool     _conf_dirty;  // Are the channel settings in flux?
    bool     _fresh;       // Has the channel been updated since the last time it was checked?

    float    _voltage;     // Primary data. Instantaneous Voltage at SENSE+ pin
    float    _current;     // Primary data. Instantaneous Amperage through the sense resistor.
    float    _power;       // Primary data. Instantaneous power through the channel.
    double   _voltage_avg; // Primary data. Rolling average of Voltage.
    double   _current_avg; // Primary data. Rolling average of power.
    double   _energy;      // Primary data. The accumulated energy through the channel.
    uint32_t _acc_time;    // The system time when we began accumulating.

    // Value update functions called by the driver at the end of I/O handling.
    int8_t _update_from_vacc_reg(uint64_t);
    int8_t _update_from_vbus_reg(uint32_t);
    int8_t _update_from_vsense_reg(uint32_t);
    int8_t _update_from_vbus_avg_reg(uint32_t);
    int8_t _update_from_vsense_avg_reg(uint32_t);
    int8_t _update_from_vpower_reg(uint32_t);

    // Scaling and parameter derivation functions.
    bool _is_accumulator_unsigned();
    uint32_t _denominator();
    float  _sample_rate();

    /*
    * Given the current channel settings, calculates and returns the PowerFSR value.
    * Units for this value are V^2/R
    */
    inline const double _power_fsr() {  return (3.200D / _SENSE_OHMS);  };
};



/* The driver. */
class PAC195x : public I2CDevice, public StateMachine<PAC195xState> {
  public:
    bool alert1_irq = false;
    bool alert2_irq = false;

    // Most interaction with this driver from the application layer is likely
    //   to be mediated via these channel objects. Depending on what hardware
    //   is found by the driver, some of these objects will be disabled.
    PAC195xChannel chan_1;
    PAC195xChannel chan_2;
    PAC195xChannel chan_3;
    PAC195xChannel chan_4;

    PAC195x(
      const PAC195xConfig*,
      const uint8_t addr,
      const uint8_t pin_alert_1,
      const uint8_t pin_alert_2,
      const uint8_t pin_pwr_dwn
    );
    ~PAC195x();

    int8_t  init();          // Setup the driver.
    int8_t  trigger();       // For applications that use one-shot sample logic.
    int8_t  refresh();       // Refresh the state of the register shadows.
    bool    scanComplete();  // Were all configured channels sampled and updated?
    bool    lowSideSensor(); // Returns true if the part is the low-side variant.
    inline int8_t poll() {   return _fsm_poll();     };

    inline uint32_t  lastRead() {        return micros_last_read;  };
    inline uint32_t  readCount() {       return read_count;        };
    inline void      resetReadCount() {  read_count = 0;           };
    inline uint32_t  accumulationCount() {   return _get_shadow_value(PAC195xRegID::ACC_COUNT);   };

    inline bool ownsIRQPin(uint8_t x) {  return ((_ALERT1_PIN == x) | (_ALERT2_PIN == x));  };
    inline bool devFound() {             return _flags.value(PAC195X_FLAG_DEVICE_PRESENT); };
    inline bool configured() {           return _flags.value(PAC195X_FLAG_USER_CONFIG);    };

    /* Functions for output and debug. */
    void printPins(StringBuilder*);
    void printRegs(StringBuilder*);
    void printDebug(StringBuilder*);
    void printChannelValues(StringBuilder*);

    /* Built-in per-instance console handler. */
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    static const char* stateStr(const PAC195xState);



  private:
    friend PAC195xChannel;   // Allows register API to remain buried.
    // Pin assignments
    const uint8_t  _ALERT1_PIN;
    const uint8_t  _ALERT2_PIN;
    const uint8_t  _PWR_DWN_PIN;
    PAC195xConfig  _desired_conf;
    FlagContainer32 _flags;

    uint8_t  _reg_shadows[163]     = {0, };  // Register shadows.
    uint32_t read_count            = 0;
    uint32_t micros_last_read      = 0;

    I2CBusOp  _busop_irq_read;
    I2CBusOp  _busop_dat_read;


    void   _set_fault(const char*);

    /* Everything below this line is up for review */
    int8_t  _post_reset_fxn();
    int8_t  _ll_pin_init();

    uint8_t _channel_count();
    int8_t  _set_scan_channels(uint32_t);
    int8_t  _apply_usr_config();
    int8_t  _send_dev_refresh(bool general_call = false);

    void     _clear_registers();
    int8_t   _set_shadow_value(const PAC195xRegID, uint32_t val);
    uint32_t _get_shadow_value(const PAC195xRegID);
    uint64_t _get_shadow_value64(const PAC195xRegID);
    uint8_t* _get_shadow_address(const PAC195xRegID);


    int8_t   _write_registers(PAC195xRegID, uint8_t count);
    int8_t   _read_registers(PAC195xRegID, uint8_t count);
    int8_t   _write_register(PAC195xRegID, uint32_t val);
    int8_t   _read_register(PAC195xRegID);
    int8_t   _proc_reg_write(PAC195xRegID);
    int8_t   _proc_reg_read(PAC195xRegID);

    /* Flag manipulation inlines */
    inline bool _servicing_irqs() {        return _flags.value(PAC195X_FLAG_SERVICING_IRQS);   };
    inline void _servicing_irqs(bool x) {  _flags.set(PAC195X_FLAG_SERVICING_IRQS, x);         };

    inline bool _scan_covers_channel(PAC195xChannel c) {
      //return (0x01 & (_reg_shadows[(uint8_t) PAC195xRegID::SCAN] >> ((uint8_t) c)));
      return false;
    };

    /* Mandatory overrides from StateMachine. */
    int8_t _fsm_poll();                     // Polling for state exit.
    int8_t _fsm_set_position(PAC195xState); // Attempt a state entry.


    /* Static accessors for register constants. TODO: Conceal. */
    static const uint8_t _reg_addr(const PAC195xRegID);
    static const uint8_t _reg_shadow_offset(const PAC195xRegID);
    static const uint8_t _reg_width(const PAC195xRegID);
    static const bool    _reg_writable(const PAC195xRegID);
    static const char* const _reg_name_str(const PAC195xRegID);
    static const PAC195xRegID _reg_id_from_addr(const uint8_t);
};

#endif  // __PAC195x_H__
