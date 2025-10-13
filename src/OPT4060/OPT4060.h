/**
*
*
*
*
* NOTES:
*   1. Driver reports brightness values in terms of lux, and color in normalized form.
*   2. Driver does not support SMBus alert.
*   3. Driver only allows interrupt generation when ALL channels are finished converting.
*      This economizes on power and bus I/O volume at the possible cost of data loss
*      for high-rate applications. TODO: It is easy to extend the driver to support
*      both cleanly. Inquire for details.
*   4. Driver does not yet support hardware triggering via the interrupt pin.
*
* Color is derived by an empirical map.
*/


#ifndef __OPT4060_DRIVER_H_
#define __OPT4060_DRIVER_H_

#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"
#include "Vector3.h"

// We only include the Image header for the sake of its color types and
//   handlers. This will allow the driver to convert triplets of lux into
//   something a webdev would recognize as a color value.
#include "Image/Image.h"

// If we are going to support interrupts from the device, we need at least one
//   entry in a global context table for this class.
#if !defined(OPT4060_INTERRUPTER_COUNT)
  #define OPT4060_INTERRUPTER_COUNT  1
#endif


/* Class flags */
#define OPT4060_FLAG_PINS_CONFIGURED    0x0002  // Have the platform GPIOs been configured.
#define OPT4060_FLAG_CONFIG_0_KNOWN     0x0004  // The register has been confirmed read or set.
#define OPT4060_FLAG_CONFIG_1_KNOWN     0x0008  // The register has been confirmed read or set.
#define OPT4060_FLAG_CONFIG_2_KNOWN     0x0010  // The register has been confirmed read or set.
#define OPT4060_FLAG_CHAN_0_KNOWN       0x0020  // The register has been confirmed read since last trigger.
#define OPT4060_FLAG_CHAN_1_KNOWN       0x0040  // The register has been confirmed read since last trigger.
#define OPT4060_FLAG_CHAN_2_KNOWN       0x0080  // The register has been confirmed read since last trigger.
#define OPT4060_FLAG_CHAN_3_KNOWN       0x0100  // The register has been confirmed read since last trigger.
#define OPT4060_FLAG_MEASURING          0x0200  // A measurement is in progress.
#define OPT4060_FLAG_IO_IN_FLIGHT       0x0400  //
#define OPT4060_FLAG_INT_BUSOP_IS_TRIG  0x0800  //

// Masks of driver state flags to simplify checking.
#define OPT4060_FLAG_PIN_CONF_MASK (OPT4060_FLAG_CONFIG_0_KNOWN | OPT4060_FLAG_CONFIG_1_KNOWN)
#define OPT4060_FLAG_ALL_CONF_MASK (OPT4060_FLAG_PIN_CONF_MASK | OPT4060_FLAG_CONFIG_2_KNOWN)
#define OPT4060_FLAG_INIT_MASK     (OPT4060_FLAG_ALL_CONF_MASK)
//| OPT4060_FLAG_PINS_CONFIGURED)
#define OPT4060_FLAG_FRESH_VALUE   (OPT4060_FLAG_CHAN_0_KNOWN | OPT4060_FLAG_CHAN_1_KNOWN | OPT4060_FLAG_CHAN_2_KNOWN | OPT4060_FLAG_CHAN_3_KNOWN)



/* Dynamic range. Enum values are register field values. */
enum class OPT4060DynRange : uint8_t {
  LUX_2200   = 0,
  LUX_4500   = 1,
  LUX_9000   = 2,
  LUX_18000  = 3,
  LUX_36000  = 4,
  LUX_72000  = 5,
  LUX_144000 = 6,
  AUTORANGE  = 12,
  INVALID    = 13
};


/* Datarate. Enum values are register field values. */
enum class OPT4060DataRate : uint8_t {
  HZ_1667 = 0,
  HZ_1000 = 1,
  HZ_556  = 2,
  HZ_294  = 3,
  HZ_154  = 4,
  HZ_79   = 5,
  HZ_40   = 6,
  HZ_20   = 7,
  HZ_10   = 8,
  HZ_5    = 9,
  HZ_2P5  = 10,
  HZ_1P25 = 11,
  INVALID = 12
};

/* Channels */
enum class OPT4060Channel : uint8_t {
  RED     = 0x00,
  GREEN   = 0x01,
  BLUE    = 0x02,
  WHITE   = 0x03,
  INVALID = 0x04     // Invalid. Not repesented in hardware.
};

/* Operating modes. Enum values are register field values. */
enum class OPT4060Mode : uint8_t {
  POWER_DOWN = 0x00,
  AR_ONESHOT = 0x01,
  ONESHOT    = 0x02,
  CONTINUOUS = 0x03,
  INVALID    = 0x04     // Invalid. Not repesented in hardware.
};

/*
* INT pin modes
* Enum values reflect actual register data.
*/
enum class OPT4060IntPin : uint8_t {
  TRIGGER       = 0x00,   // A falling edge on an active high INT pin causes a conversion to start.
  NONE          = 0x04,
  CHAN_CONV     = 0x05,
  ALL_CONV      = 0x07,
  CHAN_CONV_INV = 0x0D,
  ALL_CONV_INV  = 0x0F,
  INVALID       = 0x06    // Invalid. Not repesented in hardware.
};

/*
* Enums for registers. All registers are 16-bits wide.
* Enum values reflect actual addresses.
*/
enum class OPT4060Register : uint8_t {
  CHAN0_MSW  = 0x00,    // Read-only
  CHAN0_LSW  = 0x01,    // Read-only
  CHAN1_MSW  = 0x02,    // Read-only
  CHAN1_LSW  = 0x03,    // Read-only
  CHAN2_MSW  = 0x04,    // Read-only
  CHAN2_LSW  = 0x05,    // Read-only
  CHAN3_MSW  = 0x06,    // Read-only
  CHAN3_LSW  = 0x07,    // Read-only
  THRESH_0   = 0x08,    //
  THRESH_1   = 0x09,    //
  CONFIG_0   = 0x0A,    //
  CONFIG_1   = 0x0B,    //
  CONFIG_2   = 0x0C,    //
  DEV_ID     = 0x11,    // Read-only
  INVALID    = 0x12     // Invalid. Not repesented in hardware.
};


/**
* Container class for init-time options. Non-const parameters can be changed
*   after init, but these values reflect the state that the device should
*   initialize into.
*/
class OPT4060Opts {
  public:
    const uint8_t   ADDR;
    const uint8_t   ALRT_PIN;
    const OPT4060IntPin PIN_MODE;
    OPT4060Mode     mode;
    OPT4060DynRange range;
    OPT4060DataRate rate;

    /** Copy constructor. */
    OPT4060Opts(const OPT4060Opts &o) :
      ADDR(o.ADDR),
      ALRT_PIN(o.ALRT_PIN),
      PIN_MODE(o.PIN_MODE),
      mode(o.mode),
      range(o.range),
      rate(o.rate) {};
    OPT4060Opts(const OPT4060Opts* o) :
      ADDR(o->ADDR),
      ALRT_PIN(o->ALRT_PIN),
      PIN_MODE(o->PIN_MODE),
      mode(o->mode),
      range(o->range),
      rate(o->rate) {};

    /**
    * Constructor that accepts desired configuration.
    *
    * @param i2c address
    * @param ALERT pin
    * @param Pin mode
    * @param Operating mode
    * @param Dynamic range
    * @param Conversion Rate
    */
    OPT4060Opts(uint8_t addr, uint8_t a_pin, OPT4060IntPin pin_md, OPT4060Mode md, OPT4060DynRange rng, OPT4060DataRate cr) :
      ADDR(addr),
      ALRT_PIN(a_pin),
      PIN_MODE(pin_md),
      mode(md),
      range(rng),
      rate(cr) {};

    inline bool pinDefined() {      return (255 != ALRT_PIN); };
    inline bool haveTriggerPin() {  return (pinDefined() & (PIN_MODE == OPT4060IntPin::TRIGGER)); };
    inline bool haveAlertPin() {    return (pinDefined() & (PIN_MODE != OPT4060IntPin::TRIGGER) & (PIN_MODE != OPT4060IntPin::NONE)); };
};



/**
* The driver.
*/
class OPT4060 : public I2CDevice {
  public:
    OPT4060(const OPT4060Opts, I2CAdapter* bus = nullptr);
    ~OPT4060();

    int8_t init(I2CAdapter* bus = nullptr);
    void isr_fxn();   // Called from the ISR to dispatch status read.
    int8_t poll();

    void printDebug(StringBuilder*);
    void printChannelValues(StringBuilder*, int8_t chan = -1);
    void printRegs(StringBuilder*);
    int8_t console_handler(StringBuilder*, StringBuilder*);

    inline bool  devFound() {     return (0x0821 == _get_shadow_value(OPT4060Register::DEV_ID));   };
    inline bool  dataReady() {    return _opt4060_flag(OPT4060_FLAG_FRESH_VALUE);     };
    inline bool  initialized() {  return (OPT4060_FLAG_INIT_MASK == (_flags & OPT4060_FLAG_INIT_MASK)); };

    bool overloaded();
    int8_t reportOverload(bool autorescale);
    int8_t rescaleOnOverload(bool autorescale);

    // Get/set the number of consecutive faults for threshold logic.
    // 0 - 1 fault
    // 1 - 2 faults
    // 2 - 4 faults
    // 3 - 6 faults
    uint8_t faultCount();
    int8_t faultCount(uint8_t faultSetting);

    // Interrupt threshold accessors.
    uint32_t threshold();
    int8_t   threshold(uint32_t lux);

    int8_t conversionRate(OPT4060DataRate, bool dispatch_io = true);
    OPT4060DataRate conversionRate();
    int8_t dynamicRange(OPT4060DynRange, bool dispatch_io = true);
    OPT4060DynRange dynamicRange();
    int8_t mode(OPT4060Mode, bool dispatch_io = true);
    OPT4060Mode mode();
    OPT4060IntPin optPinMode();

    /* Data access functions */
    // These functions report normalized sensor data converted to lux.
    uint32_t milliLux(OPT4060Channel);
    int8_t   milliLux(Vector3<uint32_t>*);
    inline Vector3<float> getErrorLux() {  return Vector3<float>(_effective_res[0], _effective_res[1], _effective_res[2]);  };

    // Color functions report values as normalized against total incident light.
    uint32_t colorValue(const ImgBufferFormat);
    int8_t   colorValue(Vector3<float>*);
    int8_t   colorValue(Vector3<uint8_t>*);


    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    static const char* const odrStr(const OPT4060DataRate);
    static const char* const regStr(const OPT4060Register);
    static const char* const modeStr(const OPT4060Mode);
    static const char* const pinModeStr(const OPT4060IntPin);
    static const char* const chanStr(const OPT4060Channel);
    static const char* const rateStr(const OPT4060DataRate);
    static const char* const rangeStr(const OPT4060DynRange);
    static const uint32_t data_period_us(const OPT4060DataRate);


  private:
    OPT4060Opts _opts;
    uint16_t    _flags          = 0;
    uint32_t    _last_trig_us   = 0;    // System time at last triggering.
    uint32_t    _last_read_us   = 0;    // System time of last data read.
    uint32_t    _millilux[4]    = {0};  // Results, stored as millilux.
    float       _normalized[4]  = {0.0f};  //
    float       _effective_res[4] = {0.0f};
    uint8_t     _shadows[(uint8_t) OPT4060Register::INVALID << 1] = {0};

    I2CBusOp    _busop_trig_irq;
    I2CBusOp    _busop_chan_refresh;

    MicrosTimeout _chan_read_timer;

    int8_t   _ll_pin_init();

    void     _reset_register_values();
    uint16_t _get_shadow_value(OPT4060Register);
    void     _set_shadow_value(OPT4060Register, uint16_t);
    int8_t   _read_registers(OPT4060Register reg, uint8_t len);
    int8_t   _write_registers(OPT4060Register reg, uint8_t len);
    uint16_t _data_period_ms();
    bool     _need_to_read();       // Is data waiting for retrieval?
    int8_t   _read_channels();      // Reads the channel registers.
    int8_t   _trigger_conversion(); // Triggers the sensor, if such a thing is possible.

    int8_t _impart_config();

    int8_t _set_conversion_rate(OPT4060DataRate);
    int8_t _set_dynamic_range(OPT4060DynRange);
    int8_t _set_mode(OPT4060Mode);
    int8_t _set_irq_pin_mode(OPT4060IntPin);



    // The sensor's response is reported logarithmically. This function applies
    //   corrective adjustments to the shadow register values, and converts the
    //   results into uint32_t. These are then stored in the private _millilux
    //   member for retreival.
    int8_t _normalize_data();

    /* Semantic breakouts for flags and conditions. */
    inline bool     _is_measuring() {      return (_opt4060_flag(OPT4060_FLAG_MEASURING) | (OPT4060Mode::CONTINUOUS == _opts.mode));    };
    inline bool     _io_in_flight() {      return _opt4060_flag(OPT4060_FLAG_IO_IN_FLIGHT);     };
    inline bool     _pins_configured() {   return _opt4060_flag(OPT4060_FLAG_PINS_CONFIGURED);  };
    inline bool     _have_trig_busop() {   return _opt4060_flag(OPT4060_FLAG_INT_BUSOP_IS_TRIG);    };
    inline bool     _pin_regs_known() {    return (OPT4060_FLAG_PIN_CONF_MASK == (_flags & OPT4060_FLAG_PIN_CONF_MASK));  };
    inline bool     _all_regs_known() {    return (OPT4060_FLAG_ALL_CONF_MASK == (_flags & OPT4060_FLAG_ALL_CONF_MASK));  };


    /* Flag manipulation inlines */
    inline uint16_t _opt4060_flags() {               return _flags;          };
    inline bool _opt4060_flag(uint16_t flag) {       return (_flags & flag); };
    inline void _opt4060_clear_flag(uint16_t flag) { _flags &= ~flag;        };
    inline void _opt4060_set_flag(uint16_t flag) {   _flags |= flag;         };
    inline void _opt4060_set_flag(uint16_t flag, bool nu) {
      if (nu) _flags |= flag;
      else    _flags &= ~flag;
    };
};

#endif  // __OPT4060_DRIVER_H_
