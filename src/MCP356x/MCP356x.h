#ifndef __MCP356x_H__
#define __MCP356x_H__

#include <inttypes.h>
#include <stdint.h>
#include <stdarg.h>
#include "CppPotpourri.h"
#include "BusQueue/SPIAdapter.h"
#include "TimerTools/TimerTools.h"
#include "FiniteStateMachine.h"
#include "StringBuilder.h"
#include "FlagContainer.h"


/* In this case, these enum values translate directly to register addresses. */
enum class MCP356xRegister : uint8_t {
  ADCDATA   = 0x00,   // (it's complicated)  R
  CONFIG0   = 0x01,   // 8-bit               RW
  CONFIG1   = 0x02,   // 8-bit               RW
  CONFIG2   = 0x03,   // 8-bit               RW
  CONFIG3   = 0x04,   // 8-bit               RW
  IRQ       = 0x05,   // 8-bit               RW
  MUX       = 0x06,   // 8-bit               RW
  SCAN      = 0x07,   // 24-bit              RW
  TIMER     = 0x08,   // 24-bit              RW
  OFFSETCAL = 0x09,   // 24-bit              RW
  GAINCAL   = 0x0A,   // 24-bit              RW
  RESERVED0 = 0x0B,   // 24-bit              RW
  RESERVED1 = 0x0C,   // 8-bit               RW
  LOCK      = 0x0D,   // 8-bit               RW
  RESERVED2 = 0x0E,   // 16-bit              RW
  CRCCFG    = 0x0F    // 16-bit              R
};

/* The ADC is divided into 16 logical channels. */
enum class MCP356xChannel : uint8_t {
  SE_0   = 0x00,
  SE_1   = 0x01,
  SE_2   = 0x02,
  SE_3   = 0x03,
  SE_4   = 0x04,
  SE_5   = 0x05,
  SE_6   = 0x06,
  SE_7   = 0x07,
  DIFF_A = 0x08,
  DIFF_B = 0x09,
  DIFF_C = 0x0A,
  DIFF_D = 0x0B,
  TEMP   = 0x0C,  // Internal temperature sensor.
  AVDD   = 0x0D,  //
  VCM    = 0x0E,
  OFFSET = 0x0F
};

/* Conversion modes */
enum class MCP356xMode : uint8_t {
  ONESHOT_SHUTDOWN = 0,
  STANDBY          = 1,
  ONESHOT_STANDBY  = 2,
  CONTINUOUS       = 3
};

/*
* ADC gain ratio.
* Enum values convert directly into register values.
*/
enum class MCP356xGain : uint8_t {
  GAIN_ONETHIRD = 0,
  GAIN_1        = 1,
  GAIN_2        = 2,
  GAIN_4        = 3,
  GAIN_8        = 4,
  GAIN_16       = 5,
  GAIN_32       = 6,
  GAIN_64       = 7
};

/*
* Bias current.
* Enum values convert directly into register values.
*/
enum class MCP356xBiasCurrent : uint8_t {
  NONE           = 0,
  NANOAMPS_900   = 1,
  NANOAMPS_3700  = 2,
  NANOAMPS_15000 = 3
};

/*
* Bias current boost ratio.
* Enum values convert directly into register values.
*/
enum class MCP356xBiasBoost : uint8_t {
  HALF      = 0,
  TWOTHIRDS = 1,
  ONE       = 2,
  DOUBLE    = 3
};

/*
* Enum values convert directly into register values.
*/
enum class MCP356xOversamplingRatio : uint8_t {
  OSR_32    = 0x00,
  OSR_64    = 0x01,
  OSR_128   = 0x02,
  OSR_256   = 0x03,  // Default on reset.
  OSR_512   = 0x04,
  OSR_1024  = 0x05,
  OSR_2048  = 0x06,
  OSR_4096  = 0x07,
  OSR_8192  = 0x08,
  OSR_16384 = 0x09,
  OSR_20480 = 0x0A,
  OSR_24576 = 0x0B,
  OSR_40960 = 0x0C,
  OSR_49152 = 0x0D,
  OSR_81920 = 0x0E,
  OSR_98304 = 0x0F
};

/* Clock prescaler options. */
enum class MCP356xAMCLKPrescaler : uint8_t {
  OVER_1 = 0,
  OVER_2 = 1,
  OVER_4 = 2,
  OVER_8 = 3
};


/**
* Driver state machine positions
*/
enum class MCP356xState : uint8_t {
  UNINIT = 0,     // init() has never been called.
  PRE_INIT,       // Pin control is being established.
  RESETTING,      // Driver is resetting the ADC.
  DISCOVERY,      // Driver is probing for the ADC.
  POST_INIT,      // The initial ADC configuration is being written.
  CLK_MEASURE,    // Driver is measuring the clock.
  CALIBRATION,    // The ADC is self-calibrating.
  USR_CONF,       // User config is being written.
  IDLE,           // Powered up and calibrated, but not reading.
  READING,        // Everything running, data collection proceeding.
  FAULT,          // State machine encountered something it couldn't cope with.
  INVALID = 255   // FSM hygiene.
};


/* A callback to be notified of new values. */
typedef void (*ADCValueCallback)(MCP356xChannel chan, double voltage);


/*
* Class flags.
* NOTE: MCP356X_FLAG_USE_INTRNL_CLK takes priority over
*   MCP356X_FLAG_GENERATE_MCLK to avoid potential pin contention. If both flags
*   are set, the MCLK pin (if given) will be configured as an input, and the
*   flag directing the class to generate a clock on that pin will be ignored.
*/
// These are tracking bits for volatile and transient conditions.
#define MCP356X_FLAG_DEVICE_PRESENT   0x00000001  // Part is likely an MCP356x.
#define MCP356X_FLAG_PINS_CONFIGURED  0x00000002  // Low-level pin setup is complete.

#define MCP356X_FLAG_WROTE_OFFSET_CAL 0x00000008  // ADC is calibrated.
#define MCP356X_FLAG_VREF_DECLARED    0x00000010  // The application has given us Vref.
#define MCP356X_FLAG_CRC_ERROR        0x00000020  // The chip has reported a CRC error.
#define MCP356X_FLAG_MCLK_RUNNING     0x00000040  // MCLK is running.
#define MCP356X_FLAG_SAMPLED_AVDD     0x00000080  // This calibration-related channel was sampled.
#define MCP356X_FLAG_SAMPLED_VCM      0x00000100  // This calibration-related channel was sampled.
#define MCP356X_FLAG_SAMPLED_OFFSET   0x00000200  // This calibration-related channel was sampled.
#define MCP356X_FLAG_REFRESH_CYCLE    0x00000400  // We are undergoing a full register refresh.
#define MCP356X_FLAG_HAS_INTRNL_VREF  0x00000800  // This part was found to support an internal Vref.
#define MCP356X_FLAG_SERVICING_IRQS   0x00001000  // The class will respond to IRQ signals.
#define MCP356X_FLAG_STATE_HOLD       0x00002000  // The present state is being held.
// These are settable options.
#define MCP356X_FLAG_USE_INTRNL_VREF  0x10000000  // Internal Vref should be enabled.
#define MCP356X_FLAG_USE_INTRNL_CLK   0x20000000  // The chip should use its internal oscillator.
#define MCP356X_FLAG_3RD_ORDER_TEMP   0x40000000  // Spend CPU to make temperature conversion more accurate?
#define MCP356X_FLAG_GENERATE_MCLK    0x80000000  // MCU is to generate the clock.

// Bits indicating calibration steps.
#define MCP356X_FLAG_ALL_CAL_MASK ( \
  MCP356X_FLAG_SAMPLED_AVDD | MCP356X_FLAG_SAMPLED_VCM | \
  MCP356X_FLAG_SAMPLED_OFFSET | MCP356X_FLAG_WROTE_OFFSET_CAL)

// Bits that track static configurations.
#define MCP356X_FLAG_DRIVER_OPTS_MASK ( \
  MCP356X_FLAG_USE_INTRNL_VREF | MCP356X_FLAG_USE_INTRNL_CLK | \
  MCP356X_FLAG_3RD_ORDER_TEMP | MCP356X_FLAG_GENERATE_MCLK)

// Bits to preserve through reset.
#define MCP356X_FLAG_RESET_MASK  (MCP356X_FLAG_DEVICE_PRESENT | MCP356X_FLAG_PINS_CONFIGURED | \
                                  MCP356X_FLAG_VREF_DECLARED | MCP356X_FLAG_DRIVER_OPTS_MASK | \
                                  MCP356X_FLAG_HAS_INTRNL_VREF)


/* A class to hold enum'd config for the ADC. */
class MCP356xConfig {
  public:
    uint32_t                   scan;
    uint32_t                   flags;
    MCP356xMode                mode;
    MCP356xGain                gain;
    MCP356xBiasCurrent         bias;
    MCP356xOversamplingRatio   over;
    MCP356xAMCLKPrescaler      prescaler;

    MCP356xConfig() : scan(0), flags(0),
        mode(MCP356xMode::ONESHOT_STANDBY),
        gain(MCP356xGain::GAIN_1),
        bias(MCP356xBiasCurrent::NONE),
        over(MCP356xOversamplingRatio::OSR_256),
        prescaler(MCP356xAMCLKPrescaler::OVER_1) {};

    MCP356xConfig(
      const uint32_t SCAN,
      const uint32_t FLAGS,
      const MCP356xMode MODE,
      const MCP356xGain GAIN,
      const MCP356xBiasCurrent BIAS,
      const MCP356xOversamplingRatio OVER,
      const MCP356xAMCLKPrescaler PRESCALER
    ) : scan(SCAN), flags(FLAGS),
        mode(MODE), gain(GAIN), bias(BIAS),
        over(OVER), prescaler(PRESCALER) {};

    MCP356xConfig(const MCP356xConfig* CFG) : scan(CFG->scan), flags(CFG->flags),
        mode(CFG->mode), gain(CFG->gain), bias(CFG->bias),
        over(CFG->over), prescaler(CFG->prescaler) {};


    ~MCP356xConfig() {};
};


class MCP356x : public StateMachine<MCP356xState>, public C3PPollable, public BusOpCallback {
  public:
    MCP356x(
      const uint8_t irq_pin,
      const uint8_t cs_pin,
      const uint8_t mclk_pin,
      const uint8_t addr,
      MCP356xConfig*
    );
    ~MCP356x();

    int8_t     reset();
    int8_t     init(SPIAdapter* bus = nullptr);
    int8_t     deinit();
    int8_t     refresh();
    PollResult poll();
    bool       isIdle();
    inline void setAdapter(SPIAdapter* b) {  _BUS = b;     };

    double  valueAsVoltage(MCP356xChannel);
    int32_t value(MCP356xChannel);

    bool scanComplete();
    inline void      verbosity(uint8_t v) {       _log_verbosity = v;      };
    inline uint8_t   verbosity() {                return _log_verbosity;   };
    inline void      scansRequested(int32_t x) {  _scans_req = x;          };
    inline int32_t   scansRequested() {           return _scans_req;       };
    inline uint32_t  lastRead() {        return micros_last_read;  };
    inline uint32_t  readCount() {       return _profiler_result_read.executions();  };
    inline void      resetReadCount() {  _profiler_result_read.reset();              };

    int8_t  setOption(uint32_t);   // Set flag-based options for the ADC.

    int8_t  setOffsetCalibration(int32_t);
    int8_t  setGainCalibration(int32_t);

    int8_t  calibrate();
    int8_t  readSamples(int32_t scan_count);

    MCP356xOversamplingRatio getOversamplingRatio();
    MCP356xGain getGain();
    MCP356xBiasCurrent getBiasCurrent();
    MCP356xAMCLKPrescaler getAMCLKPrescaler();
    int8_t      readMode(MCP356xMode);
    MCP356xMode readMode();

    int8_t setGain(MCP356xGain);
    int8_t setBiasCurrent(MCP356xBiasCurrent);
    int8_t setOversamplingRatio(MCP356xOversamplingRatio);
    int8_t setAMCLKPrescaler(MCP356xAMCLKPrescaler);

    int8_t  setScanChannels(int count, ...);
    int8_t  setReferenceRange(float plus, float minus);
    inline void    setMCLKFrequency(double x) {  _mclk_freq = x;  };
    inline uint8_t getIRQPin() {        return _IRQ_PIN;  };
    inline bool    adcFound() {         return _flags.value(MCP356X_FLAG_DEVICE_PRESENT);     };
    inline bool    adcConfigured() {    return (_config_is_desired() & _config_is_written()); };
    inline bool    adcCalibrating() {   return (currentState() == MCP356xState::CALIBRATION); };
    inline bool    adcCalibrated() {    return _flags.all_set(MCP356X_FLAG_ALL_CAL_MASK);     };
    inline bool    hasInternalVref() {  return _flags.value(MCP356X_FLAG_HAS_INTRNL_VREF);    };
    bool   usingInternalVref();
    int8_t useInternalVref(bool);

    void   isr_fxn();
    inline int8_t busPriority() {          return _bus_priority;  };
    inline void   busPriority(int8_t p) {  _bus_priority = p;     };
    inline bool io_in_flight() {   return (_io_dispatched > _io_called_back);  };

    void   discardUnsettledSamples();
    float  getTemperature();
    uint16_t getSampleRate();

    inline double   getMCLKFrequency() {            return _mclk_freq;         };
    inline uint32_t getSettlingTime() {             return _settling_ms;       };
    inline uint32_t getCircuitSettleTime() {        return _circuit_settle_ms; };
    inline void setCircuitSettleTime(uint32_t ms) { _circuit_settle_ms = ms;   };

    /* Functions for output and debug. */
    void printPins(StringBuilder*);
    void printRegs(StringBuilder*);
    void printTimings(StringBuilder*);
    void printDebug(StringBuilder*);
    void printChannelValues(StringBuilder*);
    void printChannel(MCP356xChannel, StringBuilder*);

    inline void valueCallback(ADCValueCallback x) { _value_callback = x; };

    /* Built-in per-instance console handler. */
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    static const char* const stateStr(const MCP356xState);



  private:
    // Pin assignments
    const uint8_t  _IRQ_PIN;
    const uint8_t  _CS_PIN;
    const uint8_t  _MCLK_PIN;
    const uint8_t  _DEV_ADDR;
    MCP356xConfig* _desired_conf;   // TODO: Delta between these things should precipitate a re-conf cycle.
    MCP356xConfig  _current_conf;
    ADCValueCallback _value_callback = nullptr;
    uint32_t  _io_dispatched   = 0;   // How many I/O operations were sent out?
    uint32_t  _io_called_back  = 0;   // How many I/O operations came back?
    uint32_t  _irqs_noted      = 0;   // How many IRQs hit the pin?
    uint32_t  _irqs_serviced   = 0;   // How many IRQs triggered read operations?
    int32_t   _scans_req       = 0;   // How many full scan loops have we yet to take?

    FlagContainer32  _flags;                 // Aggregate boolean class state.
    StopWatch        _profiler_irq_timing;   // Profiler for IRQ response.
    StopWatch        _profiler_result_read;  // Profiler for sample readings.
    StopWatch        _profiler_full_scan;    // Profiler for full scan results.
    MillisTimeout    _discard_window;        // Enforces periods of result non-obsesrvance.
    uint32_t micros_last_read      = 0;
    uint32_t micros_last_window    = 0;

    SPIBusOp  _busop_irq_read;
    SPIBusOp  _busop_dat_read;
    SPIAdapter* _BUS      = nullptr;  // Bus reference.
    double    _mclk_freq  = 0.0;      // MCLK in Hz. Zero means undetected.
    double    _dmclk_freq = 0.0;      // Zero means undetected.
    double    _drclk_freq = 0.0;      // Data rate in Hz. Zero means undetected.
    float     _vref_plus  = 3.3;      // Voltage at the reference pins.
    float     _vref_minus = 0.0;      // Voltage at the reference pins.

    uint32_t _reg_shadows[16];     // Register shadows. NOTE: Values are the native MSB where multibyte.
    int32_t  channel_vals[16];    // Normalized channel values.
    uint32_t _channel_flags        = 0;
    uint32_t _circuit_settle_ms    = 0;  // A optional constant from the application.
    uint32_t _settling_ms          = 0;  // Settling time of the ADC alone.
    uint32_t read_accumulator      = 0;
    int8_t   _bus_priority         = 0;  // Defaults to neutral.
    int8_t   _log_verbosity        = LOG_LEV_ERROR;
    uint8_t  _slot_number          = 0;
    bool     _isr_fired            = false;

    /* State machine functions */
    int8_t   _fsm_poll();                        // Polling for state exit.
    int8_t   _fsm_set_position(MCP356xState);    // Attempt a state entry.
    int8_t   _ll_pin_init();
    int8_t   _reset_fxn();
    int8_t   _post_reset_fxn();
    int8_t   _calibrate();
    void     _set_fault(const char*);

    /* Config-write functions that actually dispatch I/O */
    int8_t  _set_gain(MCP356xGain);
    int8_t  _set_bias_current(MCP356xBiasCurrent);
    int8_t  _set_amlclk_prescaler(MCP356xAMCLKPrescaler);
    int8_t  _set_oversampling_ratio(MCP356xOversamplingRatio);
    int8_t  _set_read_mode(MCP356xMode);

    /* Mid-level register abstraction */
    uint8_t  _channel_count();
    int8_t   _set_scan_channels(uint32_t);
    int8_t   _normalize_data_register();
    uint8_t  _output_coding_bytes();
    float    _gain_value();

    int8_t  _apply_usr_config();
    bool    _config_is_written();
    bool    _config_is_desired();

    double _calculate_input_clock(unsigned long);
    int8_t _recalculate_clk_tree();
    int8_t _recalculate_settling_time();

    /* Basal register functions */
    void     _clear_registers();
    uint8_t  _get_reg_addr(MCP356xRegister);
    int8_t   _set_shadow_value(MCP356xRegister, uint32_t val);
    uint32_t _get_shadow_value(MCP356xRegister);
    int8_t   _write_register(MCP356xRegister, uint32_t val);
    int8_t   _read_register(MCP356xRegister);
    int8_t   _proc_reg_write(MCP356xRegister);
    int8_t   _proc_reg_read(MCP356xRegister);
    int8_t   _proc_irq_register();
    int8_t   _send_fast_command(uint8_t cmd);

    inline void _servicing_irqs(bool x) {  _flags.set(MCP356X_FLAG_SERVICING_IRQS, x);        };
    inline bool _servicing_irqs() {        return _flags.value(MCP356X_FLAG_SERVICING_IRQS);  };
    inline bool _vref_declared() {         return _flags.value(MCP356X_FLAG_VREF_DECLARED);   };
    bool         _mclk_in_bounds() {  return ((_mclk_freq >= 100000.0) && (_mclk_freq <= 20000000.0));  };
    inline bool _scan_covers_channel(MCP356xChannel c) {
      return (0x01 & (_reg_shadows[(uint8_t) MCP356xRegister::SCAN] >> ((uint8_t) c)));
    };

    /* Flag manipulation inlines for individual channels */
    inline void _channel_clear_new_flag(MCP356xChannel c) { _channel_flags &= ~(1 << (uint8_t) c);                 };
    inline void _channel_set_new_flag(MCP356xChannel c) {   _channel_flags |= (1 << (uint8_t) c);                  };
    inline bool _channel_has_new_value(MCP356xChannel c) {  return (_channel_flags & (1 << (uint8_t) c));          };
    inline void _channel_clear_ovr_flag(MCP356xChannel c) { _channel_flags &= ~(0x00010000 << (uint8_t) c);        };
    inline void _channel_set_ovr_flag(MCP356xChannel c) {   _channel_flags |= (0x00010000 << (uint8_t) c);         };
    inline void _channel_set_ovr_flag(MCP356xChannel c, bool nu) {
      _channel_flags = (nu) ? (_channel_flags | (0x00010000 << (uint8_t) c)) : (_channel_flags & ~(0x00010000 << (uint8_t) c));
    };
    inline bool _channel_over_range(MCP356xChannel c) {     return (_channel_flags & (0x00010000 << (uint8_t) c)); };

    static const uint16_t OSR1_VALUES[16];
    static const uint16_t OSR3_VALUES[16];
    static const EnumDefList<MCP356xState> _FSM_STATES;
    static constexpr const char* const MCP356X_LOG_TAG = "MCP356x";
};

#endif  // __MCP356x_H__
