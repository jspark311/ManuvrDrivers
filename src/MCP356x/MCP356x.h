#ifndef __MCP356x_H__
#define __MCP356x_H__

#include <inttypes.h>
#include <stdint.h>
#include <stdarg.h>
#include <CppPotpourri.h>
#include <SPIAdapter.h>
#include <StringBuilder.h>

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
* Bias current boost ratio.
* Enum values convert directly into register values.
*/
enum class MCP356xBiasCurrent : uint8_t {
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


/*
* Class flags.
* NOTE: MCP356X_FLAG_USE_INTERNAL_CLK takes priority over
*   MCP356X_FLAG_GENERATE_MCLK to avoid potential pin contention. If both flags
*   are set, the MCLK pin (if given) will be configured as an input, and the
*   flag directing the class to generate a clock on that pin will be ignored.
*/
#define MCP356X_FLAG_DEVICE_PRESENT   0x00000001  // Part is likely an MCP356x.
#define MCP356X_FLAG_PINS_CONFIGURED  0x00000002  // Low-level pin setup is complete.
#define MCP356X_FLAG_INITIALIZED      0x00000004  // Registers are initialized.
#define MCP356X_FLAG_CALIBRATED       0x00000008  // ADC is calibrated.
#define MCP356X_FLAG_VREF_DECLARED    0x00000010  // The application has given us Vref.
#define MCP356X_FLAG_CRC_ERROR        0x00000020  // The chip has reported a CRC error.
#define MCP356X_FLAG_USE_INTERNAL_CLK 0x00000040  // The chip should use its internal oscillator.
#define MCP356X_FLAG_MCLK_RUNNING     0x00000080  // MCLK is running.
#define MCP356X_FLAG_SAMPLED_AVDD     0x00000100  // This calibration-related channel was sampled.
#define MCP356X_FLAG_SAMPLED_VCM      0x00000200  // This calibration-related channel was sampled.
#define MCP356X_FLAG_SAMPLED_OFFSET   0x00000400  // This calibration-related channel was sampled.
#define MCP356X_FLAG_3RD_ORDER_TEMP   0x00000800  // Spend CPU to make temperature conversion more accurate?
#define MCP356X_FLAG_GENERATE_MCLK    0x00001000  // MCU is to generate the clock.

#define MCP356X_FLAG_MEASURING_MCLK   0x00002000  // We are in the precal phase of ODR measurement.
#define MCP356X_FLAG_REFRESH_CYCLE    0x00004000  // We are undergoing a full register refresh.
#define MCP356X_FLAG_CALIBRATING      0x00008000  // Driver is executing a calibration.

#define MCP356X_FLAG_RESET_MASK       0x00001853  // Bits to preserve through reset.
#define MCP356X_FLAG_ALL_CAL_MASK     0x00000700  // Bits indicating calibration steps.




class MCP356x : public BusOpCallback {
  public:
    bool isr_fired = false;

    MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin);
    MCP356x(uint8_t irq_pin, uint8_t cs_pin, uint8_t mclk_pin, uint8_t addr);
    ~MCP356x();

    virtual int8_t  reset();
    virtual int8_t  init(SPIAdapter*);
    inline  int8_t  init() {     return init(_BUS);    }
    int8_t  read();
    double  valueAsVoltage(MCP356xChannel);
    int32_t value(MCP356xChannel);

    bool scanComplete();
    inline uint32_t  lastRead() {        return millis_last_read;  };
    inline uint32_t  readCount() {       return read_count;        };
    inline void      resetReadCount() {  read_count = 0;           };

    int8_t  setOption(uint32_t);   // Set flag-based options for the ADC.

    int8_t  setOffsetCalibration(int32_t);
    int8_t  setGainCalibration(int32_t);
    int8_t  setGain(MCP356xGain);
    int8_t  setBiasCurrent(MCP356xBiasCurrent);
    int8_t  setAMCLKPrescaler(MCP356xAMCLKPrescaler);
    int8_t  setOversamplingRatio(MCP356xOversamplingRatio);
    int8_t  calibrate();
    MCP356xOversamplingRatio getOversamplingRatio();
    MCP356xGain getGain();

    int8_t  setScanChannels(int count, ...);
    int8_t  setReferenceRange(float plus, float minus);
    int8_t  refresh();
    inline uint8_t getIRQPin() {      return _IRQ_PIN;  };
    inline bool    adcFound() {       return _mcp356x_flag(MCP356X_FLAG_DEVICE_PRESENT);  };
    inline bool    adcConfigured() {  return _mcp356x_flag(MCP356X_FLAG_INITIALIZED);     };
    inline bool    adcCalibrating() { return _mcp356x_flag(MCP356X_FLAG_CALIBRATING);     };
    inline bool    adcCalibrated() {  return _mcp356x_flag(MCP356X_FLAG_CALIBRATED);      };

    bool isrFired() {    return isr_fired;   };

    void   discardUnsettledSamples();
    float  getTemperature();
    inline uint16_t getSampleRate() {               return reads_per_second;   };
    inline double   getMCLKFrequency() {            return _mclk_freq;         };
    inline uint32_t getSettlingTime() {             return _settling_ms;       };
    inline uint32_t getCircuitSettleTime() {        return _circuit_settle_ms; };
    inline void setCircuitSettleTime(uint32_t ms) { _circuit_settle_ms = ms;   };

    /* Functions for output and debug. */
    void printPins(StringBuilder*);
    void printRegs(StringBuilder*);
    void printTimings(StringBuilder*);
    void printData(StringBuilder*);
    void printChannelValues(StringBuilder*);
    void printChannel(MCP356xChannel, StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  protected:
    int8_t  _clear_registers();


  private:
    // Pin assignments
    const uint8_t  _IRQ_PIN;
    const uint8_t  _CS_PIN;
    const uint8_t  _MCLK_PIN;
    const uint8_t  _DEV_ADDR;

    SPIBusOp  _busop_irq_read;
    SPIAdapter* _BUS      = nullptr;  // Bus reference.
    double    _mclk_freq  = 0.0;      // MCLK in Hz. Zero means undetected.
    double    _dmclk_freq = 0.0;      // Zero means undetected.
    double    _drclk_freq = 0.0;      // Data rate in Hz. Zero means undetected.
    float     _vref_plus  = 3.3;      // Voltage at the reference pins.
    float     _vref_minus = 0.0;      // Voltage at the reference pins.

    uint32_t _reg_shadows[16];     // Register shadows. NOTE: Values are the native MSB where multibyte.
    int32_t  channel_vals[16];    // Normalized channel values.
    uint32_t _flags                = 0;
    uint32_t _channel_flags        = 0;
    uint32_t _discard_until_millis = 0;
    uint32_t _circuit_settle_ms    = 0;  // A optional constant from the application.
    uint32_t _settling_ms          = 0;  // Settling time of the ADC alone.
    uint32_t read_count            = 0;
    uint32_t read_accumulator      = 0;
    uint16_t reads_per_second      = 0;
    uint32_t millis_last_read      = 0;
    uint32_t millis_last_window    = 0;
    uint32_t _channel_backup       = 0;  // Holds scan channel selections.
    uint8_t  _slot_number          = 0;


    int8_t  _post_reset_fxn();
    int8_t  _proc_irq_register();
    int8_t  _ll_pin_init();
    uint8_t _get_reg_addr(MCP356xRegister);
    int8_t _send_fast_command(uint8_t cmd);

    uint8_t _channel_count();
    int8_t  _set_scan_channels(uint32_t);
    int8_t  _calibrate_offset();
    int8_t  _mark_calibrated();

    bool   _mclk_in_bounds();
    int8_t _detect_adc_clock();
    double _calculate_input_clock(unsigned long);
    int8_t _recalculate_clk_tree();
    int8_t _recalculate_settling_time();

    int8_t   _set_shadow_value(MCP356xRegister, uint32_t val);
    uint32_t _get_shadow_value(MCP356xRegister);
    int8_t _write_register(MCP356xRegister, uint32_t val);
    int8_t _read_register(MCP356xRegister);
    int8_t _proc_reg_write(MCP356xRegister);
    int8_t _proc_reg_read(MCP356xRegister);

    uint8_t _output_coding_bytes();
    int8_t  _normalize_data_register();
    float   _gain_value();


    inline bool _vref_declared() {  return _mcp356x_flag(MCP356X_FLAG_VREF_DECLARED);  };
    inline bool _scan_covers_channel(MCP356xChannel c) {
      return (0x01 & (_reg_shadows[(uint8_t) MCP356xRegister::SCAN] >> ((uint8_t) c)));
    };

    /* Flag manipulation inlines */
    inline uint32_t _mcp356x_flags() {                return _flags;           };
    inline bool _mcp356x_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void _mcp356x_flip_flag(uint32_t _flag) {  _flags ^= _flag;         };
    inline void _mcp356x_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void _mcp356x_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void _mcp356x_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
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
};

#endif  // __MCP356x_H__
