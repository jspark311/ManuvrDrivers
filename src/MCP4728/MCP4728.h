/*
* MCP4728 4-channel DAC with non-volatile storage.
*
* LDAC and BUSY pins are optional.
*
* TODO: The platform pins are configured properly if provided. But they are
*   presently not used. LDAC will be held low.
*
* TODO: Wall-off the NVM storage functions by a compiler flag for the ENG/CAL builds.
*/

#include "AbstractPlatform.h"
#include "I2CAdapter.h"

/*
* Class flags.
*/
#define MCP4728_FLAG_PINS_CONFIGURED  0x01  // Low-level pin setup is complete.
#define MCP4728_FLAG_DEV_FOUND        0x02  // Device positively identified.
#define MCP4728_FLAG_INITIALIZED      0x04  // Device initialized.

#define MCP4728_FLAG_CHAN_A_DIRTY     0x10
#define MCP4728_FLAG_CHAN_B_DIRTY     0x20
#define MCP4728_FLAG_CHAN_C_DIRTY     0x40
#define MCP4728_FLAG_CHAN_D_DIRTY     0x80

#define MCP4728_FLAG_RESET_MASK       0x03  // Flags that are preserved through reset.


/* Output modes per channel. */
enum class MCP4728PwrState : uint8_t {
  NORMAL       = 0x00,   //
  GND_VIA_1K   = 0x01,   //
  GND_VIA_100K = 0x02,   //
  GND_VIA_500K = 0x03    //
};

/* Voltage reference sources per channel. */
enum class MCP4728Vref : uint8_t {
  VDD       = 0x00,
  INTERNAL  = 0x01
};


class MCP4728 : public I2CDevice {
  public:
    MCP4728(const float ext_vref, const uint8_t ldac_pin, const uint8_t busy_pin, uint8_t address = 0x60);
    ~MCP4728();

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    int8_t  init(I2CAdapter* b);
    int8_t  refresh();

    int8_t          chanPowerState(uint8_t chan, MCP4728PwrState);
    MCP4728PwrState chanPowerState(uint8_t chan);
    int8_t          chanGain(uint8_t chan, uint8_t gain);
    uint8_t         chanGain(uint8_t chan);
    int8_t          chanValue(uint8_t chan, uint16_t value);
    uint16_t        chanValue(uint8_t chan);
    int8_t          chanVref(uint8_t chan, MCP4728Vref);
    MCP4728Vref     chanVref(uint8_t chan);
    inline float chanVoltage(uint8_t chan) {
      return (4 > chan) ? _DAC_VOLTS[chan] : 0.0;
    };
    int8_t  chanStore(uint8_t chan);
    int8_t  storeToNVM();

    /* Accessors taken from flags. */
    inline bool devFound() {     return _mcp4728_flag(MCP4728_FLAG_DEV_FOUND);    };
    inline bool initialized() {  return _mcp4728_flag(MCP4728_FLAG_INITIALIZED);  };
    inline bool chanDirty(uint8_t chan) {
      return _mcp4728_flag(MCP4728_FLAG_CHAN_A_DIRTY << (chan & 0x03));
    };

    void printDebug(StringBuilder*);

    static const char* pwrStateStr(MCP4728PwrState);
    static const char* vrefStr(MCP4728Vref);


  private:
    const float    _EXT_VREF;
    const uint8_t  _LDAC_PIN;
    const uint8_t  _BUSY_PIN;
    uint8_t        _flags     = 0;
    float          _DAC_VOLTS[4]   = {0.0, };
    uint8_t        _DAC_VALUES[24] = {0, };

    //int8_t  _write_channel(uint8_t chan, uint16_t val);
    int8_t  _recalculate_chan_voltage(uint8_t chan);
    int8_t  _ll_pin_init();

    inline bool _have_ldac_pin() {   return (255 != _LDAC_PIN);  };
    inline bool _have_busy_pin() {   return (255 != _BUSY_PIN);  };

    /* Flag manipulation inlines */
    inline uint8_t _mcp4728_flags() {                return _flags;           };
    inline bool _mcp4728_flag(uint8_t _flag) {       return (_flags & _flag); };
    inline void _mcp4728_clear_flag(uint8_t _flag) { _flags &= ~_flag;        };
    inline void _mcp4728_set_flag(uint8_t _flag) {   _flags |= _flag;         };
    inline void _mcp4728_set_flag(uint8_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};
