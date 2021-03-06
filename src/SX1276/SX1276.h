#include <AbstractPlatform.h>
#include <EnumeratedTypeCodes.h>
#include <SPIAdapter.h>

#ifndef __SX1276_DRIVER_H
#define __SX1276_DRIVER_H


enum class LORABand : uint8_t {
  UNASSIGNED = 0x00,
  BAND_433   = 0x01,
  BAND_868   = 0x02,
  BAND_915   = 0x03
};


/* These are the register indicies. NOT their addresses. */
enum class SX127xRegister : uint8_t {
  FIFO                 = 0x00,
  OP_MODE              = 0x01,
  FRF_MSB              = 0x02,
  FRF_MID              = 0x03,
  FRF_LSB              = 0x04,
  PA_CONFIG            = 0x05,
  LNA                  = 0x06,
  FIFO_ADDR_PTR        = 0x07,
  FIFO_TX_BASE_ADDR    = 0x08,
  FIFO_RX_BASE_ADDR    = 0x09,
  FIFO_RX_CURRENT_ADDR = 0x0A,
  IRQ_FLAGS            = 0x0B,
  RX_NB_BYTES          = 0x0C,
  PKT_RSSI_VALUE       = 0x0D,
  PKT_SNR_VALUE        = 0x0E,
  MODEM_CONFIG_1       = 0x0F,
  MODEM_CONFIG_2       = 0x10,
  PREAMBLE_MSB         = 0x11,
  PREAMBLE_LSB         = 0x12,
  PAYLOAD_LENGTH       = 0x13,
  MODEM_CONFIG_3       = 0x14,
  RSSI_WIDEBAND        = 0x15,
  DETECTION_OPTIMIZE   = 0x16,
  DETECTION_THRESHOLD  = 0x17,
  SYNC_WORD            = 0x18,
  DIO_MAPPING_1        = 0x19,
  VERSION              = 0x1A,
  INVALID              = 0x1B
};

enum class SX1276Mode : uint8_t {
  LONG_RANGE_MODE = 0x80,
  SLEEP           = 0x00,
  STDBY           = 0x01,
  TX              = 0x03,
  RX_CONTINUOUS   = 0x05,
  RX_SINGLE       = 0x06
};

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255


/* Class flags */
#define SX127X_FLAG_DEVICE_PRESENT   0x00000001  // Unambiguously found the part.
#define SX127X_FLAG_PINS_CONFIGURED  0x00000002  // Low-level pin setup is complete.
#define SX127X_FLAG_INITIALIZED      0x00000004  // Registers are initialized.

#define SX127X_FLAG_RESET_MASK       0x00000002  // Bits to preserve through reset.



/*******************************************************************************
* Options object
*******************************************************************************/

/**
* Set pin def to 255 to mark it as unused.
*/
class SX1276Opts {
  public:
    const uint8_t reset_pin;
    const uint8_t cs_pin;
    const uint8_t d0_pin;
    const uint8_t d1_pin;
    const uint8_t d2_pin;

    /** Copy constructor. */
    SX1276Opts(const SX1276Opts* o) :
      reset_pin(o->reset_pin),
      cs_pin(o->cs_pin),
      d0_pin(o->d0_pin),
      d1_pin(o->d1_pin),
      d2_pin(o->d2_pin),
      _flags(o->_flags) {};

    /**
    * Constructor.
    *
    * @param pin
    * @param Initial flags
    */
    SX1276Opts(
      uint8_t reset,
      uint8_t cs,
      uint8_t d0,
      uint8_t d1,
      uint8_t d2,
      LORABand band
    ) :
      reset_pin(reset),
      cs_pin(cs),
      d0_pin(d0),
      d1_pin(d1),
      d2_pin(d2),
      _flags((uint8_t) band) {};


    inline LORABand getBand() {     return ((LORABand) (_flags & 0x03));    };


  private:
    const uint8_t _flags;
};


/*******************************************************************************
*
*******************************************************************************/

class SX1276 : public BusOpCallback {
  public:
    SX1276(const SX1276Opts*);
    ~SX1276();

    inline bool    devFound() {      return _sx_flag(SX127X_FLAG_DEVICE_PRESENT);  };
    inline bool    initialized() {   return _sx_flag(SX127X_FLAG_INITIALIZED);     };

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    int8_t init(SPIAdapter*);

    void printDebug(StringBuilder*);
    void printRegs(StringBuilder*);
    inline void fetchLogs(StringBuilder* output) {
      if (local_log.length() > 0) {
        output->concatHandoff(&local_log);
      }
    };


  private:
    const SX1276Opts _opts;
    SPIAdapter* _BUS = nullptr;
    uint32_t _flags  = 0;
    uint8_t _tx_buffer[512] = {0, };
    uint8_t _rx_buffer[512] = {0, };
    uint8_t _shadows[27]    = {0, };
    StringBuilder local_log;
    SPIBusOp _tx_busop;
    SPIBusOp _rx_busop;

    int8_t _ll_pin_init();

    /* Flag manipulation inlines */
    inline uint32_t _sx_flags() {                return _flags;           };
    inline bool _sx_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void _sx_flip_flag(uint32_t _flag) {  _flags ^= _flag;         };
    inline void _sx_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void _sx_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void _sx_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };

    static uint8_t _get_reg_addr(const SX127xRegister);
    static SX127xRegister _reg_id_from_addr(const uint8_t addr);
};

#endif  // __SX1276_DRIVER_H
