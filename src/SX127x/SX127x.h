/*******************************************************************************
*
*******************************************************************************/

#ifndef __SX127X_DRIVER_H
#define __SX127X_DRIVER_H

#include "AbstractPlatform.h"
#include "EnumeratedTypeCodes.h"
#include "BusQueue/SPIAdapter.h"
#include "FlagContainer.h"

#define SX127X_FSM_WAYPOINT_DEPTH  16


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

enum class SX127xMode : uint8_t {
  LONG_RANGE_MODE = 0x80,
  SLEEP           = 0x00,
  STDBY           = 0x01,
  TX              = 0x03,
  RX_CONTINUOUS   = 0x05,
  RX_SINGLE       = 0x06
};


enum class SX127xState : uint8_t {
  UNINIT     = 0,  // init() has never been called.
  PREINIT    = 1,  // Pin control is being established.
  RESETTING  = 2,  // Driver is resetting the hardware.
  DISCOVERY  = 3,  // Driver is probing for the hardware.
  REGINIT    = 4,  // The initial hardware configuration is being written.
  USR_CONF   = 5,  // User config is being written.
  READY      = 6,  // Hardware and driver agree on state, and are ready to use.
  LOW_PWR    = 7,  // The hardware is in a low-power mode.
  FAULT      = 8   // State machine encountered something it couldn't cope with.
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

// Flags to preserve through reset.
#define SX127X_FLAG_RESET_MASK (SX127X_FLAG_PINS_CONFIGURED)



/*******************************************************************************
* Options object
*******************************************************************************/

/**
* Set pin def to 255 to mark it as unused.
*/
class SX127xOpts {
  public:
    const uint8_t reset_pin;
    const uint8_t cs_pin;
    const uint8_t d0_pin;
    const uint8_t d1_pin;
    const uint8_t d2_pin;

    /** Copy constructor. */
    SX127xOpts(const SX127xOpts* o) :
      reset_pin(o->reset_pin), cs_pin(o->cs_pin),
      d0_pin(o->d0_pin), d1_pin(o->d1_pin), d2_pin(o->d2_pin),
      _flags(o->_flags) {};

    /**
    * Constructor.
    *
    * @param pin
    * @param Initial flags
    */
    SX127xOpts(
      uint8_t reset,
      uint8_t cs,
      uint8_t d0,
      uint8_t d1,
      uint8_t d2,
      LORABand band
    ) :
      reset_pin(reset), cs_pin(cs), d0_pin(d0), d1_pin(d1), d2_pin(d2),
      _flags((uint8_t) band) {};


    inline LORABand band() {     return ((LORABand) (_flags & 0x03));    };


  private:
    const uint8_t _flags;   // Holds DIO states/directions/behaviors.
};


/*******************************************************************************
*
*******************************************************************************/

class SX127x : public BusOpCallback {
  public:
    SX127x(const SX127xOpts*);
    ~SX127x();

    int8_t init(SPIAdapter* b = nullptr);
    int8_t reset();
    int8_t refresh();
    inline void setAdapter(SPIAdapter* b = nullptr) {  _BUS = b;  };

    /* Flag wrappers */
    inline bool devFound() {      return _flags.value(SX127X_FLAG_DEVICE_PRESENT);  };
    inline bool initialized() {   return _flags.value(SX127X_FLAG_INITIALIZED);     };

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    /* Settings for band and relay legality. */
    void markBandIllegal(LORABand);   // Such bands will be treated as listen-only.

    void printDebug(StringBuilder*);
    void printRegs(StringBuilder*);

    /* Built-in per-instance console handler. */
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);


  private:
    const SX127xOpts _opts;
    FlagContainer32  _flags;       // Class state tracking.
    SPIAdapter*      _BUS = nullptr;
    uint32_t         _pkts_rx_good = 0;
    uint32_t         _pkts_rx_drop = 0;
    uint32_t         _pkts_tx_good = 0;
    uint32_t         _pkts_tx_drop = 0;
    uint8_t          _shadows[27]    = {0, };
    uint8_t          _verbosity      = LOG_LEV_NOTICE;
    uint8_t          _tx_buffer[512] = {0, };
    uint8_t          _rx_buffer[512] = {0, };
    SPIBusOp         _tx_busop;
    SPIBusOp         _rx_busop;

    /* FSM markers */
    uint32_t    _fsm_lockout_ms; // Used to enforce a delay between state transitions.
    SX127xState _fsm_waypoints[SX127X_FSM_WAYPOINT_DEPTH] = {SX127xState::UNINIT, };
    SX127xState _fsm_pos;
    SX127xState _fsm_pos_prior;

    int8_t _ll_pin_init();

    /* State machine functions */
    int8_t   _poll_fsm();
    int8_t   _set_fsm_position(SX127xState);
    int8_t   _set_fsm_route(int count, ...);
    int8_t   _append_fsm_route(int count, ...);
    int8_t   _prepend_fsm_state(SX127xState);
    int8_t   _advance_state_machine();
    bool     _fsm_is_waiting();
    void     _print_fsm(StringBuilder*);
    inline SX127xState _fsm_pos_next() {   return _fsm_waypoints[0];   };
    inline bool        _fsm_is_stable() {  return (SX127xState::UNINIT == _fsm_waypoints[0]);   };
};

#endif  // __SX127X_DRIVER_H
