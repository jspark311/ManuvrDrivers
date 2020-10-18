#include "SX1276.h"


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
/* Real register addresses */
static const uint8_t SX127X_REG_ADDR[27] = {
  0x00, 0x01, 0x06, 0x07, 0x08, 0x09, 0x0c, 0x0d,
  0x0e, 0x0f, 0x10, 0x12, 0x13, 0x1a, 0x1b, 0x1d,
  0x1e, 0x20, 0x21, 0x22, 0x26, 0x2c, 0x31, 0x37,
  0x39, 0x40, 0x42
};


/**
* Applies the device address and properly shifts the register address into
*   a control byte. Always sets up for incremental read/write.
* This never fails and always returns a byte to be used as the first byte
*   in an SPI transaction with the ADC.
*
* @param r The register we wish to transact with.
* @return the first byte of an SPI register transaction.
*/
uint8_t SX1276::_get_reg_addr(const SX127xRegister r) {
  uint8_t reg_idx = (uint8_t) r;
  if (27 > reg_idx) {
    return SX127X_REG_ADDR[reg_idx];
  }
  return 0;
}


/*
* This is an ISR.
*/
void sx127x_d0_isr() {
}

/*
* This is an ISR.
*/
void sx127x_d1_isr() {
}

/*
* This is an ISR.
*/
void sx127x_d2_isr() {
}


SX127xRegister SX1276::_reg_id_from_addr(const uint8_t addr) {
  switch (0x7F & addr) {
    case 0x00:   return SX127xRegister::FIFO;
    case 0x01:   return SX127xRegister::OP_MODE;
    case 0x06:   return SX127xRegister::FRF_MSB;
    case 0x07:   return SX127xRegister::FRF_MID;
    case 0x08:   return SX127xRegister::FRF_LSB;
    case 0x09:   return SX127xRegister::PA_CONFIG;
    case 0x0c:   return SX127xRegister::LNA;
    case 0x0d:   return SX127xRegister::FIFO_ADDR_PTR;
    case 0x0e:   return SX127xRegister::FIFO_TX_BASE_ADDR;
    case 0x0f:   return SX127xRegister::FIFO_RX_BASE_ADDR;
    case 0x10:   return SX127xRegister::FIFO_RX_CURRENT_ADDR;
    case 0x12:   return SX127xRegister::IRQ_FLAGS;
    case 0x13:   return SX127xRegister::RX_NB_BYTES;
    case 0x1a:   return SX127xRegister::PKT_RSSI_VALUE;
    case 0x1b:   return SX127xRegister::PKT_SNR_VALUE;
    case 0x1d:   return SX127xRegister::MODEM_CONFIG_1;
    case 0x1e:   return SX127xRegister::MODEM_CONFIG_2;
    case 0x20:   return SX127xRegister::PREAMBLE_MSB;
    case 0x21:   return SX127xRegister::PREAMBLE_LSB;
    case 0x22:   return SX127xRegister::PAYLOAD_LENGTH;
    case 0x26:   return SX127xRegister::MODEM_CONFIG_3;
    case 0x2c:   return SX127xRegister::RSSI_WIDEBAND;
    case 0x31:   return SX127xRegister::DETECTION_OPTIMIZE;
    case 0x37:   return SX127xRegister::DETECTION_THRESHOLD;
    case 0x39:   return SX127xRegister::SYNC_WORD;
    case 0x40:   return SX127xRegister::DIO_MAPPING_1;
    case 0x42:   return SX127xRegister::VERSION;
  }
  return SX127xRegister::INVALID;
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/* Constructor */
SX1276::SX1276(const SX1276Opts* o) : _opts{o} {
}


/* Destructor */
SX1276::~SX1276() {
}


int8_t SX1276::init(SPIAdapter* b) {
  int8_t pin_setup_ret = _ll_pin_init();  // Configure the pins if they are not already.
  int8_t ret = -4;
  if (pin_setup_ret >= 0) {
    ret = -3;
    if (nullptr != b) {
      ret = -2;
      _BUS = b;
      _tx_busop.setAdapter(_BUS);
      _tx_busop.shouldReap(false);
      //_tx_busop.setParams((uint8_t) _get_reg_addr(MCP356xRegister::IRQ) | 0x01);
      //_tx_busop.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) MCP356xRegister::IRQ], 1);

      _rx_busop.setAdapter(_BUS);
      _rx_busop.shouldReap(false);
      //_rx_busop.setParams((uint8_t) _get_reg_addr(MCP356xRegister::IRQ) | 0x01);
      //_rx_busop.setBuffer((uint8_t*) &_reg_shadows[(uint8_t) MCP356xRegister::IRQ], 1);
    }
  }
  return ret;
}


/**
* Debug support method. This fxn is only present in debug builds.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void SX1276::printDebug(StringBuilder* output) {
  output->concat("\n");
  output->concatf("\tReset: %u\tCS:    %u\n", _opts.reset_pin, _opts.cs_pin);
  output->concatf("\tD0:    %u\tD1:    %u\n", _opts.d0_pin, _opts.d1_pin);
  output->concatf("\tD2:    %u\n", _opts.d2_pin);
}

/**
* Debug support method. Dump the register shadows.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void SX1276::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i < sizeof(_shadows); i++) {
    output->concatf("\t0x%02x:\t0x%02x", SX127X_REG_ADDR[i], _shadows[i]);
  }
}


/**
* Setup the low-level pin details. Execution is idempotent.
*
* @return
*   -1 if the pin setup is wrong. Class must halt.
*   0  if the pin setup is complete.
*/
int8_t SX1276::_ll_pin_init() {
  int8_t ret = -1;
  if (_sx_flag(SX127X_FLAG_PINS_CONFIGURED)) {
    ret = 0;
  }
  else if (255 != _opts.cs_pin) {   // This pin is required.
    ret = pinMode(_opts.cs_pin, GPIOMode::OUTPUT);
    if (0 == ret) {
      setPin(_opts.cs_pin, true);
      if (255 != _opts.reset_pin) {
        ret = pinMode(_opts.reset_pin, GPIOMode::OUTPUT);
        if (0 == ret) {
          setPin(_opts.reset_pin, false);
        }
      }
      if (255 != _opts.d0_pin) {
        pinMode(_opts.d0_pin, GPIOMode::INPUT);
      }
      if (255 != _opts.d1_pin) {
        pinMode(_opts.d1_pin, GPIOMode::INPUT);
      }
      if (255 != _opts.d2_pin) {
        pinMode(_opts.d2_pin, GPIOMode::INPUT);
      }
    }
  }
  return ret;
}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* This is what we call when this class wants to conduct a transaction on
*   the bus. We simply forward to the bus we are bound to.
*
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t SX1276::queue_io_job(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  op->callback = this;
  return _BUS->queue_io_job(op);
}


int8_t SX1276::io_op_callahead(BusOp* _op) {
  return 0;
}


int8_t SX1276::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t    ret = BUSOP_CALLBACK_NOMINAL;

  if (!op->hasFault()) {
    uint8_t* buf   = op->buffer();
    uint     len   = op->bufferLen();
    uint8_t  ridx  = (uint8_t) _reg_id_from_addr(op->getTransferParam(0));
    switch (op->get_opcode()) {
      case BusOpcode::TX:
        for (uint i = 0; i < len; i++) {
          uint8_t value = *(buf + i);
          switch ((SX127xRegister) (i + ridx)) {
            case SX127xRegister::FIFO:
            case SX127xRegister::OP_MODE:
            case SX127xRegister::FRF_MSB:
            case SX127xRegister::FRF_MID:
            case SX127xRegister::FRF_LSB:
            case SX127xRegister::PA_CONFIG:
            case SX127xRegister::LNA:
            case SX127xRegister::FIFO_ADDR_PTR:
            case SX127xRegister::FIFO_TX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_CURRENT_ADDR:
            case SX127xRegister::IRQ_FLAGS:
            case SX127xRegister::RX_NB_BYTES:
            case SX127xRegister::PKT_RSSI_VALUE:
            case SX127xRegister::PKT_SNR_VALUE:
            case SX127xRegister::MODEM_CONFIG_1:
            case SX127xRegister::MODEM_CONFIG_2:
            case SX127xRegister::PREAMBLE_MSB:
            case SX127xRegister::PREAMBLE_LSB:
            case SX127xRegister::PAYLOAD_LENGTH:
            case SX127xRegister::MODEM_CONFIG_3:
            case SX127xRegister::RSSI_WIDEBAND:
            case SX127xRegister::DETECTION_OPTIMIZE:
            case SX127xRegister::DETECTION_THRESHOLD:
            case SX127xRegister::SYNC_WORD:
            case SX127xRegister::DIO_MAPPING_1:
            case SX127xRegister::VERSION:
              break;
            default:   // Illegal write target.
              break;
          }
        }
        break;
      case BusOpcode::RX:
        for (uint i = 0; i < len; i++) {
          uint8_t value = *(buf + i);
          switch ((SX127xRegister) (i + ridx)) {
            case SX127xRegister::FIFO:
            case SX127xRegister::OP_MODE:
            case SX127xRegister::FRF_MSB:
            case SX127xRegister::FRF_MID:
            case SX127xRegister::FRF_LSB:
            case SX127xRegister::PA_CONFIG:
            case SX127xRegister::LNA:
            case SX127xRegister::FIFO_ADDR_PTR:
            case SX127xRegister::FIFO_TX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_BASE_ADDR:
            case SX127xRegister::FIFO_RX_CURRENT_ADDR:
            case SX127xRegister::IRQ_FLAGS:
            case SX127xRegister::RX_NB_BYTES:
            case SX127xRegister::PKT_RSSI_VALUE:
            case SX127xRegister::PKT_SNR_VALUE:
            case SX127xRegister::MODEM_CONFIG_1:
            case SX127xRegister::MODEM_CONFIG_2:
            case SX127xRegister::PREAMBLE_MSB:
            case SX127xRegister::PREAMBLE_LSB:
            case SX127xRegister::PAYLOAD_LENGTH:
            case SX127xRegister::MODEM_CONFIG_3:
            case SX127xRegister::RSSI_WIDEBAND:
            case SX127xRegister::DETECTION_OPTIMIZE:
            case SX127xRegister::DETECTION_THRESHOLD:
            case SX127xRegister::SYNC_WORD:
            case SX127xRegister::DIO_MAPPING_1:
            case SX127xRegister::VERSION:
              break;
            default:   // Illegal read target.
              break;
          }
        }
        break;
      default:  // Illegal operation.
        break;
    }
  }
  return ret;
}
