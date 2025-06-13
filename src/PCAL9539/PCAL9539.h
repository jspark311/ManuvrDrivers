#ifndef __PCAL9539_DRIVER_H_
#define __PCAL9539_DRIVER_H_

#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"


/* These are the i2c register indicies. NOT their addresses. */
enum class PCAL9539RegId : uint8_t {
  INPUT_PORT0        = 0x00,
  INPUT_PORT1        = 0x01,
  OUTPUT_PORT0       = 0x02,
  OUTPUT_PORT1       = 0x03,
  POLAR_INV_PORT0    = 0x04,
  POLAR_INV_PORT1    = 0x05,
  CONFIG_PORT0       = 0x06,
  CONFIG_PORT1       = 0x07,
  OUTPUT_DRIVE_STR_0 = 0x08,
  OUTPUT_DRIVE_STR_1 = 0x09,
  OUTPUT_DRIVE_STR_2 = 0x0A,
  OUTPUT_DRIVE_STR_3 = 0x0B,
  INPUT_LATCH_0      = 0x0C,
  INPUT_LATCH_1      = 0x0D,
  PULL_ENABLE_0      = 0x0E,
  PULL_ENABLE_1      = 0x0F,
  PULL_SELECT_0      = 0x10,
  PULL_SELECT_1      = 0x11,
  IRQ_MASK_0         = 0x12,
  IRQ_MASK_1         = 0x13,
  IRQ_STATUS_0       = 0x14,
  IRQ_STATUS_1       = 0x15,
  OUTPUT_CONF        = 0x16,
  INVALID            = 0x17
};
#define PCAL9539_NUM_OF_REGISTERS   ((uint8_t) PCAL9539RegId::INVALID)


/* Class flags. */
#define PCAL9539_FLAG_PRESERVE_STATE   0x0001
#define PCAL9539_FLAG_READ_IN_FLIGHT   0x0002  // Waiting on the return of a data read.
#define PCAL9539_FLAG_WRITE_IN_FLIGHT  0x0004  // Waiting on the return of a data write.
#define PCAL9539_FLAG_DEVICE_PRESENT   0x1000
#define PCAL9539_FLAG_PINS_CONFD       0x2000
#define PCAL9539_FLAG_INITIALIZED      0x4000

// Masks of driver state flags to simplify checking.
#define PCAL9539_FLAG_INIT_MASK     (PCAL9539_FLAG_INITIALIZED | PCAL9539_FLAG_PINS_CONFD)


/*
* Driver class.
*/
class PCAL9539 : public I2CDevice, public GPIOWrapper {
  public:
    PCAL9539(I2CAdapter* bus, const uint8_t i2c_addr, const uint8_t irq_pin, const uint8_t reset_pin, const uint8_t* reg_config = nullptr);
    ~PCAL9539();

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    int8_t init(I2CAdapter* b = nullptr);
    int8_t reset();
    int8_t poll();
    bool   isrFired();
    int8_t refresh();

    // Basic usage as pins...
    int8_t   gpioMode(uint8_t pin, GPIOMode mode);
    GPIOMode gpioMode(uint8_t pin);
    int8_t   digitalWrite(uint8_t pin, bool value);
    int8_t   digitalRead(uint8_t pin);
    uint16_t getInputPinValues();
    uint16_t getOutputPinValues();
    int8_t   setPinValues(uint16_t);

    // Interrupt and callback management...
    int8_t  attachInterrupt(uint8_t pin, PinCallback, IRQCondition condition);
    int8_t  detachInterrupt(uint8_t pin);
    int8_t  detachInterrupt(PinCallback);

    inline bool devFound() {     return _pcal_flag(PCAL9539_FLAG_DEVICE_PRESENT);  };
    inline bool initialized() {  return (PCAL9539_FLAG_INIT_MASK == (_flags & PCAL9539_FLAG_INIT_MASK));  };
    inline bool preserveOnDestroy() {
      return _pcal_flag(PCAL9539_FLAG_PRESERVE_STATE);
    };
    inline void preserveOnDestroy(bool x) {
      _pcal_set_flag(PCAL9539_FLAG_PRESERVE_STATE, x);
    };

    void printDebug(StringBuilder*);
    void printPins(StringBuilder*);
    void printRegs(StringBuilder*);
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);


  private:
    const uint8_t  _IRQ_PIN;
    const uint8_t  _RESET_PIN;
    const uint8_t* _CONFIG;
    uint16_t       _flags = 0;
    uint8_t        _0_in_dat  = 0;  // Confirmed pin states.
    uint8_t        _1_in_dat  = 0;  // Confirmed pin states.
    uint8_t        _0_out_dat = 0;  // Confirmed pin states.
    uint8_t        _1_out_dat = 0;  // Confirmed pin states.
    PinCallback    _callbacks[16] = {nullptr};
    uint8_t        _shadows[PCAL9539_NUM_OF_REGISTERS] = {0};

    int8_t _ll_pin_init();
    int8_t _invoke_pin_callback(uint8_t pin, bool val);
    int8_t _write_register(PCAL9539RegId, uint8_t val);
    int8_t _write_registers(PCAL9539RegId, uint8_t len);
    int8_t _read_registers(PCAL9539RegId, uint8_t len);
    inline uint8_t _get_shadow_value(PCAL9539RegId r) {            return _shadows[((uint8_t) r)];  };
    inline void _set_shadow_value(PCAL9539RegId r, uint8_t val) {  _shadows[((uint8_t) r)] = val;   };

    // void   _reset_register_values();
    // int8_t _impart_config();

    /* Flag manipulation inlines */
    inline uint16_t _pcal_flags() {                return _flags;           };
    inline bool _pcal_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _pcal_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _pcal_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _pcal_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};

#endif  // __PCAL9539_DRIVER_H_
