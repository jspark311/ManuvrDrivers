/*
* Driver for various shift registers.
*
* NOTE: This driver assumes usage on an SPI adapter.
*/

#include <AbstractPlatform.h>
#include "SPIAdapter.h"

#ifndef __SHIFT_REG_DRIVER_H_
#define __SHIFT_REG_DRIVER_H_

/*
* Class flags.
*/
#define SHIFTREG_FLAG_PINS_CONFIGURED  0x01  // Low-level pin setup is complete.
#define SHIFTREG_FLAG_INITIALIZED      0x02  // Device initialized.
#define SHIFTREG_FLAG_ENABLED          0x04  // Is the OE pin asserted?
#define SHIFTREG_FLAG_PENDING_IO       0x08  // Is I/O ongoing?
#define SHIFTREG_FLAG_QUEUED_IO        0x10  // Is I/O queued?

#define SHIFTREG_FLAG_RESET_MASK       (SHIFTREG_FLAG_ENABLED | SHIFTREG_FLAG_PINS_CONFIGURED)


/*
* This driver was meant to support the 74HCT595, but might work for other chips
*   that have the same clock structure
* Maximum of 32 devices in chain.
* RCLK pin is rising-edge sensitive.
* Pin-0 is mapped to the first bit (Qa) of the first device in the chain.
*/
class ShiftRegisterOut : public BusOpCallback {
  public:
    ShiftRegisterOut(
      const uint8_t devs,
      const uint8_t rclk_pin,
      const uint8_t oe_pin = 255,
      const uint8_t srclr_pin = 255
    );
    ~ShiftRegisterOut() {};

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    inline void setBus(SPIAdapter* b) {         _BUS = b;         };
    inline void setCallback(PinCallback cb) {   _callback = cb;   };
    inline bool allocated() {      return (nullptr != _shadows);  };
    inline bool initialized() {    return _class_flag(SHIFTREG_FLAG_INITIALIZED);  };
    inline bool outputEnabled() {  return _class_flag(SHIFTREG_FLAG_ENABLED);      };
    inline bool pendingIO() {      return _class_flag(SHIFTREG_FLAG_PENDING_IO);   };

    int8_t  reset();
    int8_t  init();
    int8_t  outputEnabled(bool);
    int8_t  digitalWrite(uint8_t pin, bool val);
    int8_t  digitalWrite8(uint8_t dev, uint8_t val);
    int8_t  digitalRead(uint8_t pin);
    uint8_t getPinValues(uint8_t dev);
    void    printDebug(StringBuilder*);


  private:
    const uint8_t DEVS_IN_CHAIN;
    const uint8_t RCLK_PIN;
    const uint8_t OE_PIN;
    const uint8_t SRCLR_PIN;
    SPIAdapter*   _BUS        = nullptr;
    PinCallback   _callback   = nullptr;
    uint8_t*      _shadows    = nullptr;
    uint8_t       _flags      = 0;

    int8_t _ll_pin_init();
    bool   _allocated();
    int8_t _write_chain();
    inline bool _pin_valid(uint8_t pin) {  return ((pin >> 3) < DEVS_IN_CHAIN);  };

    /* Flag manipulation inlines */
    inline uint8_t _class_flags() {                return _flags;           };
    inline bool _class_flag(uint8_t _flag) {       return (_flags & _flag); };
    inline void _class_clear_flag(uint8_t _flag) { _flags &= ~_flag;        };
    inline void _class_set_flag(uint8_t _flag) {   _flags |= _flag;         };
    inline void _class_set_flag(uint8_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};

#endif   // __SHIFT_REG_DRIVER_H_
