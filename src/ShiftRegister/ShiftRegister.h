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
* This driver was meant to support the 74HCT595, but might work for other chips
*   that have the same clock structure
* Maximum of 32 devices in chain.
* RCLK pin is rising-edge sensitive.
* Pin-0 is mapped to the first bit (Qa) of the last device in the chain.
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

    inline void setBus(SPIAdapter* b) {  _BUS = b;                      };
    inline bool initialized() {          return _initd;                 };
    inline bool allocated() {            return (nullptr != _shadows);  };
    inline bool outputEnabled() {        return _enabled;               };

    int8_t  reset();
    int8_t  init();
    int8_t  outputEnabled(bool);
    int8_t  setPin(uint8_t pin, bool val);
    int8_t  readPin(uint8_t pin);
    int8_t  setPins(uint8_t dev, uint8_t val);
    uint8_t readPins(uint8_t dev);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  private:
    const uint8_t DEVS_IN_CHAIN;
    const uint8_t RCLK_PIN;
    const uint8_t OE_PIN;
    const uint8_t SRCLR_PIN;
    SPIAdapter*   _BUS        = nullptr;
    uint8_t*      _shadows    = nullptr;
    bool          _enabled    = false;
    bool          _initd      = false;
    bool          _pins_initd = false;

    int8_t _ll_pin_init();
    bool _allocated();
    inline bool _pin_valid(uint8_t pin) {  return ((pin >> 3) <= DEVS_IN_CHAIN);  };

    int8_t _write_chain();
};

#endif   // __SHIFT_REG_DRIVER_H_
