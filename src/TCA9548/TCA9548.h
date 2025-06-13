#ifndef __TCA9548_DRIVER_H_
#define __TCA9548_DRIVER_H_

#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"


class TCA9548 : public I2CDevice {
  public:
    TCA9548(I2CAdapter* bus, const uint8_t I2C_ADDR, const uint8_t RESET_PIN);
    ~TCA9548();

    int8_t init(I2CAdapter* bus = nullptr);
    int8_t reset();

    inline bool  devFound() {     return _dev_found;   };
    inline bool  initialized() {  return (_dev_found & _pins_confd & _shadow_known);  };
    inline uint8_t chanMask() {   return _shadow_r;  };
    inline bool  isStable() {  return (_shadow_known & (_shadow_w == _shadow_r));  };
    inline bool  chanActive(const uint8_t C) {  return ((_shadow_r >> C) & 0x01);  };


    int8_t chanActive(const uint8_t C, bool active);
    int8_t setMux(uint8_t BUS_MASK);

    void printDebug(StringBuilder*);
    int8_t console_handler(StringBuilder*, StringBuilder*);

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    const uint8_t _RESET_PIN;
    uint8_t  _shadow_r     = 0;  // Read-side copy of the shadow.
    uint8_t  _shadow_w     = 0;  // Write-side copy of the shadow.
    bool     _pins_confd   = false;
    bool     _dev_found    = false;
    bool     _shadow_known = false;
    I2CBusOp _busop_set_mux;

    int8_t   _ll_pin_init();

    int8_t   _read_register();
};

#endif  // __TCA9548_DRIVER_H_
