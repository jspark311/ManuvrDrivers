/*
File:   MCP23x17.h
Author: J. Ian Lindsay
Date:   2023.07.07

This driver operates in 16-bit ganged mode (BANK = 0). This not only allows
 lower operational overhead, but also a simpler list of registers. This means
 that Port-A is the first byte in the transfer order for each register, and will
 make it obnoxious to treat the I/O ports independently. So depending on
 use-case, performance might be less than it otherwise could be.

This class supports both interrupt pins (one for each 8-bit port), but having
  both offers no benefits to operation, given the driver's use of (BANK = 0).
  The optimal use of MCU pins would see INTA and INTB ctied together in a
  wired-OR configuration, and only pass in a single interrtupt pin to the
  constructor. If both pins are supplied, they will each trigger the same
  response from the driver (to read the state of the entire 16-bit port).
If possible, interrupts are leveraged for all INPUT pins to keep driver state
  as current as possible, regardless of whether or not the client software is
  expecting change-notifications on such pins.

The driver mirrors the pin states apart from the register shadows to provide
  assurances for their accuracy, and also to support its interrupt callback
  abstraction.


Copyright 2019 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#include <CppPotpourri.h>
#include <AbstractPlatform.h>
#include <I2CAdapter.h>
#include <SPIAdapter.h>

#ifndef MCP23X17_DRIVER_H
#define MCP23X17_DRIVER_H


/* These enum values indicate addresses. */
enum class MCP23x17RegID : uint8_t {
  IODIR   = 0x00,
  IPOL    = 0x02,
  GPINTEN = 0x04,
  DEFVAL  = 0x06,
  INTCON  = 0x08,
  IOCON   = 0x0A,  // Technically an 8-bit register, but is replicated at the next address.
  GPPU    = 0x0C,
  INTF    = 0x0E,
  INTCAP  = 0x10,
  GPIO    = 0x12,
  OLAT    = 0x14,
  INVALID = 0xFF   // End-of-enum. There are 11 registers.
};
#define MCP23X17_REG_COUNT  11


/*
* This part comes in two different variants: The MCP23017 (i2c) and the
*   MCP23S17 (SPI). Only the bus interface differs, and so we hold the business
*   logic of the driver in a superclass.
*/
class MCP23x17 {
  public:
    int8_t reset();
    int8_t poll();
    int8_t refresh();

    // Basic usage as pins...
    int8_t   gpioMode(uint8_t pin, GPIOMode mode);
    GPIOMode gpioMode(uint8_t pin);
    int8_t   digitalWrite(uint8_t pin, bool value);
    uint8_t  digitalRead(uint8_t pin);
    uint16_t getPinValues();
    int8_t   setPinValues(uint16_t);

    // Interrupt and callback management...
    int8_t  attachInterrupt(uint8_t pin, PinCallback, IRQCondition condition);
    int8_t  detachInterrupt(uint8_t pin);

    // Inline accessors...
    inline void preserveOnDestroy(bool x) {  _preserve_state = x;       };
    inline bool preserveOnDestroy() {        return _preserve_state;    };
    inline bool devFound() {                 return _dev_found;         };
    inline bool initialized() {              return _initialized;       };

    void printPins(StringBuilder*);
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);


  protected:
    bool   _bit7_out_only  = false;  // Used as a proxy for variant identification.
    bool   _preserve_state = false;
    bool   _pins_confd     = false;
    bool   _dev_found      = false;
    bool   _initialized    = false;

    /* This constructor is only a delegate to an extending class. */
    MCP23x17(const uint8_t RESET_PIN, const uint8_t IRQ_PIN_A, const uint8_t IRQ_PIN_B);

    int8_t _deep_init();
    void _print_debug(StringBuilder*);

    inline uint16_t _get_shadow_value(MCP23x17RegID r) {
      return ((_shadows[(uint8_t) r + 1]) | ((uint16_t) _shadows[(uint8_t) r] << 8));
    };
    inline void _set_shadow_value(MCP23x17RegID r, uint16_t val) {
      _shadows[(uint8_t) r + 1] = (uint8_t) (val & 0x00FF);
      _shadows[(uint8_t) r]     = (uint8_t) ((val >> 8) & 0x00FF);
    };

    /* Mandatory overrides for register access. */
    virtual int8_t _write_register(uint8_t addr, uint8_t* data) =0;
    virtual int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length) =0;


  private:
    const uint8_t _RESET_PIN;
    const uint8_t _IRQ_PIN_A;
    const uint8_t _IRQ_PIN_B;
    uint8_t       _shadows[MCP23X17_REG_COUNT << 1] = {0, };
    PinCallback   _callback    = nullptr;
    uint16_t      _pin_states  = 0;
    IRQCondition  _change_notice[16];

    int8_t _ll_pin_init();
    int8_t _invoke_pin_callback(uint8_t pin, bool value);
};



/*
* I2C Variant
*/
class MCP23017: public MCP23x17, public I2CDevice {
  public:
    MCP23017(uint8_t addr, const uint8_t RESET_PIN = 255, const uint8_t IRQ_PIN_A = 255, const uint8_t IRQ_PIN_B = 255);
    ~MCP23017();

    int8_t init(I2CAdapter* b = nullptr);
    void printDebug(StringBuilder*);

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    I2CBusOp  _busop_irq_read;
    I2CBusOp  _busop_pin_state;

    int8_t _write_register(uint8_t addr, uint8_t* data);
    int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length);
};



/*
* SPI Variant
*/
class MCP23S17: public MCP23x17, public BusOpCallback {
  public:
    MCP23S17(uint8_t cs_pin, uint8_t addr, const uint8_t RESET_PIN = 255, const uint8_t IRQ_PIN_A = 255, const uint8_t IRQ_PIN_B = 255);
    ~MCP23S17();

    int8_t init(SPIAdapter* b = nullptr);
    void printDebug(StringBuilder*);

    /* Overrides from BusOpCallback... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    const uint8_t _ADDR_BYTE;
    SPIBusOp  _busop_irq_read;
    SPIBusOp  _busop_pin_state;

    int8_t _write_register(uint8_t addr, uint8_t* data);
    int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length);
};

#endif // MCP23X17_DRIVER_H
