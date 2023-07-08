/*
File:   MCP23x17.h
Author: J. Ian Lindsay
Date:   2023.07.07
*/

#include <AbstractPlatform.h>
#include <FlagContainer.h>
#include <I2CAdapter.h>
#include <SPIAdapter.h>

#ifndef MCP23X17_DRIVER_H
#define MCP23X17_DRIVER_H


/* Class flags */
#define MCP23X17_FLAG_DEVICE_PRESENT   0x0001  // Part was found.
#define MCP23X17_FLAG_INITIALIZED      0x0002  // Registers are initialized.


/*
* This part comes in two different variants: The MCP23017 (i2c) and the
*   MCP23S17 (SPI). Only the bus interface differs, and so we hold the business
*   logic of the driver in a superclass.
*/
class MCP23x17 {
  public:
    inline bool  devFound() {       return _flags.value(MCP23X17_FLAG_DEVICE_PRESENT);  };
    inline bool  initialized() {    return _flags.value(MCP23X17_FLAG_INITIALIZED);     };

    int8_t poll();
    int8_t refresh();


  protected:
    FlagContainer16 _flags;

    /* This constructor is only a delegate to an extending class. */
    MCP23x17();

    /* Mandatory overrides for register access. */
    virtual int8_t _write_register(uint8_t addr, uint8_t* data) =0;
    virtual int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length) =0;
};


/*
* I2C Variant
*/
class MCP23017: public MCP23x17, public I2CDevice {
  public:
    MCP23017(uint8_t addr);
    ~MCP23017();

    int8_t init();
    void printDebug(StringBuilder*);

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);


  private:
    int8_t _write_register(uint8_t addr, uint8_t* data);
    int8_t _read_registers(uint8_t addr, uint8_t* data, uint8_t length);
};

#endif // MCP23X17_DRIVER_H
