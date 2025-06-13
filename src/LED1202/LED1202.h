/*
File:   LED1202.h
Author: J. Ian Lindsay
Date:   2025.04.28

NOTE: Driver does not support clock functions of pins A0 and A1.
NOTE: Driver is not double-buffered, and takes no meansures to control for
  reporting actual hardware state, versus data that may be in-flight.
*/

#ifndef __LED1202_LED_DRIVER_H__
#define __LED1202_LED_DRIVER_H__

#include <inttypes.h>
#include "AbstractPlatform.h"
#include "BusQueue/I2CAdapter.h"


// If we are going to support interrupts from the device, we need at least one
//   entry in a global context table for this class.
#if !defined(LED1202_INTERRUPTER_COUNT)
  #define LED1202_INTERRUPTER_COUNT  1
#endif


/* Fault and status bitmask definitions. */
#define LED1202_INT_STATUS_OVTP    0x01
#define LED1202_INT_STATUS_OPEN    0x02
#define LED1202_INT_STATUS_PAT     0x04
#define LED1202_INT_STATUS_SOF     0x08

/* Class flags */
#define LED1202_FLAG_DEVICE_PRESENT     0x0001  // Part was found.
#define LED1202_FLAG_PINS_CONFIGURED    0x0002  // Have the platform GPIOs been configured?
#define LED1202_CONFIG_READ             0x0004  //
#define LED1202_CONFIG_WRITTEN          0x0008  //
#define LED1202_FLAG_IO_IN_FLIGHT       0x0010  //

#define LED1202_FLAG_INIT_MASK   (LED1202_FLAG_PINS_CONFIGURED | LED1202_FLAG_DEVICE_PRESENT | LED1202_CONFIG_READ | LED1202_CONFIG_WRITTEN)


/*
* Enums for registers. All registers are 16-bits wide.
* Enum values reflect actual addresses.
*
* TODO: If not for the fact that we want the ability to read back the register
*   contents, we would not be enumerating all of these registers, nor providing
*   them shadow space >200 bytes is quite a bit, considering). We may yet strike
*   all of the pattern registers, in favor of a purely command-driven operation.
*   But for now, we do it by-the-book.
*/
enum class LED1202Register : uint8_t {
  DEV_ID            = 0x00,    // Read-only
  DEV_ENABLE        = 0x01,    //
  CHAN_ENABLE_L     = 0x02,    //
  CHAN_ENABLE_H     = 0x03,    //
  CONFIG            = 0x04,    //
  FAULT_STAT_MASK   = 0x05,    //
  FAULT_STAT_IRQ    = 0x06,    //
  OPEN_LED_L        = 0x07,    //
  OPEN_LED_H        = 0x08,    //
  CS0_CURRENT       = 0x09,    //
  CS1_CURRENT       = 0x0A,    //
  CS2_CURRENT       = 0x0B,    //
  CS3_CURRENT       = 0x0C,    //
  CS4_CURRENT       = 0x0D,    //
  CS5_CURRENT       = 0x0E,    //
  CS6_CURRENT       = 0x0F,    //
  CS7_CURRENT       = 0x10,    //
  CS8_CURRENT       = 0x11,    //
  CS9_CURRENT       = 0x12,    //
  CSA_CURRENT       = 0x13,    //
  CSB_CURRENT       = 0x14,    //
  PATTERN_SEQ_REP   = 0x15,    //
  PATTERN0_DURATION = 0x16,    //
  PATTERN1_DURATION = 0x17,    //
  PATTERN2_DURATION = 0x18,    //
  PATTERN3_DURATION = 0x19,    //
  PATTERN4_DURATION = 0x1A,    //
  PATTERN5_DURATION = 0x1B,    //
  PATTERN6_DURATION = 0x1C,    //
  PATTERN7_DURATION = 0x1D,    //
  PATTTERN0_CS0_L   = 0x1E,    //
  PATTTERN0_CS0_H   = 0x1F,    //
  PATTTERN0_CS1_L   = 0x20,    //
  PATTTERN0_CS1_H   = 0x21,    //
  PATTTERN0_CS2_L   = 0x22,    //
  PATTTERN0_CS2_H   = 0x23,    //
  PATTTERN0_CS3_L   = 0x24,    //
  PATTTERN0_CS3_H   = 0x25,    //
  PATTTERN0_CS4_L   = 0x26,    //
  PATTTERN0_CS4_H   = 0x27,    //
  PATTTERN0_CS5_L   = 0x28,    //
  PATTTERN0_CS5_H   = 0x29,    //
  PATTTERN0_CS6_L   = 0x2A,    //
  PATTTERN0_CS6_H   = 0x2B,    //
  PATTTERN0_CS7_L   = 0x2C,    //
  PATTTERN0_CS7_H   = 0x2D,    //
  PATTTERN0_CS8_L   = 0x2E,    //
  PATTTERN0_CS8_H   = 0x2F,    //
  PATTTERN0_CS9_L   = 0x30,    //
  PATTTERN0_CS9_H   = 0x31,    //
  PATTTERN0_CSA_L   = 0x32,    //
  PATTTERN0_CSA_H   = 0x33,    //
  PATTTERN0_CSB_L   = 0x34,    //
  PATTTERN0_CSB_H   = 0x35,    //
  PATTTERN1_CS0_L   = 0x36,    //
  PATTTERN1_CS0_H   = 0x37,    //
  PATTTERN1_CS1_L   = 0x38,    //
  PATTTERN1_CS1_H   = 0x39,    //
  PATTTERN1_CS2_L   = 0x3A,    //
  PATTTERN1_CS2_H   = 0x3B,    //
  PATTTERN1_CS3_L   = 0x3C,    //
  PATTTERN1_CS3_H   = 0x3D,    //
  PATTTERN1_CS4_L   = 0x3E,    //
  PATTTERN1_CS4_H   = 0x3F,    //
  PATTTERN1_CS5_L   = 0x40,    //
  PATTTERN1_CS5_H   = 0x41,    //
  PATTTERN1_CS6_L   = 0x42,    //
  PATTTERN1_CS6_H   = 0x43,    //
  PATTTERN1_CS7_L   = 0x44,    //
  PATTTERN1_CS7_H   = 0x45,    //
  PATTTERN1_CS8_L   = 0x46,    //
  PATTTERN1_CS8_H   = 0x47,    //
  PATTTERN1_CS9_L   = 0x48,    //
  PATTTERN1_CS9_H   = 0x49,    //
  PATTTERN1_CSA_L   = 0x4A,    //
  PATTTERN1_CSA_H   = 0x4B,    //
  PATTTERN1_CSB_L   = 0x4C,    //
  PATTTERN1_CSB_H   = 0x4D,    //
  PATTTERN2_CS0_L   = 0x4E,    //
  PATTTERN2_CS0_H   = 0x4F,    //
  PATTTERN2_CS1_L   = 0x50,    //
  PATTTERN2_CS1_H   = 0x51,    //
  PATTTERN2_CS2_L   = 0x52,    //
  PATTTERN2_CS2_H   = 0x53,    //
  PATTTERN2_CS3_L   = 0x54,    //
  PATTTERN2_CS3_H   = 0x55,    //
  PATTTERN2_CS4_L   = 0x56,    //
  PATTTERN2_CS4_H   = 0x57,    //
  PATTTERN2_CS5_L   = 0x58,    //
  PATTTERN2_CS5_H   = 0x59,    //
  PATTTERN2_CS6_L   = 0x5A,    //
  PATTTERN2_CS6_H   = 0x5B,    //
  PATTTERN2_CS7_L   = 0x5C,    //
  PATTTERN2_CS7_H   = 0x5D,    //
  PATTTERN2_CS8_L   = 0x5E,    //
  PATTTERN2_CS8_H   = 0x5F,    //
  PATTTERN2_CS9_L   = 0x60,    //
  PATTTERN2_CS9_H   = 0x61,    //
  PATTTERN2_CSA_L   = 0x62,    //
  PATTTERN2_CSA_H   = 0x63,    //
  PATTTERN2_CSB_L   = 0x64,    //
  PATTTERN2_CSB_H   = 0x65,    //
  PATTTERN3_CS0_L   = 0x66,    //
  PATTTERN3_CS0_H   = 0x67,    //
  PATTTERN3_CS1_L   = 0x68,    //
  PATTTERN3_CS1_H   = 0x69,    //
  PATTTERN3_CS2_L   = 0x6A,    //
  PATTTERN3_CS2_H   = 0x6B,    //
  PATTTERN3_CS3_L   = 0x6C,    //
  PATTTERN3_CS3_H   = 0x6D,    //
  PATTTERN3_CS4_L   = 0x6E,    //
  PATTTERN3_CS4_H   = 0x6F,    //
  PATTTERN3_CS5_L   = 0x70,    //
  PATTTERN3_CS5_H   = 0x71,    //
  PATTTERN3_CS6_L   = 0x72,    //
  PATTTERN3_CS6_H   = 0x73,    //
  PATTTERN3_CS7_L   = 0x74,    //
  PATTTERN3_CS7_H   = 0x75,    //
  PATTTERN3_CS8_L   = 0x76,    //
  PATTTERN3_CS8_H   = 0x77,    //
  PATTTERN3_CS9_L   = 0x78,    //
  PATTTERN3_CS9_H   = 0x79,    //
  PATTTERN3_CSA_L   = 0x7A,    //
  PATTTERN3_CSA_H   = 0x7B,    //
  PATTTERN3_CSB_L   = 0x7C,    //
  PATTTERN3_CSB_H   = 0x7D,    //
  PATTTERN4_CS0_L   = 0x7E,    //
  PATTTERN4_CS0_H   = 0x7F,    //
  PATTTERN4_CS1_L   = 0x80,    //
  PATTTERN4_CS1_H   = 0x81,    //
  PATTTERN4_CS2_L   = 0x82,    //
  PATTTERN4_CS2_H   = 0x83,    //
  PATTTERN4_CS3_L   = 0x84,    //
  PATTTERN4_CS3_H   = 0x85,    //
  PATTTERN4_CS4_L   = 0x86,    //
  PATTTERN4_CS4_H   = 0x87,    //
  PATTTERN4_CS5_L   = 0x88,    //
  PATTTERN4_CS5_H   = 0x89,    //
  PATTTERN4_CS6_L   = 0x8A,    //
  PATTTERN4_CS6_H   = 0x8B,    //
  PATTTERN4_CS7_L   = 0x8C,    //
  PATTTERN4_CS7_H   = 0x8D,    //
  PATTTERN4_CS8_L   = 0x8E,    //
  PATTTERN4_CS8_H   = 0x8F,    //
  PATTTERN4_CS9_L   = 0x90,    //
  PATTTERN4_CS9_H   = 0x91,    //
  PATTTERN4_CSA_L   = 0x92,    //
  PATTTERN4_CSA_H   = 0x93,    //
  PATTTERN4_CSB_L   = 0x94,    //
  PATTTERN4_CSB_H   = 0x95,    //
  PATTTERN5_CS0_L   = 0x96,    //
  PATTTERN5_CS0_H   = 0x97,    //
  PATTTERN5_CS1_L   = 0x98,    //
  PATTTERN5_CS1_H   = 0x99,    //
  PATTTERN5_CS2_L   = 0x9A,    //
  PATTTERN5_CS2_H   = 0x9B,    //
  PATTTERN5_CS3_L   = 0x9C,    //
  PATTTERN5_CS3_H   = 0x9D,    //
  PATTTERN5_CS4_L   = 0x9E,    //
  PATTTERN5_CS4_H   = 0x9F,    //
  PATTTERN5_CS5_L   = 0xA0,    //
  PATTTERN5_CS5_H   = 0xA1,    //
  PATTTERN5_CS6_L   = 0xA2,    //
  PATTTERN5_CS6_H   = 0xA3,    //
  PATTTERN5_CS7_L   = 0xA4,    //
  PATTTERN5_CS7_H   = 0xA5,    //
  PATTTERN5_CS8_L   = 0xA6,    //
  PATTTERN5_CS8_H   = 0xA7,    //
  PATTTERN5_CS9_L   = 0xA8,    //
  PATTTERN5_CS9_H   = 0xA9,    //
  PATTTERN5_CSA_L   = 0xAA,    //
  PATTTERN5_CSA_H   = 0xAB,    //
  PATTTERN5_CSB_L   = 0xAC,    //
  PATTTERN5_CSB_H   = 0xAD,    //
  PATTTERN6_CS0_L   = 0xAE,    //
  PATTTERN6_CS0_H   = 0xAF,    //
  PATTTERN6_CS1_L   = 0xB0,    //
  PATTTERN6_CS1_H   = 0xB1,    //
  PATTTERN6_CS2_L   = 0xB2,    //
  PATTTERN6_CS2_H   = 0xB3,    //
  PATTTERN6_CS3_L   = 0xB4,    //
  PATTTERN6_CS3_H   = 0xB5,    //
  PATTTERN6_CS4_L   = 0xB6,    //
  PATTTERN6_CS4_H   = 0xB7,    //
  PATTTERN6_CS5_L   = 0xB8,    //
  PATTTERN6_CS5_H   = 0xB9,    //
  PATTTERN6_CS6_L   = 0xBA,    //
  PATTTERN6_CS6_H   = 0xBB,    //
  PATTTERN6_CS7_L   = 0xBC,    //
  PATTTERN6_CS7_H   = 0xBD,    //
  PATTTERN6_CS8_L   = 0xBE,    //
  PATTTERN6_CS8_H   = 0xBF,    //
  PATTTERN6_CS9_L   = 0xC0,    //
  PATTTERN6_CS9_H   = 0xC1,    //
  PATTTERN6_CSA_L   = 0xC2,    //
  PATTTERN6_CSA_H   = 0xC3,    //
  PATTTERN6_CSB_L   = 0xC4,    //
  PATTTERN6_CSB_H   = 0xC5,    //
  PATTTERN7_CS0_L   = 0xC6,    //
  PATTTERN7_CS0_H   = 0xC7,    //
  PATTTERN7_CS1_L   = 0xC8,    //
  PATTTERN7_CS1_H   = 0xC9,    //
  PATTTERN7_CS2_L   = 0xCA,    //
  PATTTERN7_CS2_H   = 0xCB,    //
  PATTTERN7_CS3_L   = 0xCC,    //
  PATTTERN7_CS3_H   = 0xCD,    //
  PATTTERN7_CS4_L   = 0xCE,    //
  PATTTERN7_CS4_H   = 0xCF,    //
  PATTTERN7_CS5_L   = 0xD0,    //
  PATTTERN7_CS5_H   = 0xD1,    //
  PATTTERN7_CS6_L   = 0xD2,    //
  PATTTERN7_CS6_H   = 0xD3,    //
  PATTTERN7_CS7_L   = 0xD4,    //
  PATTTERN7_CS7_H   = 0xD5,    //
  PATTTERN7_CS8_L   = 0xD6,    //
  PATTTERN7_CS8_H   = 0xD7,    //
  PATTTERN7_CS9_L   = 0xD8,    //
  PATTTERN7_CS9_H   = 0xD9,    //
  PATTTERN7_CSA_L   = 0xDA,    //
  PATTTERN7_CSA_H   = 0xDB,    //
  PATTTERN7_CSB_L   = 0xDC,    //
  PATTTERN7_CSB_H   = 0xDD,    //
  // Discontinuity
  CLOCK_CONFIG      = 0xE0,    //
  INVALID           = 0xE1     // Invalid. Not repesented in hardware.
};



class LED1202 : public I2CDevice {
  public:
    LED1202(const uint8_t I2C_ADDR, const uint8_t IRQ_PIN = 255, I2CAdapter* bus = nullptr);
    ~LED1202();

    int8_t init(I2CAdapter* bus = nullptr);
    void isr_fxn();   // Called from the ISR to dispatch status read.
    int8_t reset();
    int8_t poll();
    void printDebug(StringBuilder*);
    void printRegs(StringBuilder*);
    void printChannelValues(StringBuilder*, int8_t chan = -1);
    int8_t console_handler(StringBuilder*, StringBuilder*);

    inline bool  devFound() {         return _led1202_flag(LED1202_FLAG_DEVICE_PRESENT);  };
    inline bool  initialized() {      return (LED1202_FLAG_INIT_MASK == (_flags & LED1202_FLAG_INIT_MASK));     };

    bool      enabled();
    int8_t    enabled(bool);

    // Direct channel manipulation
    // We will mostly handle device features via direct shadow lookup, rather
    //   than echoing the data into structures of our own. The device design
    //   is already packed nearly optimally, and if we are going to be shadowing
    //   the entire register space, we may as well.
    bool      led_open(uint8_t chan);
    bool      led_enabled(uint8_t chan);
    uint16_t* led_pattern(uint8_t chan);
    float     led_max_current(uint8_t chan);  // Returns Amps.
    int8_t    led_max_current(uint8_t chan, float);  // Takes Amps.

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);



  private:
    const uint8_t _IRQ_PIN;
    uint16_t _flags;
    uint8_t  _chan_milliamps[12] = {0};
    uint8_t  _shadows[(uint8_t) LED1202Register::INVALID];

    int8_t   _ll_pin_init();

    int8_t   _send_device_reset();
    void     _reset_register_values();
    int8_t   _read_registers(LED1202Register reg, uint8_t len);
    int8_t   _write_registers(LED1202Register reg, uint8_t len);


    /* Semantic breakouts for flags and conditions. */
    uint16_t _get_shadow_value16(LED1202Register reg);
    int8_t   _set_shadow_value16(LED1202Register reg, uint16_t val);
    inline uint8_t _get_shadow_value(LED1202Register reg) {   return _shadows[(uint8_t) reg]; };
    inline void    _set_shadow_value(LED1202Register reg, uint8_t val) {   _shadows[(uint8_t) reg] = val;  };

    inline bool    _io_in_flight() {      return _led1202_flag(LED1202_FLAG_IO_IN_FLIGHT);    };

    /* Flag manipulation inlines */
    inline uint16_t _led1202_flags() {                return _flags;           };
    inline bool _led1202_flag(uint16_t _flag) {       return (_flags & _flag); };
    inline void _led1202_clear_flag(uint16_t _flag) { _flags &= ~_flag;        };
    inline void _led1202_set_flag(uint16_t _flag) {   _flags |= _flag;         };
    inline void _led1202_set_flag(uint16_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};
#endif    // __LED1202_LED_DRIVER_H__
