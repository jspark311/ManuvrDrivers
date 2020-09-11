/***************************************************
This is a library for the 0.96" 16-bit Color OLED with SSD13XX driver chip
Pick one up today in the adafruit shop!
------> http://www.adafruit.com/products/684
These displays use SPI to communicate, 4 or 5 pins are required to
interface
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!
Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution


Software License Agreement (BSD License)

Copyright (c) 2012, Adafruit Industries
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holders nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

****************************************************/


#include "Image/Image.h"
#include "SPIAdapter.h"
#include "StopWatch.h"

#ifndef __SSD13XX_DRIVER_H_
#define __SSD13XX_DRIVER_H_

/* Commands that all supported models have in common. */
#define SSD13XX_CMD_SETREMAP       0xA0  // Argument is embedded in the lowest bit.
#define SSD13XX_CMD_INVERTDISPLAY  0xA7
#define SSD13XX_CMD_SETMULTIPLEX   0xA8


/* Supported SSD chipsets */
enum class SSDModel : uint8_t {
  SSD1306  = 0,
  SSD1309  = 1,
  SSD1331  = 2,
  SSD1351  = 3
};


/*
* Pin defs for this module.
* Set pin def to 255 to mark it as unused.
*/
class SSD13xxOpts {
  public:
    SSD13xxOpts(const SSD13xxOpts* p) :
      orientation(p->orientation),
      reset(p->reset),
      dc(p->dc),
      cs(p->cs),
      model(p->model)
    {};

    SSD13xxOpts(
      ImgOrientation _or,
      uint8_t _reset,
      uint8_t _dc,
      uint8_t _cs,
      SSDModel _m
    ) :
      orientation(_or),
      reset(_reset),
      dc(_dc),
      cs(_cs),
      model(_m)
    {};

    const ImgOrientation orientation;  //
    const uint8_t        reset;        //
    const uint8_t        dc;           //
    const uint8_t        cs;           //
    const SSDModel       model;        //
};



// Class to manage hardware interface with SSD13xx chipset
class SSD13xx : public Image, public BusOpCallback {
  public:
    SSD13xx(const SSD13xxOpts* opts);
    ~SSD13xx();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter*);
    inline int8_t init() {        return init(_BUS);   };
    inline bool enabled() {       return _enabled;     };
    inline bool initialized() {   return _initd;       };
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t commitFrameBuffer();

    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  private:
    const SSD13xxOpts _opts;
    bool  _enabled  = false;
    bool  _initd    = false;
    SPIAdapter* _BUS = nullptr;
    StopWatch     _stopwatch;
    SPIBusOp    _fb_data_op;  // We do this frequently enough.

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};



// Class to manage hardware interface with SSD13xx chipset
class SSD1306 : public Image, public BusOpCallback {
  public:
    SSD1306(const SSD13xxOpts* opts);
    ~SSD1306();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter*);
    inline int8_t init() {        return init(_BUS);           };
    inline bool enabled() {       return _enabled;             };
    inline bool initialized() {   return (20 == _init_state);  };
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t testDisplay(bool);
    int8_t commitFrameBuffer();

    inline void externalVCC(bool x) {         _external_vcc = x;   };
    inline void comScanDecrements(bool x) {   _com_scan_dec = x;   };
    inline void enableRemap(bool x) {         _remap_enable = x;   };
    inline void comPinConf(uint8_t x) {       _com_pin_conf = x;   };
    inline void verticalScan(uint8_t x) {     _vert_scan    = x;   };
    inline void startLine(uint8_t x) {        _start_line   = x;   };
    inline void displayOffset(uint8_t x) {    _disp_offset  = x;   };


    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  private:
    const SSD13xxOpts _opts;
    bool     _enabled      = false;
    bool     _external_vcc = false;
    bool     _com_scan_dec = false;
    bool     _remap_enable = false;
    bool     _vert_scan    = false;
    uint8_t  _com_pin_conf = 0x02;
    uint8_t  _disp_offset  = 0x00;

    uint8_t  _init_state   = 0;
    uint8_t  _start_line   = 0;

    SPIAdapter* _BUS = nullptr;
    StopWatch   _stopwatch;
    SPIBusOp    _fb_data_op;  // We do this frequently enough.

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    int8_t _internal_init_fsm();
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};


// Class to manage hardware interface with SSD13xx chipset
class SSD1309 : public Image, public BusOpCallback {
  public:
    SSD1309(const SSD13xxOpts* opts);
    ~SSD1309();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter*);
    inline int8_t init() {        return init(_BUS);           };
    inline bool enabled() {       return _enabled;             };
    inline bool initialized() {   return (19 == _init_state);  };
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t testDisplay(bool);
    int8_t commitFrameBuffer();

    inline void externalVCC(bool x) {         _external_vcc = x;   };
    inline void comScanDecrements(bool x) {   _com_scan_dec = x;   };
    inline void enableRemap(bool x) {         _remap_enable = x;   };
    inline void comPinConf(uint8_t x) {       _com_pin_conf = x;   };
    inline void verticalScan(uint8_t x) {     _vert_scan    = x;   };
    inline void startLine(uint8_t x) {        _start_line   = x;   };
    inline void displayOffset(uint8_t x) {    _disp_offset  = x;   };

    inline bool    externalVCC() {          return _external_vcc;  };
    inline bool    comScanDecrements() {    return _com_scan_dec;  };
    inline bool    enableRemap() {          return _remap_enable;  };
    inline uint8_t comPinConf() {           return _com_pin_conf;  };
    inline uint8_t verticalScan() {         return _vert_scan;     };
    inline uint8_t startLine() {            return _start_line;    };
    inline uint8_t displayOffset() {        return _disp_offset;   };


    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  private:
    const SSD13xxOpts _opts;
    bool     _enabled      = false;
    bool     _external_vcc = false;
    bool     _com_scan_dec = false;
    bool     _remap_enable = false;
    bool     _vert_scan    = false;
    uint8_t  _com_pin_conf = 0x02;
    uint8_t  _disp_offset  = 0x00;

    uint8_t  _init_state   = 0;
    uint8_t  _start_line   = 0;

    SPIAdapter* _BUS = nullptr;
    StopWatch   _stopwatch;
    SPIBusOp    _fb_data_op;  // We do this frequently enough.

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    int8_t _internal_init_fsm();
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};

#endif   // __SSD13XX_DRIVER_H_
