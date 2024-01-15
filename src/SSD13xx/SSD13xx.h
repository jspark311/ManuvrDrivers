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
#include "BusQueue/SPIAdapter.h"
#include "TimerTools/TimerTools.h"

#ifndef __SSD13XX_DRIVER_H_
#define __SSD13XX_DRIVER_H_

/* Commands that all supported models have in common. */
#define SSD13XX_CMD_SETREMAP       0xA0  // Argument is embedded in the lowest bit.
#define SSD13XX_CMD_INVERTDISPLAY  0xA7
#define SSD13XX_CMD_SETMULTIPLEX   0xA8


/* Class flags */
#define SSD13XX_FLAG_INITIALIZED   0x01  //
#define SSD13XX_FLAG_ENABLED       0x02  //
#define SSD13XX_FLAG_FB_IN_FLIGHT  0x04  //
#define SSD13XX_FLAG_EXTERNAL_VCC  0x10  //
#define SSD13XX_FLAG_COM_SCAN_DEC  0x20  //
#define SSD13XX_FLAG_REMAP_ENABLE  0x40  //
#define SSD13XX_FLAG_VERT_SCAN     0x80  //


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
      cs(p->cs)
    {};

    SSD13xxOpts(
      ImgOrientation _or,
      uint8_t _reset,
      uint8_t _dc,
      uint8_t _cs
    ) :
      orientation(_or),
      reset(_reset),
      dc(_dc),
      cs(_cs)
    {};

    const ImgOrientation orientation;  //
    const uint8_t        reset;        //
    const uint8_t        dc;           //
    const uint8_t        cs;           //
};


/*
* This is a virtual class to handle the boilerplate of this display family.
* TODO: Continue gracefully degrading back into a proper inheritance pattern.
*/
class SSD13xx : public Image {
  public:
    inline bool initialized() {              return _class_flag(SSD13XX_FLAG_INITIALIZED);   };
    inline bool enabled() {                  return _class_flag(SSD13XX_FLAG_ENABLED);       };
    inline bool externalVCC() {              return _class_flag(SSD13XX_FLAG_EXTERNAL_VCC);  };
    inline bool comScanDecrements() {        return _class_flag(SSD13XX_FLAG_COM_SCAN_DEC);  };
    inline bool enableRemap() {              return _class_flag(SSD13XX_FLAG_REMAP_ENABLE);  };
    inline bool verticalScan() {             return _class_flag(SSD13XX_FLAG_VERT_SCAN);     };

    inline void externalVCC(bool x) {        _class_set_flag(SSD13XX_FLAG_EXTERNAL_VCC, x);  };
    inline void comScanDecrements(bool x) {  _class_set_flag(SSD13XX_FLAG_COM_SCAN_DEC, x);  };
    inline void enableRemap(bool x) {        _class_set_flag(SSD13XX_FLAG_REMAP_ENABLE, x);  };
    inline void verticalScan(uint8_t x) {    _class_set_flag(SSD13XX_FLAG_VERT_SCAN, x);     };


  protected:
    const SSD13xxOpts _opts;
    const SSDModel    _model;
    uint8_t           _flags;
    StopWatch         _stopwatch;

    SSD13xx(const SSD13xxOpts*, const SSDModel);
    ~SSD13xx() {};

    /* Flag manipulation inlines */
    inline uint8_t _class_flags() {                   return _flags;           };
    inline uint8_t _class_flag(uint8_t _flag) {       return (_flags & _flag); };
    inline void    _class_clear_flag(uint8_t _flag) { _flags &= ~_flag;        };
    inline void    _class_set_flag(uint8_t _flag) {   _flags |= _flag;         };
    inline void    _class_set_flag(uint8_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};


// Class to manage hardware interface with SSD13xx chipset
class SSD1331 : public SSD13xx, public BusOpCallback {
  public:
    SSD1331(const SSD13xxOpts*);
    ~SSD1331();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter* bus = nullptr);
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t commitFrameBuffer();

    int8_t setBrightness(float);
    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    /* Built-in per-instance console handler. */
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);


  private:
    SPIAdapter* _BUS = nullptr;
    SPIBusOp    _fb_data_op;

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};


// Class to manage hardware interface with SSD13xx chipset
class SSD1351 : public SSD13xx, public BusOpCallback {
  public:
    SSD1351(const SSD13xxOpts*);
    ~SSD1351();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter*);
    inline int8_t init() {        return init(_BUS);   };
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t commitFrameBuffer();

    int8_t setBrightness(float);
    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    /* Built-in per-instance console handler. */
    int8_t console_handler(StringBuilder* text_return, StringBuilder* args);


  private:
    SPIAdapter* _BUS = nullptr;
    SPIBusOp    _fb_data_op;  // We do this frequently enough.

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};



// Class to manage hardware interface with SSD1306 chipset
class SSD1306 : public SSD13xx, public BusOpCallback {
  public:
    SSD1306(const SSD13xxOpts*);
    ~SSD1306();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter*);
    inline int8_t init() {        return init(_BUS);           };
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t testDisplay(bool);
    int8_t commitFrameBuffer();

    inline void comPinConf(uint8_t x) {       _com_pin_conf = x;   };
    inline void startLine(uint8_t x) {        _start_line   = x;   };
    inline void displayOffset(uint8_t x) {    _disp_offset  = x;   };

    int8_t setBrightness(float);
    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  private:
    uint8_t     _com_pin_conf = 0x02;
    uint8_t     _disp_offset  = 0x00;
    uint8_t     _init_state   = 0;
    uint8_t     _start_line   = 0;
    SPIAdapter* _BUS          = nullptr;
    SPIBusOp    _fb_data_op;  // We do this frequently enough.

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    int8_t _internal_init_fsm();
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};


// Class to manage hardware interface with SSD1309 chipset
class SSD1309 : public SSD13xx, public BusOpCallback {
  public:
    SSD1309(const SSD13xxOpts*);
    ~SSD1309();

    inline void setBus(SPIAdapter* b) {  _BUS = b;  };
    int8_t init(SPIAdapter*);
    inline int8_t init() {        return init(_BUS);           };
    int8_t reset();

    void setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    int8_t invertDisplay(bool);
    int8_t testDisplay(bool);
    int8_t commitFrameBuffer();

    inline void comPinConf(uint8_t x) {       _com_pin_conf = x;   };
    inline void startLine(uint8_t x) {        _start_line   = x;   };
    inline void displayOffset(uint8_t x) {    _disp_offset  = x;   };

    inline uint8_t comPinConf() {           return _com_pin_conf;  };
    inline uint8_t startLine() {            return _start_line;    };
    inline uint8_t displayOffset() {        return _disp_offset;   };

    int8_t setBrightness(float);
    int8_t enableDisplay(bool enable);
    void printDebug(StringBuilder*);

    /* Overrides from the BusAdapter interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);


  private:
    uint8_t     _com_pin_conf = 0x02;
    uint8_t     _disp_offset  = 0x00;
    uint8_t     _init_state   = 0;
    uint8_t     _start_line   = 0;
    SPIAdapter* _BUS          = nullptr;
    SPIBusOp    _fb_data_op;  // We do this frequently enough.

    int8_t _ll_pin_init();

    int8_t _send_command(uint8_t commandByte, uint8_t* dataBytes, uint8_t numDataBytes);
    int8_t _internal_init_fsm();
    inline int8_t _send_command(uint8_t commandByte) {
      return _send_command(commandByte, nullptr, 0);
    };
};

#endif   // __SSD13XX_DRIVER_H_
