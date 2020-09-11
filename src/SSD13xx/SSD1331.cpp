/***************************************************
  This is a library for the 0.96" 16-bit Color OLED with SSD13xx driver chip

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

#include "SSD13xx.h"
#include "AbstractPlatform.h"

// Timing Delays
#define SSD13XX_DELAYS_HWFILL     (3)
#define SSD13XX_DELAYS_HWLINE     (1)

// SSD13XX Commands
#define SSD13XX_CMD_DRAWLINE       0x21
#define SSD13XX_CMD_DRAWRECT       0x22
#define SSD13XX_CMD_FILL           0x26
#define SSD13XX_CMD_SETCOLUMN      0x15
#define SSD13XX_CMD_SETROW         0x75
#define SSD13XX_CMD_CONTRASTA      0x81
#define SSD13XX_CMD_CONTRASTB      0x82
#define SSD13XX_CMD_CONTRASTC      0x83
#define SSD13XX_CMD_MASTERCURRENT  0x87
#define SSD13XX_CMD_STARTLINE      0xA1
#define SSD13XX_CMD_DISPLAYOFFSET  0xA2
#define SSD13XX_CMD_NORMALDISPLAY  0xA4
#define SSD13XX_CMD_DISPLAYALLON   0xA5
#define SSD13XX_CMD_DISPLAYALLOFF  0xA6
#define SSD13XX_CMD_SETMASTER      0xAD
#define SSD13XX_CMD_DISPLAYOFF     0xAE
#define SSD13XX_CMD_DISPLAYON      0xAF
#define SSD13XX_CMD_POWERMODE      0xB0
#define SSD13XX_CMD_PRECHARGE      0xB1
#define SSD13XX_CMD_CLOCKDIV       0xB3
#define SSD13XX_CMD_PRECHARGEA     0x8A
#define SSD13XX_CMD_PRECHARGEB     0x8B
#define SSD13XX_CMD_PRECHARGEC     0x8C
#define SSD13XX_CMD_PRECHARGELEVEL 0xBB
#define SSD13XX_CMD_VCOMH          0xBE


ImgBufferFormat _get_img_fmt_for_ssd(SSDModel m) {
  switch (m) {
    case SSDModel::SSD1306:   return ImgBufferFormat::MONOCHROME;
    case SSDModel::SSD1309:   return ImgBufferFormat::MONOCHROME;
    case SSDModel::SSD1331:   return ImgBufferFormat::R5_G6_B5;
    case SSDModel::SSD1351:   return ImgBufferFormat::R5_G6_B5;
  }
  return ImgBufferFormat::MONOCHROME;
}

uint32_t _get_img_x_for_ssd(SSDModel m) {
  switch (m) {
    case SSDModel::SSD1306:   return 128;
    case SSDModel::SSD1309:   return 128;
    case SSDModel::SSD1331:   return 96;
    case SSDModel::SSD1351:   return 128;
  }
  return 0;
}

uint32_t _get_img_y_for_ssd(SSDModel m) {
  switch (m) {
    case SSDModel::SSD1306:   return 64;
    case SSDModel::SSD1309:   return 64;
    case SSDModel::SSD1331:   return 64;
    case SSDModel::SSD1351:   return 128;
  }
  return 0;
}


/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor.
*/
SSD13xx::SSD13xx(const SSD13xxOpts* o)
  : Image(_get_img_x_for_ssd(o->model),
    _get_img_y_for_ssd(o->model),
    _get_img_fmt_for_ssd(o->model)),
    _opts(o),
    _fb_data_op(BusOpcode::TX, this, o->cs, false)
{
  _is_framebuffer(true);
}


/**
* Destructor. Should never be called.
*/
SSD13xx::~SSD13xx() {}


/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/**
* Called prior to the given bus operation beginning.
* Returning 0 will allow the operation to continue.
* Returning anything else will fail the operation with IO_RECALL.
*   Operations failed this way will have their callbacks invoked as normal.
*
* @param  _op  The bus operation that is about to run.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t SSD13xx::io_op_callahead(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  bool data_xfer = (op->transferParamLength() == 0);
  if (data_xfer) {  // Was this a frame refresh?
    _stopwatch.markStart();
    _lock(true);
  }
  setPin(_opts.dc, data_xfer);
  return 0;
}

/**
* When a bus operation completes, it is passed back to its issuing class.
* This driver never reads back from the device. Assume all ops are WRITEs.
* The initialization chain is carried forward in this function, so that we don't
*   swamp the bus queue. Display will enable at the end of the init chain.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t SSD13xx::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    return BUSOP_CALLBACK_ERROR;
  }

  if (op->transferParamLength() == 0) {  // Was this a frame refresh?
    _lock(false);
    _stopwatch.markStop();
    if (_enabled && _initd) {
      //ret = BUSOP_CALLBACK_RECYCLE;
    }
  }
  else {
    uint8_t arg_buf[4];
    switch (op->getTransferParam(0)) {
      case SSD13XX_CMD_DISPLAYON:
        _enabled = true;
        //commitFrameBuffer();
        break;
      case SSD13XX_CMD_DISPLAYOFF:   _enabled = false;      break;
      case SSD13XX_CMD_SETREMAP:
        arg_buf[0] = 0x00;
        _send_command(SSD13XX_CMD_STARTLINE, arg_buf, 1);   // 0xA1
        break;
      case SSD13XX_CMD_STARTLINE:
        arg_buf[0] = 0x00;
        _send_command(SSD13XX_CMD_DISPLAYOFFSET, arg_buf, 1);   // 0xA2
        break;
      case SSD13XX_CMD_DISPLAYOFFSET:
        _send_command(SSD13XX_CMD_NORMALDISPLAY);    // 0xA4
        break;
      case SSD13XX_CMD_NORMALDISPLAY:
        arg_buf[0] = 0x3F;  // 0x3F 1/64 duty
        _send_command(SSD13XX_CMD_SETMULTIPLEX, arg_buf, 1);   // 0xA8
        break;
      case SSD13XX_CMD_SETMULTIPLEX:
        arg_buf[0] = 0x8E;
        _send_command(SSD13XX_CMD_SETMASTER, arg_buf, 1);    // 0xAD
        break;
      case SSD13XX_CMD_SETMASTER:
        arg_buf[0] = 0x0B;
        _send_command(SSD13XX_CMD_POWERMODE, arg_buf, 1);    // 0xB0
        break;
      case SSD13XX_CMD_POWERMODE:
        arg_buf[0] = 0x31;
        _send_command(SSD13XX_CMD_PRECHARGE, arg_buf, 1);    // 0xB1
        break;
      case SSD13XX_CMD_PRECHARGE:
        arg_buf[0] = 0xF0;  // 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
        _send_command(SSD13XX_CMD_CLOCKDIV, arg_buf, 1);    // 0xB3
        break;
      case SSD13XX_CMD_CLOCKDIV:
        arg_buf[0] = 0x64;
        _send_command(SSD13XX_CMD_PRECHARGEA, arg_buf, 1);    // 0x8A
        break;
      case SSD13XX_CMD_PRECHARGEA:
        arg_buf[0] = 0x78;
        _send_command(SSD13XX_CMD_PRECHARGEB, arg_buf, 1);    // 0x8B
        break;
      case SSD13XX_CMD_PRECHARGEB:
        arg_buf[0] = 0x64;
        _send_command(SSD13XX_CMD_PRECHARGEC, arg_buf, 1);    // 0x8C
        break;
      case SSD13XX_CMD_PRECHARGEC:
        arg_buf[0] = 0x3A;
        _send_command(SSD13XX_CMD_PRECHARGELEVEL, arg_buf, 1);    // 0xBB
        break;
      case SSD13XX_CMD_PRECHARGELEVEL:
        arg_buf[0] = 0x3E;
        _send_command(SSD13XX_CMD_VCOMH, arg_buf, 1);      // 0xBE
        break;
      case SSD13XX_CMD_VCOMH:
        arg_buf[0] = 0x06;
        _send_command(SSD13XX_CMD_MASTERCURRENT, arg_buf, 1);    // 0x87
        break;
      case SSD13XX_CMD_MASTERCURRENT:
        arg_buf[0] = 0x91;
        _send_command(SSD13XX_CMD_CONTRASTA, arg_buf, 1);    // 0x81
        break;
      case SSD13XX_CMD_CONTRASTA:
        arg_buf[0] = 0x50;
        _send_command(SSD13XX_CMD_CONTRASTB, arg_buf, 1);    // 0x82
        break;
      case SSD13XX_CMD_CONTRASTB:
        arg_buf[0] = 0x7D;
        _send_command(SSD13XX_CMD_CONTRASTC, arg_buf, 1);    // 0x83
        break;
      case SSD13XX_CMD_CONTRASTC:
        _initd = true;  // This is the last register written in the init sequence.
        _send_command(SSD13XX_CMD_DISPLAYON);  //--turn on oled panel
        break;
      default:
        break;
    }
  }
  return ret;
}


/**
* This is what is called when the class wants to conduct a transaction on the bus.
* Note that this is different from other class implementations, in that it checks for
*   callback population before clobbering it. This is because this class is also the
*   SPI driver. This might end up being reworked later.
*
* @param  _op  The bus operation to execute.
* @return Zero on success, or appropriate error code.
*/
int8_t SSD13xx::queue_io_job(BusOp* _op) {
  // This is the choke-point whereby any parameters to the operation that are
  //   uniform for this driver can be set.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->setCSPin(_opts.cs);
  op->csActiveHigh(false);
  return _BUS->queue_io_job(op);
}


/*!
  @brief   SPI displays set an address window rectangle for blitting pixels
  @param  x  Top left corner x coordinate
  @param  y  Top left corner x coordinate
  @param  w  Width of window
  @param  h  Height of window
*/
void SSD13xx::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint8_t x1 = x;
  uint8_t y1 = y;
  if (x1 > 95) x1 = 95;
  if (y1 > 63) y1 = 63;

  uint8_t x2 = (x+w-1);
  uint8_t y2 = (y+h-1);
  if (x2 > 95) x2 = 95;
  if (y2 > 63) y2 = 63;

  if (x1 > x2) {
    strict_swap(&x1, &x2);
  }
  if (y1 > y2) {
    strict_swap(&y1, &y2);
  }

  uint8_t arg_buf[4] = {x1, x2, y1, y2};
  _send_command(0x15, &arg_buf[0], 2); // Column addr set
  _send_command(0x75, &arg_buf[2], 2); // Column addr set
}


int8_t SSD13xx::commitFrameBuffer() {
  int8_t ret = -3;
  if (_fb_data_op.isIdle()) {
    //_fb_data_op.setBuffer(_buffer, bytesUsed());  // Buffer is in Image superclass.
    ret = _BUS->queue_io_job(&_fb_data_op);
  }
  return ret;
}


int8_t SSD13xx::invertDisplay(bool flipped) {
  return _send_command(SSD13XX_CMD_INVERTDISPLAY);
}


/*!
    @brief   Initialize SSD13xx chip.
    Connects to the SSD13xx over SPI and sends initialization procedure commands
    @param    freq  Desired SPI clock frequency
*/
int8_t SSD13xx::init(SPIAdapter* b) {
  int8_t ret = -1;
  _BUS = b;
  if (nullptr != _BUS) {
    ret--;
    _initd    = false;
    _fb_data_op.setAdapter(_BUS);
    _fb_data_op.shouldReap(false);
    if (0 == _ll_pin_init()) {
      ret--;
      if (reallocate()) {
        ret--;
        _fb_data_op.setBuffer(_buffer, bytesUsed());  // Buffer is in Image superclass.
        uint8_t arg_buf[4];
        // Initialization Sequence begins here and is carried forward by the
        //   io_callback.
        _send_command(SSD13XX_CMD_DISPLAYOFF);    // 0xAE
        arg_buf[0] = 0x72; // RGB Color
        if (0 == _send_command(SSD13XX_CMD_SETREMAP, arg_buf, 1)) {
          ret = 0;
        }
      }
    }
  }
  return ret;
}


/*!
  @brief  Change whether display is on or off
  @param   enable True if you want the display ON, false OFF
*/
int8_t SSD13xx::enableDisplay(bool enable) {
  int8_t ret = -1;
  if (_initd) {
    ret--;
    if (enable ^ _enabled) {
      ret = _send_command(enable ? SSD13XX_CMD_DISPLAYON : SSD13XX_CMD_DISPLAYOFF);
    }
    else {
      ret = 0;
    }
  }
  else {
    ret = init();
  }
  return ret;
}


/*!
 @brief   Adafruit_SPITFT Send Command handles complete sending of commands and data
 @param   commandByte       The Command Byte
 @param   dataBytes         A pointer to the Data bytes to send
 @param   numDataBytes      The number of bytes we should send
 */
int8_t SSD13xx::_send_command(uint8_t commandByte, uint8_t* buf, uint8_t buf_len) {
  int8_t ret = -2;
  if (nullptr != _BUS) {
    ret--;
    SPIBusOp* op = (SPIBusOp*) _BUS->new_op(BusOpcode::TX, (BusOpCallback*) this);
    if (nullptr != op) {
      switch (buf_len) {
        case 0:   op->setParams(commandByte);                                       break;
        case 1:   op->setParams(commandByte, *(buf + 0));                           break;
        case 2:   op->setParams(commandByte, *(buf + 0), *(buf + 1));               break;
        case 3:   op->setParams(commandByte, *(buf + 0), *(buf + 1), *(buf + 2));   break;
        default:
          op->setParams(commandByte);
          op->setBuffer(buf, buf_len);
          break;
      }
      ret = queue_io_job(op);
    }
  }
  return ret;
}


/*
* Reset is technically optional. CS and DC are obligatory.
*
* @return 0 on success. -1 otherwise.
*/
int8_t SSD13xx::reset() {
  int8_t ret = -1;
  if (255 != _opts.reset) {
    setPin(_opts.reset, false);  // Hold the display in reset.
    sleep_us(500);
    setPin(_opts.reset, true);
    sleep_us(500);
    ret = 0;
  }
  return ret;
}


/*
* Reset is technically optional. CS and DC are obligatory.
*
* @return 0 on success. -1 otherwise.
*/
int8_t SSD13xx::_ll_pin_init() {
  int8_t ret = -1;
  if (255 != _opts.reset) {
    pinMode(_opts.reset, GPIOMode::OUTPUT);
    reset();
  }
  if (255 != _opts.cs) {
    pinMode(_opts.cs, GPIOMode::OUTPUT);
    setPin(_opts.cs, true);
    if (255 != _opts.dc) {
      pinMode(_opts.dc, GPIOMode::OUTPUT);
      setPin(_opts.dc, true);
      ret = 0;
    }
  }
  return ret;
}


/**
* Debug support method.
*
* @param   StringBuilder* The buffer into which this fxn should write its output.
*/
void SSD13xx::printDebug(StringBuilder* output) {
  output->concatf("SSD13xx (%u x %u) %s", x(), y(), PRINT_DIVIDER_1_STR);
  output->concatf("\tLocked:    %c\n", (locked() ? 'y': 'n'));
  output->concatf("\tInitd:     %c\n", (_initd ? 'y': 'n'));
  output->concatf("\tEnabled:   %c\n", (_enabled ? 'y': 'n'));
  output->concatf("\tDirty:     %c\n", (_is_dirty() ? 'y': 'n'));
  output->concatf("\tAllocated: %c\n", (allocated() ? 'y': 'n'));
  output->concatf("\tPixels:    %u\n", pixels());
  output->concatf("\tBits/pix:  %u\n", _bits_per_pixel());
  output->concatf("\tBytes:     %u\n\n", bytesUsed());
  StopWatch::printDebugHeader(output);
  _stopwatch.printDebug("Redraw", output);
}
