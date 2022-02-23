/*
 *    MAX7219.cpp - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    This permission notice shall be included in all copies or
 *    substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */


#include "MAX7219.h"

//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

//static SPISettings spi_settings_led(10000000, MSBFIRST, SPI_MODE0);

/*
* Segments to be switched on for characters and digits on
* 7-Segment Displays
*/
const static uint8_t charTable[] = {
  0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
  0B01111111,0B01111011,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,
  0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00000000,0B00000000,0B00000000,0B00000000,0B10000000,0B00000001,0B10000000,0B00000000,
  0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
  0B01111111,0B01111011,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
  0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B01111011,
  0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00000000,0B00000000,
  0B01100111,0B00000000,0B00000101,0B00000000,0B00001111,0B00011100,0B00000000,0B00000000,
  0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00001000,
  0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B01111011,
  0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00010101,0B00011101,
  0B01100111,0B00000000,0B00000101,0B00000000,0B00001111,0B00011100,0B00000000,0B00000000,
  0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000
};



MAX7219::MAX7219(const SPIAdapter* ada, const uint8_t csPin, int numDevices) : _BUS(ada), _CS_PIN(csPin), _MAX_DEVICES(numDevices) {
  for(int i=0;i<64;i++) status[i]=0x00;
}


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
* @param  _op  The bus operation that was completed.
* @return 0 to run the op, or non-zero to cancel it.
*/
int8_t MAX7219::io_op_callahead(BusOp* _op) {
  return 0;   // Permit the transfer, always.
}

/**
* When a bus operation completes, it is passed back to its issuing class.
*
* @param  _op  The bus operation that was completed.
* @return BUSOP_CALLBACK_NOMINAL on success, or appropriate error code.
*/
int8_t MAX7219::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    c3p_log(LOG_LEV_ERROR, __PRETTY_FUNCTION__, "Rejected a callback because the BusOp failed.");
    return BUSOP_CALLBACK_ERROR;
  }
  return BUSOP_CALLBACK_NOMINAL;
}


int8_t MAX7219::queue_io_job(BusOp* _op) {
  // This is the choke-point whereby any parameters to the operation that are
  //   uniform for this driver can be set.
  SPIBusOp* op = (SPIBusOp*) _op;
  op->setCSPin(_CS_PIN);
  op->csActiveHigh(false);
  return ((SPIAdapter*)_BUS)->queue_io_job(op);
}




void MAX7219::shutdown(int addr, bool b) {
    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(b)
        spiTransfer(addr, OP_SHUTDOWN,0);
    else
        spiTransfer(addr, OP_SHUTDOWN,1);
}

void MAX7219::setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(limit>=0 && limit<8)
        spiTransfer(addr, OP_SCANLIMIT,limit);
}

void MAX7219::setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(intensity>=0 && intensity<16)
        spiTransfer(addr, OP_INTENSITY,intensity);
}

void MAX7219::clearDisplay(int addr) {
    int offset;

    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    offset=addr*8;
    for(int i=0;i<8;i++) {
        status[offset+i]=0;
        spiTransfer(addr, i+1,status[offset+i]);
    }
}

void MAX7219::setLed(int addr, int row, int column, bool state) {
    int offset;
    uint8_t val=0x00;

    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(row<0 || row>7 || column<0 || column>7)
        return;
    offset=addr*8;
    val=0B10000000 >> column;
    if(state)
        status[offset+row]=status[offset+row]|val;
    else {
        val=~val;
        status[offset+row]=status[offset+row]&val;
    }
    spiTransfer(addr, row+1,status[offset+row]);
}

void MAX7219::setRow(int addr, int row, uint8_t value) {
    int offset;
    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(row<0 || row>7)
        return;
    offset=addr*8;
    status[offset+row]=value;
    spiTransfer(addr, row+1,status[offset+row]);
}

void MAX7219::setColumn(int addr, int col, uint8_t value) {
    uint8_t val;

    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(col<0 || col>7)
        return;
    for(int row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        setLed(addr,row,col,val);
    }
}

void MAX7219::setDigit(int addr, int digit, uint8_t value, bool dp) {
    int offset;
    uint8_t v;

    if(addr<0 || addr>=_MAX_DEVICES)
        return;
    if(digit<0 || digit>7 || value>15)
        return;
    offset=addr*8;
    v = charTable[value];
    if(dp) {
      v|=0B10000000;
    }
    status[offset+digit]=v;
    spiTransfer(addr, digit+1,v);
}

void MAX7219::setChar(int addr, int digit, char value, bool dp) {
  int offset;
  uint8_t index,v;

  if(addr<0 || addr>=_MAX_DEVICES) return;
  if(digit<0 || digit>7) return;
  offset=addr*8;
  index=(uint8_t)value;
  if(index >127) {
    //no defined beyond index 127, so we use the space char
    index=32;
  }
  v = charTable[index];
  if(dp) {
    v|=0B10000000;
  }
  status[offset+digit]=v;
  spiTransfer(addr, digit+1,v);
}

int8_t MAX7219::spiTransfer(int addr, volatile uint8_t opcode, volatile uint8_t data) {
  int8_t ret = -1;
  SPIBusOp* op = (SPIBusOp*) ((SPIAdapter*)_BUS)->new_op(BusOpcode::TX, (BusOpCallback*) this);
  op->setParams(opcode, data);
  ret = queue_io_job(op);
  return ret;
}


void MAX7219::init() {
  setPin(_CS_PIN, true);
  pinMode(_CS_PIN, GPIOMode::OUTPUT);
  for(int i=0;i<_MAX_DEVICES;i++) {
    spiTransfer(i,OP_DISPLAYTEST,0);
    //scanlimit is set to max on startup
    setScanLimit(i,7);
    //decode is done in source
    spiTransfer(i, OP_DECODEMODE, 0);
    shutdown(i,false);    // Wake up the MAX72xx
    clearDisplay(i);      // Clear the display.
    setIntensity(0,10);   // Set a default brightness.
  }
}


/*
* Renders a double to the display with number of decimal places determine by precision
*   precision is a number from 0 to 6 indicating the desired decimial places
*/
void MAX7219::showDouble(double x, uint precision) {
  unsigned long lodp = abs(int(x));
  double rodp = (x < 0) ? (lodp - x) : (x - lodp);
  //clearDisplay(0);

  if ((x < 0) && (lodp >= 100)) {  setRow(0, 7, 0x01); }     // Minus
  else {                           setRow(0, 7, 0x00); }     // Blank space

  if (lodp >= 100) { setDigit(0, 6, (lodp / 100) % 10, false); }
  else if ((x < 0) && (lodp >= 10)) {  setRow(0, 6, 0x01);     } // Minus
  else {             setRow(0, 6, 0x00);                       } // Blank space

  if (lodp >= 10) {  setDigit(0, 5, (lodp / 10) % 10, false);  }
  else if (x < 0) {  setRow(0, 5, 0x01);                       } // Minus
  else {             setRow(0, 5, 0x00);                       } // Blank space

  setDigit(0, 4, lodp % 10, true);
  setDigit(0, 3, int(rodp * 10) % 10, false);
  setDigit(0, 2, int(rodp * 100) % 10, false);
  setDigit(0, 1, int(rodp * 1000) % 10, false);
  setDigit(0, 0, int(rodp * 10000) % 10, false);
}


/*
* Renders a double to the display with number of decimal places determine by precision
*   precision is a number from 0 to 6 indicating the desired decimial places
*/
//void MAX7219::showDouble(double x, uint precision) {
//  uint8_t idx = 8;
//  unsigned long lodp = abs(int(x));
//  unsigned long rhm  = pow(10, (idx-(1 + (x < 0) ? 1 : 0)) - precision);
//  unsigned long temp = rhm;
//  double rodp = (x < 0) ? (lodp - x) : (x - lodp);
//  clearDisplay(0);
//
//  // Calculate the left-padding for the given precision.
//  while (temp > 9) {
//    temp = temp / 10;
//    if (temp > lodp) {
//      setRow(0, --idx, 0x00);  // Leading blank space
//    }
//  }
//
//  if (x < 0) {
//    //setRow(0, --idx, 0x01);  // Minus.
//  }
//
//  while ((idx > 0) && (0 < lodp) && (idx > precision)) {
//    rhm = rhm / 10;
//    unsigned long k = (lodp / rhm);
//    lodp = lodp % rhm;
//    setDigit(0, --idx, k, (0 >= lodp));
//  }
//
//  while ((idx > 0) && (0 < rodp)) {
//    rodp = rodp * 10;
//    temp = int(rodp) % 10;
//    rodp = rodp - temp;
//    setDigit(0, --idx, temp, false);
//  }
//}


/*
* Renders a double to the display.
*/
void MAX7219::showDoubleRaw(double x) {
  clearDisplay(0);
  uint8_t idx = 8;
  unsigned long lodp = abs(int(x));
  unsigned long temp = 0;
  double rodp;
  unsigned int  ruler = 9999999;

  if (x < 0) {
    setRow(0, --idx, 0x01);  // Minus.
    rodp = (lodp - x);
  }
  else {
    rodp = (x - lodp);
  }
  bool have_val_rodp  = (rodp > 0);

  while ((idx > 0) && (ruler > 0)) {
    if (lodp > ruler) {
      temp = lodp / (ruler + 1);
      lodp = lodp - ((ruler + 1) * temp);
      setDigit(0, --idx, temp, false);
    }

    if (9 == ruler) {
      setDigit(0, --idx, lodp, have_val_rodp);
    }
    ruler = ruler / 10;
  }

  if (have_val_rodp) {
    while ((idx > 0) && (0 < rodp)) {
      rodp = rodp * 10;
      temp = int(rodp) % 10;
      rodp = rodp - temp;
      setDigit(0, --idx, temp, false);
    }
  }
}
