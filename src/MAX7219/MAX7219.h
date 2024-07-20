/*
 *    MAX7219.h - A library for controling Leds with a MAX7219/MAX7221
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

#ifndef MAX7219_H
#define MAX7219_H

#include <stdint.h>
#include "BusQueue/SPIAdapter.h"


class MAX7219 : public BusOpCallback {
  public:
    /*
    * Create a new controler
    * Params :
    * csPin    pin for selecting the device
    * numDevices maximum number of devices that can be controled
    */
    MAX7219(const SPIAdapter*, const uint8_t csPin, int numDevices=1);

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    void init();
    void showDouble(double x, uint32_t precision);
    void showDoubleRaw(double x);


    /*
    * Gets the number of devices attached to this MAX7219.
    * Returns :
    * int  the number of devices on this MAX7219
    */
    inline int getDeviceCount() {    return _MAX_DEVICES;    };

    /*
    * Set the shutdown (power saving) mode for the device
    * Params :
    * addr The address of the display to control
    * status If true the device goes into power-down mode. Set to false
    *    for normal operation.
    */
    void shutdown(int addr, bool status);

    /*
    * Set the number of digits (or rows) to be displayed.
    * See datasheet for sideeffects of the scanlimit on the brightness
    * of the display.
    * Params :
    * addr address of the display to control
    * limit  number of digits to be displayed (1..8)
    */
    void setScanLimit(int addr, int limit);

    /*
    * Set the brightness of the display.
    * Params:
    * addr   the address of the display to control
    * intensity  the brightness of the display. (0..15)
    */
    void setIntensity(int addr, int intensity);

    /*
    * Switch all Leds on the display off.
    * Params:
    * addr address of the display to control
    */
    void clearDisplay(int addr);

    /*
    * Set the status of a single Led.
    * Params :
    * addr address of the display
    * row  the row of the Led (0..7)
    * col  the column of the Led (0..7)
    * state  If true the led is switched on,
    *    if false it is switched off
    */
    void setLed(int addr, int row, int col, bool state);

    /*
    * Set all 8 Led's in a row to a new state
    * Params:
    * addr address of the display
    * row  row which is to be set (0..7)
    * value  each bit set to 1 will light up the
    *    corresponding Led.
    */
    void setRow(int addr, int row, uint8_t value);

    /*
    * Set all 8 Led's in a column to a new state
    * Params:
    * addr address of the display
    * col  column which is to be set (0..7)
    * value  each bit set to 1 will light up the
    *    corresponding Led.
    */
    void setColumn(int addr, int col, uint8_t value);

    /*
    * Display a hexadecimal digit on a 7-Segment Display
    * Params:
    * addr address of the display
    * digit  the position of the digit on the display (0..7)
    * value  the value to be displayed. (0x00..0x0F)
    * dp sets the decimal point.
    */
    void setDigit(int addr, int digit, uint8_t value, bool dp);

    /*
    * Display a character on a 7-Segment display.
    * There are only a few characters that make sense here :
    *  '0','1','2','3','4','5','6','7','8','9','0',
    *  'A','b','c','d','E','F','H','L','P',
    *  '.','-','_',' '
    * Params:
    * addr address of the display
    * digit  the position of the character on the display (0..7)
    * value  the character to be displayed.
    * dp sets the decimal point.
    */
    void setChar(int addr, int digit, char value, bool dp);


  private :
    const SPIAdapter* _BUS;
    const uint8_t _CS_PIN;
    const uint8_t _MAX_DEVICES;  // The maximum number of devices we use

    uint8_t spidata[16];   // The array for shifting the data to the devices
    uint8_t status[64];    // We keep track of the led-status for all devices in this array

    /* Send out a single command to the device */
    int8_t spiTransfer(int addr, uint8_t opcode, uint8_t data);
};

#endif  // MAX7219_H
