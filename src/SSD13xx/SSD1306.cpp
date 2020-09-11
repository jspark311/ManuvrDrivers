#include "SSD13xx.h"
#include "AbstractPlatform.h"

#define SSD1306_CMD_MEMMODE          0x20  // Takes an additional byte argument.
#define SSD1306_CMD_COL_ADDR         0x21  // Takes an additional byte argument.
#define SSD1306_CMD_PAGEADDR         0x22  // Takes two additional byte arguments.
#define SSD1306_CMD_SCROLL_OFF       0x2E
#define SSD1306_CMD_STARTLINE        0x40  // Argument is embedded in the lowest 6-bits.
#define SSD1306_CMD_CONTRAST         0x81
#define SSD1306_CMD_CHARGEPUMP       0x8D  // Takes an additional byte argument.
#define SSD1306_CMD_DISPLAYALLOFF    0xA4
#define SSD1306_CMD_DISPLAYALLON     0xA5
#define SSD1306_CMD_NORMALDISPLAY    0xA6
#define SSD1306_CMD_DISPLAYOFF       0xAE
#define SSD1306_CMD_DISPLAYON        0xAF
#define SSD1306_CMD_SET_PAGE_START   0xB0
#define SSD1306_CMD_COMSCANINC       0xC0
#define SSD1306_CMD_COMSCANDEC       0xC8
#define SSD1306_CMD_DISPLAYOFFSET    0xD3  // Argument is embedded in the lowest 6-bits.
#define SSD1306_CMD_CLOCKDIV         0xD5  // Takes an additional byte argument.
#define SSD1306_CMD_SETVCOMDETECT    0xD8  // Takes an additional byte argument.
#define SSD1306_CMD_PRECHARGE        0xD9  // Takes an additional byte argument.
#define SSD1306_CMD_SETCOMPINS       0xDA  // Takes an additional byte argument.


extern ImgBufferFormat _get_img_fmt_for_ssd(SSDModel);
extern uint32_t _get_img_x_for_ssd(SSDModel);
extern uint32_t _get_img_y_for_ssd(SSDModel);


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
SSD1306::SSD1306(const SSD13xxOpts* o)
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
SSD1306::~SSD1306() {}


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
int8_t SSD1306::io_op_callahead(BusOp* _op) {
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
int8_t SSD1306::io_op_callback(BusOp* _op) {
  SPIBusOp* op = (SPIBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  // There is zero chance this object will be a null pointer unless it was done on purpose.
  if (op->hasFault()) {
    return BUSOP_CALLBACK_ERROR;
  }

  if (op == &_fb_data_op) {  // Was this a frame refresh?
    _lock(false);
    _stopwatch.markStop();
    if (_enabled && initialized()) {
      //ret = BUSOP_CALLBACK_RECYCLE;
    }
  }
  else if (initialized()) {
    // This driver never reads back from the display. All BusOps are TX.
    uint8_t arg_buf[4];
    // NOTE: The check below won't account for commands that have arguments
    //   built into the same byte. Should probably have a mask and heuristic
    //   check in the default case if we are ever interested in acting upon those.
    switch (op->getTransferParam(0)) {
      case SSD1306_CMD_DISPLAYOFF:
        _enabled = false;
        break;
      case SSD1306_CMD_DISPLAYON:
        _enabled = true;
        commitFrameBuffer();
        break;
      default:
        break;
    }
  }
  else {
    _internal_init_fsm();
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
int8_t SSD1306::queue_io_job(BusOp* _op) {
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
void SSD1306::setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  uint8_t x1 = x;
  uint8_t y1 = y;
  if (x1 > _x) x1 = _x;
  if (y1 > _y) y1 = _y;

  uint8_t x2 = (x+w-1);
  uint8_t y2 = (y+h-1);
  if (x2 > _x) x2 = _x;
  if (y2 > _y) y2 = _y;

  if (x1 > x2) {
    strict_swap(&x1, &x2);
  }
  if (y1 > y2) {
    strict_swap(&y1, &y2);
  }

  uint8_t arg_buf[4] = {x1, x2, y1, y2};
  //_send_command(0x15, &arg_buf[0], 2); // Column addr set
  //_send_command(0x75, &arg_buf[2], 2); // Column addr set
}


int8_t SSD1306::commitFrameBuffer() {
  int8_t ret = -3;
  if (_fb_data_op.isIdle()) {
    //_fb_data_op.setBuffer(_buffer, bytesUsed());  // Buffer is in Image superclass.
    ret = _BUS->queue_io_job(&_fb_data_op);
  }
  return ret;
}


int8_t SSD1306::invertDisplay(bool flipped) {
  return _send_command(SSD13XX_CMD_INVERTDISPLAY);
}


/*!
    @brief   Initialize SSD1306 chip.
    Starts the init chain for the SSD1306.
    @param    b  Pointer to bus adapter to which we are attached.
*/
int8_t SSD1306::init(SPIAdapter* b) {
  int8_t ret = -1;
  _BUS = b;
  if (nullptr != _BUS) {
    ret--;
    _init_state = 0;
    _fb_data_op.setAdapter(_BUS);
    _fb_data_op.shouldReap(false);
    if (0 == _ll_pin_init()) {
      ret--;
      if (reallocate()) {
        ret--;
        _fb_data_op.setBuffer(_buffer, bytesUsed());  // Buffer is in Image superclass.
        // Initialization Sequence begins here and is carried forward by the
        //   io_callback.
        if (0 == _send_command(SSD1306_CMD_DISPLAYOFF, nullptr, 0)) {
          ret = 0;
        }
      }
    }
  }
  return ret;
}


int8_t SSD1306::testDisplay(bool x) {
  int8_t ret = -1;
  if (initialized()) {
    ret--;
    if (0 == _send_command(x ? SSD1306_CMD_DISPLAYALLON : SSD1306_CMD_DISPLAYALLOFF, nullptr, 0)) {
      ret = 0;
    }
  }
  return ret;
}


/*
* Init needs to happen in blocks of operations with delays in between. This
*   function is called after a bus transaction to send the next batch of init
*   operations.
*/
int8_t SSD1306::_internal_init_fsm() {
  uint8_t arg_buf[3] = {0, 0, 0};
  switch (_init_state) {
    case 0:
      if (0 == _send_command(SSD1306_CMD_DISPLAYALLOFF, nullptr, 0)) {  _init_state++;  }
      break;
    case 1:
      arg_buf[0] = 0;
      if (0 == _send_command(SSD1306_CMD_CLOCKDIV, arg_buf, 1)) {       _init_state++;  }
      break;
    case 2:
      arg_buf[0] = y() - 1;
      if (0 == _send_command(SSD13XX_CMD_SETMULTIPLEX, arg_buf, 1)) {   _init_state++;  }
      break;

    case 3:
      arg_buf[0] = _disp_offset;
      if (0 == _send_command(SSD1306_CMD_DISPLAYOFFSET, arg_buf, 1)) {  _init_state++;  }
      break;
    case 4:
      if (0 == _send_command(SSD1306_CMD_STARTLINE | (_start_line & 0x3F), nullptr, 0)) {      _init_state++;  }
      break;
    case 5:
      // If the flags tell us that we are supplied externally, we leave the
      //   charge pump off. Otherwise, turn it on.
      arg_buf[0] = _external_vcc ? 0x10 : 0x14;
      if (0 == _send_command(SSD1306_CMD_CHARGEPUMP, arg_buf, 1)) {     _init_state++;  }
      break;

    case 6:
      arg_buf[0] = _vert_scan ? 1 : 0;  // Horizontal memory increment.
      if (0 == _send_command(SSD1306_CMD_MEMMODE, arg_buf, 1)) {        _init_state++;  }
      break;
    case 7:
      if (0 == _send_command(SSD13XX_CMD_SETREMAP | (_remap_enable ? 1 : 0), nullptr, 0)) {   _init_state++;  }
      break;
    case 8:
      if (0 == _send_command(_com_scan_dec ? SSD1306_CMD_COMSCANDEC : SSD1306_CMD_COMSCANINC, nullptr, 0)) {     _init_state++;  }
      break;

    case 9:
      arg_buf[0] = _com_pin_conf;
      if (0 == _send_command(SSD1306_CMD_SETCOMPINS, arg_buf, 1)) {     _init_state++;  }
      break;
    case 10:
      // TODO: Are we contrast limited on external vcc? Why this differential?
      arg_buf[0] = _external_vcc ? 0x9F : 0xCF;
      if (0 == _send_command(SSD1306_CMD_CONTRAST, arg_buf, 1)) {       _init_state++;  }
      break;

    case 11:
      // TODO: Why this differential?
      arg_buf[0] = _external_vcc ? 0x22 : 0xF1;
      if (0 == _send_command(SSD1306_CMD_PRECHARGE, arg_buf, 1)) {      _init_state++;  }
      break;

    case 12:
      arg_buf[0] = 0x40;
      if (0 == _send_command(SSD1306_CMD_SETVCOMDETECT, arg_buf, 1)) {  _init_state++;  }
      break;
    case 13:
      if (0 == _send_command(SSD1306_CMD_DISPLAYALLOFF, nullptr, 0)) {  _init_state++;  }
      break;
    case 14:
      if (0 == _send_command(SSD13XX_CMD_INVERTDISPLAY & 0xA6, nullptr, 0)) {  _init_state++;  }
      break;
    case 15:
      if (0 == _send_command(SSD1306_CMD_SCROLL_OFF, nullptr, 0)) {     _init_state++;  }
      break;

    // This driver assumes a 128x64 layout. So we use the entire memory space.
    case 16:
      arg_buf[0] = 0;
      arg_buf[1] = 0x0F;
      if (0 == _send_command(SSD1306_CMD_PAGEADDR, arg_buf, 2)) {       _init_state++;  }
      break;
    case 17:
      arg_buf[0] = 0;
      arg_buf[1] = x()-1;
      if (0 == _send_command(SSD1306_CMD_COL_ADDR, arg_buf, 2)) {       _init_state++;  }
      break;

    case 18:
      if (0 == _send_command(SSD1306_CMD_DISPLAYON, nullptr, 0)) {      _init_state++;  }
      _enabled = true;
      break;

    case 19:
      // Initialized and enabled.
      commitFrameBuffer();
      _init_state++;
      break;
    default:
      break;
  }
  return 0;
}


/*!
  @brief  Change whether display is on or off
  @param   enable True if you want the display ON, false OFF
*/
int8_t SSD1306::enableDisplay(bool enable) {
  int8_t ret = -1;
  if (initialized()) {
    ret--;
    if (enable ^ _enabled) {
      ret = _send_command(enable ? SSD1306_CMD_DISPLAYON : SSD1306_CMD_DISPLAYOFF);
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
int8_t SSD1306::_send_command(uint8_t commandByte, uint8_t* buf, uint8_t buf_len) {
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
int8_t SSD1306::reset() {
  int8_t ret = -1;
  _init_state = 0;
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
int8_t SSD1306::_ll_pin_init() {
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
void SSD1306::printDebug(StringBuilder* output) {
  output->concatf("SSD1306 (%u x %u) %s", x(), y(), PRINT_DIVIDER_1_STR);
  output->concatf("\tLocked:    %c\n", (locked() ? 'y': 'n'));
  output->concatf("\tInitd:     %c (state %u)\n", (initialized() ? 'y': 'n'), _init_state);
  output->concatf("\tEnabled:   %c\n", (_enabled ? 'y': 'n'));
  output->concatf("\tDirty:     %c\n", (_is_dirty() ? 'y': 'n'));
  output->concatf("\tAllocated: %c\n", (allocated() ? 'y': 'n'));
  output->concatf("\tPixels:    %u\n", pixels());
  output->concatf("\tBits/pix:  %u\n", _bits_per_pixel());
  output->concatf("\tBytes:     %u\n\n", bytesUsed());
  StopWatch::printDebugHeader(output);
  _stopwatch.printDebug("Redraw", output);
}
