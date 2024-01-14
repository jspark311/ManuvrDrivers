/*
* This driver operates only in "Mode-1", in the datasheet's parlance.
*
* Vector data produced by this driver is given as LH_POS_Z, with Z+ representing
*   UP, or yaw to the right, depending on ACC/GYR.
*
* Interrupt pins are optional, and if provided, they are used this way...
*   INT1 INT2
*   ---------|----------------------------------------------------------------
*    No   No | No IRQs configured. Reads are timed by dead-reckoning.
*    No  Yes | Activity on INT2.
*    Yes  No | FIFO high-water mark on INT1.
*    Yes Yes | FIFO high-water mark on INT1. Activity on INT2.
*/


#ifndef __DRIVER_LSM6DSOX_H__
#define __DRIVER_LSM6DSOX_H__

#include "AbstractPlatform.h"
#include "BusQueue/SPIAdapter.h"
#include "Vector3.h"
#include "Pipes/TripleAxisPipe/TripleAxisPipe.h"


/*
* Class flags.
*/
#define LSM6DSOX_FLAG_PINS_CONFIGURED  0x00000001  // Low-level pin setup is complete.
#define LSM6DSOX_FLAG_DEV_FOUND        0x00000002  // Device positively identified.
#define LSM6DSOX_FLAG_INITIALIZED      0x00000004  // Device initialized.
#define LSM6DSOX_FLAG_CALIBRATED       0x00000008  // Offsets calibrated.
#define LSM6DSOX_FLAG_IRQ_USES_ALT_ISR 0x00000010  // IRQ pins are used by hardware, but not this driver.
#define LSM6DSOX_FLAG_NEW_DATA_ACC     0x00000020  // Accelerometer data was refreshed.
#define LSM6DSOX_FLAG_NEW_DATA_GYRO    0x00000040  // Gyroscope data was refreshed.
#define LSM6DSOX_FLAG_NEW_DATA_TEMP    0x00000080  // Temperature data was refreshed.


// Flags that are preserved through reset.
#define LSM6DSOX_FLAG_RESET_MASK  (LSM6DSOX_FLAG_PINS_CONFIGURED | LSM6DSOX_FLAG_DEV_FOUND | LSM6DSOX_FLAG_IRQ_USES_ALT_ISR)


/*
* NOTE: These are indicies. Not real register addresses. Only order is reflected.
*/
enum class LSM6DSOXRegister : uint8_t {
  // Name                     Idx       Addr   Offset  Width  Writable
  // ------------------------------------------------------------------
  FUNC_CONFIG_ACCESS       = 0x00,   // 0x01     0x00      1      true
  PIN_CTRL_REG             = 0x01,   // 0x02     0x01      1      true
  S4S_TPH                  = 0x02,   // 0x04     0x02      2      true
  S4S_RR                   = 0x03,   // 0x06     0x04      1      true
  FIFO_CTRL1               = 0x04,   // 0x07     0x05      1      true
  FIFO_CTRL2               = 0x05,   // 0x08     0x06      1      true
  FIFO_CTRL3               = 0x06,   // 0x09     0x07      1      true
  FIFO_CTRL4               = 0x07,   // 0x0A     0x08      1      true
  COUNTER_BDR_REG1         = 0x08,   // 0x0B     0x09      1      true
  COUNTER_BDR_REG2         = 0x09,   // 0x0C     0x0A      1      true
  INT1_CTRL                = 0x0A,   // 0x0D     0x0B      1      true
  INT2_CTRL                = 0x0B,   // 0x0E     0x0C      1      true
  WHO_AM_I                 = 0x0C,   // 0x0F     0x0D      1     false
  CTRL1_XL                 = 0x0D,   // 0x10     0x0E      1      true
  CTRL2_G                  = 0x0E,   // 0x11     0x0F      1      true
  CTRL3_C                  = 0x0F,   // 0x12     0x10      1      true
  CTRL4_C                  = 0x10,   // 0x13     0x11      1      true
  CTRL5_C                  = 0x11,   // 0x14     0x12      1      true
  CTRL6_C                  = 0x12,   // 0x15     0x13      1      true
  CTRL7_G                  = 0x13,   // 0x16     0x14      1      true
  CTRL8_XL                 = 0x14,   // 0x17     0x15      1      true
  CTRL9_XL                 = 0x15,   // 0x18     0x16      1      true
  CTRL10_C                 = 0x16,   // 0x19     0x17      1      true
  ALL_INT_SRC              = 0x17,   // 0x1A     0x18      1     false
  WAKE_UP_SRC              = 0x18,   // 0x1B     0x19      1     false
  TAP_SRC                  = 0x19,   // 0x1C     0x1A      1     false
  D6D_SRC                  = 0x1A,   // 0x1D     0x1B      1     false
  STATUS_REG               = 0x1B,   // 0x1E     0x1C      1     false
  OUT_TEMP                 = 0x1C,   // 0x20     0x1D      2     false
  OUTX_G                   = 0x1D,   // 0x22     0x1F      2     false
  OUTY_G                   = 0x1E,   // 0x24     0x21      2     false
  OUTZ_G                   = 0x1F,   // 0x26     0x23      2     false
  OUTX_A                   = 0x20,   // 0x28     0x25      2     false
  OUTY_A                   = 0x21,   // 0x2A     0x27      2     false
  OUTZ_A                   = 0x22,   // 0x2C     0x29      2     false
  EMB_FUNC_STATUS_MAINPAGE = 0x23,   // 0x35     0x2B      1     false
  FSM_STATUS_A_MAINPAGE    = 0x24,   // 0x36     0x2C      1     false
  FSM_STATUS_B_MAINPAGE    = 0x25,   // 0x37     0x2D      1     false
  MLC_STATUS_MAINPAGE      = 0x26,   // 0x38     0x2E      1     false
  STATUS_MASTER_MAINPAGE   = 0x27,   // 0x39     0x2F      1     false
  FIFO_STATUS1             = 0x28,   // 0x3A     0x30      1     false
  FIFO_STATUS2             = 0x29,   // 0x3B     0x31      1     false
  TIMESTAMP0               = 0x2A,   // 0x40     0x32      1     false
  TIMESTAMP1               = 0x2B,   // 0x41     0x33      1     false
  TIMESTAMP2               = 0x2C,   // 0x42     0x34      1     false
  TIMESTAMP3               = 0x2D,   // 0x43     0x35      1     false
  UI_STATUS_REG_OIS        = 0x2E,   // 0x49     0x36      1     false
  UI_OUTX_G_OIS            = 0x2F,   // 0x4A     0x37      2     false
  UI_OUTY_G_OIS            = 0x30,   // 0x4C     0x39      2     false
  UI_OUTZ_G_OIS            = 0x31,   // 0x4E     0x3B      2     false
  UI_OUTX_A_OIS            = 0x32,   // 0x50     0x3D      2     false
  UI_OUTY_A_OIS            = 0x33,   // 0x52     0x3F      2     false
  UI_OUTZ_A_OIS            = 0x34,   // 0x54     0x41      2     false
  TAP_CFG0                 = 0x35,   // 0x56     0x43      1      true
  TAP_CFG1                 = 0x36,   // 0x57     0x44      1      true
  TAP_CFG2                 = 0x37,   // 0x58     0x45      1      true
  TAP_THS_6D               = 0x38,   // 0x59     0x46      1      true
  INT_DUR2                 = 0x39,   // 0x5A     0x47      1      true
  WAKE_UP_THS              = 0x3A,   // 0x5B     0x48      1      true
  WAKE_UP_DUR              = 0x3B,   // 0x5C     0x49      1      true
  FREE_FALL                = 0x3C,   // 0x5D     0x4A      1      true
  MD1_CFG                  = 0x3D,   // 0x5E     0x4B      1      true
  MD2_CFG                  = 0x3E,   // 0x5F     0x4C      1      true
  S4S_ST_CMD_CODE          = 0x3F,   // 0x60     0x4D      1      true
  S4S_DT_REG               = 0x40,   // 0x61     0x4E      1      true
  I3C_BUS_AVB              = 0x41,   // 0x62     0x4F      1      true
  INTERNAL_FREQ_FINE       = 0x42,   // 0x63     0x50      1     false
  UI_INT_OIS               = 0x43,   // 0x6F     0x51      1      true   // R/W is contingent
  UI_CTRL1_OIS             = 0x44,   // 0x70     0x52      1      true   // R/W is contingent
  UI_CTRL2_OIS             = 0x45,   // 0x71     0x53      1      true   // R/W is contingent
  UI_CTRL3_OIS             = 0x46,   // 0x72     0x54      1      true   // R/W is contingent
  X_OFS_USR                = 0x47,   // 0x73     0x55      1      true
  Y_OFS_USR                = 0x48,   // 0x74     0x56      1      true
  Z_OFS_USR                = 0x49,   // 0x75     0x57      1      true
  FIFO_DATA_OUT_TAG        = 0x4A,   // 0x78     0x58      1     false
  FIFO_DATA_OUT_X          = 0x4B,   // 0x79     0x59      2     false
  FIFO_DATA_OUT_Y          = 0x4C,   // 0x7B     0x5B      2     false
  FIFO_DATA_OUT_Z          = 0x4D,   // 0x7D     0x5D      2     false
  INVALID                  = 0x4E    // The end of the enum. Not a real register.
};


/*
* Merged ODR enum for external use.
*/
enum class LSM6DSOX_ODR : uint8_t {
  ODR_0     = 0x00,  // Sensor disabled.
  ODR_12_5  = 0x01,
  ODR_26    = 0x02,
  ODR_52    = 0x03,
  ODR_104   = 0x04,
  ODR_208   = 0x05,
  ODR_416   = 0x06,
  ODR_833   = 0x07,
  ODR_1660  = 0x08,
  ODR_3330  = 0x09,
  ODR_6660  = 0x0A,
  ODR_1_6   = 0x0B,  // Special ultra-low power mode.
  INVALID   = 0xFF
};



class LSM6DSOX : public BusOpCallback {
  public:
    LSM6DSOX(const uint8_t cs_pin, const uint8_t int1_pin, const uint8_t int2_pin);
    ~LSM6DSOX();

    /* Overrides from the BusOpCallback interface */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);
    int8_t queue_io_job(BusOp*);

    int8_t  poll();
    int8_t  init(SPIAdapter* b = nullptr);
    int8_t  reset();
    int8_t  refresh();
    bool    enable(bool enable,
                   LSM6DSOX_ODR LSM6DSOX_acc_ODR  = DEFAULT_ODR,
                   LSM6DSOX_ODR LSM6DSOX_gyro_ODR = DEFAULT_ODR);
    bool enabled();
    int8_t enableTemp(LSM6DSOX_ODR ODR_Temp = DEFAULT_TEMP);

    inline void attachPipe(TripleAxisPipe* tap) {   _pipeline = tap;   };
    inline void efferentGnomon(GnomonType n) {      _NXT_FMT  = n;     };
    inline GnomonType efferentGnomon() {            return _NXT_FMT;   };

    /* Accessors taken from flags. */
    inline bool devFound() {      return _class_flag(LSM6DSOX_FLAG_DEV_FOUND);     };
    inline bool initialized() {   return _class_flag(LSM6DSOX_FLAG_INITIALIZED);   };
    inline bool calibrated() {    return _class_flag(LSM6DSOX_FLAG_CALIBRATED);    };
    inline bool dataReadyAcc() {  return _class_flag(LSM6DSOX_FLAG_NEW_DATA_ACC);  };
    inline bool dataReadyGyro() { return _class_flag(LSM6DSOX_FLAG_NEW_DATA_GYRO); };
    inline bool dataReadyTemp() { return _class_flag(LSM6DSOX_FLAG_NEW_DATA_TEMP); };

    /* Functions to deal with interrupt features. */
    inline void forceINTUse(bool x) {   _class_set_flag(LSM6DSOX_FLAG_IRQ_USES_ALT_ISR, x);   };
    inline bool usingINT1() {   return ((_INT1_PIN != 255) | (_class_flag(LSM6DSOX_FLAG_IRQ_USES_ALT_ISR)));  };
    inline bool usingINT2() {   return ((_INT2_PIN != 255) | (_class_flag(LSM6DSOX_FLAG_IRQ_USES_ALT_ISR)));  };

    /* Accessors taken straight from registers. */
    bool temperatureEnabled();
    bool accLowPowerMode();
    bool accFSMode();
    bool gyroLowPowerMode();
    uint8_t readStatus();

    uint16_t accODR();
    uint16_t gyroODR();
    Vector3<float>* getAcc();
    Vector3<float>* getGyro();
    double getTemperature();

    void printDebug(StringBuilder*);
    void printRegisters(StringBuilder*);
    void printBusOps(StringBuilder*);
    int console_handler(StringBuilder*, StringBuilder*);


  private:
    const static LSM6DSOX_ODR DEFAULT_ODR  = LSM6DSOX_ODR::ODR_104;
    const static LSM6DSOX_ODR DEFAULT_TEMP = LSM6DSOX_ODR::ODR_12_5;
    const uint8_t  _CS_PIN;
    const uint8_t  _INT1_PIN;
    const uint8_t  _INT2_PIN;
    uint32_t       _flags        = 0;
    SPIAdapter*    _BUS          = nullptr;
    TripleAxisPipe* _pipeline    = nullptr; // We are a source for this pipeline.
    Vector3<float> _acc;          // Given in g's.
    Vector3<float> _gyro;         // Given in deg/sec.
    Vector3<float> _offset_acc;   // Given in g's.
    Vector3<float> _offset_gyro;  // Given in deg/sec.
    Vector3<float> _err_acc;      // Given in g's.
    Vector3<float> _err_gyro;     // Given in deg/sec.
    double         _data_scale_acc = 1.0;   // Bits-per-g.
    double         _data_scale_gyro = 1.0;  // Bits-per-radian/sec.
    float          _temperature    = 0.0;   // In Celcius.
    uint32_t       _last_read_axes = 0;
    uint32_t       _last_read_gyro = 0;
    uint32_t       _last_read_fifo = 0;
    uint32_t       _last_read_temp = 0;
    uint16_t       _fifo_remaining = 0;    // How much data is left in the FIFO?
    uint8_t        reg_shadows[95] = {0, };
    uint8_t        _verbosity      = 7;    // How chatty is the class?
    GnomonType      _NXT_FMT = GnomonType::LH_POS_Z; // Sensor default

    SPIBusOp       _imu_data_refresh;
    SPIBusOp       _fifo_lev_refresh;


    uint8_t _temp_bits_per_sample();
    uint8_t _acc_bits_per_sample();
    uint8_t _gyro_bits_per_sample();
    int8_t  _acc_get_sample_range();
    int16_t _gyro_get_sample_range();
    int8_t  _read_axes();
    int8_t  _read_temperature();
    int8_t  _read_fifo_level();

    /* Basal register access and utility fxn's */
    int8_t  _clear_registers();
    int8_t  _check_register_value_for_write(LSM6DSOXRegister, unsigned int val);
    int8_t  _set_shadow_value(LSM6DSOXRegister, unsigned int val);
    unsigned int _get_shadow_value(LSM6DSOXRegister);

    int8_t  _write_register(LSM6DSOXRegister, uint8_t val);
    int8_t  _write_registers(LSM6DSOXRegister, uint8_t len);
    int8_t  _read_register(LSM6DSOXRegister);
    int8_t  _read_registers(LSM6DSOXRegister, uint8_t len);
    int8_t  _bus_read_callback(uint8_t reg_addr, uint8_t* buf, uint16_t len);
    int8_t  _bus_write_callback(uint8_t reg_addr, uint8_t* buf, uint16_t len);
    int8_t  _ll_pin_init();
    int8_t  _impart_initial_config();

    inline bool _have_int1_pin() {   return (255 != _INT1_PIN);  };
    inline bool _have_int2_pin() {   return (255 != _INT2_PIN);  };

    /* Flag manipulation inlines */
    inline uint32_t _class_flags() {                    return _flags;           };
    inline uint32_t _class_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void     _class_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void     _class_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void     _class_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};


#endif  // __DRIVER_LSM6DSOX_H__
