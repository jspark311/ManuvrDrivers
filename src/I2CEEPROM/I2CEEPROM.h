/*
* This class extends the Storage API and implements an i2c EEPROM.
* Data is unencrypted.
*
* Data conventions for this class:
*   A page is considered empty if it has a next_block pointer equal to -1 (unsigned).
*
* TODO: An allocation-tracking table would take up 64 bytes for a 32K EEPROM.
*   It might be worth it to save the burdensome I/O seeking for free blocks
*   everytime we want to store data. It might also allow us to select
*   blocks more intelligently.
* Possible block selection strategy: Blocks holding record metadata (the
*   first block of any record) starts at low addresses and grows upward.
*   Strictly data blocks stored at the rear and grows downward. But data is
*   still stored in ascending order such that access is kept as non-random
*   as fragmentation allows.
*/

#ifndef __I2C_EEPROM_H_
#define __I2C_EEPROM_H_

#include "Storage.h"
#include "I2CAdapter.h"



/* These are the enumerated positions in the driver's finite state machine. */
enum class I2CEEPROMFSM : uint8_t {
  UNINIT      = 0,   // Who are we? What is this place?
  PRE_INIT    = 1,   //
  FORMATTING  = 2,   // The driver is setting up the storage for first use.
  ALLOCATING  = 3,   // The driver is searching for free blocks.
  IDLE        = 4,   // Stable. Waiting for I/O requests from application.
  READING     = 5,   // Reading a record.
  WRITING     = 6,   // Writing a record.
  FAULT       = 7    // Application needs to address a problem.
};


/*
* Concrete Storage implementation for an i2c EEPROM.
*/
class I2CEEPROM : public Storage, public I2CDevice {
  public:
    I2CEEPROM(const uint32_t bits, const uint16_t page_size, const uint8_t addr = 0x50);
    ~I2CEEPROM();

    /* Mandatory overrides from Storage.h */
    StorageErr wipe(uint32_t offset, uint32_t len);
    uint8_t    blockAddrSize() {  return DEV_ADDR_SIZE_BYTES;  };
    int8_t     allocateBlocksForLength(uint32_t, DataRecord*);

    StorageErr persistentWrite(uint8_t* buf, unsigned int len, uint32_t offset);
    StorageErr persistentRead(uint8_t* buf,  unsigned int len, uint32_t offset);

    /* Overrides from I2CDevice... */
    int8_t io_op_callahead(BusOp*);
    int8_t io_op_callback(BusOp*);

    int8_t init(I2CAdapter*);
    void printDebug(StringBuilder*);


  private:
    // Derived constants.
    const uint8_t  OVERHEAD_SIZE_BYTES;  // Linked Lists have two addresses.
    const uint16_t PAYLOAD_SIZE_BYTES;   // This many bytes left over per page.

    uint32_t     _op_len_rem     = 0;  // Used to track remaining length in self-generated I/O.
    uint8_t      _addr_ptr[4]    = {0, };  // The current value of the address pointer in the chip.
    uint8_t*     _page_buffer    = nullptr;
    uint8_t*     _alloc_table    = nullptr;  // Tracks allocation of blocks. 1-bit per block.
    DataRecord*  _current_record = nullptr;  // The object currently using the driver for I/O.
    I2CEEPROMFSM _fsm_pos        = I2CEEPROMFSM::UNINIT;
    I2CEEPROMFSM _fsm_pos_prior  = I2CEEPROMFSM::UNINIT;

    I2CBusOp _addr_ptr_op;    // A commonly-used bus operation.
    I2CBusOp _data_io_op;     // A commonly-used bus operation.

    inline const uint _allocation_table_size();
    void _mark_block_allocated(const uint32_t BLKADDR, const bool allocd);
    bool _is_block_allocated(const uint32_t BLKADDR);

    uint32_t _address_ptr();
    int8_t   _set_address_ptr(uint32_t);
    int8_t   _increment_address_ptr(uint32_t);
    int8_t   _store_address_ptr(uint32_t);
    int8_t   _busy_check();

    /* Driver state machine functions */
    int8_t   _set_fsm_position(I2CEEPROMFSM);
};

#endif    // __I2C_EEPROM_H_
