#include "I2CEEPROM.h"


/**
* Constructor
*/
I2CEEPROM::I2CEEPROM(const uint32_t bits, const uint16_t page_size, const uint8_t addr)
  : Storage(bits >> 3, page_size), I2CDevice(addr),
    OVERHEAD_SIZE_BYTES(DEV_ADDR_SIZE_BYTES << 1),
    PAYLOAD_SIZE_BYTES(page_size - OVERHEAD_SIZE_BYTES),
    _addr_ptr_op(BusOpcode::TX, (I2CDevice*) this),
    _data_io_op(BusOpcode::RX, (I2CDevice*) this)
{
  _pl_set_flag(PL_FLAG_MEDIUM_WRITABLE | PL_FLAG_MEDIUM_READABLE | PL_FLAG_MEDIUM_MOUNTED);
}

/**
* Destructor
*/
I2CEEPROM::~I2CEEPROM() {
  if (nullptr != _page_buffer) {
    free(_page_buffer);
    _page_buffer = nullptr;
  }
  if (nullptr != _alloc_table) {
    free(_alloc_table);
    _alloc_table = nullptr;
  }
}


/*******************************************************************************
* Storage functions
*******************************************************************************/

/*
* Wipes must be page-aligned, and of a length equal to a multiple of DEV_BLOCK_SIZE.
*/
StorageErr I2CEEPROM::wipe(uint32_t offset, uint32_t range) {
  StorageErr ret = StorageErr::BUSY;
  if ((DEV_SIZE_BYTES <= (offset + DEV_BLOCK_SIZE)) | (0 != (range % DEV_BLOCK_SIZE))) {
    return StorageErr::BAD_PARAM;
  }
  if (0 == _busy_check()) {
    ret = StorageErr::HW_FAULT;
    for (uint i = 0; i < DEV_BLOCK_SIZE; i++) {  *(_page_buffer + i) = 0xFF;  }
    _op_len_rem = strict_min(range, (DEV_SIZE_BYTES - offset));  // Constrain.
    if (0 == _set_address_ptr(offset)) {
      _data_io_op.set_opcode(BusOpcode::TX);
      _data_io_op.setBuffer(_page_buffer, DEV_BLOCK_SIZE);
      _set_fsm_position(I2CEEPROMFSM::FORMATTING);
      if (0 == queue_io_job(&_data_io_op)) {
        ret = StorageErr::NONE;
      }
    }
    else {
      // TODO: Might also be HW.
      return StorageErr::BAD_PARAM;
    }
  }
  return ret;
}


int8_t I2CEEPROM::allocateBlocksForLength(uint32_t len, DataRecord* rec) {
  int8_t ret = -1;
  if (0 == _busy_check()) {
    _op_len_rem = len;
    _current_record = rec;
    _set_fsm_position(I2CEEPROMFSM::ALLOCATING);
  }
  return ret;
}


/**
*
*
* @param buf is the buffer containing the data.
* @param len is the number of bytes to write.
* @param addr is the in-chip address of the byte to start at.
* @return StorageErr
*/
StorageErr I2CEEPROM::persistentWrite(uint8_t* buf, unsigned int len, uint32_t addr) {
  StorageErr ret = StorageErr::BUSY;
  if (0 == _busy_check()) {
    ret = StorageErr::HW_FAULT;
    _pl_set_flag(PL_FLAG_BUSY_WRITE);
    if (addr != _address_ptr()) {
      // If the write is non-sequential, we must precede it with an address.
      if (0 != _set_address_ptr(addr)) {
        // TODO: Might also be HW.
        return StorageErr::BAD_PARAM;
      }
    }
    const uint32_t BYTES_NEXT_LEN = strict_min((uint32_t) len, (uint32_t) DEV_BLOCK_SIZE);
    for (uint i = 0; i < BYTES_NEXT_LEN; i++) {
      *(_page_buffer + i) = *(buf + i);
    }
    _data_io_op.set_opcode(BusOpcode::TX);
    _data_io_op.setBuffer(_page_buffer, (uint16_t) BYTES_NEXT_LEN);
    _set_fsm_position(I2CEEPROMFSM::WRITING);
    if (0 == queue_io_job(&_data_io_op)) {
      ret = StorageErr::NONE;
    }
  }
  return ret;
}


/**
* Dispatches I/O to read a given amount of data into the given buffer.
*
* @param buf is the buffer to receive the data.
* @param len is the number of bytes to read.
* @param addr is the in-chip address of the byte to start at.
* @return StorageErr
*/
StorageErr I2CEEPROM::persistentRead(uint8_t* buf, unsigned int len, uint32_t addr) {
  StorageErr ret = StorageErr::BUSY;
  if (0 == _busy_check()) {
    ret = StorageErr::HW_FAULT;
    if (addr != _address_ptr()) {
      // If the read is non-sequential, we must precede it with an address.
      if (0 != _set_address_ptr(addr)) {
        // TODO: Might also be HW.
        return StorageErr::BAD_PARAM;
      }
    }
    const uint32_t BYTES_NEXT_LEN = strict_min((uint32_t) len, (uint32_t) DEV_BLOCK_SIZE);
    _data_io_op.set_opcode(BusOpcode::RX);
    _data_io_op.setBuffer(_page_buffer, (uint16_t) BYTES_NEXT_LEN);
    _set_fsm_position(I2CEEPROMFSM::READING);
    if (0 == queue_io_job(&_data_io_op)) {
      ret = StorageErr::NONE;
    }
  }
  return ret;
}



/*******************************************************************************
* I2C functions
*******************************************************************************/

/**
*
*
* @return  0 on success
*         -1 on mem allocation failure
*/
int8_t I2CEEPROM::init(I2CAdapter* b) {
  int8_t ret = -1;
  if (nullptr == _bus) {
    _bus = b;   // Idempotent bus assignment.
  }
  if (nullptr == _page_buffer) {
    _page_buffer = (uint8_t*) malloc(DEV_BLOCK_SIZE);
    if (nullptr == _page_buffer) return ret;
  }
  if (nullptr == _alloc_table) {
    const uint ALLOC_TABLE_SIZE = _allocation_table_size();
    _alloc_table = (uint8_t*) malloc(ALLOC_TABLE_SIZE);
    if (nullptr == _alloc_table) return ret;
    for (uint i = 0; i < ALLOC_TABLE_SIZE; i++) {  *(_alloc_table + i) = 0;  }
  }

  _addr_ptr_op.shouldReap(false);
  _addr_ptr_op.sub_addr = -1;
  _addr_ptr_op.dev_addr = _dev_addr;
  _addr_ptr_op.set_opcode(BusOpcode::TX);
  _addr_ptr_op.setBuffer(_addr_ptr, DEV_ADDR_SIZE_BYTES);

  _data_io_op.shouldReap(false);
  _data_io_op.sub_addr = -1;
  _data_io_op.dev_addr = _dev_addr;
  _data_io_op.set_opcode(BusOpcode::RX);
  _data_io_op.setBuffer(_page_buffer, DEV_BLOCK_SIZE);

  if (0 == queue_io_job(&_addr_ptr_op)) {
    _set_fsm_position(I2CEEPROMFSM::PRE_INIT);
  }
  return ret;
}


void I2CEEPROM::printDebug(StringBuilder* output) {
  Storage::printStorage(output);
  output->concatf("\t op_len_rem:\t %u\n", _op_len_rem);
  output->concatf("\t Locked by DataRecord: %c\n", (nullptr != _current_record) ? 'y':'n');
  output->concatf("\t address_ptr:\t %u\n", _address_ptr());
  output->concatf("\t fsm_pos/prior:\t %u / %u\n", (uint8_t) _fsm_pos, (uint8_t) _fsm_pos_prior);

  if (nullptr != _page_buffer) {
    output->concat("\t Page buffer:\n");
    StringBuilder::printBuffer(output, _page_buffer, DEV_BLOCK_SIZE, "\t\t");
  }

  if (nullptr != _alloc_table) {
    output->concat("\t Allocation table:\n");
    StringBuilder::printBuffer(output, _alloc_table, _allocation_table_size(), "\t\t");
  }
}


/**
* One of the most common things to do in this driver is to set the internal
*   address pointer.
* This function immediately changes the value of the address pointer shadow.
*
* @param a is the new address.
* @return 0 on success, negative otherwise.
*/
int8_t I2CEEPROM::_set_address_ptr(uint32_t a) {
  int8_t ret = -1;
  if (0 == _store_address_ptr(a)) {
    if (0 == queue_io_job(&_addr_ptr_op)) {
      ret = 0;
    }
  }
  return ret;
}

uint32_t I2CEEPROM::_address_ptr() {
  uint32_t tmp = 0;
  for (uint8_t i = 0; i < DEV_ADDR_SIZE_BYTES; i++) {
    tmp = (tmp << 8) + _addr_ptr[i];
  }
  return tmp;
}

int8_t I2CEEPROM::_increment_address_ptr(uint32_t x) {
  uint32_t tmp = _address_ptr();
  tmp = tmp + x;   // Do not allow address wrap-around. Fail instead.
  return _store_address_ptr(tmp);
}

int8_t I2CEEPROM::_store_address_ptr(uint32_t x) {
  if (DEV_SIZE_BYTES > x) {
    for (uint8_t i = 0; i < DEV_ADDR_SIZE_BYTES; i++) {
      _addr_ptr[i] = (uint8_t) (x >> (i << 3)) & 0xFF;
    }
    return 0;
  }
  return -1;
}


int8_t I2CEEPROM::_busy_check() {
  int8_t ret = -1;
  if (!isBusy()) {
    if (_addr_ptr_op.isIdle() && _data_io_op.isIdle()) {
      ret = 0;
    }
  }
  return ret;
}


int8_t I2CEEPROM::_set_fsm_position(I2CEEPROMFSM new_state) {
  int8_t ret = 0;
  _fsm_pos_prior = _fsm_pos;
  _fsm_pos       = new_state;
  switch (new_state) {
    case I2CEEPROMFSM::IDLE:
      _pl_clear_flag(PL_FLAG_BUSY_READ | PL_FLAG_BUSY_WRITE);
      break;
    case I2CEEPROMFSM::ALLOCATING:
    case I2CEEPROMFSM::READING:
      _pl_set_flag(PL_FLAG_BUSY_READ);
      break;
    case I2CEEPROMFSM::FORMATTING:
    case I2CEEPROMFSM::WRITING:
      _pl_set_flag(PL_FLAG_BUSY_WRITE);
      break;
    default:
      break;
  }
  return ret;
}


/*******************************************************************************
* Allocation table functions
*******************************************************************************/

const uint I2CEEPROM::_allocation_table_size() {
  return ((DEV_TOTAL_BLOCKS >> 3) + ((DEV_TOTAL_BLOCKS & 0x07) ? 1:0));
}

void I2CEEPROM::_mark_block_allocated(const uint32_t BLKADDR, const bool allocd) {
  if (nullptr != _alloc_table) {
    const uint INDEX_COMPONENT = BLKADDR >> 3;
    const uint SHIFT_COMPONENT = BLKADDR & 0x07;
    const uint8_t BIT_MASK    = 1 << SHIFT_COMPONENT;
    const uint8_t CURRENT_VAL = *(_alloc_table + INDEX_COMPONENT) & ~BIT_MASK;
    *(_alloc_table + INDEX_COMPONENT) = CURRENT_VAL | (allocd ? BIT_MASK:0);
  }
}

bool I2CEEPROM::_is_block_allocated(const uint32_t BLKADDR) {
  const uint INDEX_COMPONENT = BLKADDR >> 3;
  const uint SHIFT_COMPONENT = BLKADDR & 0x07;
  return ((*(_alloc_table + INDEX_COMPONENT) >> SHIFT_COMPONENT) & 0x01);
}




/*******************************************************************************
* ___     _       _                      These members are mandatory overrides
*  |   / / \ o   | \  _     o  _  _      for implementing I/O callbacks. They
* _|_ /  \_/ o   |_/ (/_ \/ | (_ (/_     are also implemented by Adapters.
*******************************************************************************/

/* Transfers always permitted. */
int8_t I2CEEPROM::io_op_callahead(BusOp* _op) {   return 0;   }


/*
* Register I/O calls back to this function for BOTH devices (MAG/IMU). So we
*   split the function up into two halves in private scope in the superclass.
* Bus operations that call back with errors are ignored.
*/
// TODO: The cyclomatic complexity in this function is killing me. I won't
//   remember _anything_ in a week.
int8_t I2CEEPROM::io_op_callback(BusOp* _op) {
  I2CBusOp* op = (I2CBusOp*) _op;
  int8_t ret = BUSOP_CALLBACK_NOMINAL;

  if (op->hasFault()) {
    _set_fsm_position(I2CEEPROMFSM::FAULT);
  }

  if (&_addr_ptr_op == op) {
    if (I2CEEPROMFSM::PRE_INIT == _fsm_pos) {
      // We just reset the chip's address pointer. If we did this from PREINIT,
      //   we should begin reading the data region.
      _data_io_op.set_opcode(BusOpcode::RX);
      _data_io_op.setBuffer(_page_buffer, DEV_BLOCK_SIZE);
      _set_fsm_position((0 != queue_io_job(&_data_io_op)) ? I2CEEPROMFSM::FAULT : I2CEEPROMFSM::ALLOCATING);
    }
  }
  else {
    // The only other kind of transfer we do is data exchange with the page
    //   buffer.
    const uint32_t THIS_BLOCK_ADDR = _address_ptr();
    uint32_t next_addr = THIS_BLOCK_ADDR;
    uint32_t next_len  = DEV_BLOCK_SIZE;

    // Increment out local shadow of the address pointer to save I/O.
    _increment_address_ptr(op->bufferLen());

    switch (op->get_opcode()) {
      case BusOpcode::TX:
        if (I2CEEPROMFSM::FORMATTING == _fsm_pos) {
          // It is safe to assume the operation length will be the full size
          //   of the page buffer.
          _op_len_rem = _op_len_rem - op->bufferLen();
          _mark_block_allocated(THIS_BLOCK_ADDR, false);
          if (0 < _op_len_rem) {
            ret = BUSOP_CALLBACK_RECYCLE;
          }
          else {
            _set_fsm_position(I2CEEPROMFSM::IDLE);
          }
        }
        else if (I2CEEPROMFSM::ALLOCATING == _fsm_pos) {
        }
        else if (nullptr != _current_record) {
          switch (_current_record->buffer_request_from_storage(&next_addr, _page_buffer, &next_len)) {
            case 0:    // Buffer updated. Recycle BusOp.
              op->setBuffer(_page_buffer, next_len);
              ret = BUSOP_CALLBACK_RECYCLE;
              break;
            default:   // We are done writing the record.
              _current_record = nullptr;
              _set_fsm_position(I2CEEPROMFSM::IDLE);
              break;
          }
        }
        break;

      case BusOpcode::RX:
        if (I2CEEPROMFSM::ALLOCATING == _fsm_pos) {
          // We are searching for a free block.
          uint32_t empty_val = 0;
          uint32_t buf_addr  = 0;
          for (uint i = 0; i < DEV_ADDR_SIZE_BYTES; i++) {
            // Copy out the block address.
            empty_val = (empty_val << 8) | 0xFF;
            buf_addr  = (buf_addr << 8)  | *(_page_buffer + 1);
          }
          bool allocd = (buf_addr != empty_val);

          if (!allocd) {
            // We just found a free block.
            // Take the payload size out of the remaining bytes to allocate.
            const uint BYTES_AVAILABLE = DEV_BLOCK_SIZE - DEV_ADDR_SIZE_BYTES;
            if (nullptr != _current_record) {
              _current_record->append_block_to_list(THIS_BLOCK_ADDR);
              _mark_block_allocated(THIS_BLOCK_ADDR, true);
            }
            if (_op_len_rem > BYTES_AVAILABLE) {
              // Not all bytes allocated. Keep looking.
              _op_len_rem = _op_len_rem - BYTES_AVAILABLE;
              _set_address_ptr(THIS_BLOCK_ADDR + DEV_BLOCK_SIZE);
              ret = BUSOP_CALLBACK_RECYCLE;
            }
            else {
              // Enough free bytes were found to satisfy a pending alloc request.
              _set_fsm_position(I2CEEPROMFSM::IDLE);
              if (nullptr != _current_record) {
                _current_record->alloc_complete(true);
              }
            }
          }
          else {
            _mark_block_allocated(THIS_BLOCK_ADDR, true);
            // Block is in-use. Advance to next block if possible.
            if ((THIS_BLOCK_ADDR + DEV_BLOCK_SIZE) < DEV_SIZE_BYTES) {
              _set_address_ptr(THIS_BLOCK_ADDR + DEV_BLOCK_SIZE);
              ret = BUSOP_CALLBACK_RECYCLE;
            }
            else {
              // This is the last block in the device.
              if (nullptr != _current_record) {
                _current_record->alloc_complete(false);
              }
              _set_fsm_position(I2CEEPROMFSM::IDLE);
            }
          }
        }
        else if (nullptr != _current_record) {
          switch (_current_record->buffer_offer_from_storage(&next_addr, _page_buffer, &next_len)) {
            case 0:    // Buffer updated. Recycle BusOp.
              op->setBuffer(_page_buffer, next_len);
              ret = BUSOP_CALLBACK_RECYCLE;
              break;
            default:   // We are done reading the record.
              _current_record = nullptr;
              _set_fsm_position(I2CEEPROMFSM::IDLE);
              break;
          }
        }
        break;

      default:
        break;
    }

    if ((BUSOP_CALLBACK_RECYCLE == ret) && ((THIS_BLOCK_ADDR + op->bufferLen()) != next_addr)) {
      // If the next block is discontinuous from this one, set the address pointer.
      if (0 != _set_address_ptr(next_addr)) {
        // TODO: If we fail to set the address pointer, fault. Don't recycle the BusOp.
        ret = BUSOP_CALLBACK_NOMINAL;
      }
    }
  }
  return ret;
}
