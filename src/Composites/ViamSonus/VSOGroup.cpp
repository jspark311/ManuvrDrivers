/*
File:   VSGroup.cpp
Author: J. Ian Lindsay
Date:   2019.12.07


This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include "ViamSonus.h"
#include <string.h>


/*******************************************************************************
* VSOGroup functions
*******************************************************************************/
VSOGroup::VSOGroup(const ViamSonus* hw) : VSGroup(hw, VSOG_FIXED_SER_SIZE) {}

VSOGroup::VSOGroup(const ViamSonus* hw, const uint8_t* buf, const unsigned int len) : VSOGroup(hw) {
  if ((VSCG_FIXED_SER_SIZE + VSOG_FIXED_SER_SIZE) <= len) {
    _unserialize_top(buf, len);
  }
}

VSOGroup::~VSOGroup() {}


int8_t VSOGroup::_add_channel(uint8_t chan, int8_t pos) {
  int8_t ret = -2;
  if ((0 <= pos) && (pos < 8)) {
    uint32_t mask = 0x0F << (pos << 2);
    _bind_order = (_bind_order & ~mask) | (chan << (pos << 2));
    ret = 0;
  }
  return ret;
}

/*
* Returns the first open position, or -1 if there are none.
*/
int8_t VSOGroup::_next_position() {
  for (uint8_t i = 0; i < 8; i++) {
    if (0x0F == ((_bind_order >> (4*i)) & 0x0F)) {   return i;     }
  }
  return -1;
}

int8_t VSOGroup::_channel_at_position(int8_t pos) {
  int8_t ret = -1;
  if (pos < 8) {
    uint8_t temp = (_bind_order >> (pos << 2)) & 0x0F;
    if (temp < 8) {
      // Any number outside the output channel max index results in an error.
      ret = temp;
    }
  }
  return ret;
}


/**
* Swap the channel assignments at the given positions.
* Error-checks parameters, since public fxn.
*/
int8_t VSOGroup::swapChannelPositions(int8_t pos0, int8_t pos1) {
  int8_t ret = -2;
  int8_t chan0 = _channel_at_position(pos0);
  int8_t chan1 = _channel_at_position(pos1);
  if ((chan0 >= 0) && (chan0 < 8)) {
    ret = -1;
    if ((chan1 >= 0) && (chan1 < 8)) {
      uint32_t mask0 = 0x0F << (pos0 << 2);
      uint32_t mask1 = 0x0F << (pos1 << 2);
      uint32_t new_bind_order = _bind_order & ~(mask0 | mask1);
      new_bind_order |= chan0 << (pos1 << 2);
      new_bind_order |= chan1 << (pos0 << 2);
      _bind_order_q = new_bind_order;
      ret = 0;
    }
  }
  return ret;
}


/**
* Returns true if we have changes that need to be applied to hardware.
* For an output channel, all that means is channel assignments or their order.
*/
bool VSOGroup::_dirty() {
  return (_bind_order_q != _bind_order);
}


/**
* Applies any stacked changes to the hardware.
* No special treatment of defer is needed here, since we are only dealing with
*   the switch, and not the potentiometers.
*/
int8_t VSOGroup::_apply_to_hardware(bool defer) {
  int8_t ret = -1;
  for (uint8_t i = 0; i < _count; i++) {
    // TODO: Traverse bound_groups list and store the channel lists locally.
    //   then make sure the swap gets pushed into hardware.
  }
  return ret;
}


uint8_t VSOGroup::_serialize(uint8_t* buf, unsigned int len) {
  uint8_t offset = 0;
  *(buf + offset++) = 'O';  // Marks this as an output channel.
  *(buf + offset++) = (uint8_t) 0xFF & (_bind_order >> 24);
  *(buf + offset++) = (uint8_t) 0xFF & (_bind_order >> 16);
  *(buf + offset++) = (uint8_t) 0xFF & (_bind_order >> 8);
  *(buf + offset++) = (uint8_t) 0xFF & _bind_order;
  return offset;
}


uint8_t VSOGroup::_unserialize(const uint8_t* buf, const unsigned int len) {
  int8_t ret = 0;
  if (len >= VSOG_FIXED_SER_SIZE) {
    _bind_order = (*(buf + 0) << 24) | (*(buf + 1) << 16) | (*(buf + 2) << 8) | *(buf + 3);
    ret = VSOG_FIXED_SER_SIZE;
  }
  return ret;
}


/*******************************************************************************
* Group relationship functions
*******************************************************************************/

int8_t  VSOGroup::attachGroup(VSIGroup* ig) {
  int8_t ret = -1;
  return ret;
}


int8_t  VSOGroup::detachGroup(VSIGroup* ig) {
  int8_t ret = -1;
  if (_bound_groups.remove(ig)) {
    // We had the group. Reflect the changes in hardware by unsetting the routes.
    // TODO: We can't assume the channel cardinality is the same.
    for (uint8_t i = 0; i < _count; i++) {
      uint8_t ochan = _channel_at_position(i);
      uint8_t ichan = ig->channelAtPosition(i);
      ret = (int8_t) ((ViamSonus*)_VS)->unroute(ochan, ichan);
      if (0 != ret) {
        return ret;
      }
    }
  }
  return ret;
}
