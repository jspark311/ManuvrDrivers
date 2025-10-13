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
* VSIGroup
*******************************************************************************/
VSIGroup::VSIGroup(const ViamSonus* hw) : VSGroup(hw, VSIG_FIXED_SER_SIZE) {}

VSIGroup::VSIGroup(const ViamSonus* hw, const uint8_t* buf, const unsigned int len) : VSIGroup(hw) {
  if ((VSCG_FIXED_SER_SIZE + VSIG_FIXED_SER_SIZE) <= len) {
    _unserialize_top(buf, len);
  }
}

VSIGroup::~VSIGroup() {}


int8_t VSIGroup::_add_channel(uint8_t chan, int8_t pos) {
  int8_t ret = -2;
  if ((0 <= pos) && (pos < 12)) {
    uint8_t idx = pos >> 1;
    uint8_t mask = 0x0F << ((pos & 1) << 2);
    _bind_order[idx] = (_bind_order[idx] & ~mask) | (chan << ((pos & 1) << 2));
    ret = 0;
  }
  return ret;
}

/*
* Returns the first open position, or -1 if there are none.
*/
int8_t VSIGroup::_next_position() {
  for (uint8_t i = 0; i < 6; i++) {
    if (0x0F == (_bind_order[i] & 0x0F)) {          return (2 * i);         }
    if (0x0F == ((_bind_order[i] >> 4) & 0x0F)) {   return (1 + (2 * i));   }
  }
  return -1;
}


int8_t VSIGroup::_channel_at_position(int8_t pos) {
  int8_t ret = -2;
  if (pos < 12) {
    uint8_t temp = (_bind_order[pos >> 1] >> ((pos & 1) << 2)) & 0x0F;
    if (temp < 12) {
      // Any number outside the input channel max index results in an error.
      ret = temp;
    }
  }
  return ret;
}


/**
* Swap the channel assignments at the given positions. We do NOT swap their
*   volumes in hardware, so we must also swap their indexed references to not
*   give the impression of a dirty class state if they happen to differ.
* Error-checks parameters, since public fxn.
*/
int8_t VSIGroup::swapChannelPositions(int8_t pos0, int8_t pos1) {
  int8_t ret = -2;
  int8_t chan0 = _channel_at_position(pos0);
  int8_t chan1 = _channel_at_position(pos1);
  if ((chan0 >= 0) && (chan0 < 12)) {
    ret = -1;
    if ((chan1 >= 0) && (chan1 < 12)) {
      if ((pos0 >> 1) == (pos1 >> 1)) {
        _bind_order_q[pos0 >> 1] = (chan1 << ((pos0 & 1) << 2)) | (chan0 << ((pos1 & 1) << 2));
      }
      else {
        uint8_t mask0 = 0x0F << ((pos0 & 1) << 2);
        uint8_t mask1 = 0x0F << ((pos1 & 1) << 2);
        uint8_t nbo0  = _bind_order[pos0 >> 1] & ~mask0;
        uint8_t nbo1  = _bind_order[pos1 >> 1] & ~mask1;
        nbo0 |= chan1 << ((pos0 & 1) << 2);
        nbo1 |= chan0 << ((pos1 & 1) << 2);
        _bind_order_q[pos0 >> 1] = nbo0;
        _bind_order_q[pos1 >> 1] = nbo1;
      }
      uint8_t tmp_vol = _chan_vol_q[pos0];
      _chan_vol_q[pos0] = _chan_vol_q[pos1];
      _chan_vol_q[pos1] = tmp_vol;
      ret = 0;
    }
  }
  return ret;
}


/**
* Returns the volume at the given position in the group.
* TODO: Presently construes the first channel's volume as being the global
*   group volume. This is not correct.
*/
uint8_t VSIGroup::getVolume() {
  uint8_t ret = 0;
  for (uint8_t i = 0; i < _count; i++) {
    int8_t chan = _channel_at_position(i);
    if ((chan < 0) && (chan >= 12)) {
      return -2;
    }
    ret = strict_max(ret, (uint8_t) ((ViamSonus*)_VS)->getVolume(i));
  }
  return ret;
}


/**
* Returns the volume at the given position in the group.
* Error-checks parameters, since public fxn.
*/
int8_t VSIGroup::setVolume(uint8_t vol) {
  int8_t ret = -1;
  for (uint8_t i = 0; i < _count; i++) {
    int8_t chan = _channel_at_position(i);
    if ((chan < 0) && (chan >= 12)) {
      return -2;
    }
    _chan_vol_q[i] = vol;
    ret = 0;
  }
  return ret;
}


/**
* Returns the volume at the given position in the group.
* Error-checks parameters, since public fxn.
*/
int8_t VSIGroup::volumeAtPosition(int8_t pos) {
  int8_t ret = -1;
  int8_t chan = _channel_at_position(pos);
  if ((chan >= 0) && (chan < 12)) {
    ret = (int8_t) ((ViamSonus*)_VS)->getVolume(chan);
  }
  return ret;
}


/**
* Returns true if we have changes that need to be applied to hardware.
* For an input channel, this means either the channel assignments, their order,
*   or their volumes.
*/
bool VSIGroup::_dirty() {
  for (unsigned int i = 0; i < sizeof(_bind_order); i++) {
    if (_bind_order[i] != _bind_order_q[i]) {
      return true;
    }
  }
  for (unsigned int i = 0; i < _count; i++) {
    if (volumeAtPosition(i) != _chan_vol_q[i]) {
      return true;
    }
  }
  return false;
}


void VSIGroup::printDebug(StringBuilder* output) {
  VSGroup::printDebug(output);
  output->concat("    Channel volumes: { ");
  for (uint8_t i = 0; i < _count; i++) {
    output->concatf("%d %s",
      volumeAtPosition(i),
      ((i == (_count-1)) ? "}\n" : ", ")
    );
  }
}


/**
* Applies any stacked changes to the hardware.
* Since this call will also update any volumes, and since the pots don't
*   have the same defered action capability that the switch does, we order
*   things at this function with awareness as to the order of events that
*   happen in the hardware.
*/
int8_t VSIGroup::_apply_to_hardware(bool defer) {
  int8_t ret = -1;
  return ret;
}


uint8_t VSIGroup::_serialize(uint8_t* buf, unsigned int len) {
  uint8_t offset = 0;
  *(buf + offset++) = 'I';  // Marks this as an input channel.
  for (uint8_t i = 0; i < 6; i++) {
    *(buf + offset++) = _bind_order[i];
  }
  return offset;
}


uint8_t VSIGroup::_unserialize(const uint8_t* buf, const unsigned int len) {
  int8_t ret = 0;
  if (len >= VSIG_FIXED_SER_SIZE) {
    for (uint8_t i = 0; i < 6; i++) {
      _bind_order[i] = *(buf + i);
    }
    ret = VSIG_FIXED_SER_SIZE;
  }
  return ret;
}



/*******************************************************************************
* Group relationship functions
*******************************************************************************/

int8_t  VSIGroup::attachGroup(VSOGroup* og) {
  int8_t ret = -1;
  if (!_bound_groups.contains(og)) {
    _bound_groups.insert(og);
    ret = og->attachGroup(this);
  }
  return ret;
}


int8_t  VSIGroup::detachGroup(VSOGroup* og) {
  int8_t ret = -1;
  if (_bound_groups.contains(og)) {
    _bound_groups.remove(og);
    ret = og->detachGroup(this);
  }
  return ret;
}
