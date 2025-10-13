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
* VSGroup
*******************************************************************************/
VSGroup::VSGroup(const ViamSonus* hw, const uint8_t bs) : _VS(hw), _BASE_SER_SIZE(bs) {}


VSGroup::~VSGroup() {
  if (nullptr != _name) {
    free(_name);
    _name = nullptr;
  }
}


/*
* Returns -3 if this group already contains the channel.
*         -2 if the channel is out-of-range.
*         -1 on some other error.
*         Channel position on success.
*/
int8_t VSGroup::addChannel(uint8_t chan) {
  int8_t ret = _add_channel(chan, _next_position());
  if (0 <= ret) {
    _count++;
  }
  return ret;
}


int8_t VSGroup::reverseChannelOrder() {
  int8_t ret = 0;
  uint8_t i = 0;
  while ((0 == ret) & (i < (_count >> 1))) {
    ret = swapChannelPositions(i, (_count-1) - i);
    i++;
  }
  if (0 == ret) {
    //ret = _apply_to_hardware(false);
  }
  return ret;
}


void VSGroup::printDebug(StringBuilder* output) {
  if (nullptr != _name) {
    output->concatf("    Assigned name:   %s\n", _name);
  }
  output->concatf("    Dirty:           %c\n", _dirty() ? 'y' : 'n');
  if (0 < _count) {
    output->concat("    Bound channels:  { ");
    for (uint8_t i = 0; i < _count; i++) {
      output->concatf("%d%s",
        _channel_at_position(i),
        ((i == (_count-1)) ? " }\n" : ", ")
      );
    }
  }
}


uint32_t VSGroup::serialize(uint8_t* buf, unsigned int len) {
  const uint32_t FULLSIZE = serialized_len();
  if (FULLSIZE <= len) {
    memset(buf, 0, FULLSIZE);
    uint8_t offset = _serialize(buf, len);
    *(buf + offset++) = (uint8_t) 0xFF & (_flags >> 8);
    *(buf + offset++) = (uint8_t) 0xFF & _flags;

    for (uint32_t i = 0; i < (FULLSIZE - offset); i++) {
      *(buf + offset++) = *(_name + i);
    }
    return FULLSIZE;
  }
  return 0;
}


/*
* Returns the full length required to store this channel group.
*/
uint32_t VSGroup::serialized_len() {
  // +1 for group type byte.
  // +2 for flags.
  // +1 for null-term of empty string.
  uint32_t ret   = _BASE_SER_SIZE + VSCG_FIXED_SER_SIZE;
  uint8_t  s_len = 0;
  if (nullptr != _name) {
    while ((VSCG_MAX_NAME_LENGTH > s_len) && (0 != *(_name + s_len))) {
      s_len++;
    }
  }
  return ret + s_len;
}


void VSGroup::setName(const char* n) {
  if (nullptr != _name) {
    free(_name);
    _name = nullptr;
  }
  int len = strlen(n) + 1;
  _name = (char*) malloc(len);
  if (nullptr != _name) {
    *(_name + len) = 0;
    memcpy(_name, n, len);
  }
}


/*
* No buffer bounds checking is done here. It is assumed it will be done in the
*  constructor.
*/
uint8_t VSGroup::_unserialize_top(const uint8_t* buf, const unsigned int len) {
  uint8_t offset = _unserialize(buf+1, len-1);
  _flags = (*(buf + offset + 1) << 8) + *(buf + offset + 2);
  offset += 3;  // Account for type code and flags.
  uint8_t s_len = 0;
  while ((VSCG_MAX_NAME_LENGTH > s_len) && (0 != *(buf + offset + s_len))) {
    s_len++;
  }
  _name = (char*) malloc(s_len + 1);
  if (nullptr != _name) {
    memcpy(_name, (buf + offset), s_len);
    *(_name + s_len) = 0;
    return offset + s_len + 1;
  }
  return 0;
}
