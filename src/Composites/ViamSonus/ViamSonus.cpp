/*
File:   ViamSonus.cpp
Author: J. Ian Lindsay
Date:   2019.11.18


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



const char* const ViamSonus::errorToStr(ViamSonusError err) {
  switch (err) {
    case ViamSonusError::INPUT_DISPLACED:   return "INPUT_DISPLACED";
    case ViamSonusError::DEVICE_DISABLED:   return "DEVICE_DISABLED";
    case ViamSonusError::PEGGED_MAX:        return "PEGGED_MAX";
    case ViamSonusError::PEGGED_MIN:        return "PEGGED_MIN";
    case ViamSonusError::NO_ERROR:          return "NO_ERROR";
    case ViamSonusError::ABSENT:            return "ABSENT";
    case ViamSonusError::BUS:               return "BUS";
    case ViamSonusError::ALREADY_AT_MAX:    return "ALREADY_AT_MAX";
    case ViamSonusError::ALREADY_AT_MIN:    return "ALREADY_AT_MIN";
    case ViamSonusError::INVALID_POT:       return "INVALID_POT";
    case ViamSonusError::UNROUTE_FAILED:    return "UNROUTE_FAILED";
    case ViamSonusError::BAD_COLUMN:        return "BAD_COLUMN";
    case ViamSonusError::BAD_ROW:           return "BAD_ROW";
    case ViamSonusError::SWITCH_COLLISION:  return "SWITCH_COLLISION";
    case ViamSonusError::GEN_SWITCH_FAULT:  return "GEN_SWITCH_FAULT";
    default:                                return "UNKNOWN";
  }
}


/*
* Constructor. Here is all of the setup work. Takes the i2c addresses of the hardware as arguments.
*/
ViamSonus::ViamSonus(const uint8_t reset_pin) :
  cp_switch(
    ADG2128Opts(
      0x70,       // The device address on the i2c bus
      reset_pin,  // ADG2128 reset pin
      true, true  // Allow arbitrary routing. The hardware allows it.
    )
  ),
  pot0(0x28), pot1(0x29), pot2(0x2A), pot3(0x2B), pot4(0x2C), pot5(0x2D)
{
  for (uint8_t i = 0; i < 12; i++) {   // Setup our input channels.
    inputs[i].name      = nullptr;
    inputs[i].i_chan    = i;
    inputs[i].o_chans   = 0;
    inputs[i].flags     = 0;
  }
  for (uint8_t i = 0; i < 8; i++) {    // Setup our output channels.
    outputs[i].name      = nullptr;
    outputs[i].i_chans   = 0;
    outputs[i].o_chan    = i;
    outputs[i].flags     = 0;
  }
}

/*
* Constructor. Here is all of the setup work. Takes the i2c addresses of the hardware as arguments.
*/
ViamSonus::ViamSonus(const uint8_t* buf, const unsigned int len) :
  cp_switch(
    *(buf+6),
    len
  ),
  pot0(0x28), pot1(0x29), pot2(0x2A), pot3(0x2B), pot4(0x2C), pot5(0x2D)
{
  for (uint8_t i = 0; i < 12; i++) {   // Setup our input channels.
    inputs[i].name      = nullptr;
    inputs[i].i_chan    = i;
    inputs[i].o_chans   = 0;
    inputs[i].flags     = 0;
  }
  for (uint8_t i = 0; i < 8; i++) {    // Setup our output channels.
    outputs[i].name      = nullptr;
    outputs[i].i_chans   = 0;
    outputs[i].o_chan    = i;
    outputs[i].flags     = 0;
  }
  unserialize(buf, len);
}



/*
* Destructor
*/
ViamSonus::~ViamSonus() {
  while (_pending_ops.hasNext()) {
    delete _pending_ops.get();
  }
  // Destroy the objects that represent our hardware. Downstream operations will
  //   put the hardware into an inert state, so that needn't be done here.
  for (uint8_t i = 0; i < 12; i++) {
    if (inputs[i].name) {
      free(inputs[i].name);
    }
  }
  for (uint8_t i = 0; i < 8; i++) {
    if (outputs[i].name) {
      free(outputs[i].name);
    }
  }
  _clear_o_groups();
  _clear_i_groups();
}


bool ViamSonus::allDevsFound() {
  return (pot0.devFound() && pot1.devFound() && pot2.devFound() && pot3.devFound() && pot4.devFound() && pot5.devFound() && cp_switch.devFound());
}



/*
* Do all the bus-related init.
*/
ViamSonusError ViamSonus::init(I2CAdapter* bus) {
  ViamSonusError ret = ViamSonusError::ABSENT;
  bool all_success = true;
  all_success &= (DIGITALPOT_ERROR::NO_ERROR == pot0.init(bus));
  all_success &= (DIGITALPOT_ERROR::NO_ERROR == pot1.init(bus));
  all_success &= (DIGITALPOT_ERROR::NO_ERROR == pot2.init(bus));
  all_success &= (DIGITALPOT_ERROR::NO_ERROR == pot3.init(bus));
  all_success &= (DIGITALPOT_ERROR::NO_ERROR == pot4.init(bus));
  all_success &= (DIGITALPOT_ERROR::NO_ERROR == pot5.init(bus));
  all_success &= (ADG2128_ERROR::NONE == cp_switch.init(bus));
   // If we are this far, it means we've successfully refreshed all the device classes
   //   to reflect the state of the hardware. Now to parse that data into structs that
   //   mean something to us at this level...
   //for (uint8_t i = 0; i < 12; i++) {  // Routes...
   //  uint8_t temp_byte = cp_switch.getCols(inputs[i].i_chan);
   //  for (uint8_t j = 0; j < 8; j++) {
   //    if (0x01 & temp_byte) {
   //      CPOutputChannel* temp_output = getOutputByCol(j);
   //      temp_output->i_chans |= 1 << i;
   //    }
   //    temp_byte = temp_byte >> 1;
   //  }
   //}

  if (all_success) {
    ret = ViamSonusError::NO_ERROR;
  }
  return ret;
}


ViamSonusError ViamSonus::reset() {
  ViamSonusError ret = ViamSonusError::GEN_SWITCH_FAULT;
  ADG2128_ERROR res = cp_switch.reset();
  if (ADG2128_ERROR::NONE == res) {
    // Clear out our local records of what is connected where.
    for (uint8_t i = 0; i < 8; i++) {
      inputs[i].o_chans  = 0;
      outputs[i].i_chans = 0;
    }
    for (uint8_t i = 8; i < 12; i++) {
      inputs[i].o_chans  = 0;
    }
    ret = ViamSonusError::NO_ERROR;
  }
  return ret;
}


ViamSonusError ViamSonus::poll() {
  ViamSonusError ret = ViamSonusError::NO_ERROR;
  if (0 < _pending_ops.size()) {
    uint32_t n_mils = (uint32_t) _pending_ops.getPriority(0);
    bool run_loop = (n_mils <= millis());
    while (run_loop) {
      VSPendingOperation* vspo = _pending_ops.dequeue();
      switch (vspo->op) {
        case VSOpcode::CHAN_VOLUME_SET:
          break;
        case VSOpcode::CHAN_VOLUME_INC:
          break;
        case VSOpcode::CHAN_VOLUME_DEC:
          break;
        case VSOpcode::CHAN_ROUTE:
          break;
        case VSOpcode::CHAN_UNROUTE:
          break;
        case VSOpcode::GRP_VOLUME_SET:
          break;
        case VSOpcode::GRP_VOLUME_INC:
          break;
        case VSOpcode::GRP_VOLUME_DEC:
          break;
        case VSOpcode::GRP_ROUTE:
          break;
        case VSOpcode::GRP_UNROUTE:
          break;
        case VSOpcode::ADC_READ:
          break;
        case VSOpcode::UNDEFINED:
        default:
          break;
      }
      if (0 < vspo->recycle_count) {
        vspo->recycle_count--;
        vspo->at_ms += vspo->recycle_period;
        _pending_ops.insert(vspo, vspo->at_ms);
      }
      else {
        delete vspo;
      }

      if (0 < _pending_ops.size()) {
        n_mils = (uint32_t) _pending_ops.getPriority(0);
        run_loop = (n_mils <= millis());
      }
      else {
        run_loop = false;
      }
    }
  }
  return ret;
}


ViamSonusError ViamSonus::preserveOnDestroy(bool preserve) {
  ViamSonusError ret = ViamSonusError::GEN_SWITCH_FAULT;
  cp_switch.preserveOnDestroy(preserve);
  for (uint8_t i = 0; i < 6; i++) {
    if (DIGITALPOT_ERROR::NO_ERROR != _getPotRef(i)->storeWipers()) {
      return ret;
    }
    ret = ViamSonusError::NO_ERROR;
    _vs_set_flag(VIAMSONUS_FLAG_PRESERVE_STATE);
  }
  return ret;
}


ViamSonusError ViamSonus::refresh() {
  ViamSonusError ret = ViamSonusError::GEN_SWITCH_FAULT;
  // ADG2128_ERROR res = cp_switch.refresh();
  // if (ADG2128_ERROR::NONE == res) {
  //   DIGITALPOT_ERROR dp_ret = DIGITALPOT_ERROR::NO_ERROR;
  //   if ((DIGITALPOT_ERROR::NO_ERROR == dp_ret) & _vs_flag(VIAMSONUS_FLAG_FOUND_POT_0)) {  dp_ret = pot0.refresh();  }
  //   if ((DIGITALPOT_ERROR::NO_ERROR == dp_ret) & _vs_flag(VIAMSONUS_FLAG_FOUND_POT_1)) {  dp_ret = pot1.refresh();  }
  //   if ((DIGITALPOT_ERROR::NO_ERROR == dp_ret) & _vs_flag(VIAMSONUS_FLAG_FOUND_POT_2)) {  dp_ret = pot2.refresh();  }
  //   if ((DIGITALPOT_ERROR::NO_ERROR == dp_ret) & _vs_flag(VIAMSONUS_FLAG_FOUND_POT_3)) {  dp_ret = pot3.refresh();  }
  //   if ((DIGITALPOT_ERROR::NO_ERROR == dp_ret) & _vs_flag(VIAMSONUS_FLAG_FOUND_POT_4)) {  dp_ret = pot4.refresh();  }
  //   if ((DIGITALPOT_ERROR::NO_ERROR == dp_ret) & _vs_flag(VIAMSONUS_FLAG_FOUND_POT_5)) {  dp_ret = pot5.refresh();  }
  //   ret = (DIGITALPOT_ERROR::NO_ERROR != dp_ret) ? ViamSonusError::BUS : ViamSonusError::NO_ERROR;
  // }
  return ret;
}


/*
*/
ViamSonusError ViamSonus::nameInput(uint8_t row, const char* name) {
  if (row > 11) return ViamSonusError::BAD_ROW;
  ViamSonusError ret = ViamSonusError::GEN_SWITCH_FAULT;
  int len = strlen(name);
  if (inputs[row].name) {
    free(inputs[row].name);
  }
  inputs[row].name = (char *) malloc(len + 1);
  if (nullptr != inputs[row].name) {
    *(inputs[row].name + len) = 0;
    for (int i = 0; i < len; i++) *(inputs[row].name + i) = *(name + i);
    ret = ViamSonusError::NO_ERROR;
  }
  return ret;
}


/*
*/
ViamSonusError ViamSonus::nameOutput(uint8_t col, const char* name) {
  if (col > 7) return ViamSonusError::BAD_COLUMN;
  ViamSonusError ret = ViamSonusError::GEN_SWITCH_FAULT;
  int len = strlen(name);
  if (outputs[col].name) {
    free(outputs[col].name);
  }
  outputs[col].name = (char *) malloc(len + 1);
  if (nullptr != outputs[col].name) {
    *(outputs[col].name + len) = 0;
    for (int i = 0; i < len; i++) *(outputs[col].name + i) = *(name + i);
    ret = ViamSonusError::NO_ERROR;
  }
  return ret;
}



ViamSonusError ViamSonus::unroute(uint8_t col, uint8_t row) {
  if (col > 7)  return ViamSonusError::BAD_COLUMN;
  if (row > 11) return ViamSonusError::BAD_ROW;
  ViamSonusError ret = ViamSonusError::UNROUTE_FAILED;
  if (ADG2128_ERROR::NONE == cp_switch.unsetRoute(col, row)) {
    outputs[col].i_chans &= ~(1 << row);
    inputs[row].o_chans  &= ~(1 << col);
    ret = ViamSonusError::NO_ERROR;
  }
  return ret;
}


ViamSonusError ViamSonus::unroute(uint8_t col) {
  if (col > 7)  return ViamSonusError::BAD_COLUMN;
  ViamSonusError ret = ViamSonusError::NO_ERROR;
  for (int i = 0; i < 12; i++) {
    if (ADG2128_ERROR::NONE == cp_switch.unsetRoute(col, i, (11 == i))) {
      outputs[col].i_chans &= ~(1 << i);
      inputs[i].o_chans    &= ~(1 << col);
    }
    else {
      ret = ViamSonusError::UNROUTE_FAILED;
    }
  }
  return ret;
}


/*
* Remember: This is the class responsible for ensuring that we don't cross-wire a circuit.
*   The crosspoint switch class is generalized, and knows nothing about the circuit it is
*   embedded within. It will follow whatever instructions it is given. So if we tell it
*   to route two inputs to the same output, it will oblige and possibly fry hastily-built
*   hardware. So under that condition, we unroute prior to routing and return a code to
*   indicate that we've done so.
*/
ViamSonusError ViamSonus::route(uint8_t col, uint8_t row) {
  if (col > 7)  return ViamSonusError::BAD_COLUMN;
  if (row > 11) return ViamSonusError::BAD_ROW;

  ViamSonusError ret = ViamSonusError::NO_ERROR;
  if (0 != outputs[col].i_chans) {
    // We already have channels bound.
    if (0 != (outputs[col].i_chans & ~(1 << row))) {
      // There are routes attached beyond the one requested.
      // TODO: Implications for mix after this line.
      if (ViamSonusError::NO_ERROR == unroute(col)) {
        ret = ViamSonusError::INPUT_DISPLACED;
      }
      else {
        ret = ViamSonusError::UNROUTE_FAILED;
      }
    }
    else {
      // The requested route already exists.
    }
  }

  if (((int8_t)ret) >= 0) {
    ADG2128_ERROR result = cp_switch.setRoute(outputs[col].o_chan, row);
    switch (result) {
      case ADG2128_ERROR::NONE:
        outputs[col].i_chans |= (1 << row);
        inputs[row].o_chans  |= (1 << col);
        ret = ViamSonusError::NO_ERROR;
        break;
      case ADG2128_ERROR::ABSENT:
        ret = ViamSonusError::ABSENT;
        break;
      case ADG2128_ERROR::BUS:
        ret = ViamSonusError::BUS;
        break;
      case ADG2128_ERROR::BAD_COLUMN:
        ret = ViamSonusError::BAD_COLUMN;
        break;
      case ADG2128_ERROR::BAD_ROW:
        ret = ViamSonusError::BAD_ROW;
        break;
      default:
        ret = ViamSonusError::GEN_SWITCH_FAULT;
        break;
    }
  }
  return ret;
}


ViamSonusError ViamSonus::setVolume(uint8_t row, uint8_t vol) {
  if (row > 11)  return ViamSonusError::BAD_ROW;
  ViamSonusError ret = (ViamSonusError) _getPotRef(row)->setValue(row & 0x01, vol);
  return ret;
}


int16_t ViamSonus::getVolume(uint8_t row) {
  if (row > 11)  return 0;
  return (int16_t) _getPotRef(row)->getValue(row & 0x01);
}


void ViamSonus::printDebug(StringBuilder* output) {
  output->concat("ViamSonus 2.0\n");
  #if defined(ADG2128_CONSOLE)
    cp_switch.printDebug(output);
  #endif  // ADG2128_CONSOLE
  #if defined(DS1881_CONSOLE)
    pot0.printDebug(output);
    pot1.printDebug(output);
    pot2.printDebug(output);
    pot3.printDebug(output);
    pot4.printDebug(output);
    pot5.printDebug(output);
  #endif  // DS1881_CONSOLE
}


void ViamSonus::printGroups(StringBuilder* output) {
  for (uint8_t i = 0; i < 8; i++) {
    if (nullptr != ogroups[i]) {
      ogroups[i]->printDebug(output);
    }
  }
  for (uint8_t i = 0; i < 12; i++) {
    if (nullptr != igroups[i]) {
      igroups[i]->printDebug(output);
    }
  }
}


void ViamSonus::printChannels(StringBuilder* output) {
  for (uint8_t i = 0; i < 8; i++) {
    dumpOutputChannel(i, output);
  }
}


void ViamSonus::dumpOutputChannel(uint8_t chan, StringBuilder* output) {
  if (chan > 7)  {
    output->concat("dumpOutputChannel() was passed an out-of-bounds id.\n");
    return;
  }

  if (outputs[chan].name) {
    output->concatf("    Output: %s\n", outputs[chan].name);
  }
  else {
    output->concatf("    Output channel %u\n", chan);
  }

  if (outputs[chan].i_chans == 0) {
    output->concat("\tPresently unbound\n");
  }
  else {
    output->concat("\tBound to inputs ");
    for (uint8_t i = 0; i < 12; i++) {
      if ((1 << i) & outputs[chan].i_chans) {
        //dumpInputChannel(&inputs[i], output);
        if (nullptr != inputs[i].name) {
          output->concatf("\"%s\", ", inputs[i].name);
        }
        else {
          output->concatf("%u, ", i);
        }
      }
    }
    output->concat("\n");
  }
}


void ViamSonus::dumpInputChannel(CPInputChannel *chan, StringBuilder* output) {
  if (chan == nullptr) {
    output->concat("dumpInputChannel() was passed a NULL InputChannel.\n");
    return;
  }
  if (chan->name) {
    output->concatf("    Input: %s", chan->name);
  }
  else {
    output->concatf("    Input channel %u", chan->i_chan);
  }
  DS1881* pot = _getPotRef(chan->i_chan);
  output->concatf("\tVolume:  %u\n", pot->getValue(chan->i_chan & 1));
}


void ViamSonus::dumpInputChannel(uint8_t chan, StringBuilder* output) {
  if (chan > 11) {
    output->concatf("dumpInputChannel() was passed an out-of-bounds input channel (%u).\n", chan);
    return;
  }
  dumpInputChannel(&inputs[chan], output);
}



#if defined(CONFIG_VIAM_SONUS_CONSOLE)
void ViamSonus::console_help(StringBuilder* text_return) {
  text_return->concat("ViamSonus router command help:\n");
  text_return->concat("\t info          Dump overview\n");
  text_return->concat("\t channels      Dump channels\n");
  text_return->concat("\t group         Dump groups\n");
  text_return->concat("\t init          Attempt initialization\n");
  text_return->concat("\t refresh       Refresh all hardware shadows\n");
  text_return->concat("\t reset         Disconnect all inputs\n");
  text_return->concat("\t switch        Cross-point switch subconsole\n");
  text_return->concat("\t pot <0-5>     Input volume control subconsole\n");

  //text_return->concat("\t input <0-5>   Input volume control subconsole\n");
  //text_return->concat("\t pot <0-5>     Input volume control subconsole\n");
}


int8_t ViamSonus::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = -1;
  if (0 < args->count()) {
    ret = 0;
    char* cmd = args->position_trimmed(0);
    if (0 == StringBuilder::strcasecmp(cmd, "info")) {
      printDebug(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("init() returns %s.\n", ViamSonus::errorToStr(init(nullptr)));
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
      text_return->concatf("refresh() returns %s.\n", ViamSonus::errorToStr(refresh()));
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "reset")) {
      text_return->concatf("reset() returns %s.\n", ViamSonus::errorToStr(reset()));
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "channels")) {
      printChannels(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "groups")) {
      printGroups(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "switch")) {
      // Discard the first argument and shunt into the
      // switch's console handler.
      args->drop_position(0);
      cp_switch.console_handler(text_return, args);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "pot")) {
      if (1 < args->count()) {
        const uint8_t pot_idx = (uint8_t) args->position_as_int(1);
        // Discard the first two arguments and shunt into the
        // potentiometer's console handler.
        args->drop_position(0);
        args->drop_position(0);
        switch (pot_idx) {
          case 0:  pot0.console_handler(text_return, args);  break;
          case 1:  pot1.console_handler(text_return, args);  break;
          case 2:  pot2.console_handler(text_return, args);  break;
          case 3:  pot3.console_handler(text_return, args);  break;
          case 4:  pot4.console_handler(text_return, args);  break;
          case 5:  pot5.console_handler(text_return, args);  break;
          default:
            text_return->concatf("Invalid pot index: %u\n", pot_idx);
            break;
        }
      }
      else {
        text_return->concatf("Usage: %s <0-5> <potentiometer arguments>\n", cmd);
      }
    }
    else {
      ret = -1;
    }
  }

  if (0 > ret) {
    console_help(text_return);
  }
  return ret;
}
#endif




/*
* No bounds-checking.
*/
DS1881* ViamSonus::_getPotRef(uint8_t row) {
  DS1881* refs[] = {&pot0, &pot1, &pot2, &pot3, &pot4, &pot5};
  return refs[row >> 1];
}


/*
* Stores everything about the class in the provided buffer in this format...
*   Offset                    | Data
*   --------------------------|------------------------
*   0                         | Serializer version
*   1                         | Flags bits 24-31 (MSB)
*   2                         | Flags bits 16-23
*   3                         | Flags bits 8-15
*   4                         | Flags bits 0-7 (LSB)
*   5                         | ADG2128 data
*   5+ADG2128_SERIALIZE_SIZE  | DS1881E data x6
*   TODO: Input channel data
*   TODO: Output channel data
*
* Returns the number of bytes written to the buffer.
*/
uint32_t ViamSonus::serialize(uint8_t* buf, unsigned int len) {
  uint32_t offset = 0;
  //if (len >= VIAMSONUS_SERIALIZE_SIZE) {
  //  uint32_t f = _flags & VIAMSONUS_FLAG_SERIAL_MASK;
  //  *(buf + offset++) = VIAMSONUS_SERIALIZE_VERSION;
  //  *(buf + offset++) = (uint8_t) 0xFF & (f >> 24);
  //  *(buf + offset++) = (uint8_t) 0xFF & (f >> 16);
  //  *(buf + offset++) = (uint8_t) 0xFF & (f >> 8);
  //  *(buf + offset++) = (uint8_t) 0xFF & f;
  //  if (ADG2128_SERIALIZE_SIZE == cp_switch.serialize((buf + offset), len-offset)) {
  //    offset += ADG2128_SERIALIZE_SIZE;
  //    for (uint8_t i = 0; i < 6; i++) {
  //      if (DS1881_SERIALIZE_SIZE == _getPotRef(i <<1 )->serialize((buf + offset), len-offset)) {
  //        offset += DS1881_SERIALIZE_SIZE;
  //      }
  //    }
  //  }

  //  if ((VIAMSONUS_SERIALIZE_SIZE == offset) && (0 < (len - offset))) {
  //    // If we packed the basics with success, and still have space left in
  //    //   the buffer, we start looking for definitions of channel groups.
  //    for (uint8_t i = 0; i < 8; i++) {
  //      if (nullptr != ogroups[i]) {
  //        if (ogroups[i]->serialized_len() > (len - offset)) {
  //          return 0;
  //        }
  //        offset += ogroups[i]->serialize((buf + offset), (len - offset));
  //      }
  //    }
  //    for (uint8_t i = 0; i < 12; i++) {
  //      if (nullptr != igroups[i]) {
  //        if (igroups[i]->serialized_len() > (len - offset)) {
  //          return 0;
  //        }
  //        offset += igroups[i]->serialize((buf + offset), (len - offset));
  //      }
  //    }
  //  }
  //}
  return offset;
}



int8_t ViamSonus::unserialize(const uint8_t* buf, const unsigned int len) {
  uint32_t offset = 0;
  uint32_t expected_sz = 0xFFFFFFFF;
  int8_t ret = -1;
  if (len >= VIAMSONUS_SERIALIZE_SIZE) {  // The minimum length.
    uint32_t f = (*(buf + 1) << 24) | (*(buf + 2) << 16) | (*(buf + 3) << 8) | *(buf + 4);
    switch (*(buf + offset++)) {
      case VIAMSONUS_SERIALIZE_VERSION:
        expected_sz = VIAMSONUS_SERIALIZE_SIZE;
        offset += 4;  // We'll have already constructed with _ADDR.
        // TODO: Serialization won't work until porting away from Arduiino is done.
        //if (0 != cp_switch.unserialize((buf + offset), len-offset)) {
        //  return -3;
        //}
        offset += ADG2128_SERIALIZE_SIZE;  // We'll have already constructed with _ADDR.
        _flags = (_flags & ~VIAMSONUS_FLAG_SERIAL_MASK) | (f & VIAMSONUS_FLAG_SERIAL_MASK);
        for (uint8_t i = 0; i < 6; i++) {
          // TODO: Serialization won't work until porting away from Arduiino is done.
          //if (0 != _getPotRef(i << 1)->unserialize((buf + offset), len-offset)) {
          //  return -4;
          //}
          offset += DS1881_SERIALIZE_SIZE;
        }
        ret = (expected_sz == offset) ? 0 : -5;
        break;
      default:  // Unhandled serializer version.
        return -2;
    }
  }
  if ((0 == ret) && (1 < (len - offset))) {
    // If we took apart the basics with success, and still have space left in
    //   the buffer, we start looking for definitions of channel groups.
    uint8_t nu_ogrp_idx = 0;
    uint8_t nu_igrp_idx = 0;
    _clear_o_groups();
    _clear_i_groups();
    while ((0 == ret) && (1 < (len - offset))) {
      switch (*(buf + offset)) {
        case 'I':
          igroups[nu_igrp_idx] = new VSIGroup((const ViamSonus*) this, (buf + offset), len-offset);
          if (nullptr != igroups[nu_igrp_idx]) {
            offset += igroups[nu_igrp_idx]->serialized_len();
            nu_igrp_idx++;
          }
          else {
            ret = -6;
          }
          break;
        case 'O':
          ogroups[nu_ogrp_idx] = new VSOGroup((const ViamSonus*) this, (buf + offset), len-offset);
          if (nullptr != ogroups[nu_ogrp_idx]) {
            offset += ogroups[nu_ogrp_idx]->serialized_len();
            nu_ogrp_idx++;
          }
          else {
            ret = -7;
          }
          break;
        default:
          ret = -5;
          break;
      }
    }
  }
  return ret;
}


/*******************************************************************************
* VSGroup handling
*******************************************************************************/

VSOGroup* ViamSonus::createOutputGroup(const char* n) {
  VSOGroup* ret = nullptr;
  uint8_t nu_grp_idx = 0;
  while ((nu_grp_idx < 8) && (nullptr != ogroups[nu_grp_idx])) {
    nu_grp_idx++;
  }
  if (nu_grp_idx < 8) {
    ret = new VSOGroup((const ViamSonus*) this);
    if (nullptr != ret) {
      ret->setName(n);
      ogroups[nu_grp_idx] = ret;
    }
  }
  return ret;
}


VSIGroup* ViamSonus::createInputGroup(const char* n) {
  VSIGroup* ret = nullptr;
  uint8_t nu_grp_idx = 0;
  while ((nu_grp_idx < 12) && (nullptr != igroups[nu_grp_idx])) {
    nu_grp_idx++;
  }
  if (nu_grp_idx < 12) {
    ret = new VSIGroup((const ViamSonus*) this);
    if (nullptr != ret) {
      ret->setName(n);
      igroups[nu_grp_idx] = ret;
    }
  }
  return ret;
}


void ViamSonus::_clear_o_groups() {
  for (uint8_t i = 0; i < 8; i++) {
    if (nullptr != ogroups[i]) {
      delete ogroups[i];
      ogroups[i] = nullptr;
    }
  }
}


void ViamSonus::_clear_i_groups() {
  for (uint8_t i = 0; i < 12; i++) {
    if (nullptr != igroups[i]) {
      delete igroups[i];
      igroups[i] = nullptr;
    }
  }
}
