/*
File:   ViamSonus.h
Author: J. Ian Lindsay
Date:   2019.11.16

Copyright 2019 Manuvr, Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.


Digipots start at i2c address 0x28, and there are 6 of them.
Crosspoint switch is at address 0x70.
*/

#ifndef __VIAMSONUS_DRIVER_H__
#define __VIAMSONUS_DRIVER_H__

#include <StringBuilder.h>
#include <PriorityQueue.h>

// These hardware drivers from this repo are required.
#include "../../DS1881/DS1881.h"
#include "../../ADG2128/ADG2128.h"

class ViamSonus;
class VSOGroup;
class VSIGroup;

#define VIAMSONUS_SERIALIZE_VERSION  1
#define VIAMSONUS_SERIALIZE_SIZE     (5 + ADG2128_SERIALIZE_SIZE + (DS1881_SERIALIZE_SIZE*6))

#define CONFIG_VIAM_SONUS_CONSOLE 1   // TODO: Make this a build feature flag.

/* Values that set limits on channel storage requirements. */
#define VSIG_FIXED_SER_SIZE    6
#define VSOG_FIXED_SER_SIZE    4
#define VSCG_FIXED_SER_SIZE    4
#define VSCG_MAX_NAME_LENGTH  32

/* ViamSonus class flags */
#define VIAMSONUS_FLAG_PRESERVE_STATE 0x00000001  // Preserve hardware states.

#define VIAMSONUS_FLAG_RESET_MASK     0xFE000000  // Bits to preserve through reset.
#define VIAMSONUS_FLAG_SERIAL_MASK    0x000000FF  // Only these bits are serialized.


/* InputChannel flags */
#define VSCG_FLAG_ALLOW_MIX           0x0001  // This channel is safe to mix.
#define VSCG_FLAG_MONITOR             0x0002  // Monitor this channel via ADC.
#define VSCG_FLAG_AUTO_APPLY          0x0004  // This group should apply to hardware on every change.


enum class ViamSonusError : int8_t {
  INPUT_DISPLACED  = 4,   // There was no error, but a channel-routing operation has displaced a previously-routed input.
  DEVICE_DISABLED  = 3,   // A caller tried to set a wiper while the device is disabled. This may work...
  PEGGED_MAX       = 2,   // There was no error, but a call to change a wiper setting pegged the wiper at its highest position.
  PEGGED_MIN       = 1,   // There was no error, but a call to change a wiper setting pegged the wiper at its lowest position.
  NO_ERROR         = 0,   // There was no error.
  ABSENT           = -1,  // A part on the board appears to not be connected to the bus.
  BUS              = -2,  // Something went wrong with the i2c bus.
  ALREADY_AT_MAX   = -3,  // A caller tried to increase the value of the wiper beyond its maximum.
  ALREADY_AT_MIN   = -4,  // A caller tried to decrease the value of the wiper below its minimum.
  INVALID_POT      = -5,  // There aren't that many potentiometers.
  UNROUTE_FAILED   = -6,  // We tried to unroute a signal from an output and failed.
  BAD_COLUMN       = -7,  // Column was out-of-bounds.
  BAD_ROW          = -8,  // Row was out-of-bounds.
  SWITCH_COLLISION = -9,  // Command would violate a row or col cardinality constraint.
  GEN_SWITCH_FAULT = -10  // A general switch failure.
};

/*
* These are operations that happen on timers to facilitate certain use-cases,
*   ease peak resource usage, or implement UI features.
*/
enum class VSOpcode : uint8_t {
  UNDEFINED        = 0,   // Default uninitialized value.
  CHAN_VOLUME_SET  = 1,   // Set the channel volume to the value at the operand.
  CHAN_VOLUME_INC  = 2,   // Bump the channel volume by the amount at the operand.
  CHAN_VOLUME_DEC  = 3,   // Cut the channel volume by the amount at the operand.
  CHAN_ROUTE       = 4,   // Routes the input channel to the output channel.
  CHAN_UNROUTE     = 5,   // Unroutes the input channel from the output channel.
  GRP_VOLUME_SET   = 6,   // Set the group volume to the value at the operand.
  GRP_VOLUME_INC   = 7,   // Bump the group volume by the amount at the operand.
  GRP_VOLUME_DEC   = 8,   // Cut the group volume by the amount at the operand.
  GRP_ROUTE        = 9,   // Routes the input group to the output group.
  GRP_UNROUTE      = 10,  // Unroutes the input group from the output group.
  ADC_READ         = 11   // Reads an ADC associated with a channel.
};


// This struct defines an input pin on the PCB.
typedef struct cps_input_channel_t {
  char*    name;       // A name for this input. Not required.
  uint8_t  i_chan;     // The row in the crosspoint switch associated with this input.
  uint8_t  o_chans;    // A flag field indicating which columns are presently bound to this row.
  uint16_t flags;      // Flags on this channel.
} CPInputChannel;


// This struct defines an output pin on the PCB.
typedef struct cps_output_channel_t {
  char*    name;       // A name for this output. Not required.
  uint16_t i_chans;    // A flag field indicating which rows are presently bound to this column.
  uint8_t  o_chan;     // The column in the crosspoint switch associated with this output.
  uint8_t  flags;      // Flags on this channel.
} CPOutputChannel;


/* This class is used to schedule class operations. */
class VSPendingOperation {
  public:
    uint32_t    at_ms          = 0;
    uint8_t     recycle_count  = 0;
    uint8_t     recycle_period = 0;
    VSOpcode op = VSOpcode::UNDEFINED;
};


/* This pure virtual class represents an grouping of channels. */
class VSGroup {
  public:
    int8_t addChannel(uint8_t chan);
    int8_t reverseChannelOrder();
    virtual int8_t swapChannelPositions(int8_t pos0, int8_t pos1) =0;
    virtual void printDebug(StringBuilder*);
    //int8_t apply();

    uint32_t serialize(uint8_t* buf, unsigned int len);
    uint32_t serialized_len();

    void setName(const char*);

    inline const char* getName() {   return _name;   };
    inline uint8_t channelCount() {  return _count;  };
    inline int8_t  channelAtPosition(int8_t pos) {
      return _channel_at_position(pos);
    };

    inline bool autoApply() {        return _grp_flag(VSCG_FLAG_AUTO_APPLY);      };



  protected:
    const ViamSonus* _VS;         // Pointer to the responsible hardware.
    const uint8_t _BASE_SER_SIZE; // Constant for the fixed size of serialization.
    uint8_t  _count = 0;          // Channel count in this group.
    uint16_t _flags = 0;          // Flags on this group.
    char*    _name  = nullptr;    // A name for this group. Not required.

    VSGroup(const ViamSonus*, const uint8_t);
    virtual ~VSGroup();

    uint8_t _unserialize_top(const uint8_t* buf, const unsigned int len);

    virtual uint8_t _serialize(uint8_t* buf, unsigned int len) =0;
    virtual uint8_t _unserialize(const uint8_t* buf, const unsigned int len) =0;
    virtual int8_t  _add_channel(uint8_t chan, int8_t pos) =0;
    virtual int8_t  _next_position() =0;
    virtual int8_t  _channel_at_position(int8_t pos) =0;
    virtual int8_t  _apply_to_hardware(bool defer) =0;
    virtual bool    _dirty() =0;

    /* Flag manipulation inlines */
    inline uint8_t _grp_flags() {                return _flags;           };
    inline bool _grp_flag(uint8_t _flag) {       return (_flags & _flag); };
    inline void _grp_clear_flag(uint8_t _flag) { _flags &= ~_flag;        };
    inline void _grp_set_flag(uint8_t _flag) {   _flags |= _flag;         };
    inline void _grp_set_flag(uint8_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};


/* This class represents an input group. */
class VSIGroup : public VSGroup {
  public:
    VSIGroup(const ViamSonus*);
    VSIGroup(const ViamSonus*, const uint8_t* buf, const unsigned int len);
    ~VSIGroup();

    int8_t swapChannelPositions(int8_t pos0, int8_t pos1);
    void printDebug(StringBuilder*);

    /* Input-specific API */
    //bool allowMix();
    uint8_t getVolume();
    int8_t  setVolume(uint8_t);
    int8_t  volumeAtPosition(int8_t pos);
    int8_t  attachGroup(VSOGroup*);
    int8_t  detachGroup(VSOGroup*);


  protected:
    uint8_t _serialize(uint8_t* buf, unsigned int len);
    uint8_t _unserialize(const uint8_t* buf, const unsigned int len);
    int8_t  _add_channel(uint8_t chan, int8_t pos);
    int8_t  _next_position();
    int8_t  _channel_at_position(int8_t pos);
    int8_t  _apply_to_hardware(bool defer);
    bool    _dirty();

  private:
    PriorityQueue<VSOGroup*> _bound_groups;
    // A bit-packed ordered list of channels that compose the group.
    uint8_t  _bind_order[6]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t  _bind_order_q[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t  _chan_vol_q[12]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};



/* This class represents an output group. */
class VSOGroup : public VSGroup {
  public:
    VSOGroup(const ViamSonus*);
    VSOGroup(const ViamSonus*, const uint8_t* buf, const unsigned int len);
    ~VSOGroup();

    int8_t swapChannelPositions(int8_t pos0, int8_t pos1);
    int8_t  attachGroup(VSIGroup*);
    int8_t  detachGroup(VSIGroup*);


  protected:
    uint8_t _serialize(uint8_t* buf, unsigned int len);
    uint8_t _unserialize(const uint8_t* buf, const unsigned int len);
    int8_t  _add_channel(uint8_t chan, int8_t pos);
    int8_t  _next_position();
    int8_t  _channel_at_position(int8_t pos);
    int8_t  _apply_to_hardware(bool defer);
    bool    _dirty();

  private:
    PriorityQueue<VSIGroup*> _bound_groups;
    uint32_t  _bind_order   = 0xFFFFFFFF;   // A bit-packed ordered list of channels that compose the group.
    uint32_t  _bind_order_q = 0xFFFFFFFF;   // A bit-packed ordered list of channels that compose the group.
};



/*
* This class represents a complete Viam Sonus board.
* The 8-pin group are the outputs, and the 12-pin group are inputs. Please see
*   the schematic for more clarity.
*/
class ViamSonus {
  public:
    ViamSonus(const uint8_t reset_pin);
    ViamSonus(const uint8_t* buf, const unsigned int len);
    ~ViamSonus();

    ViamSonusError init(I2CAdapter* b = nullptr);
    ViamSonusError reset();
    ViamSonusError poll();
    ViamSonusError refresh();
    ViamSonusError preserveOnDestroy(bool);

    /* API level-1: Dealing with groups of channels. */
    VSOGroup* createOutputGroup(const char*);
    VSIGroup* createInputGroup(const char*);

    /* API level-0: Dealing with discrete channels directly. */
    ViamSonusError route(uint8_t col, uint8_t row);       // Establish a route to the given output from the given input.
    ViamSonusError unroute(uint8_t col, uint8_t row);     // Disconnect the given output from the given input.
    ViamSonusError unroute(uint8_t col);                  // Disconnect the given output from all inputs.

    ViamSonusError nameInput(uint8_t row, const char*);   // Name the input channel.
    ViamSonusError nameOutput(uint8_t col, const char*);  // Name the output channel.

    ViamSonusError setVolume(uint8_t row, uint8_t vol);   // Set the volume of a given input channel.
    int16_t getVolume(uint8_t row);                       // Get the volume of a given input channel.

    inline CPInputChannel*  getInputByRow(uint8_t row) {   return &inputs[row % 12];     };
    inline CPOutputChannel* getOutputByCol(uint8_t col) {  return &outputs[col & 0x07];  };

    // Save this hardware state into a buffer for later restoration.
    uint32_t serialize(uint8_t* buf, unsigned int len);
    int8_t   unserialize(const uint8_t* buf, const unsigned int len);

    bool allDevsFound();

    void dumpInputChannel(CPInputChannel* chan, StringBuilder*);
    void dumpInputChannel(uint8_t chan, StringBuilder*);
    void dumpOutputChannel(uint8_t chan, StringBuilder*);
    void printDebug(StringBuilder*);    void printGroups(StringBuilder*);
    void printChannels(StringBuilder*);


    #if defined(CONFIG_VIAM_SONUS_CONSOLE)
      /* Built-in per-instance console handler. */
      void console_help(StringBuilder* text_return);
      int8_t console_handler(StringBuilder* text_return, StringBuilder* args);
    #endif

    static const char* const errorToStr(ViamSonusError);


  private:
    CPInputChannel  inputs[12];  // TODO: These are on borrowed time.
    CPOutputChannel outputs[8];  // TODO: These are on borrowed time.
    VSIGroup*       igroups[12];
    VSOGroup*       ogroups[8];
    uint32_t _flags = 0;
    ADG2128  cp_switch;
    DS1881   pot0;
    DS1881   pot1;
    DS1881   pot2;
    DS1881   pot3;
    DS1881   pot4;
    DS1881   pot5;
    PriorityQueue<VSPendingOperation*> _pending_ops;
    uint16_t _input_claims  = 0;
    uint8_t  _output_claims = 0;

    DS1881* _getPotRef(uint8_t row);

    void _clear_o_groups();
    void _clear_i_groups();

    inline bool _preserve_on_destroy() {
      return _vs_flag(VIAMSONUS_FLAG_PRESERVE_STATE);
    };

    /* Flag manipulation inlines */
    inline uint32_t _vs_flags() {                return _flags;           };
    inline bool _vs_flag(uint32_t _flag) {       return (_flags & _flag); };
    inline void _vs_clear_flag(uint32_t _flag) { _flags &= ~_flag;        };
    inline void _vs_set_flag(uint32_t _flag) {   _flags |= _flag;         };
    inline void _vs_set_flag(uint32_t _flag, bool nu) {
      if (nu) _flags |= _flag;
      else    _flags &= ~_flag;
    };
};

#endif   // __VIAMSONUS_DRIVER_H__
