/*
File:   MCP356x_Util.cpp
Author: J. Ian Lindsay
*/

#include "MCP356x.h"

static const char* CHAN_NAMES[16] = {
  "SE_0", "SE_1", "SE_2", "SE_3", "SE_4", "SE_5", "SE_6", "SE_7",
  "DIFF_A", "DIFF_B", "DIFF_C", "DIFF_D", "TEMP", "AVDD", "VCM", "OFFSET"
};


/**
* Static function to convert enum to string.
*/
const char* MCP356x::stateStr(const MCP356xState e) {
  return _FSM_STATES.enumStr(e);
}


void MCP356x::printRegs(StringBuilder* output) {
  output->concatf("[0]  ADCDATA    = 0x%08x\n", _get_shadow_value(MCP356xRegister::ADCDATA));
  output->concatf("[1]  CONFIG0    = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG0));
  output->concatf("[2]  CONFIG1    = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG1));
  output->concatf("[3]  CONFIG2    = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG2));
  output->concatf("[4]  CONFIG3    = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG3));
  output->concatf("[5]  IRQ        = 0x%02x\n", _get_shadow_value(MCP356xRegister::IRQ));
  output->concatf("[6]  MUX        = 0x%02x\n", _get_shadow_value(MCP356xRegister::MUX));
  output->concatf("[7]  SCAN       = 0x%06x\n", _get_shadow_value(MCP356xRegister::SCAN));
  output->concatf("[8]  TIMER      = 0x%06x\n", _get_shadow_value(MCP356xRegister::TIMER));
  output->concatf("[9]  OFFSETCAL  = 0x%06x\n", _get_shadow_value(MCP356xRegister::OFFSETCAL));
  output->concatf("[10] GAINCAL    = 0x%06x\n", _get_shadow_value(MCP356xRegister::GAINCAL));
  output->concatf("[11] RESERVED0  = 0x%06x\n", _get_shadow_value(MCP356xRegister::RESERVED0));
  output->concatf("[12] RESERVED1  = 0x%02x\n", _get_shadow_value(MCP356xRegister::RESERVED1));
  output->concatf("[13] LOCK       = 0x%02x\n", _get_shadow_value(MCP356xRegister::LOCK));
  output->concatf("[14] RESERVED2  = 0x%04x\n", _get_shadow_value(MCP356xRegister::RESERVED2));
  output->concatf("[15] CRCCFG     = 0x%04x\n", _get_shadow_value(MCP356xRegister::CRCCFG));
}


void MCP356x::printPins(StringBuilder* output) {
  output->concatf("IRQ:   %u\n", _IRQ_PIN);
  output->concatf("CS:    %u\n", _CS_PIN);
  output->concatf("MCLK:  %u\n", _MCLK_PIN);
}


void MCP356x::printTimings(StringBuilder* output) {
  output->concatf("\tMeasuring MCLK: %c\n", (_measuring_clock() ? 'y' : 'n'));
  output->concatf("\tMCLK                = %.4f MHz\n", _mclk_freq / 1000000.0);
  output->concatf("\tDMCLK               = %.4f MHz\n", _dmclk_freq / 1000000.0);
  output->concatf("\tData rate           = %.4f KHz\n", _drclk_freq / 1000.0);
  output->concatf("\tReal sample rate    = %u\n", reads_per_second);
  output->concatf("\tADC settling time   = %u\n", getSettlingTime());
  output->concatf("\tTotal settling time = %u\n", _circuit_settle_ms);
  output->concatf("\tLast read (micros)  = %u\n", micros_last_read);
}


void MCP356x::printDebug(StringBuilder* output) {
  StringBuilder prod_str("MCP356");
  if (adcFound()) {
    prod_str.concatf("%d", _channel_count() >> 1);
    if (hasInternalVref()) prod_str.concat('R');
  }
  else prod_str.concat("x (not found)");

  StringBuilder::styleHeader2(output, (const char*) prod_str.string());
  if (stateStable()) {
    output->concatf("\tState stable:   %s\n", stateStr(_current_state));
  }
  else {
    output->concatf("\tCurrent State:\t%s\n", stateStr(_current_state));
    output->concatf("\tPrior State:  \t%s\n", stateStr(_prior_state));
    output->concatf("\tDesired State:\t%s\n", stateStr(_desired_state));
  }
  if (adcFound()) {
    output->concatf("\tChannels:       %u\n", _channel_count());
    output->concatf("\tClock running:  %c\n", (_mcp356x_flag(MCP356X_FLAG_MCLK_RUNNING) ? 'y' : 'n'));
    output->concatf("\tConfigured:     %c\n", (adcConfigured() ? 'y' : 'n'));
    output->concatf("\tCalibrated:     %c\n", (adcCalibrated() ? 'y' : 'n'));
    if (!adcCalibrated()) {
      output->concatf("\t  SAMPLED_OFFSET: %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_OFFSET) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_VCM:    %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_VCM) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_AVDD:   %c\n", (_mcp356x_flag(MCP356X_FLAG_SAMPLED_AVDD) ? 'y' : 'n'));
    }
    output->concatf("\tCRC Error:      %c\n", (_mcp356x_flag(MCP356X_FLAG_CRC_ERROR) ? 'y' : 'n'));
    output->concatf("\tisr_fired:      %c\n", (isr_fired ? 'y' : 'n'));
    output->concatf("\tRead count:     %u\n", read_count);
    output->concatf("\tGain:           x%.2f\n", _gain_value());
    uint8_t _osr_idx = (uint8_t) getOversamplingRatio();
    output->concatf("\tOversampling:   x%u\n", OSR1_VALUES[_osr_idx] * OSR3_VALUES[_osr_idx]);
    output->concatf("\tVref source:    %sternal\n", (usingInternalVref() ? "In": "Ex"));
    output->concatf("\tVref declared:  %c\n", (_vref_declared() ? 'y' : 'n'));
    output->concatf("\tVref range:     %.3f / %.3f\n", _vref_minus, _vref_plus);
    output->concatf("\tClock SRC:      %sternal\n", (_mcp356x_flag(MCP356X_FLAG_USE_INTERNAL_CLK) ? "In" : "Ex"));
    if (_scan_covers_channel(MCP356xChannel::TEMP)) {
      output->concatf("\tTemperature:    %.2fC\n", getTemperature());
      output->concatf("\tThermo fitting: %s\n", (_mcp356x_flag(MCP356X_FLAG_3RD_ORDER_TEMP) ? "3rd-order" : "Linear"));
    }
    if (adcCalibrated()) {
      output->concat("\t");
      printChannel(MCP356xChannel::OFFSET, output);
      output->concat("\t");
      printChannel(MCP356xChannel::VCM, output);
      output->concat("\t");
      printChannel(MCP356xChannel::AVDD, output);
    }
  }
}


/**
* Prints a single channel.
*/
void MCP356x::printChannel(MCP356xChannel chan, StringBuilder* output) {
  output->concatf(
    "%s:\t%.6fv\t%s\n",
    CHAN_NAMES[((uint8_t) chan) & 0x0F],
    valueAsVoltage(chan),
    _channel_over_range(chan) ? "OvR" : " "
  );
}


/**
* Prints the values of all enabled channels.
*/
void MCP356x::printChannelValues(StringBuilder* output) {
  if (adcFound()) {
    for (uint8_t i = 0; i < 16; i++) {
      MCP356xChannel chan = (MCP356xChannel) i;
      if (_scan_covers_channel(chan)) {
        printChannel(chan, output);
      }
    }
  }
  else {
    output->concat("MCP356x not found.\n");
  }
}



/*******************************************************************************
* Console callback
* These are built-in handlers for using this instance via a console.
*******************************************************************************/

/**
* @page console-handlers
* @section mcp356x-tools MCP356x tools
*
* This is the console handler for using the MCP356x ADC. If invoked without
*   arguments, it will print channel values, as are being observed by hardware.
*
* @subsection cmd-actions Actions
*
* Action    | Description | Additional arguments
* --------- | ----------- | --------------------
* `init`    | Manually invoke the driver's `init()` function. | None
* `reset`   | Manually invoke the driver's `reset()` function. | None
* `irq`     | Force an ISR cycle. | None
* `refresh` | Refresh the register shadows from the hardware. | None
*/
int8_t MCP356x::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  if (0 < args->count()) {
    char* cmd = args->position_trimmed(0);

    if (0 == StringBuilder::strcasecmp(cmd, "info")) {
      printDebug(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "timings")) {
      printTimings(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "regs")) {
      printRegs(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "busops")) {
      _busop_irq_read.printDebug(text_return);
      _busop_dat_read.printDebug(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "temperature")) {
      text_return->concatf("MCP356x temperature: %u.\n", (uint8_t) getTemperature());
    }

    else if (0 == StringBuilder::strcasecmp(cmd, "pins")) {
      printPins(text_return);
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "gain")) {
      if (1 < args->count()) {
        ret = setGain((MCP356xGain) args->position_as_int(1));
      }
      text_return->concatf("MCP356x gain is now %u.\n", 1 << ((uint8_t) getGain()));
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "oversampling")) {
      if (1 < args->count()) {
        ret = setOversamplingRatio((MCP356xOversamplingRatio) args->position_as_int(1));
      }
      text_return->concatf("Oversampling ratio is now %u.\n", (uint8_t) getOversamplingRatio());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "refresh")) {
      text_return->concatf("MCP356x refresh() returns %d.\n", refresh());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
      text_return->concatf("MCP356x init() returns %d.\n", init());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "read")) {
      text_return->concatf("MCP356x read() returns %d\n", read());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "irq")) {
      isr_fired = true;
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "state")) {
      bool print_state_map = (2 > args->count());
      if (!print_state_map) {
        MCP356xState state = (MCP356xState) args->position_as_int(1);
        switch (state) {
          case MCP356xState::PREINIT:
          case MCP356xState::RESETTING:
          case MCP356xState::DISCOVERY:
          case MCP356xState::POST_INIT:
          case MCP356xState::CLK_MEASURE:
          case MCP356xState::CALIBRATION:
          case MCP356xState::USR_CONF:
          case MCP356xState::IDLE:
          case MCP356xState::READING:
            text_return->concatf("MCP356x setDesiredState(%s)\n", MCP356x::stateStr(state));
            break;
          case MCP356xState::UNINIT:
          case MCP356xState::FAULT:
            text_return->concatf("MCP356x illegal desired state (%s).\n", MCP356x::stateStr(state));
            setDesiredState(state);
            break;
          default:
            print_state_map = true;
            break;
        }
      }

      if (print_state_map) {
        for (uint8_t i = 0; i < 11; i++) {
          MCP356xState ste = (MCP356xState) i;
          text_return->concatf("%u:\t%s", i, MCP356x::stateStr(ste));
          if (ste == _current_state) {
            text_return->concat("  <--- Current\n");
          }
          else if (ste == _prior_state) {
            text_return->concat("  <--- Prior\n");
          }
          else if (ste == _desired_state) {
            text_return->concat("  <--- Desired\n");
          }
          else {
            text_return->concat("\n");
          }
        }
      }
    }
  }
  else {
    printChannelValues(text_return);
  }

  return ret;
}
