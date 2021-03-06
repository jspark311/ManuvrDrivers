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
  switch (e) {
    case MCP356xState::UNINIT:        return "UNINIT";
    case MCP356xState::PREINIT:       return "PREINIT";
    case MCP356xState::RESETTING:     return "RESETTING";
    case MCP356xState::DISCOVERY:     return "DISCOVERY";
    case MCP356xState::REGINIT:       return "REGINIT";
    case MCP356xState::CLK_MEASURE:   return "CLK_MEASURE";
    case MCP356xState::CALIBRATION:   return "CALIBRATION";
    case MCP356xState::USR_CONF:      return "USR_CONF";
    case MCP356xState::IDLE:          return "IDLE";
    case MCP356xState::READING:       return "READING";
    case MCP356xState::FAULT:         return "FAULT";
    default:   break;
  }
  return "INVALID";
}


void MCP356x::printRegs(StringBuilder* output) {
  output->concatf("reg_shadows[0] (ADCDATA)     = 0x%08x\n", _get_shadow_value(MCP356xRegister::ADCDATA));
  output->concatf("reg_shadows[1] (CONFIG0)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG0));
  output->concatf("reg_shadows[2] (CONFIG1)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG1));
  output->concatf("reg_shadows[3] (CONFIG2)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG2));
  output->concatf("reg_shadows[4] (CONFIG3)     = 0x%02x\n", _get_shadow_value(MCP356xRegister::CONFIG3));
  output->concatf("reg_shadows[5] (IRQ)         = 0x%02x\n", _get_shadow_value(MCP356xRegister::IRQ));
  output->concatf("reg_shadows[6] (MUX)         = 0x%02x\n", _get_shadow_value(MCP356xRegister::MUX));
  output->concatf("reg_shadows[7] (SCAN)        = 0x%06x\n", _get_shadow_value(MCP356xRegister::SCAN));
  output->concatf("reg_shadows[8] (TIMER)       = 0x%06x\n", _get_shadow_value(MCP356xRegister::TIMER));
  output->concatf("reg_shadows[9] (OFFSETCAL)   = 0x%06x\n", _get_shadow_value(MCP356xRegister::OFFSETCAL));
  output->concatf("reg_shadows[10] (GAINCAL)    = 0x%06x\n", _get_shadow_value(MCP356xRegister::GAINCAL));
  output->concatf("reg_shadows[11] (RESERVED0)  = 0x%06x\n", _get_shadow_value(MCP356xRegister::RESERVED0));
  output->concatf("reg_shadows[12] (RESERVED1)  = 0x%02x\n", _get_shadow_value(MCP356xRegister::RESERVED1));
  output->concatf("reg_shadows[13] (LOCK)       = 0x%02x\n", _get_shadow_value(MCP356xRegister::LOCK));
  output->concatf("reg_shadows[14] (RESERVED2)  = 0x%04x\n", _get_shadow_value(MCP356xRegister::RESERVED2));
  output->concatf("reg_shadows[15] (CRCCFG)     = 0x%04x\n", _get_shadow_value(MCP356xRegister::CRCCFG));
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


void MCP356x::printData(StringBuilder* output) {
  output->concat("    ---< MCP356x >-----------------------------------------------\n");
  if (stateStable()) {
    output->concatf("\tState stable:   %s\n", stateStr(_current_state));
  }
  else {
    output->concatf("\tCurrent State:\t%s\n", stateStr(_current_state));
    output->concatf("\tPrior State:  \t%s\n", stateStr(_prior_state));
    output->concatf("\tDesired State:\t%s\n", stateStr(_desired_state));
  }
  output->concatf("\tFound:          %c\n", (adcFound() ? 'y' : 'n'));
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
  for (uint8_t i = 0; i < 16; i++) {
    MCP356xChannel chan = (MCP356xChannel) i;
    if (_scan_covers_channel(chan)) {
      switch (chan) {
        case MCP356xChannel::TEMP:
          output->concatf("Die temperature     = %.2fC\n", getTemperature());
          break;
        default:
          printChannel(MCP356xChannel::VCM, output);
          break;
      }
    }
  }
}


/**
* Allow the application to retreive the log.
*
* @param l is a reference to the buffer which should receive the log.
*/
void MCP356x::fetchLog(StringBuilder* l) {
  if (_local_log.length() > 0) {
    if (nullptr != l) {
      _local_log.string();
      l->concatHandoff(&_local_log);
    }
  }
}
