/*
File:   MCP356x_Util.cpp
Author: J. Ian Lindsay
*/

#include "MCP356x.h"

static const char* const MCP356X_CHAN_NAMES[16] = {
  "SE_0", "SE_1", "SE_2", "SE_3", "SE_4", "SE_5", "SE_6", "SE_7",
  "DIFF_A", "DIFF_B", "DIFF_C", "DIFF_D", "TEMP", "AVDD", "VCM", "OFFSET"
};

static const char* const MCP356X_REG_NAMES[16] = {
  "ADCDATA", "CONFIG0", "CONFIG1", "CONFIG2", "CONFIG3", "IRQ", "MUX", "SCAN",
  "TIMER", "OFFSETCAL", "GAINCAL", "RESERVED0", "RESERVED1", "LOCK", "RESERVED2", "CRCCFG"
};


/**
* Static function to convert enum to string.
*/
const char* const MCP356x::stateStr(const MCP356xState e) {  return _FSM_STATES.enumStr(e);  }

void MCP356x::printRegs(StringBuilder* output) {
  for (uint8_t i = 0; i <= (uint8_t) MCP356xRegister::CRCCFG; i++) {
    const MCP356xRegister TMP_REG     = (MCP356xRegister) i;
    const uint32_t        TMP_REG_VAL = _get_shadow_value(TMP_REG);
    output->concatf("[%2d] %10s = 0x%08x\n", i, MCP356X_REG_NAMES[i], TMP_REG_VAL);
  }
}

void MCP356x::printPins(StringBuilder* output) {
  output->concatf("IRQ:   %u  (State: %c)\n", _IRQ_PIN, (readPin(_IRQ_PIN) ? '1':'0'));
  output->concatf("CS:    %u\n", _CS_PIN);
  output->concatf("MCLK:  %u\n", _MCLK_PIN);
}

void MCP356x::printTimings(StringBuilder* output) {
  output->concatf("\tDiscard window:     %s\n", (_discard_window.expired() ? "expired" : "pending"));
  output->concatf("\tMCLK                = %.4f MHz\n", _mclk_freq / 1000000.0);
  output->concatf("\tDMCLK               = %.4f MHz\n", _dmclk_freq / 1000000.0);
  output->concatf("\tData rate           = %.4f KHz\n", _drclk_freq / 1000.0);
  output->concatf("\tReal sample rate    = %u\n", getSampleRate());
  output->concatf("\tADC settling time   = %u\n", getSettlingTime());
  output->concatf("\tTotal settling time = %u\n", _circuit_settle_ms);
  output->concatf("\tLast read (micros)  = %u\n", micros_last_read);
  StopWatch::printDebugHeader(output);
  _profiler_irq_timing.printDebug("IRQ cycle", output);
  _profiler_result_read.printDebug("Result read", output);
}


void MCP356x::printDebug(StringBuilder* output) {
  StringBuilder prod_str("MCP356");
  if (adcFound()) {
    prod_str.concatf("%d", _channel_count() >> 1);
    if (hasInternalVref()) prod_str.concat('R');
  }
  else prod_str.concat("x (not found)");

  StringBuilder::styleHeader2(output, (const char*) prod_str.string());

  output->concatf("\tI/O in-flight:    %c\n",   io_in_flight() ? 'y':'n');
  output->concatf("\t  Dispatched:     %u\n",   _io_dispatched);
  output->concatf("\t  Called back:    %u\n",   _io_called_back);
  output->concatf("\tISR Fired (noted / srvcd):    %c (%u / %u)\n", (_isr_fired ? 'y':'n'), _irqs_noted, _irqs_serviced);
  printFSM(output);
  output->concatf("\tPins conf'd:    %c\n", (_flags.value(MCP356X_FLAG_PINS_CONFIGURED) ? 'y' : 'n'));

  if (adcFound()) {
    output->concatf("\tChannels:       %u\n", _channel_count());
    output->concatf("\tClock running:  %c\n", (_flags.value(MCP356X_FLAG_MCLK_RUNNING) ? 'y' : 'n'));
    output->concatf("\tConfigured:     %c\n", (adcConfigured() ? 'y' : 'n'));
    if (!adcConfigured()) {
      output->concatf("\t  ...as desired:  %c\n", (_config_is_desired() ? 'y' : 'n'));
      output->concatf("\t  ...in hardware: %c\n", (_config_is_written() ? 'y' : 'n'));
    }
    output->concatf("\tCalibrated:     %c\n", (adcCalibrated() ? 'y' : 'n'));
    if (!adcCalibrated()) {
      output->concatf("\t  SAMPLED_OFFSET: %c\n", (_flags.value(MCP356X_FLAG_SAMPLED_OFFSET) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_VCM:    %c\n", (_flags.value(MCP356X_FLAG_SAMPLED_VCM) ? 'y' : 'n'));
      output->concatf("\t  SAMPLED_AVDD:   %c\n", (_flags.value(MCP356X_FLAG_SAMPLED_AVDD) ? 'y' : 'n'));
    }
    output->concatf("\tCRC Error:      %c\n", (_flags.value(MCP356X_FLAG_CRC_ERROR) ? 'y' : 'n'));
    output->concatf("\tRead count:     %u\n", _profiler_result_read.executions());
    output->concatf("\tGain:           x%.2f\n", _gain_value());
    uint8_t _osr_idx = (uint8_t) getOversamplingRatio();
    output->concatf("\tOversampling:   x%u\n", OSR1_VALUES[_osr_idx] * OSR3_VALUES[_osr_idx]);
    output->concatf("\tVref source:    %sternal\n", (usingInternalVref() ? "In": "Ex"));
    output->concatf("\tVref declared:  %c\n", (_vref_declared() ? 'y' : 'n'));
    output->concatf("\tVref range:     %.3f / %.3f\n", _vref_minus, _vref_plus);
    output->concatf("\tClock SRC:      %sternal\n", (_flags.value(MCP356X_FLAG_USE_INTRNL_CLK) ? "In" : "Ex"));
    if (_scan_covers_channel(MCP356xChannel::TEMP)) {
      output->concatf("\tTemperature:    %.2fC\n", getTemperature());
      output->concatf("\tThermo fitting: %s\n", (_flags.value(MCP356X_FLAG_3RD_ORDER_TEMP) ? "3rd-order" : "Linear"));
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
    MCP356X_CHAN_NAMES[((uint8_t) chan) & 0x0F],
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
    else if (0 == StringBuilder::strcasecmp(cmd, "poll")) {
      text_return->concatf("MCP356x poll() returns %d\n", poll());
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "irq")) {
      if (_busop_irq_read.isIdle()) {
        if (0 == _BUS->queue_io_job(&_busop_irq_read, _bus_priority)) {
          _io_dispatched++;
          text_return->concat("IRQ read dispatched.\n");
        }
      }
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "irqsim")) {
      text_return->concat("Simulated IRQ triggered.\n");
      isr_fxn();
    }
    else if (0 == StringBuilder::strcasecmp(cmd, "fsm")) {
      char* sub_cmd = args->position_trimmed(1);
      if (0 == StringBuilder::strcasecmp(sub_cmd, "map")) {
        printFSM(text_return);
        text_return->concat("\nStates: ");
        StringBuilder tmp;
        _FSM_STATES.exportKeys(&tmp);
        tmp.implode(", ");
        text_return->concatHandoff(&tmp);
      }
      else {
        // Shunt into the FSM object's console handler.
        args->drop_position(0);
        ret = fsm_console_handler(text_return, args);
      }
    }
  }
  else {
    printChannelValues(text_return);
  }

  return ret;
}
