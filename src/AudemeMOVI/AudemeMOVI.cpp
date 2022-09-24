/**
* @file      AudemeMOVI.cpp
* @brief     Header for Manuvr fork of the Audeme MOVI board.
* @details   This driver is a re-write of [MOVIArduinoAPI](https://github.com/audeme/MOVIArduinoAPI).
*            It was hard-forked at Version 1.13 to divorce it from Arduino.
*            Their license is reproduced below, and in the LICENSE file in this
*            directory.
* @author    J. Ian Lindsay
* @date      2022.08.19
*
* Useful commentary from their original code is relicated below, with edits to
*   reflect new realities.
*
* Communication rate between MOVI and Arduino. 9600bps is the default. As of
*   firmware 1.10 changing the bitrate is possible with a config file on the
*   SDcard.
*
* Refactor note: If ever there was a reason to set your compiler to fail any
*   build that would contain unhandled switch() cases, this driver is sufficient.
*   While extending and debugging, ensure that ALL cases are handled for ALL
*   switch()'ing on enums. Ideally, there would be no default cases anywhere.
*                                  ---I. Lindsay Sat 10 Sep 2022 09:40:23 PM MDT
*/

/********************************************************************
 This is a library for the Audeme MOVI Voice Control Shield
 ----> http://www.audeme.com/MOVI/
 This code is inspired and maintained by Audeme but open to change
 and organic development on GITHUB:
 ----> https://github.com/audeme/MOVIArduinoAPI
 Written by Gerald Friedland for Audeme LLC.
 Contact: fractor@audeme.com
 BSD license, all text above must be included in any redistribution.
 ********************************************************************/

#include "AudemeMOVI.h"


/*******************************************************************************
*      _______.___________.    ___   .___________. __    ______     _______.
*     /       |           |   /   \  |           ||  |  /      |   /       |
*    |   (----`---|  |----`  /  ^  \ `---|  |----`|  | |  ,----'  |   (----`
*     \   \       |  |      /  /_\  \    |  |     |  | |  |        \   \
* .----)   |      |  |     /  _____  \   |  |     |  | |  `----.----)   |
* |_______/       |__|    /__/     \__\  |__|     |__|  \______|_______/
*
* Static members and initializers should be located here.
*******************************************************************************/
const char* const MOVI_EVENT_STRING = "MOVIEvent[";


static const char* const movi_state_str(const MOVIState E) {
  switch (E) {
    case MOVIState::INIT:          return "INIT";
    case MOVIState::PENDING_BOOT:  return "PENDING_BOOT";
    case MOVIState::CONF_SYNC:     return "CONF_SYNC";
    case MOVIState::IDLE:          return "IDLE";
    case MOVIState::SPEAKING:      return "SPEAKING";
    case MOVIState::LISTENING:     return "LISTENING";
    case MOVIState::TRAINING:      return "TRAINING";
    case MOVIState::PENDING_PWR:   return "PENDING_PWR";
    case MOVIState::POWERED_OFF:   return "POWERED_OFF";
    case MOVIState::FAULT:         return "FAULT";
    default:                       return "UNKNOWN";
  }
}


static const char* movi_get_event_string(const MOVIEvent C) {
  switch (C) {
    case MOVIEvent::SHIELD_IDLE:        return "SHIELD_IDLE";
    case MOVIEvent::SENTENCE_LISTING:   return "SENTENCE_LISTING";
    case MOVIEvent::SENTENCE_DEF:       return "SENTENCE_DEF";
    case MOVIEvent::LOG_RELAY:          return "LOG_RELAY";
    case MOVIEvent::PONG:               return "PONG";
    case MOVIEvent::INIT_COMPLETE:      return "INIT_COMPLETE";
    case MOVIEvent::VERSION_REPORT:     return "VERSION_REPORT";
    case MOVIEvent::VERSION_REPORT_FW:  return "VERSION_REPORT_FW";
    case MOVIEvent::VERSION_REPORT_HW:  return "VERSION_REPORT_HW";
    case MOVIEvent::READY:              return "READY";
    case MOVIEvent::TRAINING_SKIPPED:   return "TRAINING_SKIPPED";
    case MOVIEvent::CALLSIGN_REPORT:    return "CALLSIGN_REPORT";
    case MOVIEvent::BEGIN_LISTEN:       return "BEGIN_LISTEN";
    case MOVIEvent::END_LISTEN:         return "END_LISTEN";
    case MOVIEvent::BEGIN_SAY:          return "BEGIN_SAY";
    case MOVIEvent::END_SAY:            return "END_SAY";
    case MOVIEvent::CALLSIGN_DETECTED:  return "CALLSIGN_DETECTED";
    case MOVIEvent::RAW_WORDS:          return "RAW_WORDS";
    case MOVIEvent::SENTENCE_RECOG:     return "SENTENCE_RECOG";
    case MOVIEvent::PASSWORD_ATTEMPT:   return "PASSWORD_ATTEMPT";
    case MOVIEvent::PASSWORD_ACCEPT:    return "PASSWORD_ACCEPT";
    case MOVIEvent::SENTENCE_QUEUED:    return "SENTENCE_QUEUED";
    case MOVIEvent::SENTENCES_TRAINED:  return "SENTENCES_TRAINED";
    case MOVIEvent::CALLSIGN_TRAINED:   return "CALLSIGN_TRAINED";
    case MOVIEvent::VOLUME_SETTING:     return "VOLUME_SETTING";
    case MOVIEvent::THRESHOLD_SETTING:  return "THRESHOLD_SETTING";
    case MOVIEvent::BEEP_OFF:           return "BEEP_OFF";
    case MOVIEvent::BEEP_ON:            return "BEEP_ON";
    case MOVIEvent::RESPONSES_OFF:      return "RESPONSES_OFF";
    case MOVIEvent::RESPONSES_ON:       return "RESPONSES_ON";
    case MOVIEvent::SYS_MESSAGES_OFF:   return "SYS_MESSAGES_OFF";
    case MOVIEvent::WELCOME_OFF:        return "WELCOME_OFF";
    case MOVIEvent::WELCOME_ON:         return "WELCOME_ON";
    case MOVIEvent::FEMALE_VOICE:       return "FEMALE_VOICE";
    case MOVIEvent::MALE_VOICE:         return "MALE_VOICE";
    case MOVIEvent::SYNTH_ESPEAK:       return "SYNTH_ESPEAK";
    case MOVIEvent::SYNTH_PICO:         return "SYNTH_PICO";
    case MOVIEvent::MIC_DEBUG_ON:       return "MIC_DEBUG_ON";
    case MOVIEvent::MIC_DEBUG_OFF:      return "MIC_DEBUG_OFF";
    case MOVIEvent::STOPPED:            return "STOPPED";
    case MOVIEvent::RESTARTED:          return "RESTARTED";
    case MOVIEvent::ABORTED:            return "ABORTED";
    case MOVIEvent::SHUTDOWN_NOTICE:    return "SHUTDOWN_NOTICE";
    case MOVIEvent::PASSWORD_REJECT:    return "PASSWORD_REJECT";
    case MOVIEvent::INVALID_PARAM:      return "INVALID_PARAM";
    case MOVIEvent::SYNTH_REJECT:       return "SYNTH_REJECT";
    case MOVIEvent::SILENCE:            return "SILENCE";
    case MOVIEvent::UNKNOWN_SENTENCE:   return "UNKNOWN_SENTENCE";
    case MOVIEvent::NOISE_ALARM:        return "NOISE_ALARM";
    default:                            return "UNSUPPORTED";
  }
}


static const char* movi_get_command_string(const MOVICommandCode C) {
  switch (C) {
    case MOVICommandCode::INIT:            return "INIT";
    case MOVICommandCode::PING:            return "PING";
    case MOVICommandCode::ASK:             return "ASK";
    case MOVICommandCode::SAY:             return "SAY";
    case MOVICommandCode::SETSYNTHESPEAK:  return "SETSYNTHESPEAK";
    case MOVICommandCode::SETSYNTHPICO:    return "SETSYNTHPICO";
    case MOVICommandCode::VOLUME:          return "VOLUME";
    case MOVICommandCode::THRESHOLD:       return "THRESHOLD";
    case MOVICommandCode::PASSWORD:        return "PASSWORD";
    case MOVICommandCode::TRAINSENTENCES:  return "TRAINSENTENCES";
    case MOVICommandCode::NEWSENTENCES:    return "NEWSENTENCES";
    case MOVICommandCode::ADDSENTENCE:     return "ADDSENTENCE";
    case MOVICommandCode::CALLSIGN:        return "CALLSIGN";
    case MOVICommandCode::FACTORY:         return "FACTORY";
    case MOVICommandCode::STOP:            return "STOP";
    case MOVICommandCode::RESTART:         return "RESTART";
    case MOVICommandCode::PAUSE:           return "PAUSE";
    case MOVICommandCode::UNPAUSE:         return "UNPAUSE";
    case MOVICommandCode::FINISH:          return "FINISH";
    case MOVICommandCode::PLAY:            return "PLAY";
    case MOVICommandCode::ABORT:           return "ABORT";
    case MOVICommandCode::RESPONSES:       return "RESPONSES";
    case MOVICommandCode::WELCOMEMESSAGE:  return "WELCOMEMESSAGE";
    case MOVICommandCode::BEEPS:           return "BEEPS";
    case MOVICommandCode::FEMALE:          return "FEMALE";
    case MOVICommandCode::MALE:            return "MALE";
    case MOVICommandCode::HELP:            return "HELP";
    case MOVICommandCode::SHUTDOWN:        return "SHUTDOWN";
    case MOVICommandCode::ABOUT:           return "ABOUT";
    case MOVICommandCode::VERSION:         return "VERSION";
    case MOVICommandCode::HWVERSION:       return "HWVERSION";
    case MOVICommandCode::VOCABULARY:      return "VOCABULARY";
    case MOVICommandCode::TRAIN:           return "TRAIN";
    case MOVICommandCode::MEM:             return "MEM";
    case MOVICommandCode::MICDEBUG:        return "MICDEBUG";
    default:                               return "<UNKNOWN>";
  }
}

/*
* This function is essentially of a list of expected replies from the hardware
*   indexed by the commands that elicit them.
* For commands that don't wait for replies, this function will return
*   MOVIEvent::UNSUPPORTED, and this will be construed as "no expected reply"
*   by class logic.
* NOTE: Some commands have many possible non-error responses (IE, "BEEP").
*   Unless something smarter is required, these should be handled by treating
*   those commands as non-blocking, and passively observing the event (when it
*   arrives). If a case is found where this treatment is insufficient to
*   maintain FSM uniformity, it will need to be re-worked. Such commands have
*   been tagged "1:MANY".
*/
static const MOVIEvent movi_get_reply_by_command(const MOVICommandCode C) {
  switch (C) {
    // TODO: These commands are unhandled.
    case MOVICommandCode::PASSWORD:        return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::NEWSENTENCES:    return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::FACTORY:         return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::PAUSE:           return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::UNPAUSE:         return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::PLAY:            return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::TRAIN:           return MOVIEvent::UNSUPPORTED;
    case MOVICommandCode::MICDEBUG:        return MOVIEvent::UNSUPPORTED;

    // These commands stall the TX queue until the response comes back.
    case MOVICommandCode::ASK:             return MOVIEvent::BEGIN_LISTEN;
    case MOVICommandCode::ABORT:           return MOVIEvent::ABORTED;
    case MOVICommandCode::STOP:            return MOVIEvent::STOPPED;
    case MOVICommandCode::RESTART:         return MOVIEvent::RESTARTED;
    case MOVICommandCode::TRAINSENTENCES:  return MOVIEvent::SENTENCES_TRAINED;
    case MOVICommandCode::ADDSENTENCE:     return MOVIEvent::SENTENCE_QUEUED;
    case MOVICommandCode::SAY:             return MOVIEvent::BEGIN_SAY;
    case MOVICommandCode::SETSYNTHESPEAK:  return MOVIEvent::SYNTH_ESPEAK;
    case MOVICommandCode::SETSYNTHPICO:    return MOVIEvent::SYNTH_PICO;
    case MOVICommandCode::VOLUME:          return MOVIEvent::VOLUME_SETTING;
    case MOVICommandCode::THRESHOLD:       return MOVIEvent::THRESHOLD_SETTING;
    case MOVICommandCode::CALLSIGN:        return MOVIEvent::CALLSIGN_TRAINED;
    case MOVICommandCode::FEMALE:          return MOVIEvent::FEMALE_VOICE;
    case MOVICommandCode::MALE:            return MOVIEvent::MALE_VOICE;
    case MOVICommandCode::SHUTDOWN:        return MOVIEvent::SHUTDOWN_NOTICE;
    case MOVICommandCode::ABOUT:           return MOVIEvent::VERSION_REPORT;
    case MOVICommandCode::VERSION:         return MOVIEvent::VERSION_REPORT_FW;
    case MOVICommandCode::HWVERSION:       return MOVIEvent::VERSION_REPORT_HW;
    case MOVICommandCode::INIT:            return MOVIEvent::INIT_COMPLETE;
    case MOVICommandCode::PING:            return MOVIEvent::PONG;

    // These commands do not stall the TX queue.
    case MOVICommandCode::FINISH:          // 1:MANY
    case MOVICommandCode::RESPONSES:       // 1:MANY
    case MOVICommandCode::WELCOMEMESSAGE:  // 1:MANY
    case MOVICommandCode::BEEPS:           // 1:MANY
    case MOVICommandCode::HELP:            // Generates lots of LOG_RELAY.
    case MOVICommandCode::VOCABULARY:      // Generates lots of SENTENCE_DEF.
    case MOVICommandCode::MEM:             // Generates response lines without tags.
    default:                               return MOVIEvent::UNSUPPORTED;
  }
}



/*******************************************************************************
*   ___ _              ___      _ _              _      _
*  / __| |__ _ ______ | _ ) ___(_) |___ _ _ _ __| |__ _| |_ ___
* | (__| / _` (_-<_-< | _ \/ _ \ | / -_) '_| '_ \ / _` |  _/ -_)
*  \___|_\__,_/__/__/ |___/\___/_|_\___|_| | .__/_\__,_|\__\___|
*                                          |_|
* Constructors/destructors, class initialization functions and so-forth...
*******************************************************************************/

/**
* Constructor
*/
MOVI::MOVI(UARTAdapter* u, bool debugonoff) :
  _uart(u), _flags(debugonoff ? MOVI_FLAG_DEBUGGING : 0),
  _hw_ver(0.0), _fw_ver(0.0), _fsm_lockout_ms(0),
  _fsm_pos(MOVIState::UNKNOWN),
  _fsm_pos_prior(MOVIState::UNKNOWN),
  _fsm_pos_target(MOVIState::IDLE),
  _volume(0), _threshold(0),
  _callback(nullptr)
{
  _uart->readCallback(this);  // Associate ourselves with the UART.
}


/**
* Destructor
*/
MOVI::~MOVI() {
  _flags.clear(MOVI_FLAG_READY_MASK);
  _uart->readCallback(nullptr);  // Disconnect from the UART.
  _purge_waiting_io();
}


/*
* Implementation of BufferAccepter.
*/
int8_t MOVI::provideBuffer(StringBuilder* buf) {
  if (!buf->isEmpty()) {
    if (buf->contains('\n')) {
      // If the buffer contains a newline, we take that to be a message ending.
      buf->split("\n");
      //while (buf->count()) {
        char* clean_reply = buf->position_trimmed(0);
        _response.concat(clean_reply);
        //c3p_log(LOG_LEV_INFO, __PRETTY_FUNCTION__, "UART ---> MOVI: %s\n", clean_reply);
        buf->drop_position(0);  // Drop the data we just used. UART retains the rest.
      //}
      return 1;  // Accept with claim to leave the remaining buffer in the UART.
    }
  }
  return -1;
}


/**
*
* @return 0 on success. -1 on failure. 1 on new event.
*/
int8_t MOVI::poll() {
  int8_t ret = 0;

  // Iterate through the inbound lines and handle them each in order.
  while (_response.count() > 0) {
    char* line = StringBuilder::strcasestr(_response.position_trimmed(0), MOVI_EVENT_STRING);
    if (line) {
      MOVIEvent e_code = _handle_event_line(line);
      if (MOVIEvent::UNSUPPORTED != e_code) {
        // DEBUG: Anything unsupported will be dumped.
      }

      // If this event matches that expected by the currently-blocking command,
      //   clean it up.
      MOVICmdRespPair* waiting_cmd = _waiting_cmds.get(0);
      if ((nullptr != waiting_cmd) && (waiting_cmd->was_sent)) {
        // A waiting command may be resolved if...
        bool resolve_cmd = (waiting_cmd->EXPECT == e_code);  // ...the expected event arrives
        // TODO: resolve_cmd     |= _event_is_error_response(e_code);
        if (resolve_cmd) {
          _waiting_cmds.remove(waiting_cmd);
          delete waiting_cmd;
        }
      }
    }
    else {
      c3p_log(LOG_LEV_NOTICE, __PRETTY_FUNCTION__, "%s", _response.position_trimmed(0));
    }
    _response.drop_position(0);
  }

  // After the I/O has been processed, poll the driver state machine
  //   and return the result.
  //ret = _poll_fsm();
  _poll_fsm();
  return ret;
}


/*
* Fetch any un-heeded text from the recognizer.
*/
void MOVI::getResult(StringBuilder* out) {
  if (!_result.isEmpty()) {
    out->concatHandoff(&_result);
  }
}


void MOVI::ask(const char* question) {
  // Empty string makes ask faster when there is no question.
  if ((nullptr != question) && (strlen(question) > 0)) {
    say(question);
  }
  _queue_command(MOVICommandCode::ASK);
}


void MOVI::setSynthesizer(int synth, const char* commandline) {
  _queue_command(((0 != synth) ? MOVICommandCode::SETSYNTHPICO : MOVICommandCode::SETSYNTHESPEAK), commandline);
}


void MOVI::setVolume(int volume) {
  StringBuilder str_bldr;
  str_bldr.concatf("%d", volume);
  _queue_command(MOVICommandCode::VOLUME, (const char*) str_bldr.string());
}

// Sets the noise threshold of the recognizer. Values vary between 2 and 95. Factory default is 5.
void MOVI::setThreshold(int threshold) {
  StringBuilder str_bldr;
  str_bldr.concatf("%d", threshold);
  _queue_command(MOVICommandCode::THRESHOLD, (const char*) str_bldr.string());
}


void MOVI::password(const char* question, const char* passkey) {
  say(question);
  _passstring.clear();
  _passstring.concat(passkey);
  _passstring.toUpper();
  _passstring.trim();
  _queue_command(MOVICommandCode::PASSWORD);
}


bool MOVI::train() {
  if (_flags.value(MOVI_FLAG_IN_TRAINING)) {
    _queue_command(MOVICommandCode::TRAINSENTENCES, "");
    return true;
  }
  return false;
}


void MOVI::callSign(const char* callsign) {
  if (!_flags.value(MOVI_FLAG_CALLSIGN_TRAINED)) {
    _queue_command(MOVICommandCode::CALLSIGN, callsign);
  }
  _flags.set(MOVI_FLAG_CALLSIGN_TRAINED);
}


bool MOVI::addSentence(const char* sentence) {
  // Needs a new MOVI instance (typically restart Arduino). This avoids training only part of the sentence set.
  if (!_flags.value(MOVI_FLAG_SENTENCES_ADDED)) {
    //intraining = sendCommand("NEWSENTENCES","","210");
    //_flags.set(MOVI_FLAG_IN_TRAINING, _queue_command(MOVICommandCode::NEWSENTENCES,"","210"));
  }
  if (_flags.value(MOVI_FLAG_IN_TRAINING)) {
    //_flags.set(MOVI_FLAG_IN_TRAINING, _queue_command(MOVICommandCode::ADDSENTENCE,sentence,"211"));
    _flags.set(MOVI_FLAG_SENTENCES_ADDED);
  }
  return _flags.value(MOVI_FLAG_IN_TRAINING);
}


bool MOVI::isReady() {
  bool ret = _uart->initialized();
  ret &= (MOVI_FLAG_READY_MASK == (_flags.raw & MOVI_FLAG_READY_MASK));
  return ret;
}


/*
* Internal function for sending commands in a controlled manner.
*/
bool MOVI::_queue_command(const MOVICommandCode CMD_CODE, const char* p) {
  bool ret = false;
  if (true) {  // TODO: If we are going to enforce a maximum queue size, do it here.
    MOVICmdRespPair* new_cmd = new MOVICmdRespPair(CMD_CODE, movi_get_reply_by_command(CMD_CODE));
    if (new_cmd) {
      if ((nullptr != p) && (strlen(p) > 0)) {
        new_cmd->outbound.concat((char*) p);   // Casting ensures that a copy is made.
      }
      ret = (0 <= _waiting_cmds.insert(new_cmd));
    }
  }
  return ret;
}


/**
* Cleans up any outstanding I/O tracking.
*
* @return The number of waiting I/O exchnages that were purged.
*/
int MOVI::_purge_waiting_io() {
  int ret = 0;
  MOVICmdRespPair* current = _waiting_cmds.remove();
  while (current) {
    delete current;
    current = _waiting_cmds.remove();
    ret++;
  }
  return ret;
}


/**
* Given a sanitized line of response from the hardware, parse and validate the
*   event code, and copy any associated text into the provided detail buffer.
*
* @param line is the isolated line from the UART.
* @param detail is where this function should write the details of the event.
* @return MOVIEvent::UNSUPPORTED on failure. Any other MOVIEvent on success.
*/
MOVIEvent MOVI::_decompose_event_string(char* line, StringBuilder* detail) {
  const int FULL_TAG_LEN = 16;   // strlen("MOVIEvent[000]: ");
  const int LINE_LEN     = strlen(line);
  if (LINE_LEN >= FULL_TAG_LEN) {  // Anything shorter than this is garbage.
    const int LEADER_LEN   = 10;   // strlen("MOVIEvent[");
    MOVIEvent RETURN_VALUE = (MOVIEvent) atoi(line + LEADER_LEN);
    switch (RETURN_VALUE) {
      case MOVIEvent::SHIELD_IDLE:
      case MOVIEvent::SENTENCE_LISTING:
      case MOVIEvent::SENTENCE_DEF:
      case MOVIEvent::LOG_RELAY:
      case MOVIEvent::PONG:
      case MOVIEvent::INIT_COMPLETE:
      case MOVIEvent::VERSION_REPORT:
      case MOVIEvent::VERSION_REPORT_FW:
      case MOVIEvent::VERSION_REPORT_HW:
      case MOVIEvent::READY:
      case MOVIEvent::TRAINING_SKIPPED:
      case MOVIEvent::CALLSIGN_REPORT:
      case MOVIEvent::BEGIN_LISTEN:
      case MOVIEvent::END_LISTEN:
      case MOVIEvent::BEGIN_SAY:
      case MOVIEvent::END_SAY:
      case MOVIEvent::CALLSIGN_DETECTED:
      case MOVIEvent::RAW_WORDS:
      case MOVIEvent::SENTENCE_RECOG:
      case MOVIEvent::PASSWORD_ATTEMPT:
      case MOVIEvent::PASSWORD_ACCEPT:
      case MOVIEvent::SENTENCE_QUEUED:
      case MOVIEvent::SENTENCES_TRAINED:
      case MOVIEvent::CALLSIGN_TRAINED:
      case MOVIEvent::VOLUME_SETTING:
      case MOVIEvent::THRESHOLD_SETTING:
      case MOVIEvent::BEEP_OFF:
      case MOVIEvent::BEEP_ON:
      case MOVIEvent::RESPONSES_OFF:
      case MOVIEvent::RESPONSES_ON:
      case MOVIEvent::SYS_MESSAGES_OFF:
      case MOVIEvent::WELCOME_OFF:
      case MOVIEvent::WELCOME_ON:
      case MOVIEvent::FEMALE_VOICE:
      case MOVIEvent::MALE_VOICE:
      case MOVIEvent::SYNTH_ESPEAK:
      case MOVIEvent::SYNTH_PICO:
      case MOVIEvent::MIC_DEBUG_ON:
      case MOVIEvent::MIC_DEBUG_OFF:
      case MOVIEvent::STOPPED:
      case MOVIEvent::RESTARTED:
      case MOVIEvent::ABORTED:
      case MOVIEvent::SHUTDOWN_NOTICE:
      case MOVIEvent::PASSWORD_REJECT:
      case MOVIEvent::INVALID_PARAM:
      case MOVIEvent::SYNTH_REJECT:
      case MOVIEvent::SILENCE:
      case MOVIEvent::UNKNOWN_SENTENCE:
      case MOVIEvent::NOISE_ALARM:
        if (LINE_LEN > FULL_TAG_LEN) {
          detail->concat(line + FULL_TAG_LEN);
        }
        return RETURN_VALUE;

      default:
        c3p_log(LOG_LEV_DEBUG, __PRETTY_FUNCTION__, "Unknown event: %s", line);
        break;
    }
  }
  return MOVIEvent::UNSUPPORTED;
}



/**
* Handles and returns the event in the given line.
* Does not advance state machine.
* May queue commands, but does not consider existing command queue.
*
* @param line is the isolated line from the UART.
* @return MOVIEvent::UNSUPPORTED on failure. Any other MOVIEvent on success.
*/
MOVIEvent MOVI::_handle_event_line(char* line) {
  StringBuilder detail;
  MOVIEvent e_code = _decompose_event_string(line, &detail);

  if (_callback) {
    _callback(e_code, 0, (char*)detail.string());
  }
  else {
    c3p_log(LOG_LEV_INFO, __PRETTY_FUNCTION__, "MOVIEvent: %s", movi_get_event_string(e_code));
  }

  switch (e_code) {
    case MOVIEvent::SHIELD_IDLE:
      break;

    case MOVIEvent::SENTENCE_LISTING:      // A trained sentence report is beginning.
      _vocabulary.clear();
      break;

    case MOVIEvent::SENTENCE_DEF:          // HW is relaying a sentence it knows.
      if (MOVIState::CONF_SYNC == _fsm_pos) {
        // Give time for more sentence definitions to come in.
        _fsm_lockout_ms = millis() + 20;
      }
      // TODO: Strip the numeral and leading space.
      _vocabulary.concat(detail.string());
      break;

    case MOVIEvent::LOG_RELAY:             // Print to our log directly.
      c3p_log(LOG_LEV_NOTICE, __PRETTY_FUNCTION__, "%s", detail.string());
      break;

    case MOVIEvent::PONG:             // Hardware confirms communication.
      if (!_flags.value(MOVI_FLAG_SHIELD_FOUND)) {
        _flags.set(MOVI_FLAG_SHIELD_FOUND);
      }
      break;

    case MOVIEvent::INIT_COMPLETE:   // INIT completed. Version data attached.
      if (!_flags.value(MOVI_FLAG_SHIELD_FOUND)) {
        _flags.set(MOVI_FLAG_SHIELD_FOUND);
      }
      break;

    case MOVIEvent::VERSION_REPORT:
    case MOVIEvent::VERSION_REPORT_FW:
    case MOVIEvent::VERSION_REPORT_HW:
    case MOVIEvent::READY:
      break;

    case MOVIEvent::TRAINING_SKIPPED:
      // This is probably an error case. The state machine shouldn't have
      //   dispatched a training command unless there was confirmation that
      //   a new sentence was added.
      // TODO: Wipe the vocab shadow, and re-mirror it from the hardware.
      _flags.clear(MOVI_FLAG_IN_TRAINING);
      break;

    case MOVIEvent::CALLSIGN_REPORT:
      if (1 < detail.split(":")) {
        _callsign.clear();
        detail.drop_position(0);
        _callsign.concat(detail.position_trimmed(0));
      }
      break;

    case MOVIEvent::BEGIN_LISTEN:        _flags.set(MOVI_FLAG_HW_LISTENING);    break;
    case MOVIEvent::END_LISTEN:          _flags.clear(MOVI_FLAG_HW_LISTENING);  break;
    case MOVIEvent::BEGIN_SAY:           _flags.set(MOVI_FLAG_HW_SPEAKING);     break;
    case MOVIEvent::END_SAY:             _flags.clear(MOVI_FLAG_HW_SPEAKING);   break;

    case MOVIEvent::CALLSIGN_DETECTED:    break;
    case MOVIEvent::RAW_WORDS:            break;
    case MOVIEvent::SENTENCE_RECOG:       break;
    case MOVIEvent::PASSWORD_ATTEMPT:     break;
    case MOVIEvent::PASSWORD_ACCEPT:      break;

    case MOVIEvent::SENTENCE_QUEUED:
      // TODO: Embark on an FSM sequence to do/queue training automatically.
      break;

    case MOVIEvent::SENTENCES_TRAINED:    // This signifies the end of training.
    case MOVIEvent::CALLSIGN_TRAINED:     // This signifies the end of training.
      _flags.clear(MOVI_FLAG_IN_TRAINING);
      break;

    case MOVIEvent::VOLUME_SETTING:
      if (1 < detail.split("[")) {
        detail.drop_position(0);
        _volume = atoi(detail.position_trimmed(0));
      }
      break;

    case MOVIEvent::THRESHOLD_SETTING:   break;
    case MOVIEvent::BEEP_OFF:        _flags.clear(MOVI_FLAG_HW_BEEP);            break;
    case MOVIEvent::BEEP_ON:         _flags.set(MOVI_FLAG_HW_BEEP);              break;
    case MOVIEvent::RESPONSES_OFF:   _flags.clear(MOVI_FLAG_HW_ANNOUNCE_RECOG);  break;
    case MOVIEvent::RESPONSES_ON:    _flags.set(MOVI_FLAG_HW_ANNOUNCE_RECOG);    break;

    case MOVIEvent::SYS_MESSAGES_OFF:    break;

    case MOVIEvent::WELCOME_OFF:     _flags.clear(MOVI_FLAG_HW_ANNOUNCE_BOOT);   break;
    case MOVIEvent::WELCOME_ON:      _flags.set(MOVI_FLAG_HW_ANNOUNCE_BOOT);     break;
    case MOVIEvent::FEMALE_VOICE:    _flags.clear(MOVI_FLAG_HW_ANNOUNCE_BOOT);   break;
    case MOVIEvent::MALE_VOICE:      _flags.set(MOVI_FLAG_HW_IS_MALE);           break;
    case MOVIEvent::SYNTH_ESPEAK:    _flags.clear(MOVI_FLAG_HW_SYNTH_PICO);      break;
    case MOVIEvent::SYNTH_PICO:      _flags.set(MOVI_FLAG_HW_SYNTH_PICO);        break;

    case MOVIEvent::MIC_DEBUG_ON:
    case MOVIEvent::MIC_DEBUG_OFF:

    case MOVIEvent::STOPPED:
    case MOVIEvent::RESTARTED:
    case MOVIEvent::ABORTED:
    case MOVIEvent::SHUTDOWN_NOTICE:
    case MOVIEvent::PASSWORD_REJECT:
      break;

    case MOVIEvent::INVALID_PARAM:
      break;

    case MOVIEvent::SYNTH_REJECT:
    case MOVIEvent::SILENCE:
    case MOVIEvent::UNKNOWN_SENTENCE:
    case MOVIEvent::NOISE_ALARM:
      break;

    case MOVIEvent::UNSUPPORTED:
      break;
  }
  return e_code;
}


/*
* This function contains the state machine logic that is advanced as a
*   consequence of polling. Some state transitions must be called elsewhere.
*/
MOVIState MOVI::_poll_fsm() {
  MOVIState ret = MOVIState::UNKNOWN;

  switch (_fsm_pos) {
    // Exit conditions: Unconditional.
    case MOVIState::UNKNOWN:
      _fsm_pos_pending = MOVIState::INIT;
      break;

    // Exit conditions: The UART is ready to use.
    case MOVIState::INIT:
      if (_uart->initialized() && _uart->txCapable() && _uart->rxCapable()) {
        _fsm_pos_pending = MOVIState::PENDING_BOOT;
      }
      break;

    // Exit conditions: The hardware replied to an INIT and a PING.
    case MOVIState::PENDING_BOOT:
      if (_flags.value(MOVI_FLAG_SHIELD_FOUND)) {
        _fsm_pos_pending = MOVIState::CONF_SYNC;
      }
      break;

    // Exit conditions: The hardware been configured and shadowed, and the UART
    //   has been idle and flushed for a long enough period of time that we can
    //   be confident that the entire sentence vocabulary has been received.
    case MOVIState::CONF_SYNC:
      if (_uart->flushed() && (0 == _waiting_cmds.size())) {
        _fsm_pos_pending = MOVIState::IDLE;
      }
      break;

    // Exit conditions:
    case MOVIState::IDLE:
      if (_flags.value(MOVI_FLAG_IN_TRAINING)) {
        //ret = _set_fsm_position(MOVIState::TRAINING);
      }
      else if (_flags.value(MOVI_FLAG_HW_SPEAKING)) {
        //ret = _set_fsm_position(MOVIState::SPEAKING);
      }
      else if (_flags.value(MOVI_FLAG_HW_LISTENING)) {
        //ret = _set_fsm_position(MOVIState::LISTENING);
      }
      break;

    // Exit conditions: The hardware is no longer speaking.
    // NOTE: SPEAKING is a single-branch digresion from whatever MOVIState we
    //   entered from.
    case MOVIState::SPEAKING:
      if (!_flags.value(MOVI_FLAG_HW_SPEAKING)) {
        _fsm_pos_pending = _fsm_pos_prior;
      }
      break;

    // Exit conditions: The hardware is no longer listening.
    // NOTE: LISTENING is a single-branch digresion from whatever MOVIState we
    //   entered from.
    case MOVIState::LISTENING:
      if (!_flags.value(MOVI_FLAG_HW_LISTENING)) {
        _fsm_pos_pending = _fsm_pos_prior;
      }
      break;

    // Exit conditions: The training flag has been cleared by a notice
    //   from the hardware.
    case MOVIState::TRAINING:
      if (!_flags.value(MOVI_FLAG_IN_TRAINING)) {
        _fsm_pos_pending = MOVIState::IDLE;
      }
      break;

    case MOVIState::PENDING_PWR:
      break;

    case MOVIState::POWERED_OFF:     // The only way out is calling init().
      break;

    case MOVIState::FAULT:         // If the driver is in FAULT, do nothing.
      break;
    default:   // Can't automatically exit from an unknown state.
      break;
  }


  if (MOVIState::UNKNOWN == _fsm_pos_pending) {
    // If no state transition happened, and the state allows it, we
    //   can process outbound queue.

    // TODO: If the FSM state allows it...
    // TODO: If the UART is flushed...
    // If there is an unsent command at the head of the queue, dispatch it.
    MOVICmdRespPair* waiting_cmd = _waiting_cmds.get(0);
    if ((nullptr != waiting_cmd) && !waiting_cmd->was_sent) {
      bool send = false;
      StringBuilder str_bldr(movi_get_command_string(waiting_cmd->CMD));
      //if (!_flags.value(MOVI_FLAG_SENTENCES_ADDED) || _flags.value(MOVI_FLAG_IN_TRAINING)) {
        //  send = true;
      //}
      //else if (isReady()) {
        send = true;
      //}
      if (send) {
        if (!waiting_cmd->outbound.isEmpty()) {
          str_bldr.concatf(" %s", (char*) waiting_cmd->outbound.string());
        }
        str_bldr.concat('\n');
        waiting_cmd->was_sent = (0 <= _uart->provideBuffer(&str_bldr));

        if (waiting_cmd->was_sent & (MOVIEvent::UNSUPPORTED == waiting_cmd->EXPECT)) {
          _waiting_cmds.remove(waiting_cmd);
          delete waiting_cmd;
        }
      }
    }
  }

  if (MOVIState::UNKNOWN != _fsm_pos_pending) {
    // If there is a state-change pending, set it.
    if (0 == _set_fsm_position(_fsm_pos_pending)) {
      // State set succeeded.
      ret = _fsm_pos_pending;
    }
  }
  return _fsm_pos;    // Return the current state.
}


/**
* @return 1 on success, 0 on no action/no failure, -1 on failure.
*/
int8_t MOVI::_set_fsm_position(MOVIState new_state) {
  int8_t ret = 0;
  bool state_entry_success = false;   // Fail by default.
  if (!_fsm_is_waiting()) {
    switch (new_state) {
      // Entry into INIT means the class members have been reset and all memory
      //   is allocated,
      case MOVIState::INIT:
        _flags.raw = (_flags.raw & MOVI_FLAG_RESET_MASK);
        _hw_ver = 0.0f;
        _fw_ver = 0.0f;
        _fsm_lockout_ms = 0;
        _threshold = 0;
        _volume = 0;
        _result.clear();
        _passstring.clear();
        _response.clear();
        _vocabulary.clear();
        _purge_waiting_io();
        state_entry_success = true;
        break;

      // Entry into PENDING_BOOT means the messages were dispatched to confirm
      //   hardware operation.
      case MOVIState::PENDING_BOOT:
        if (_flags.value(MOVI_FLAG_INIT_SENT)) {
          state_entry_success = _queue_command(MOVICommandCode::PING);
        }
        else {
          _flags.set(MOVI_FLAG_INIT_SENT, _queue_command(MOVICommandCode::INIT));
        }
        break;

      case MOVIState::CONF_SYNC:
        _queue_command(MOVICommandCode::VOLUME);
        _queue_command(MOVICommandCode::RESPONSES);
        _queue_command(MOVICommandCode::WELCOMEMESSAGE);
        _queue_command(MOVICommandCode::BEEPS);
        _queue_command(MOVICommandCode::VOCABULARY);
        state_entry_success = true;
        break;

      case MOVIState::IDLE:
        state_entry_success = true;
        break;

      case MOVIState::SPEAKING:
        state_entry_success = true;
        break;

      case MOVIState::LISTENING:
        state_entry_success = true;
        break;

      case MOVIState::TRAINING:
        state_entry_success = true;
        break;

      case MOVIState::PENDING_PWR:
        state_entry_success = true;
        break;

      case MOVIState::POWERED_OFF:
        state_entry_success = true;
        break;

      // We allow fault entry to be done this way.
      case MOVIState::FAULT:
        state_entry_success = true;
        break;

      default:   // Any attempt to enter an unlisted state will cause a FAULT.
        _fsm_pos_prior = _fsm_pos;
        _fsm_pos = MOVIState::FAULT;
        c3p_log(LOG_LEV_ERROR, __PRETTY_FUNCTION__, "illegal state: %s (%u)", (uint8_t) new_state, movi_state_str(new_state));
        ret = -1;
        break;
    }

    if (state_entry_success) {
      c3p_log(LOG_LEV_NOTICE, __PRETTY_FUNCTION__, "MOVI moved state (%s ---> %s)", movi_state_str(_fsm_pos), movi_state_str(new_state));
      _fsm_pos_prior = _fsm_pos;
      _fsm_pos       = new_state;
      ret = 1;
    }
  }
  return ret;
}


bool MOVI::_fsm_is_waiting() {
  bool ret = false;
  if (0 != _fsm_lockout_ms) {
    ret = !(millis() >= _fsm_lockout_ms);
    if (!ret) {
      _fsm_lockout_ms = 0;
    }
  }
  return ret;
}


void MOVI::_print_work_queue(StringBuilder* output) {
  int print_depth = _waiting_cmds.size();
  output->concatf("-- Queue Listing (%d total)\n", print_depth);
  for (int i = 0; i < print_depth; i++) {
    _waiting_cmds.get(i)->printDebug(output);
  }
}


void MOVI::_print_vocabulary(StringBuilder* output) {
  int print_depth = _vocabulary.count();
  output->concatf("-- Vocabulary shadow (%d total):\n", print_depth);
  for (int i = 0; i < print_depth; i++) {
    output->concatf("\t %s\n", _vocabulary.position(i));
  }
}


void MOVI::_print_fsm(StringBuilder* output) {
  output->concatf("\t Prior state:   %s\n", movi_state_str(_fsm_pos_prior));
  output->concatf("\t Current state: %s%s%s\n", movi_state_str(_fsm_pos), (_fsm_is_waiting() ? " (LOCKED)":""), (_fsm_is_stable() ? " (STABLE)":""));
  if (!_fsm_is_stable()) {
    output->concatf("\t Target state:  %s\n", movi_state_str(_fsm_pos_target));
  }
  if (_fsm_is_waiting()) {
    output->concatf("\t FSM locked for another %ums\n", _fsm_lockout_ms - millis());
  }
}


void MOVI::printDebug(StringBuilder* text_return){
  StringBuilder hdr_txt("MOVI Shield");
  if (!_callsign.isEmpty()) {
    hdr_txt.concatf(" [%s]", _callsign.string());
  }
  StringBuilder::styleHeader1(text_return, (const char*) hdr_txt.string());

  _print_fsm(text_return);
  text_return->concatf("\t Version (HW/FW):  \t%.2f / %.2f\n", _hw_ver, _fw_ver);
  text_return->concatf("\t Volume:           \t%u\n", _volume);
  text_return->concatf("\t Noise threshold:  \t%u\n", _threshold);
  _print_vocabulary(text_return);
  _print_work_queue(text_return);
}



int MOVI::console_handler(StringBuilder* text_return, StringBuilder* args) {
  int ret = 0;
  char*  cmd = args->position_trimmed(0);
  if (args->count() == 0) {
    printDebug(text_return);
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "init")) {
    text_return->concatf("MOVI init %s.\n", _queue_command(MOVICommandCode::INIT) ? "succeded" : "failed");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "ping")) {
    text_return->concatf("MOVI ping %s.\n", _queue_command(MOVICommandCode::PING) ? "succeded" : "failed");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "help")) {
    text_return->concatf("MOVI help %s.\n", _queue_command(MOVICommandCode::HELP) ? "succeded" : "failed");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "male")) {
    text_return->concatf("MOVI MALE %s.\n", _queue_command(MOVICommandCode::MALE) ? "succeded" : "failed");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "female")) {
    text_return->concatf("MOVI FEMALE %s.\n", _queue_command(MOVICommandCode::FEMALE) ? "succeded" : "failed");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "volume")) {
    if (1 < args->count()) {
      int arg1 = args->position_as_int(1);
      setVolume(arg1);
      text_return->concatf("MOVI setVolume(%d)\n", arg1);
    }
    else {
      text_return->concatf("Volume: %u\n", _volume);
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "threshold")) {
    if (1 < args->count()) {
      int arg1 = args->position_as_int(1);
      setThreshold(arg1);
      text_return->concatf("MOVI setThreshold(%d)\n", arg1);
    }
    else {
      text_return->concatf("Threshold: %u\n", _threshold);
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "say")) {
    args->drop_position(0);
    if (0 < args->count()) {
      args->implode(" ");
      text_return->concatf("say(\"%s\")\n", args->string());
      say((char*)args->string());
    }
    else {
      text_return->concat("Usage: say <what?>\n");
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "train")) {
    args->drop_position(0);
    if (0 < args->count()) {
      args->implode(" ");
      text_return->concatf("say(\"%s\")\n", args->string());
      say((char*)args->string());
    }
    else {
      text_return->concat("Usage: train <sentence>\n");
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "callsign")) {
    if (1 < args->count()) {
      callSign((const char*) args->position_trimmed(1));
    }
    else {
      text_return->concatf("Callsign: %s\n", _callsign.string());
    }
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "restart")) {
    restartDialog();
    text_return->concat("MOVI restart command sent.\n");
  }
  else if (0 == StringBuilder::strcasecmp(cmd, "purge")) {
    text_return->concatf("MOVI purged %d commands.\n", _purge_waiting_io());
  }
  else {
    ret = -1;
  }
  return ret;
}


/*******************************************************************************
* MOVICmdRespPair functions
*******************************************************************************/

void MOVICmdRespPair::printDebug(StringBuilder* text_return){
  text_return->concatf("\t %15s %s", movi_get_command_string(CMD), outbound.string());
  if (was_sent) {    text_return->concat("\t[SENT]");    }
  if (MOVIEvent::UNSUPPORTED != EXPECT) {
    text_return->concatf("\t[EXPECTS %s]", movi_get_event_string(EXPECT));
  }
  text_return->concat('\n');
}
