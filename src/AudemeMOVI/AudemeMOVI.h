/**
* @file      AudemeMOVI.h
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
*   SDcard. But the bandwidth requirement is so low that likely isn't worth
*   changing. GPS uses more.
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

#ifndef ____MOVIShield__
#define ____MOVIShield__

#include "StringBuilder.h"
#include "CppPotpourri.h"
#include "FlagContainer.h"
#include "LightLinkedList.h"
#include "UARTAdapter.h"

#define MOVI_API_VERSION 9.99f

/* Class flags */
#define MOVI_FLAG_INIT_SENT           0x00000001 // Has the MOVI init been sent?
#define MOVI_FLAG_SHIELD_FOUND        0x00000002 // Has the MOVI been confirmed to be present?
#define MOVI_FLAG_CONFIGURED          0x00000004 // Has the hardware been configured and shadowed?
#define MOVI_FLAG_IN_TRAINING         0x00000008 // Has the hardware been trained for sentences?
#define MOVI_FLAG_CALLSIGN_TRAINED    0x00000010 // Has the callsign been trained?
#define MOVI_FLAG_SENTENCES_ADDED     0x00000020 // Has the hardware been trained on at least one sentence?
#define MOVI_FLAG_DEBUGGING           0x00000040 // Class debugging enabled.
#define MOVI_FLAG_HW_SPEAKING         0x20000000 //
#define MOVI_FLAG_HW_LISTENING        0x40000000 //
#define MOVI_FLAG_HW_IS_MALE          0x80000000 // If this isn't set, the voice is female.
#define MOVI_FLAG_HW_BEEP             0x10000000 // Setting survives power-cycle.
#define MOVI_FLAG_HW_ANNOUNCE_BOOT    0x20000000 // Setting survives power-cycle.
#define MOVI_FLAG_HW_ANNOUNCE_RECOG   0x40000000 // Setting survives power-cycle.
#define MOVI_FLAG_HW_SYNTH_PICO       0x80000000 //

// These flags survive class reset.
#define MOVI_FLAG_RESET_MASK ( \
  MOVI_FLAG_DEBUGGING | MOVI_FLAG_HW_ANNOUNCE_BOOT | MOVI_FLAG_HW_BEEP | \
  MOVI_FLAG_HW_ANNOUNCE_RECOG | MOVI_FLAG_HW_SYNTH_PICO)

// These flags being set indicates hardware readiness.
#define MOVI_FLAG_READY_MASK ( \
  MOVI_FLAG_INIT_SENT | MOVI_FLAG_SHIELD_FOUND | MOVI_FLAG_CONFIGURED | \
  MOVI_FLAG_CALLSIGN_TRAINED | MOVI_FLAG_SENTENCES_ADDED)

/*
* MOVI events. These enum values match the return codes from the hardware.
* Observation of these codes being returned from the hardware is taken as
*   evidence that the associated command (if applicable) has completed.
*/
enum class MOVIEvent : int16_t {
  SHIELD_IDLE       = 0,
  SENTENCE_LISTING  = 2,     // Beginning trained sentence report.
  SENTENCE_DEF      = 3,     // An element of the trained sentence report.
  LOG_RELAY         = 4,     // The board is printing a line of text for human consumption.
  PONG              = 100,   // Comm check response.
  INIT_COMPLETE     = 101,   // The MOVI firmware version is being reported after an INIT.
  VERSION_REPORT    = 102,   // The MOVI is reporting the full hardware ID string.
  VERSION_REPORT_FW = 103,   // The MOVI firmware version is being reported.
  VERSION_REPORT_HW = 104,   // The MOVI firmware version is being reported.
  READY             = 105,   // The MOVI board is ready for interaction.
  TRAINING_SKIPPED  = 110,   // Training completed without doing any work.
  CALLSIGN_REPORT   = 112,   // Callsign report.
  BEGIN_LISTEN      = 140,   //
  END_LISTEN        = 141,   //
  BEGIN_SAY         = 150,   //
  END_SAY           = 151,   //
  CALLSIGN_DETECTED = 200,   //
  RAW_WORDS         = 201,   // Recognized the sentence as relayed verbatim.
  SENTENCE_RECOG    = 202,   // Recognized the sentence with the given index (#<idx>).
  PASSWORD_ATTEMPT  = 203,   //
  PASSWORD_ACCEPT   = 204,   //
  //NEWSENTENCES_RESP = 210,   // TODO: Naming.
  SENTENCE_QUEUED   = 211,   // A sentence has been added to the training queue.
  SENTENCES_TRAINED = 212,   // The training queue has been processed.
  CALLSIGN_TRAINED  = 213,   // Callsign training complete.
  VOLUME_SETTING    = 220,   // Volume setting.
  THRESHOLD_SETTING = 230,   // Noise threshold setting.
  BEEP_OFF          = 240,   //
  BEEP_ON           = 241,   //
  RESPONSES_OFF     = 250,   //
  RESPONSES_ON      = 251,   //
  SYS_MESSAGES_OFF  = 252,   // The system messages were turned off.
  WELCOME_OFF       = 254,   //
  WELCOME_ON        = 255,   //
  FEMALE_VOICE      = 256,   //
  MALE_VOICE        = 257,   //
  SYNTH_ESPEAK      = 258,   //
  SYNTH_PICO        = 259,   //
  MIC_DEBUG_ON      = 260,   //
  MIC_DEBUG_OFF     = 261,   //
  STOPPED           = 290,   // Notice that the hardware has stopped.
  RESTARTED         = 291,   // Notice that the hardware has rebooted.
  ABORTED           = 292,   // Notice that the hardware aborted a play or say.
  SHUTDOWN_NOTICE   = 299,   // Notice that the hardware has shut down.
  PASSWORD_REJECT   = 404,   //
  INVALID_PARAM     = 430,   // Invalid parameter.
  SYNTH_REJECT      = 450,   //
  SILENCE           = 501,   // A recognition cycle found only silence.
  UNKNOWN_SENTENCE  = 502,   //
  NOISE_ALARM       = 530,   //
  UNSUPPORTED       = -1     // Internal enum for set closure purposes.
};

/* MOVI commands. These enum values are mapped to strings used as commands. */
enum class MOVICommandCode : uint8_t {
  INIT,
  PING,
  ASK,
  SAY,
  SETSYNTHESPEAK,
  SETSYNTHPICO,
  VOLUME,
  THRESHOLD,
  PASSWORD,
  TRAINSENTENCES,
  NEWSENTENCES,
  ADDSENTENCE,
  CALLSIGN,
  FACTORY,
  STOP,
  RESTART,
  PAUSE,
  UNPAUSE,
  FINISH,
  PLAY,
  ABORT,
  RESPONSES,
  WELCOMEMESSAGE,
  BEEPS,
  FEMALE,
  MALE,
  HELP,
  SHUTDOWN,
  ABOUT,
  VERSION,
  HWVERSION,
  VOCABULARY,
  TRAIN,
  MEM,
  MICDEBUG
};

/* MOVI driver states. */
enum class MOVIState : uint8_t {
  UNKNOWN      = 0,
  INIT         = 1,
  PENDING_BOOT = 2,
  CONF_SYNC    = 3,
  IDLE         = 4,
  SPEAKING     = 5,
  LISTENING    = 6,
  TRAINING     = 7,
  PENDING_PWR  = 8,
  POWERED_OFF  = 9,
  FAULT        = 0xFF
};


/* Callbacks for drivers that provide extra GPI pins. */
typedef void (*MOVICallback)(MOVIEvent, uint8_t, char*);



/* A class to track asynchronous commands and responses. */
class MOVICmdRespPair {
  public:
    MOVICmdRespPair(const MOVICommandCode C, MOVIEvent exp_reply) :
      EXPECT(exp_reply), CMD(C), was_sent(false) {};
    ~MOVICmdRespPair() {};

    const MOVIEvent EXPECT;
    const MOVICommandCode CMD;  // The command being waited on.
    bool was_sent;
    StringBuilder outbound;  // The full string sent to the module.

    void printDebug(StringBuilder*);
};


/*
* The API class itself.
* Implements BufferAccepter, and ties itself to the given UART on init.
*/
class MOVI : public BufferAccepter {
  public:
    MOVI(UARTAdapter*, bool debugonoff = false);
    ~MOVI();

    /* Implementation of BufferAccepter. */
    int8_t provideBuffer(StringBuilder* buf);

    // Gets the result string of an event. For example: MOVIEvent[201]: LET THERE LIGHT results in "LET THERE BE
    // LIGHT\n". The resulting string might need trimming for comparison to other strings. The resulting string
    // is uppercase and does not contain any numbers, punctuation or special characters.
    void getResult(StringBuilder*);

    /* Version accessors. */
    inline float getAPIVersion() {       return MOVI_API_VERSION; };
    inline float getHardwareVersion() {  return _hw_ver;          };
    inline float getFirmwareVersion() {  return _fw_ver;          };

    // By default, init waits for MOVI to be booted and resets some settings.
    //   If the recognizer had been stopped with stopDialog() it is restarted.
    //   Only initializes the API and doesn't wait for MOVI to be ready.
    //int8_t init();    NOTE: Made automatic by state machine.

    // This method is called in loop() to get an event from the recognizer. 0 stand for no event. A postive number
    // denotes a sentence number. A negative value defines an event number. Event numbers are the negatives of
    // the numbers displayed on the serial monitor. For example: MOVIEvent[200] would return -200.
    int8_t poll();

    // Sets MOVI's synthesizer to one of SYNTH_ESPEAK or SYNTH_PICO with optional command line parameters.
    void setSynthesizer(int synth, const char* commandline = nullptr);

    // Sets the output volume of the speaker.
    void  setVolume(int volume);

    // Sets the noise threshold of the recognizer. Values vary between 2 and 95. Factory default is 5.
    void  setThreshold(int threshold);

    // This method checks if the training set contains new sentences since the last training. If so, it trains all
    // sentences added in this MOVI instance. Once training is performed, no more sentences can be added
    // and training cannot be invoked again in the same instance.
    bool train();

    // This method sets the callsign to the parameter given. If the callsign has
    //   previously been set to the same value, nothing happens. Only one
    //   callsign can be trained per MOVI instance. The callsign can be the
    //   empty string. The MOVI will react to any noise above the threshold.
    void callSign(const char* callsign);

    // Speak the optional given sentence and then immediately listen without requiring a callsign.
    void ask(const char* question = nullptr);

    inline bool waitingForHardware() {    return !_waiting_cmds.hasNext();    };

    // Makes MOVI speak the sentence given as first parameter. Then MOVI's password function is used to query for
    // a password. The API comapres the passkey with the password and return either PASSWORD_REJECT or
    // PASSWORD_ACCEPT as an event. The passkey is not transferred to or saved on the MOVI board.
    // IMPORTANT: The passkey must consist only of words contained in the trained sentences
    // and must not contain digits or other non-letter characters except one space between the words.
    // (e.g., ask(F("Password please",password);)
    void password(const char* question, const char* passkey);

    // This method adds a sentence to the training set. Sentences must not contain any punctuation or numbers.
    // Everything must be spelled out. No special characters, umlauts or accents. Uppercase or lowercase does
    // not matter.
    // (e.g., addsentence(F("Light On");)
    bool addSentence(const char* sentence);

    // This method can be used to determine if MOVI is ready to receive commands, e.g. when MOVI has been initialized with init(false).
    bool isReady();


    /* Inline wrappers for publicly-exposed commands **************************/

    // Resets MOVI to factory default. This method should only be used in setup()
    //   and only if needed. All trained sentences and callsigns are untrained. The
    //   preferrable method for a factory reset is to use the serial monitor.
    inline void  factoryDefault() {            _queue_command(MOVICommandCode::FACTORY);        };

    // Stops the recognizer and synthesizer.
    inline void  stopDialog() {                _queue_command(MOVICommandCode::STOP);           };

    // Restarts the recognizer and synthesizer manually.
    inline void  restartDialog() {             _queue_command(MOVICommandCode::RESTART);        };

    // Makes MOVI speak the sentence given as parameter using the speech synthesizer.
    inline void  say(const char* sentence) {   _queue_command(MOVICommandCode::SAY, sentence);  };
    inline void  say(StringBuilder* sentence) {  say((const char*) sentence->string());      };

    // Pauses the recognizer until an upause(), ask(), say() or password() command.
    inline void  pause() {                     _queue_command(MOVICommandCode::PAUSE);          };

    // Silently interrupts a pause. See pause()
    inline void  unpause() {                   _queue_command(MOVICommandCode::UNPAUSE);        };

    // Finishes up the currently executing sentence recognition.
    inline void  finish() {                    _queue_command(MOVICommandCode::FINISH);         };

    // Play an audio file (wave format) located on the update partition of the SDcard.
    inline void  play(const char* filename) {  _queue_command(MOVICommandCode::PLAY, filename);    };

    // Aborts a play or say command immediately.
    inline void  abort() {                     _queue_command(MOVICommandCode::ABORT,"");    };

    // Turns the spoken responses as a result of recognition events (e.g. silence or noise) on or off.
    inline void  responses(bool on) {           _queue_command(MOVICommandCode::RESPONSES, (on ? "ON":"OFF"));  };

    // Turns off the spoken welcome message indicating the call sign.
    inline void  welcomeMessage(bool on) {      _queue_command(MOVICommandCode::WELCOMEMESSAGE, (on ? "ON":"OFF"));  };

    // Turns the recognition beeps on or off.
    inline void  beeps(bool on) {               _queue_command(MOVICommandCode::BEEPS, (on ? "ON":"OFF"));   };

    // Sets the gender of the speech synthesizer.
    inline void  setVoiceGender(bool female) {  _queue_command(female ? MOVICommandCode::FEMALE:MOVICommandCode::MALE);   };

    // Trim for console support.
    int  console_handler(StringBuilder* text_return, StringBuilder* args);
    void printDebug(StringBuilder*);

    inline void  setCallback(MOVICallback x) {  _callback = x;   };


  private:
    UARTAdapter*    _uart;        // This is the pipe to the hardware.
    FlagContainer32 _flags;       // Class state tracking.
    float           _hw_ver;      // stores hardware version
    float           _fw_ver;      // stores firmware version
    uint32_t        _fsm_lockout_ms; // Used to enforce a delay between state transitions.
    MOVIState       _fsm_pos;
    MOVIState       _fsm_pos_prior;
    MOVIState       _fsm_pos_target;
    uint8_t         _volume;
    uint8_t         _threshold;
    MOVICallback    _callback;
    StringBuilder   _result;      // Stores the last-noted speech-capture result.
    StringBuilder   _passstring;  // stores the passkey for a password() request
    StringBuilder   _response;    // Treated as a tokenized list of complete lines.
    StringBuilder   _vocabulary;  // Treated as a tokenized list of sentences to recognize.
    StringBuilder   _callsign;    // The hardware's idea about its own name.
    LinkedList<MOVICmdRespPair*> _waiting_cmds;

    /* Command dispatch and tracking */
    bool      _queue_command(const MOVICommandCode, const char* parameter = nullptr);
    int       _purge_waiting_io();
    void      _print_work_queue(StringBuilder*);

    /* Event handling */
    MOVIEvent _handle_event_line(char* line);
    MOVIEvent _decompose_event_string(char* line, StringBuilder* detail);

    /* Application glue */
    void      _print_vocabulary(StringBuilder*);

    /* State machine functions */
    int8_t _poll_fsm();
    int8_t _set_fsm_position(MOVIState);
    bool   _fsm_is_waiting();
    void   _print_fsm(StringBuilder*);
    inline bool _fsm_is_stable() {   return (_fsm_pos == _fsm_pos_target);   };
};

#endif //____MOVIShield__)
