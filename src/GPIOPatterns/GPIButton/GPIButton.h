/*
File:   GPIButton.h
Author: J. Ian Lindsay
Date:   2023.07.11

This header provides common support for buttons connected to GPIO pins.

NOTE: Strict attention must be paid to the ISR safety of the core operation of
  these classes. Inline should be used wherever possible.

TODO: Convert this class to a template that uses an enum for the ID field.
*/

#ifndef __UI_BUTTONS_H__
#define __UI_BUTTONS_H__

#include <stdint.h>
#include "AbstractPlatform.h"
#include "TimerTools.h"

/**
* @brief Used to determine a buttons current state
*/
enum class GPIButtonState : uint8_t {
  UNKNOWN  = 0x00,
  RELEASE  = 0x01,
  PRESS    = 0x02,
  LPRESS   = 0x03
};


/*******************************************************************************
* This class represents a single button (with constraints and behaviors) that is
*   abstracted from its source.
*
* NOTE: This class uses discrete bool members over flags for better atomicity.
* NOTE: This class handles software debounce and long-press. But long-press
*   detection needs external scheduling, and the detection will be limited by
*   the temporal resolution of that arrangement.
* NOTE: If long-press detection or software debounce isn't desired for a given
*   button, pass a value of zero for its period argument during construction.
* NOTE: This class will not initialize into a state where the button is pressed.
*   If the button is held when init() is called, class state will not evolve
*   until poll() is called and the button reads as inactive. State reports will
*   be reliable at that time.
*******************************************************************************/
class GPIButton {
  public:
    const uint8_t ID;   // TODO: Template and enum.

    GPIButton(const uint8_t CAST_BTN_ID, const uint8_t PIN, const GPIOMode, const bool ACTIVE_LOW, const uint16_t DEBOUNCE_MS, const uint16_t LP_MS = 0);
    ~GPIButton();

    inline bool initialized() {           return (GPIButtonState::UNKNOWN != _debounced_state);  };
    inline bool shortPressed() {          return (GPIButtonState::PRESS == _debounced_state);    };
    inline bool longPressed() {           return (GPIButtonState::LPRESS == _debounced_state);   };
    inline bool pressed() {               return (shortPressed() | longPressed());  };
    inline bool lpAware() {               return (_lp_timer.period() > 0);          };
    inline bool needsPolling() {          return (_long_pressing & _debouncing);    };
    inline uint32_t lastChange() {        return _last_change;      };
    inline uint32_t lastPolling() {       return _last_polling;     };
    inline GPIButtonState priorState() {  return _prior_state;      };
    inline GPIButtonState state() {       return _debounced_state;  };

    /* Called once to setup pins and state certainty. */
    int8_t init();
    int8_t deinit();

    /* Called repeatedly to update and read the button. */
    int8_t poll();
    uint32_t next_poll_delay();
    GPIButtonState getFreshState();


  private:
    const uint8_t   _PIN;               // TODO: Template and enum.
    const GPIOMode  _PIN_MODE;          // What GPIOMode should the pin be?
    const bool      _PIN_ACT_LOW;       // If true, the physical state will be inverted.
    bool            _pin_setup;         // Pin setup is complete.
    bool            _debouncing;        // If true, the debounce timeout is being observed.
    bool            _long_pressing;     // If true, the LP timeout is being observed.
    bool            _pin_filter;        // True state of the button on last polling.
    GPIButtonState  _prior_state;       // The state that the button was in before now.
    GPIButtonState  _debounced_state;   // The state that made it past debouncing.
    GPIButtonState  _observed_state;    // The last state that we were asked for.
    unsigned int    _last_change;       // System time of last change in state.
    unsigned int    _last_polling;      // System time of last polling.
    MillisTimeout   _debounce_timeout;  // A timeout for optional software debounce.
    MillisTimeout   _lp_timer;          // A timeout for optional long-press tracking.

    bool _apply_debounce(const bool PIN_ACTIVE);
};

#endif // __UI_BUTTONS_H__
