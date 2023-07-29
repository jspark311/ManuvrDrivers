/*
File:   GPIButton.cpp
Author: J. Ian Lindsay
Date:   2023.07.12

*/

#include "GPIButton.h"


GPIButton::GPIButton(
  const uint8_t CAST_BTN_ID,
  const uint8_t PIN,
  const GPIOMode GPIOMODE,
  const bool ACTIVE_LOW,
  const uint16_t DEBOUNCE_MS,
  const uint16_t LP_MS
) :
  ID(CAST_BTN_ID),
  _PIN(PIN), _PIN_MODE(GPIOMODE), _PIN_ACT_LOW(ACTIVE_LOW),
  _pin_setup(false), _debouncing(false),
  _long_pressing(false), _pin_filter(false),
  _prior_state(GPIButtonState::UNKNOWN),
  _debounced_state(GPIButtonState::UNKNOWN),
  _observed_state(GPIButtonState::UNKNOWN),
  _last_change(0), _last_polling(0),
  _debounce_timeout(DEBOUNCE_MS), _lp_timer(LP_MS) {}


GPIButton::~GPIButton() {
  deinit();
}



int8_t GPIButton::init() {
  int8_t ret = 0;
  if (!_pin_setup) {
    _pin_setup = (0 <= pinMode(_PIN, _PIN_MODE));
    if (_pin_setup) {
      // Call poll() to make a best effort at getting an initial state in a
      //   non-blocking way. But since the class is gated internally against
      //   initializing in a button-active state, this polling might not yield
      //   knowledge.
      poll();
    }
    else {
      ret = -1;
    }
  }
  return ret;
}


int8_t GPIButton::deinit() {
  int8_t ret = 0;   // Succeed by default
  if (_pin_setup) {
    pinMode(_PIN, GPIOMode::INPUT);
    _pin_setup       = false;
    _pin_filter      = false;
    _debouncing      = false;
    _long_pressing   = false;
    _prior_state     = GPIButtonState::UNKNOWN;
    _debounced_state = GPIButtonState::UNKNOWN;
    _observed_state  = GPIButtonState::UNKNOWN;
    //unsetPinFxn(_PIN);
  }
  return ret;
}


/**
* This function will be called from an ISR, and should be isomorphic with timer
*   service in idle time. That is: This function should be called from both the
*   pin-change ISR, as well as the periodic timer polling.
*
* @return 1 on state-change, 0 on no change, -1 on no init.
*/
int8_t GPIButton::poll() {
  const uint32_t NOW = millis();
  int8_t ret = -1;
  if (_pin_setup) {
    const bool CURRENT_PIN = (_PIN_ACT_LOW ^ readPin(_PIN));
    ret = 0;
    if (initialized()) {
      if (_apply_debounce(CURRENT_PIN)) {
        // Signal passed the debounce filter. Consider the existing debounced
        //   state, and decide if this represents an outward-facing change.
        // If this button is not long-press aware, all we need to do is set the
        //   state within our debounce window.
        bool state_changed = (pressed() != _pin_filter);
        GPIButtonState new_state = (_pin_filter ? GPIButtonState::PRESS : GPIButtonState::RELEASE);

        if (lpAware()) {                     // If this button tracks long-press...
          if (state_changed & _pin_filter) { // ...and a button became active...
            _lp_timer.reset();               // ...reset the long-press timer...
            _long_pressing = true;           // ...and begin observing it.
          }
          else if (!_pin_filter) {           // If the button was released...
            _long_pressing = false;          // ...stop tracking long-press.
          }
          else if (!_long_pressing) {        // The button was pressed. If we are
            _lp_timer.reset();               // not long-pressing, start doing so.
            _long_pressing = true;           // This is a guardrail for expired().
          }
          else if (_lp_timer.expired()) {    // Check to see if the button has
            _long_pressing = false;          // been held past threshold. Stop
            state_changed = true;            // tracking and mark state if so.
            new_state = GPIButtonState::LPRESS;
          }
        }

        if (state_changed) {
          _prior_state = _debounced_state;
          _debounced_state = new_state;  // Update the new stable outward-facing button state.
          _last_change = NOW;            // Update the change timestamp.
          ret = 1;                       // Indicate state shift in return.
        }
      }
    }
    else if (!CURRENT_PIN) {
      // If the class has an intialized pin, and no known state, it could only
      //   be because the button hasn't yet been seen to be inactive until now.
      // NOTE: By assigning states to these members, we suppress debouncing and
      //   notification of initial state.
      _prior_state     = _debounced_state;
      _debounced_state = GPIButtonState::RELEASE;
      _observed_state  = _debounced_state;
      _last_change = NOW;
      ret = 1;  // Indicate a shift of state.
    }
  }
  _last_polling = NOW;
  return ret;
}


/**
* Function makes an estimate about the amount of delay needed to answer any
*   timeouts that might be pending.
* If no polling is required, will return zero. This is distinguishable from
*   simply "being very close to the time when the next poll() is due" because
*   if that were actually true, this function would add 1 to the real number
*   so that a boundary condition in the polling timer would be assured that it
*   only needs to run once, in the worst case.
* User actions might obviate the need to poll the class. So the timer should
*   expect that the poll() function might not result in an evolution of state.
*
* @return The number of milliseconds that should elapse before next poll().
*/
uint32_t GPIButton::next_poll_delay() {
  // TODO: Exploitation of integer overflow is not very portable, since the
  //   compiler gets to decide what to do about it.
  // TODO: If you are going to keep it this way, at least make this function
  //   return unsigned int, rather than a width-specified integer. We are dealing
  //   with a system time value (which may be 64-bit).
  uint32_t ret = 0xFFFFFFFF;
  if (needsPolling()) {
    if (_debouncing) {      ret = strict_min(ret, _debounce_timeout.period());  }
    if (_long_pressing) {   ret = strict_min(ret, _lp_timer.period());  }
  }
  ret++;
  return ret;
}


/**
* This is a public accessor that gates its return according to changes since the
*   prior call to this function. This is an optional API nicity that allows the
*   client code to simply poll() a button followed by getFreshState() for
*   detection of state changes that merit attention.
* More sophisticated uses might call this from a thread.
*
* @return UNKNOWN if there is no fresh state, or else, the button state.
*/
GPIButtonState GPIButton::getFreshState() {
  GPIButtonState ret = GPIButtonState::UNKNOWN;
  if (_debounced_state != _observed_state) {
    _observed_state = _debounced_state;
    ret = _observed_state;
  }
  return ret;
}


/**
* This function applies the debounce logic to the pin and gates modification of
*   the recognized button state; adding a latency equal to the debounce time on
*   both sides of the button cycle.
*
* Rules:
*   1. There is no inter-press deadband. That would be a separate parameter.
*   2. This is the only place in the class where _pin_filter is changed.
*   3. If the debounce sample starts and stops on the same pin state when the
*        debounce timer is serviced, the state change is allowed to pass.
*
*             |-Debounce-|     |-Debounce-|       |-Debounce-| |-Debounce-|
* pin_state:  ┌────────────────┐                  ┌─┐┌─┐┌┐┌────┐
*          ───┘                └──────────────────┘ └┘ └┘└┘    └────────────────
* ButtonState:           ┌────────────────┐                  ┌────────────┐
*          ──────────────┘                └──────────────────┘            └─────
*
*
* @return true to allow state-change observation, false to inhibit it
*/
bool GPIButton::_apply_debounce(const bool PIN_ACTIVE) {
  bool ret = true;
  const bool PIN_CHANGED = (_pin_filter ^ PIN_ACTIVE);
  if (_debounce_timeout.period() > 0) {  // If we are using the debounce feature...
    ret = false;                         // ...change our default answer to inhibit.

    if (_debouncing) {
      const bool DEBOUNCE_EXPIRED = _debounce_timeout.expired();
      // We're already intra-debounce. Check for timer expiration, and note the
      //   state of the pin if it hasn't changed.
      if (DEBOUNCE_EXPIRED) {
        _debouncing = false;
        if (!PIN_CHANGED) {    // If the pin didn't change at the end of the
          ret = true;          //   debounce time, allow the state change.
        }
        else {                 // Otherwise, passively note the new state.
          _pin_filter = PIN_ACTIVE;
        }
      }
    }
    else if (PIN_CHANGED) {
      // We are not debouncing, but the pin state changed. Store the current pin
      //   state, begin debounce, and return an inhibit signal.
      // Presumably this is from an interrupt from a legitmate button press. If
      //   the state lasts longer than the debounce timer, we'll allow it when
      //   the timer calls next calls poll().
      _debounce_timeout.reset();
      _pin_filter = PIN_ACTIVE;
      _debouncing = true;
    }
    else if (PIN_ACTIVE) {
      // We aren't debouncing, and the pin didn't change, but is still active.
      // Do nothing, but allow state change so that long-press timers can be tested.
      ret = _long_pressing;
    }
  }

  if (ret) {
    // If the debounce filter is going to allow state change, be sure the state
    //   is accurate.
    _pin_filter = PIN_ACTIVE;
  }
  return ret;
}
