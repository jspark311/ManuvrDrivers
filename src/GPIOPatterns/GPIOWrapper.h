#ifndef __C3P_GPIO_WRAPPER_H__
#define __C3P_GPIO_WRAPPER_H__

#include "CppPotpourri.h"
#include "AbstractPlatform.h"

/**
* This is an optional helper class that wraps the basic GPIO API into an
*   instance of the GPIOWrapper abstraction. Mostly, this is useful for some
*   drivers that are not written against the assumption that the
*   globally-scoped (and platform provided) GPIO functions are the proper
*   pin-set to use.
*/
class PlatformGPIOShim : public GPIOWrapper {
public:
  inline int8_t pinMode(uint8_t pin, GPIOMode m) {  return ::pinMode(pin, m);   };
  inline int8_t setPin(uint8_t pin, bool val) {     return ::setPin(pin, val);  };
  inline int8_t readPin(uint8_t pin) {              return ::readPin(pin);      };
};


#endif  // __C3P_GPIO_WRAPPER_H__
