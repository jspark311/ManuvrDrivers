# ManuvrDrivers

A collection of non-blocking flexible hardware drivers written on top of [CppPotpourri](https://github.com/jspark311/CppPotpourri).


## This repo's relationship to CppPotpourri

[CppPotpourri](https://github.com/jspark311/CppPotpourri) was developed, in part, to allow asynchronous I/O and to ease the maintenance burden of drivers written to take advantage of it. CppPotpourri provides a platform agnostic collection of embedded-friendly libraries and abstract interfaces to hardware which covers nearly all of the basic stuff that a given driver will need (pin manipulation and interrupts, I2C/SPI/UART, RTCs, RNG, threading and delays). Thus, drivers that stay confined to the API provided by CppPotpourri will be hardware agnostic as well.

One major benefit to this abstraction is that drivers written under one environment don't require porting for use under any other environment.

### Notes on platform support

In order to use this library, your project must provide the unimplemented functions in CppPotpourri's [AbstractPlatform header](https://github.com/jspark311/CppPotpourri/blob/master/src/AbstractPlatform.h). Some of these are provided by CppPotpourri itself, and your project may not use everything in the `AbstractPlatform` definition (depending on what drivers you include). The easiest way to determine what you need to implement is to write the first-draft of your project, and see what the linker complains about.

I maintain a collection of platform examples in the [ManuvrPlatforms](https://github.com/jspark311/ManuvrPlatforms) repo. But you are advised not to rely on that code in any way. If you _do_ decide to use it as it stands, my advice for each supported platform would be found in those README files.

#### Example projects that have extended CppPotpourri in this manner

##### [MotherFlux0r](https://github.com/jspark311/Motherflux0r)

  * Based on the Teensy4/teensyduino library and automake
  * Makes extensive use of drivers in this repo

##### [Calor Sentinam](https://github.com/jspark311/CalorSentinam)

  * Based on an ESP32 under the ESP-IDF environment
  * Uses many of the same drivers and application code as MotherFlux0r

##### [File-Librarian](https://github.com/jspark311/File-Librarian)

  * Linux console application
  * No hardware driver use, but does show how to extend the platform to Linux using automake.


## Which drivers are ready to use?

Drivers in the [ManuvrDrivers header file](https://github.com/jspark311/ManuvrDrivers/blob/master/src/ManuvrDrivers.h) are generally useful and won't break builds for platform-dependence reasons. That is, your project should be able to `#include <ManuvrDrivers.h>` and get all the support that is listed there. Your linker should cull any compiled drivers that your project doesn't depend upon (you _are_ using `-gc-sections`, right????). Drivers will be promoted to inclusion in this header when they meet the following criteria:

  * The driver operates asynchronously with regard to all I/O (it never blocks).
  * The driver has no reliance on any concrete platform features, unless it is cased-off in the preprocessor.

Drivers not included in `ManuvrDrivers.h` _might_ be worth using on a given platform, and might contain useful code for your own efforts. But they should not be expected to work.

### General philosophy of driver construction

I/O takes a long time. Priority #1 in all cases is _never block on I/O_. I know... It's hard to write drivers that way. You suddenly need tricks like finite state machines and concurrency-safe buffers and flags. But the pay off is being able to do lots more with far less in your main program.

Drivers should handle their own pin-related concerns, but should _never_ manage the bus adapters to which they are connected, nor should they make assumptions about what bus. This is often the first thing I fix in other people's drivers when I begin porting them: Remove all assumptions about `Wire`, and all calls that have the flavor `Wire.begin()`. That's the I2CAdapter's job. `BusAdapter`'s are typically passed in as a reference, either on construction, or as an argument to the driver's `init()` function. See the [BME280 driver](https://github.com/jspark311/ManuvrDrivers/blob/master/src/BME280/BME280.h) for an example of how a sensor can be supported on two different bus types (I2C or SPI).

Drivers should be able to cope as much as possible if the best-case resources are not provided. For example, some parts have a RESET pin, but that pin is not always connected to the MCU. If the driver wants to make RESET mandatory, it should not `init()` without it. Preferably, the driver would be written to cope with the absence of the pin by using either a software reset feature of the hardware (if supported), or clobbering the registers with the part's reset values as specified in the data sheet (as was done in [SX1503's reset() function](https://github.com/jspark311/ManuvrDrivers/blob/master/src/SX1503/SX1503.cpp#L193)). This adds program weight and complexity. But flash is cheap, and time spent re-writing drivers is not.

CppPotpourri is heap-heavy. The `BusAdapter` class offers a preallocation pool of ready-to-use jobs that drivers can pull from. But sometimes, it is faster and more flexible to hard-allocate private `BusOp` objects for I/O operations that are (commonly used) and/or (require special treatment). See the `_fb_data_op` member in the [SSD13xx driver](https://github.com/jspark311/ManuvrDrivers/blob/master/src/SSD13xx/SSD13xx.h#L199) for an example of such a case. The memory location of the private `SPIBusOp` allows the driver to recognize it among the torrent of other I/O operations, and quickly change the module's D/~C pin. It also allows the driver to not dispatch redundant framebuffer writes (which take a lot of time on the bus).

I/O can be minimized by keeping memory-resident copies of the hardware's registers, and then answering application calls against register accessors from the shadows, rather than invoking redundant I/O and blocking while waiting for the value to come back. Put differently: Drivers should cache the known states of their hardware, and (optionally) provide a callback mechanism and concurrency guards so that I/O in-progress (which might still fail) isn't mistaken for the current state of the hardware. See the [74HCT595 shift-register driver](https://github.com/jspark311/ManuvrDrivers/blob/master/src/ShiftRegister/ShiftRegister.h) for an example of this assurance being implemented.

#### Notes on behavior of destructors

The envisioned use-case for these drivers is to be instantiated once, and never destroyed. This has the benefit of keeping (usually unused) code out of the binary. The toolchain is generally not smart enough to know if a given instance will _actually need_ destructors. Nevertheless, in drivers that do heap allocation, it will (should) be free'd. Any case where heap is NOT free'd in a destructor is a mistake on my part. BusOp hygiene is not as good, and it is conceivable that a BusOp whose life-cycle outlasts that of the class that is assigned as its callback will precipitate a hard-fault if the driver class is torn down. If this untidiness hurts your use-case, I will gladly accept patches.


## Possible conflicts in Arduino from other repos

If you are using any of my atomized Arduino libraries for specific drivers, you might have namespace conflicts. The list of libraries follows:

  * [Arduino-ADG2128](https://github.com/jspark311/Arduino-ADG2128)
  * [Arduino-DS1881](https://github.com/jspark311/Arduino-DS1881)
  * [Arduino-SX150x](https://github.com/jspark311/Arduino-SX150x)
  * [Arduino-SX8634](https://github.com/jspark311/Arduino-SX8634)

My advice would be to abandon those synchronous drivers, and re-work your sketch to use the async version. The synchronous versions of these libraries were written more as demonstration pieces, and aren't maintained unless a major bug is found. Their async counterparts in this repo are much better by all standards except simplicity.


## Planned changes

### Cryptography hardware

It is the ultimate goal of CppPotpourri to facilitate integration of cryptographic code in a manner similar to how it abstracts hardware. But this is not ready yet, and will be provided by a separate repo. Until such time as that integration is complete, please don't try to enable the cryptographic features. It will only break the build and frustrate you.


## Licenses

Many of the drivers in this repo were ported from synchronous drivers written by others. The original author's commentary was preserved where appropriate, alongside my refactor notes. Such drivers inherit their original author's licensing terms. The [AMG88xx driver](https://github.com/jspark311/ManuvrDrivers/blob/master/src/AMG88xx/AMG88xx.h) is a good example of this.

Those drivers that are my original authorship are covered by the LICENSE file.
