# ManuvrDrivers

A collection of non-blocking flexible hardware drivers written on top of CppPotpourri.


## This repo's relationship to CppPotpourri

[CppPotpourri](https://github.com/jspark311/CppPotpourri) was developed, in part, to allow asynchronous I/O and to ease the maintenance burden of drivers written to take advantage of it. CppPotpourri provides a platform agnostic collection of embedded-friendly libraries and abstract interfaces to hardware which covers nearly all of the basic stuff that a given driver will need (pin manipulation and interrupts, I2C/SPI/UART, RTCs, RNG, threading and delays). Thus, drivers that stay confined to the API provided by CppPotpourri will be hardware agnostic as well.

One major benefit to this abstraction is that drivers written under one environment can be easily ported to any other supported environment.


## Notes on platform support

In order to use this library, your project must provide the unimplemented functions in CppPotpourri's [AbstractPlatform header](https://github.com/jspark311/CppPotpourri/blob/master/src/AbstractPlatform.h). Some of these are provided by CppPotpourri itself, and your project may not use everything in the `AbstractPlatform` definition (depending on what drivers you include). The easiest way to determine what you need to implement is to write the first-draft of your project, and see what the linker complains about.

I maintain a collection of platform examples in the [ManuvrPlatforms](https://github.com/jspark311/ManuvrPlatforms) repo. But you are advised not to rely on that code in any way. If you _do_ decide to use it, my advice would be to hard-fork the parts you want into your own tree, and build it with the rest of your top-level code. If you are in Arduino, that would mean copying the relevant files into your sketch folder.

### Example projects that have extended CppPotpourri in this manner

### [MotherFlux0r](https://github.com/jspark311/Motherflux0r)

  * Based on the Teensy4/teensyduino library and automake
  * Makes extensive use of drivers in this repo

### [File-Librarian](https://github.com/jspark311/File-Librarian)

  * Linux console application
  * No hardware driver use, but does show how to extend the platform to Linux using automake.


## Which drivers are ready to use?

Drivers in the [ManuvrDrivers header file](https://github.com/jspark311/ManuvrDrivers/blob/master/src/ManuvrDrivers.h) are generally useful and won't break builds for platform-dependence reasons. That is, your project should be able to `#include <ManuvrDrivers.h>` and get all the support that is listed there. Your linker should cull any compiled drivers that your project doesn't depend upon (you _are_ using `-gc-sections`, right????). Drivers will be promoted to inclusion in this header when they meet the following criteria:

  * The driver operates asynchronously with regard to all I/O (it never blocks).
  * The driver has no reliance on any concrete platform features, unless it is cased-off in the preprocessor.

Drivers not included in ManuvrDrivers.h _might_ be worth using on a given platform, and might contain useful code for your own efforts. But they should not be expected to work.


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

Those drivers that are my original authorship are covered by the license below.

Copyright (c) 2019 J. Ian Lindsay

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

This permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
