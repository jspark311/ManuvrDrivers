###########################################################################
# Makefile for driver tests
# Author: J. Ian Lindsay
###########################################################################
export FIRMWARE_NAME      = DriverTests
export PROGRAM_VERSION    = 0
export OPTIMIZATION       = -O0 -g --coverage -fno-omit-frame-pointer
export C_STANDARD         = gnu99
export CPP_STANDARD       = gnu++17

###########################################################################
# This is the build policy for the behavior of warnings in the library,
#   and should reflect the contractual assurances given at that level.
###########################################################################
WARNING_BEHAVIOR   = -Wall -Wextra              # Enable all non-ABI warnings.
WARNING_BEHAVIOR  += -Wno-unused-variable       # Sloppy, but not a risk.
WARNING_BEHAVIOR  += -Wno-unused-parameter      # Not a problem.
WARNING_BEHAVIOR  += -Wno-unused-value          # Sometimes things are volatile.
WARNING_BEHAVIOR  += -Wno-ignored-qualifiers    # We might want to ignore qualifiers.
WARNING_BEHAVIOR  += -Wno-implicit-fallthrough  # Code often reads better with this.

# The warnings below will be considered build-failing problems.
#WARNING_BEHAVIOR  += -Werror   # One day....
#WARNING_BEHAVIOR  += -Werror=sign-compare
#WARNING_BEHAVIOR  += -Werror=strict-aliasing          # Type-punning needs to be explicitly noted.
WARNING_BEHAVIOR  += -Werror=delete-non-virtual-dtor  # Unacceptable risk that is easy to close.
WARNING_BEHAVIOR  += -Werror=format-extra-args        # Something else is broken.
WARNING_BEHAVIOR  += -Werror=type-limits              # Exposes bad assumptions about type suitability.
WARNING_BEHAVIOR  += -Werror=parentheses              # Algebra and C aren't the same thing.


###########################################################################
# Environmental awareness...
###########################################################################
SHELL          = /bin/sh
export CC      = $(shell which gcc)
export CXX     = $(shell which g++)
export AR      = $(shell which ar)
export SZ      = $(shell which size)
export MAKE    = $(shell which make)
export GCOV    = $(shell which gcov)

BUILD_ROOT    = .
OUTPUT_PATH   = $(BUILD_ROOT)/build
COVERAGE_PATH = $(OUTPUT_PATH)/coverage
ANALYZER_PATH = $(OUTPUT_PATH)/static-analysis


###########################################################################
# Source files, includes, and linker directives...
###########################################################################
INCLUDES    = -I$(BUILD_ROOT)/../../src
INCLUDES   += -I$/lib/CppPotpourri/src
INCLUDES   += -I$/lib/ManuvrPlatforms/Linux

# Use the line below for discrete test executables.
# TODO: Find a way to agglomerate the coverage data from the running of several
#   executables. Until then, we run all the tests in a single translation unit.
#SOURCES_TESTS_CPP  = TAPTest.cpp GPIOTest.cpp PCA195xTest.cpp
SOURCES_TESTS_CPP  = AllTests.cpp


# C3P itself
SOURCES_CPP  = lib/CppPotpourri/src/AbstractPlatform.cpp
SOURCES_CPP += lib/CppPotpourri/src/AsyncSequencer.cpp
SOURCES_CPP += lib/CppPotpourri/src/EnumeratedTypeCodes.cpp
SOURCES_CPP += lib/CppPotpourri/src/CppPotpourri.cpp
SOURCES_CPP += lib/CppPotpourri/src/Quaternion.cpp
SOURCES_CPP += lib/CppPotpourri/src/Meta/Bikeshed/SensorWrapper.cpp
SOURCES_CPP += lib/CppPotpourri/src/TimeSeries/TimeSeries.cpp
SOURCES_CPP += lib/CppPotpourri/src/TimeSeries/SensorFilter.cpp
SOURCES_CPP += lib/CppPotpourri/src/StringBuilder.cpp
SOURCES_CPP += lib/CppPotpourri/src/Battery.cpp
SOURCES_CPP += lib/CppPotpourri/src/uuid.cpp
SOURCES_CPP += lib/CppPotpourri/src/BusQueue/BusQueue.cpp
SOURCES_CPP += lib/CppPotpourri/src/BusQueue/I2CDevice.cpp
SOURCES_CPP += lib/CppPotpourri/src/BusQueue/I2CBusOp.cpp
SOURCES_CPP += lib/CppPotpourri/src/BusQueue/I2CAdapter.cpp
#SOURCES_CPP += lib/CppPotpourri/src/BusQueue/SPIAdapter.cpp
#SOURCES_CPP += lib/CppPotpourri/src/BusQueue/SPIBusOp.cpp
SOURCES_CPP += lib/CppPotpourri/src/BusQueue/UARTAdapter.cpp
SOURCES_CPP += lib/CppPotpourri/src/C3PRandom/C3PRandom.cpp
SOURCES_CPP += lib/CppPotpourri/src/C3PRandom/PerlinNoise.cpp
SOURCES_CPP += lib/CppPotpourri/src/M2MLink/M2MLink.cpp
SOURCES_CPP += lib/CppPotpourri/src/M2MLink/M2MMsg.cpp
SOURCES_CPP += lib/CppPotpourri/src/M2MLink/M2MMsgHdr.cpp
SOURCES_CPP += lib/CppPotpourri/src/Identity/Identity.cpp
SOURCES_CPP += lib/CppPotpourri/src/Identity/IdentityUUID.cpp
SOURCES_CPP += lib/CppPotpourri/src/Storage/RecordTypes/ConfRecord.cpp
SOURCES_CPP += lib/CppPotpourri/src/Storage/DataRecord.cpp
SOURCES_CPP += lib/CppPotpourri/src/Storage/SimpleDataRecord.cpp
SOURCES_CPP += lib/CppPotpourri/src/Console/C3PConsole.cpp
SOURCES_CPP += lib/CppPotpourri/src/Pipes/BufferAccepter/GPSWrapper/GPSWrapper.cpp
SOURCES_CPP += lib/CppPotpourri/src/Pipes/TripleAxisPipe/TripleAxisCompass.cpp
SOURCES_CPP += lib/CppPotpourri/src/Pipes/TripleAxisPipe/TripleAxisPipe.cpp
SOURCES_CPP += lib/CppPotpourri/src/Pipes/BufferAccepter/BufferAccepter.cpp
SOURCES_CPP += lib/CppPotpourri/src/Pipes/BufferAccepter/Base64/C3P_B64.cpp
#SOURCES_CPP += lib/CppPotpourri/src/Pipes/BufferAccepter/LineCoDec/LineCoDec.cpp
SOURCES_CPP += lib/CppPotpourri/src/C3PValue/C3PType.cpp
SOURCES_CPP += lib/CppPotpourri/src/C3PValue/C3PValue.cpp
SOURCES_CPP += lib/CppPotpourri/src/C3PValue/KeyValuePair.cpp
SOURCES_CPP += lib/CppPotpourri/src/Image/Image.cpp
SOURCES_CPP += lib/CppPotpourri/src/TimerTools/StopWatch.cpp
SOURCES_CPP += lib/CppPotpourri/src/TimerTools/C3PScheduler.cpp
SOURCES_CPP += lib/CppPotpourri/src/TimerTools/C3PTrace.cpp
SOURCES_CPP += lib/CppPotpourri/src/cbor-cpp/cbor.cpp

SOURCES_CPP += lib/ManuvrPlatforms/Linux/src/Linux.cpp
SOURCES_CPP += lib/ManuvrPlatforms/Linux/src/C3PLinuxFile.cpp
SOURCES_CPP += lib/ManuvrPlatforms/Linux/src/LinuxStdIO.cpp
SOURCES_CPP += lib/ManuvrPlatforms/Linux/src/LinuxI2C.cpp
SOURCES_CPP += lib/ManuvrPlatforms/Linux/src/LinuxUART.cpp
#SOURCES_CPP += lib/ManuvrPlatforms/Linux/src/SPIAdapter.cpp



CXXFLAGS  = $(OPTIMIZATION)
CXXFLAGS += $(WARNING_BEHAVIOR) -D_GNU_SOURCE #-fprofile-dir=$(COVERAGE_PATH)
CXXFLAGS += -fsingle-precision-constant -Wdouble-promotion -fno-rtti -fno-exceptions
CXXFLAGS += -static
CXXFLAGS += -DCONFIG_C3P_CBOR
CXXFLAGS += -DCONFIG_C3P_IMG_SUPPORT
CXXFLAGS += -DCONFIG_C3P_TRACE_ENABLED
CXXFLAGS += -DCONFIG_C3P_I2C

LIBS  = -L. -lm -lpthread -lstdc++

###########################################################################
# Are we on a 64-bit system? If so, we'll need to specify
#   that we want a 32-bit build...
#
# TODO: we need to make both 32-bit and 64-bit builds. The library is used under
#   both conditions. It would be nice if this Makefile "just handled it".
#
# Thanks, estabroo...
# http://www.linuxquestions.org/questions/programming-9/how-can-make-makefile-detect-64-bit-os-679513/
###########################################################################
LBITS = $(shell getconf LONG_BIT)
ifeq ($(LBITS),64)
	# This is no longer required on 64-bit platforms. But it is being retained in
	#   case 32-bit problems need to be debugged.
  CXXFLAGS += -m32
endif


###########################################################################
# exports, consolidation....
###########################################################################
OBJS       = $(SOURCES_CPP:.cpp=.o)      # List of the object files to build.
DEPS       = $(SOURCES_CPP:.cpp=.d)      # List of the dependency files.
GCNO_FILES = $(OBJS:.o=.gcno)            # These are generated by coverage reporting.
GCDA_FILES = $(OBJS:.o=.gcda)            # These are generated by coverage reporting.
GCOV_FILES = $(OBJS:.o=.gcov)            # These are generated by coverage reporting.
TESTS      = $(SOURCES_TESTS_CPP:.cpp=)  # The list of discrete test binaries to build.

GCOV_FLAGS  = --demangled-names
GCOV_FLAGS += --object-directory $(COVERAGE_PATH)
#GCOV_FLAGS += --source-prefix $(COVERAGE_PATH)
#GCOV_FLAGS += --preserve-paths
GCOV_FLAGS += --branch-probabilities
GCOV_FLAGS += --branch-counts
#GCOV_FLAGS += --display-progress
GCOV_FLAGS += --function-summaries
GCOV_FLAGS += --relative-only

ANALYZER_FLAGS  = $(INCLUDES)
ANALYZER_FLAGS += --std=$(CPP_STANDARD) -j6 --cppcheck-build-dir=$(ANALYZER_PATH)
ANALYZER_FLAGS += --enable=all --report-progress --force

export CXXFLAGS

###########################################################################
# Parameter unification and make targets
###########################################################################

.PHONY: all

all: coverage check

%.o : %.cpp
	$(CXX) -std=$(CPP_STANDARD) $(INCLUDES) $(CXXFLAGS) -c -o $@ $^

%.o : %.c
	$(CC) -std=$(C_STANDARD) $(INCLUDES) $(CFLAGS) -c -o $(OUTPUT_PATH)/$(@F) $^

%.o : %.S
	$(CC) -std=$(C_STANDARD) -x assembler-with-cpp $(INCLUDES) $(CFLAGS) -c -o $(OUTPUT_PATH)/$(@F) $^

# Creates the output directory structure for all of the produced artifacts.
outputpath:
	mkdir -p $(OUTPUT_PATH)
	mkdir -p $(COVERAGE_PATH)
	mkdir -p $(ANALYZER_PATH)

# Build a static library for C3P and platform support.
lib: outputpath $(OBJS)
#	$(MAKE) -C lib

# 32-bit tests
alltests: lib
	@echo $(CXXFLAGS) $(INCLUDES) $(LIBS)
	@$(foreach test,$(TESTS),$(CXX) $(CXXFLAGS) $(INCLUDES) $(LIBS) -Wl,--gc-sections $(OBJS) $(test).cpp -o $(OUTPUT_PATH)/$(test);)
	@echo 'Built tests:  $(TESTS)'

# Run the test binaries and move their coverage files.
gencoverage: alltests
	@echo 'Beginning test execution...'
	@$(foreach test,$(TESTS),$(OUTPUT_PATH)/$(test) >> $(OUTPUT_PATH)/$(test).log;)
	# Coverage files are in an inconvenient location. Put them all into one path.
	-mv $(OUTPUT_PATH)/AllTests-AllTests.gcno $(COVERAGE_PATH)/AllTests.gcno 2> /dev/null
	-mv $(OUTPUT_PATH)/AllTests-AllTests.gcda $(COVERAGE_PATH)/AllTests.gcda 2> /dev/null
	-mv $(OUTPUT_PATH)/*.gcov $(GCOV_FILES) $(GCNO_FILES) $(GCDA_FILES) $(COVERAGE_PATH) 2> /dev/null
	@echo 'Unit testing complete.'

coverage: gencoverage
	$(GCOV) $(GCOV_FLAGS) $(SOURCES_CPP) AllTests.cpp
	lcov --capture --ignore-errors gcov --directory $(COVERAGE_PATH) --output-file $(COVERAGE_PATH)/coverage.info
	genhtml $(COVERAGE_PATH)/coverage.info --output-directory $(COVERAGE_PATH)
	@echo 'Coverage measurement complete.'

check: outputpath
	@echo 'Doing static analysis...'
	find ../../src -type f \( -name \*.cpp -o -name \*.c -o -name \*.h \) -exec wc -l {} + > $(OUTPUT_PATH)/line_counts.log
	cppcheck $(SOURCES_CPP) $(ANALYZER_FLAGS) > $(OUTPUT_PATH)/report.log
	@echo 'Static analysis complete.'

clean:
	rm -rf $(OUTPUT_PATH) $(DEPS) $(OBJS) $(GCNO_FILES) $(GCDA_FILES) *.gcno *.gcda *.gcov *.o *.a
