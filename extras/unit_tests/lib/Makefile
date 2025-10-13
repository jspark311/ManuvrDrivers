###########################################################################
# Makefile for LibC3P, as used by driver testing.
# Author: J. Ian Lindsay
#
# Builds the platform support, C3P, and drivers. Packages everything into
#   a single archive file for linking against the top-level application.
# Relies on exports from top-level Makefile.
###########################################################################
LIBRARY_NAME       = libC3P

###########################################################################
# Source file definitions...
###########################################################################
INCLUDES     = -iquote. -iquoteCppPotpourri/src -iquoteManuvrPlatforms/Linux

# C3P itself
SOURCES_CPP  = CppPotpourri/src/AbstractPlatform.cpp
SOURCES_CPP += CppPotpourri/src/AsyncSequencer.cpp
SOURCES_CPP += CppPotpourri/src/EnumeratedTypeCodes.cpp
SOURCES_CPP += CppPotpourri/src/CppPotpourri.cpp
SOURCES_CPP += CppPotpourri/src/Quaternion.cpp
SOURCES_CPP += CppPotpourri/src/Meta/Bikeshed/SensorWrapper.cpp
SOURCES_CPP += CppPotpourri/src/TimeSeries/TimeSeries.cpp
SOURCES_CPP += CppPotpourri/src/TimeSeries/SensorFilter.cpp
SOURCES_CPP += CppPotpourri/src/StringBuilder.cpp
SOURCES_CPP += CppPotpourri/src/Battery.cpp
SOURCES_CPP += CppPotpourri/src/uuid.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/BusQueue.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/I2CDevice.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/I2CBusOp.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/I2CAdapter.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/SPIAdapter.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/SPIBusOp.cpp
SOURCES_CPP += CppPotpourri/src/BusQueue/UARTAdapter.cpp
SOURCES_CPP += CppPotpourri/src/C3PRandom/C3PRandom.cpp
SOURCES_CPP += CppPotpourri/src/C3PRandom/PerlinNoise.cpp
SOURCES_CPP += CppPotpourri/src/M2MLink/M2MLink.cpp
SOURCES_CPP += CppPotpourri/src/M2MLink/M2MMsg.cpp
SOURCES_CPP += CppPotpourri/src/M2MLink/M2MMsgHdr.cpp
SOURCES_CPP += CppPotpourri/src/Identity/Identity.cpp
SOURCES_CPP += CppPotpourri/src/Identity/IdentityUUID.cpp
SOURCES_CPP += CppPotpourri/src/Storage/RecordTypes/ConfRecord.cpp
SOURCES_CPP += CppPotpourri/src/Storage/SimpleDataRecord.cpp
SOURCES_CPP += CppPotpourri/src/Console/C3PConsole.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/GPSWrapper/GPSWrapper.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/TripleAxisPipe/TripleAxisCompass.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/TripleAxisPipe/TripleAxisPipe.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/BufferAccepter.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/Base64/C3P_B64.cpp
SOURCES_CPP += CppPotpourri/src/Pipes/BufferAccepter/LineCoDec/LineCoDec.cpp
SOURCES_CPP += CppPotpourri/src/C3PValue/C3PType.cpp
SOURCES_CPP += CppPotpourri/src/C3PValue/C3PValue.cpp
SOURCES_CPP += CppPotpourri/src/C3PValue/KeyValuePair.cpp
SOURCES_CPP += CppPotpourri/src/Image/Image.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/ImageUtils.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/ImageGraph.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/ImgPerlinNoise.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/ImageScalers.cpp
SOURCES_CPP += CppPotpourri/src/Image/ImageUtils/BlobPlotter.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIMagnifier.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUI3AxisRender.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIBlobRender.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIButton.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIC3PScheduler.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIC3PValue.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIChecklist.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUI.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUICryptoBurrito.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIDataRecord.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIGraphing.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIGroup.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIIdentity.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIKeyValuePair.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIMagnifier.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIMLink.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIOptionsView.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUISensorFilter.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUISlider.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIStopWatch.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUIStorage.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUITabBar.cpp
SOURCES_CPP += CppPotpourri/src/Image/GfxUI/GfxUITextArea.cpp
SOURCES_CPP += CppPotpourri/src/TimerTools/StopWatch.cpp
SOURCES_CPP += CppPotpourri/src/TimerTools/C3PScheduler.cpp
SOURCES_CPP += CppPotpourri/src/TimerTools/C3PTrace.cpp
SOURCES_CPP += CppPotpourri/src/cbor-cpp/cbor.cpp

SOURCES_CPP += ManuvrPlatforms/Linux/src/Linux.cpp
SOURCES_CPP += ManuvrPlatforms/Linux/src/C3PLinuxFile.cpp
SOURCES_CPP += ManuvrPlatforms/Linux/src/LinuxStdIO.cpp
SOURCES_CPP += ManuvrPlatforms/Linux/src/LinuxI2C.cpp
SOURCES_CPP += ManuvrPlatforms/Linux/src/LinuxUART.cpp
#SOURCES_CPP += ManuvrPlatforms/Linux/src/SPIAdapter.cpp


###########################################################################
# exports, consolidation....
###########################################################################
OBJS  = $(SOURCES_C:.c=.o) $(SOURCES_CPP:.cpp=.o) $(SOURCES_S:.S=.o)
DEP_FILES = $(SOURCES_C:.c=.d) $(SOURCES_CPP:.cpp=.d) $(SOURCES_S:.S=.d)


###########################################################################
# Rules for building the firmware follow...
###########################################################################

.PHONY:  $(LIBRARY_NAME).a

all:  $(LIBRARY_NAME).a

%.o : %.cpp
	$(CXX) -std=$(CPP_STANDARD) $(CXXFLAGS) $(INCLUDES) -c -o $@ $^

%.o : %.c
	$(CC) -std=$(C_STANDARD) $(CFLAGS) $(INCLUDES) -c -o $@ $^

%.o : %.S
	$(CC) -std=$(C_STANDARD) -x assembler-with-cpp $(CFLAGS) $(INCLUDES) -c -o $@ $^

$(LIBRARY_NAME).a: $(OBJS)
	$(AR) rcs $(LIBRARY_NAME).a $(OBJS)

clean:
	rm -f *.d *.o *.su *~ $(OBJS) $(DEP_FILES) $(LIBRARY_NAME).a
