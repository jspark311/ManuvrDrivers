#
# ESP-IDF component
#
message(STATUS "==============================================")
message(STATUS "ManuvrDrivers")
message(STATUS "==============================================")

set(src_dirs)
list(APPEND src_dirs "src/AMG88xx" )
list(APPEND src_dirs "src/BME280" )
list(APPEND src_dirs "src/VEML6075" )
list(APPEND src_dirs "src/TSL2561")
list(APPEND src_dirs "src/LTC294x" )
list(APPEND src_dirs "src/BQ24155" )
#list(APPEND src_dirs "src/LSM9DS1" )
list(APPEND src_dirs "src/SX1503" )
list(APPEND src_dirs "src/SX8634")
list(APPEND src_dirs "src/SSD13xx" )
#list(APPEND src_dirs "src/SX127x" )
list(APPEND src_dirs "src/MCP4728" )
list(APPEND src_dirs "src/ADG2128" )
list(APPEND src_dirs "src/DS1881")
list(APPEND src_dirs "src/PAC195x" )
list(APPEND src_dirs "src/MCP23x17" )
list(APPEND src_dirs "src/TMP102" )
list(APPEND src_dirs "src/LSM6DSOX")
#list(APPEND src_dirs "src/Composites/ManuvrPMU" )
list(APPEND src_dirs "src/AudemeMOVI")
list(APPEND src_dirs "src/GPIOPatterns/GPIButton")

idf_component_register(
  SRC_DIRS "${src_dirs}"
  INCLUDE_DIRS "src"
  PRIV_INCLUDE_DIRS "."
  REQUIRES CppPotpourri
)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-ignored-qualifiers -Wno-implicit-fallthrough)
