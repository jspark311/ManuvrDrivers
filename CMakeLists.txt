#
# ESP-IDF component
#
message(STATUS "==============================================")
message(STATUS "ManuvrDrivers")
message(STATUS "==============================================")

# idf_component_set_property(ManuvrDrivers COMPONENT_NAME "ManuvrDrivers")
# idf_component_set_property(ManuvrDrivers COMPONENT_TYPE LIBRARY)

idf_component_register(
  SRC_DIRS "src/AMG88xx" "src/BME280" "src/VEML6075" "src/TSL2561"
    "src/LTC294x" "src/BQ24155" "src/LSM9DS1" "src/SX1503" "src/SX8634"
    "src/SSD13xx" "src/SX127x" "src/MCP4728" "src/ADG2128" "src/DS1881"
    "src/PAC195x" "src/MCP23x17" "src/TMP102" "src/LSM6DSOX"
    "src/Composites/ManuvrPMU" "src/AudemeMOVI"
    "src/GPIOPatterns/GPIButton"
  INCLUDE_DIRS "src"
  REQUIRES CppPotpourri
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-ignored-qualifiers -Wno-implicit-fallthrough)
