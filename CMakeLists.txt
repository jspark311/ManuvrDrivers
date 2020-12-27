#
# ESP-IDF component
#
message(STATUS "==============================================")
message(STATUS "ManuvrDrivers")
message(STATUS "==============================================")

# idf_component_set_property(ManuvrDrivers COMPONENT_NAME "ManuvrDrivers")
# idf_component_set_property(ManuvrDrivers COMPONENT_TYPE LIBRARY)

file(GLOB_RECURSE SOURCE_FILES
  *.c
  *.cpp
)

idf_component_register(
  SRC_DIRS "src" "src/AMG88xx" "src/BME280" "src/VEML6075" "src/TSL2561"
    "src/LTC294x" "src/BQ24155" "src/LSM9DS1" "src/SX1503" "src/SX8634"
    "src/SSD13xx" "src/SX1276" "src/MCP4728" "src/ADG2128" "src/DS1881"
  INCLUDE_DIRS "src"
  REQUIRES CppPotpourri
)
