cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Add the compile-time define globally (affects all components)
add_compile_definitions(WOKWI_SIM=1)

project(heater_control)