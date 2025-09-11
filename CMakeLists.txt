# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/boards)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.0.0)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
include(${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# ====================================================================================
set(PICO_BOARD adcs CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(${CMAKE_CURRENT_LIST_DIR}/pico-sdk/pico_sdk_init.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(samwise-adcs C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

file(GLOB top CONFIGURE_DEPENDS "src/*.cpp")
file(GLOB gnc CONFIGURE_DEPENDS "src/gnc/*.cpp")
file(GLOB gnc_estimation CONFIGURE_DEPENDS "src/gnc/estimation/*.cpp")
file(GLOB gnc_control CONFIGURE_DEPENDS "src/gnc/control/*.cpp")
file(GLOB gnc_estimation CONFIGURE_DEPENDS "src/gnc/estimation/*.cpp")
file(GLOB gnc_models CONFIGURE_DEPENDS "src/gnc/models/*.cpp")
file(GLOB gnc_utils CONFIGURE_DEPENDS "src/gnc/utils/*.cpp")
file(GLOB gnc_world CONFIGURE_DEPENDS "src/gnc/world/*.cpp")
file(GLOB drivers CONFIGURE_DEPENDS "src/drivers/*.cpp")
file(GLOB drivers_power_monitor CONFIGURE_DEPENDS "src/drivers/power_monitor/*.cpp")
file(GLOB drivers_gps CONFIGURE_DEPENDS "src/drivers/gps/*.cpp")
file(GLOB drivers_imu CONFIGURE_DEPENDS "src/drivers/imu/*.cpp")
file(GLOB drivers_magnetometer CONFIGURE_DEPENDS "src/drivers/magnetometer/*.cpp")
file(GLOB drivers_magnetorquers CONFIGURE_DEPENDS "src/drivers/magnetorquers/*.cpp")
file(GLOB drivers_neopixel CONFIGURE_DEPENDS "src/drivers/neopixel/*.cpp")
file(GLOB drivers_sun_sensors CONFIGURE_DEPENDS "src/drivers/sun_sensors/*.cpp")
file(GLOB drivers_picubed CONFIGURE_DEPENDS "src/drivers/picubed/*.cpp")
file(GLOB drivers_watchdog CONFIGURE_DEPENDS "src/drivers/watchdog/*.cpp")
file(GLOB drivers_external CONFIGURE_DEPENDS "src/drivers/external/*.[c|cpp]")
file(GLOB scheduler CONFIGURE_DEPENDS "src/scheduler/*.cpp")
file(GLOB states CONFIGURE_DEPENDS "src/states/*.cpp")
file(GLOB tasks CONFIGURE_DEPENDS "src/tasks/*.cpp")
file(GLOB tests CONFIGURE_DEPENDS "src/tests/*.cpp")

add_executable(samwise-adcs ${top} ${gnc} ${gnc_estimation} ${gnc_control} ${gnc_models} ${gnc_utils} ${drivers} ${drivers_power_monitor} ${drivers_gps} ${drivers_imu} ${drivers_magnetometer} ${drivers_magnetorquers} ${drivers_neopixel} ${drivers_sun_sensors} ${drivers_picubed} ${drivers_watchdog} ${drivers_external} ${scheduler} ${states} ${tasks} ${tests})

pico_set_program_name(samwise-adcs "samwise_adcs")
pico_set_program_version(samwise-adcs "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(samwise-adcs 0)
pico_enable_stdio_usb(samwise-adcs 1)

# Add the standard library to the build
target_link_libraries(samwise-adcs
  hardware_i2c
  hardware_spi
  hardware_uart
  hardware_flash
  hardware_pwm
  hardware_adc
  pico_stdlib 
  hardware_pio
)

# Add the standard include files to the build
target_include_directories(samwise-adcs PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

target_include_directories(samwise-adcs PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

pico_add_extra_outputs(samwise-adcs)