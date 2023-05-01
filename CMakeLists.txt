cmake_minimum_required(VERSION 3.17.5 FATAL_ERROR)
project(uf-ipmc VERSION 0.3.0)

find_package(x2o 0.3.0 REQUIRED)
find_package(swatch 1.5.3 REQUIRED)

configure_file(CONFIG/CONFIG.toml CONFIG/CONFIG.toml)
configure_file(ipmc.service ipmc.service)

set(BOARD "VU13P-QSFPDD")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_COMPILER /usr/bin/gcc)
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS "-lrt")
set(CMAKE_CXX_FLAGS "-lrt")

if(${BOARD} STREQUAL "KU15P")
  message("-- BOARD = ${BOARD}")
  set(SRCS "SRC/picmg.c"
           "SRC/ipmc.c"
           "SRC/fan.c"
           "SRC/debug.c"
           "SRC/strings.c"
           "SRC/ipmi.c"
           "SRC/i2c.c"
           "SRC/timer.c"
           "SRC/event.c"
           "SRC/ws.c"
           "SRC/sensor.c"
           "SRC/main.c"
           "SRC/logger.c"
           "SRC/i2c-sensor.c"
           "SRC/toml.c"
           "SRC/user-sensor.c"
           "SRC/semaphore.c"
           "SRC/user-payload.c")
  set(HDRS "SRC/ipmi.h"
           "SRC/i2c.h"
           "SRC/picmg.h"
           "SRC/event.h"
           "SRC/sensor.h"
           "SRC/ws.h"
           "SRC/timer.h"
           "SRC/debug.h"
           "SRC/fan.h"
           "SRC/ipmc.h"
           "SRC/error.h"
           "SRC/strings.h"
           "SRC/logger.h"
           "SRC/i2c-sensor.h"
           "SRC/toml.h"
           "SRC/user-sensor.h"
           "SRC/semaphore.h"
           "SRC/user-payload.h")
elseif(${BOARD} STREQUAL "VU13P-QSFPDD")
  message("-- BOARD = ${BOARD}")
  set(SRCS "SRC/picmg.c"
           "SRC/ipmc.c"
           "SRC/fan.c"
           "SRC/debug.c"
           "SRC/strings.c"
           "SRC/ipmi.c"
           "SRC/i2c.c"
           "SRC/timer.c"
           "SRC/event.c"
           "SRC/ws.c"
           "SRC/sensor.c"
           "SRC/main.c"
           "SRC/logger.c"
           "SRC/i2c-sensor.c"
           "SRC/toml.c"
           "SRC/user-sensor-octopus-qsfpdd.c"
           "SRC/semaphore.c"
           "SRC/user-payload-octopus-qsfpdd.c"
           "SRC/octopus.c")
  set(HDRS "SRC/ipmi.h"
           "SRC/i2c.h"
           "SRC/picmg.h"
           "SRC/event.h"
           "SRC/sensor.h"
           "SRC/ws.h"
           "SRC/timer.h"
           "SRC/debug.h"
           "SRC/fan.h"
           "SRC/ipmc.h"
           "SRC/error.h"
           "SRC/strings.h"
           "SRC/logger.h"
           "SRC/i2c-sensor.h"
           "SRC/toml.h"
           "SRC/user-sensor-octopus-qsfpdd.h"
           "SRC/semaphore.h"
           "SRC/user-payload-octopus-qsfpdd.h"
           "SRC/octopus.h")
elseif(${BOARD} STREQUAL "VU13P-QSFP")
  message("-- BOARD = ${BOARD}")
  set(SRCS "SRC/picmg.c"
           "SRC/ipmc.c"
           "SRC/fan.c"
           "SRC/debug.c"
           "SRC/strings.c"
           "SRC/ipmi.c"
           "SRC/i2c.c"
           "SRC/timer.c"
           "SRC/event.c"
           "SRC/ws.c"
           "SRC/sensor.c"
           "SRC/main.c"
           "SRC/logger.c"
           "SRC/i2c-sensor.c"
           "SRC/toml.c"
           "SRC/user-sensor-octopus-qsfp.c"
           "SRC/semaphore.c"
           "SRC/user-payload-octopus-qsfp.c"
           "SRC/octopus.c")
  set(HDRS "SRC/ipmi.h"
           "SRC/i2c.h"
           "SRC/picmg.h"
           "SRC/event.h"
           "SRC/sensor.h"
           "SRC/ws.h"
           "SRC/timer.h"
           "SRC/debug.h"
           "SRC/fan.h"
           "SRC/ipmc.h"
           "SRC/error.h"
           "SRC/strings.h"
           "SRC/logger.h"
           "SRC/i2c-sensor.h"
           "SRC/toml.h"
           "SRC/user-sensor-octopus-qsfp.h"
           "SRC/semaphore.h"
           "SRC/user-payload-octopus-qsfp.h"
           "SRC/octopus.h")
elseif(${BOARD} STREQUAL "VU13P-QSFP30")
  message("-- BOARD = ${BOARD}")
  set(SRCS "SRC/picmg.c"
           "SRC/ipmc.c"
           "SRC/fan.c"
           "SRC/debug.c"
           "SRC/strings.c"
           "SRC/ipmi.c"
           "SRC/i2c.c"
           "SRC/timer.c"
           "SRC/event.c"
           "SRC/ws.c"
           "SRC/sensor.c"
           "SRC/main.c"
           "SRC/logger.c"
           "SRC/i2c-sensor.c"
           "SRC/toml.c"
           "SRC/user-sensor-vu13p-qsfp30.c"
           "SRC/semaphore.c"
           "SRC/user-payload-vu13p-qsfp30.c"
           "SRC/octopus.c")
  set(HDRS "SRC/ipmi.h"
           "SRC/i2c.h"
           "SRC/picmg.h"
           "SRC/event.h"
           "SRC/sensor.h"
           "SRC/ws.h"
           "SRC/timer.h"
           "SRC/debug.h"
           "SRC/fan.h"
           "SRC/ipmc.h"
           "SRC/error.h"
           "SRC/strings.h"
           "SRC/logger.h"
           "SRC/i2c-sensor.h"
           "SRC/toml.h"
           "SRC/user-sensor-vu13p-qsfp30.h"
           "SRC/semaphore.h"
           "SRC/user-payload-vu13p-qsfp30.h"
           "SRC/octopus.h")
elseif(${BOARD} STREQUAL "VU13P-QSFP30-REV3")
  message("-- BOARD = ${BOARD}")
  set(SRCS "SRC/picmg.c"
           "SRC/ipmc.c"
           "SRC/fan.c"
           "SRC/debug.c"
           "SRC/strings.c"
           "SRC/ipmi.c"
           "SRC/i2c.c"
           "SRC/timer.c"
           "SRC/event.c"
           "SRC/ws.c"
           "SRC/sensor.c"
           "SRC/main.c"
           "SRC/logger.c"
           "SRC/i2c-sensor.c"
           "SRC/toml.c"
           "SRC/user-sensor-vu13p-qsfp30-rev3.cpp"
           "SRC/semaphore.c"
           "SRC/user-payload-vu13p-qsfp30-rev3.cpp")
  set(HDRS "SRC/ipmi.h"
           "SRC/i2c.h"
           "SRC/picmg.h"
           "SRC/event.h"
           "SRC/sensor.h"
           "SRC/ws.h"
           "SRC/timer.h"
           "SRC/debug.h"
           "SRC/fan.h"
           "SRC/ipmc.h"
           "SRC/error.h"
           "SRC/strings.h"
           "SRC/logger.h"
           "SRC/i2c-sensor.h"
           "SRC/toml.h"
           "SRC/user-sensor-vu13p-qsfp30-rev3.h"
           "SRC/semaphore.h"
           "SRC/user-payload-vu13p-qsfp30-rev3.h")
endif()

add_executable(IPMC_EXEC ${SRCS} ${HDRS})
add_library(uf_ipmc SHARED ${SRCS} ${HDRS})

#target_link_libraries(${PROJECT_NAME} PUBLIC x2o)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/CONFIG/CONFIG.toml DESTINATION /usr/local/share/)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/ipmc.service DESTINATION /usr/lib/systemd/system/)

include(${swatch_CMAKE_DIR}/../cactus-utilities.cmake)
set(PLUGIN_CONFIG_FILE herd.yml)
cactus_add_phase2_plugin_rules(
    UF_IPMC
    uf_ipmc
    "Boost 1.53.0 REQUIRED COMPONENTS chrono filesystem regex thread unit_test_framework"
)
