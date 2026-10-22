# SPDX-License-Identifier: Apache-2.0
# Derived from zephyr/samples/subsys/ipc/openamp_rsc_table.
cmake_minimum_required(VERSION 3.20.0)

# micro-ROS (west.yml). Its Zephyr module lives in a subdirectory, so west
# doesn't find it on its own. It builds libmicroros from the sources pinned
# in micro_ros.yml.
list(APPEND ZEPHYR_EXTRA_MODULES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/micro_ros_zephyr_module/modules/libmicroros)
set(MICROROS_REPOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../modules/lib/micro_ros)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(dontfall_m4)

target_include_directories(app PRIVATE ${LIBMETAL_INCLUDE_DIR} ${OPENAMP_INCLUDE_DIR} ${PLATFORM_DIR})
target_sources(app PRIVATE
    src/main.c
    src/ipc.c
    src/link_led.c
    src/uros.c
    src/uros_transport.c
    src/imu.c
    src/imu_health.c
    src/tilt.c
    src/supervisor.c
    src/control.c
    src/diag.c
    src/wdog.c
)

# libmicroros's code and constants in RETRAM (overlay: retram node, linker
# region RETRAM), our own code stays in SRAM1. See retram.ld.
zephyr_linker_sources(PRE_TEXT_SECTIONS retram.ld)
