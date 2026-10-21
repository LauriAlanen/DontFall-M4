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
    src/protocol.c
    src/heartbeat.c
    src/uros.c
    src/uros_transport.c
    src/imu.c
    src/imu_health.c
    src/tilt.c
)

# Code in RETRAM (overlay: retram node, linker region RETRAM). Only .text
# moves, its rodata stays in SRAM1. NOCOPY: remoteproc loads the code to
# its address, so no copy from a load address at boot.
# (m4-memory-plan step 2: a small test file proves the mechanism.)
zephyr_code_relocate(FILES src/protocol.c LOCATION RETRAM_TEXT NOCOPY)
