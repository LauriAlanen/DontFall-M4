# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tilt_test)

# the filter itself comes straight from the firmware sources
target_include_directories(app PRIVATE ../../app/src)
target_sources(app PRIVATE src/main.c ../../app/src/tilt.c)
