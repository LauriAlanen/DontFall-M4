# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(balance_test)

# the simulated robot is test code, it never goes into the firmware
target_sources(app PRIVATE src/main.c src/plant.c)

# -DBALANCE_CSV=1 prints the runs for plot.py (off by default)
if(BALANCE_CSV)
  target_compile_definitions(app PRIVATE BALANCE_CSV)
endif()
