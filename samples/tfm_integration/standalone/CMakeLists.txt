cmake_minimum_required(VERSION 3.13.1)
# Copyright (c) 2017, NXP
#
# SPDX-License-Identifier: Apache-2.0
#

set(REMOTE_ZEPHYR_DIR ${CMAKE_CURRENT_BINARY_DIR}/ns_img-prefix/src/ns_img-build/zephyr)

if("${BOARD}" STREQUAL "lpcxpresso55s69_cpu0")
  set(BOARD_REMOTE "lpcxpresso55s69_ns")
else()
  message(FATAL_ERROR "${BOARD} was not supported for this sample")
endif()

message(INFO " ${BOARD} compile as Master in this sample")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tfm_sample)

enable_language(C ASM)

target_sources(app PRIVATE src/main_master.c)

include(ExternalProject)

ExternalProject_Add(
  ns_img
  SOURCE_DIR ${APPLICATION_SOURCE_DIR}/remote
  INSTALL_COMMAND ""      # This particular build system has no install command
  CMAKE_CACHE_ARGS -DBOARD:STRING=${BOARD_REMOTE}
  CMAKE_CACHE_ARGS -DDTC_OVERLAY_FILE:STRING=${DTC_OVERLAY_FILE}
  BUILD_BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/ns_img-prefix/src/ns_img-build/zephyr/zephyr.bin"
  BUILD_ALWAYS True
)

# add_dependencies(slave_core_inc_target ns_img)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
