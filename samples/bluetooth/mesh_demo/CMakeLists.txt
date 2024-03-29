# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
set(QEMU_EXTRA_FLAGS -s)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(mesh_demo)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_BOARD_BBC_MICROBIT app PRIVATE src/microbit.c)

if(NODE_ADDR)
  zephyr_compile_definitions(NODE_ADDR=${NODE_ADDR})
endif()

if (CONFIG_BUILD_WITH_TFM)
  target_include_directories(app PRIVATE
    $<TARGET_PROPERTY:tfm,TFM_BINARY_DIR>/api_ns/interface/include
  )
endif()
