# SPDX-License-Identifier: Apache-2.0

if (CONFIG_SOC_ESP32_APPCPU)
  zephyr_sources(soc_appcpu.c)
else()
  zephyr_sources(
    soc.c
    loader.c
    esp32-mp.c
    )
endif()

zephyr_library_sources_ifdef(CONFIG_NEWLIB_LIBC newlib_fix.c)

zephyr_library_sources_ifdef(CONFIG_GDBSTUB gdbstub.c)

zephyr_library_sources_ifdef(CONFIG_PM power.c)
zephyr_library_sources_ifdef(CONFIG_POWEROFF poweroff.c)

# get flash size to use in esptool as string
math(EXPR esptoolpy_flashsize "${CONFIG_FLASH_SIZE} / 0x100000")

if(CONFIG_BOOTLOADER_ESP_IDF)
  include(ExternalProject)

  ## we use hello-world project, but I think any can be used.
  set(espidf_components_dir   ${ESP_IDF_PATH}/components)
  set(espidf_prefix    ${CMAKE_BINARY_DIR}/esp-idf)
  set(espidf_build_dir ${espidf_prefix}/build)

  ExternalProject_Add(
    EspIdfBootloader
    PREFIX ${espidf_prefix}
    SOURCE_DIR ${espidf_components_dir}/bootloader/subproject
    BINARY_DIR ${espidf_build_dir}/bootloader
    CONFIGURE_COMMAND
    ${CMAKE_COMMAND} -G${CMAKE_GENERATOR}
    -S ${espidf_components_dir}/bootloader/subproject
    -B ${espidf_build_dir}/bootloader -DSDKCONFIG=${espidf_build_dir}/sdkconfig
    -DIDF_PATH=${ESP_IDF_PATH} -DIDF_TARGET=${CONFIG_SOC_SERIES}
    -DPYTHON_DEPS_CHECKED=1
    -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_ASM_COMPILER=${CMAKE_ASM_COMPILER}
    -DCMAKE_SYSTEM_NAME=${CMAKE_SYSTEM_NAME}
    -DPYTHON=${PYTHON_EXECUTABLE}
    BUILD_COMMAND
    ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ""      # This particular build system has no install command
    )

  ExternalProject_Add(
    EspPartitionTable
    SOURCE_DIR ${espidf_components_dir}/partition_table
    BINARY_DIR ${espidf_build_dir}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND
    ${PYTHON_EXECUTABLE} ${ESP_IDF_PATH}/components/partition_table/gen_esp32part.py -q
    --offset 0x8000 --flash-size ${esptoolpy_flashsize}MB ${ESP_IDF_PATH}/components/partition_table/partitions_singleapp.csv ${espidf_build_dir}/partitions_singleapp.bin
    INSTALL_COMMAND ""
    )

  set_property(TARGET bintools PROPERTY disassembly_flag_inline_source)

  add_dependencies(app EspIdfBootloader EspPartitionTable)

  board_finalize_runner_args(esp32 "--esp-flash-bootloader=${espidf_build_dir}/bootloader/bootloader.bin")

  board_finalize_runner_args(esp32 "--esp-flash-partition_table=${espidf_build_dir}/partitions_singleapp.bin")

  board_finalize_runner_args(esp32 "--esp-partition-table-address=0x8000")

endif()

if(CONFIG_MCUBOOT OR CONFIG_BOOTLOADER_ESP_IDF)

  if(CONFIG_BUILD_OUTPUT_BIN)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${PYTHON_EXECUTABLE} ${ESP_IDF_PATH}/components/esptool_py/esptool/esptool.py
      ARGS --chip esp32 elf2image --flash_mode dio --flash_freq 40m --flash_size ${esptoolpy_flashsize}MB
      -o ${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.bin
      ${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.elf)
  endif()

  if(CONFIG_MCUBOOT)
    board_finalize_runner_args(esp32 "--esp-flash-bootloader=${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.bin")
  endif()

endif()

## When building for APPCPU
if (CONFIG_SOC_ESP32_APPCPU)

  if(CONFIG_BUILD_OUTPUT_BIN)
    set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
      COMMAND ${PYTHON_EXECUTABLE} ${ESP_IDF_PATH}/tools/esp_bin2c_array.py
      ARGS -i ${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.bin
      -o ${CMAKE_BINARY_DIR}/zephyr/${CONFIG_KERNEL_BIN_NAME}.c
      -a "esp32_appcpu_fw_array")
  endif()

else()
  set_property(TARGET bintools PROPERTY disassembly_flag_inline_source)

  # get code-partition slot0 address
  dt_nodelabel(dts_partition_path NODELABEL "slot0_partition")
  dt_reg_addr(img_0_off PATH ${dts_partition_path})

  # get code-partition boot address
  dt_nodelabel(dts_partition_path NODELABEL "boot_partition")
  dt_reg_addr(boot_off PATH ${dts_partition_path})

  board_finalize_runner_args(esp32 "--esp-boot-address=${boot_off}")
  board_finalize_runner_args(esp32 "--esp-app-address=${img_0_off}")
endif()

if(CONFIG_MCUBOOT)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/mcuboot.ld CACHE INTERNAL "")
elseif(CONFIG_SOC_ESP32_APPCPU)
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/default_appcpu.ld CACHE INTERNAL "")
else()
  set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/default.ld CACHE INTERNAL "")
endif()
