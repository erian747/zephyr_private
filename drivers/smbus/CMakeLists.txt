# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/smbus.h)

zephyr_library()

zephyr_library_sources(smbus_utils.c)

zephyr_library_sources_ifdef(CONFIG_SMBUS_SHELL		smbus_shell.c)
zephyr_library_sources_ifdef(CONFIG_SMBUS_INTEL_PCH	intel_pch_smbus.c)
zephyr_library_sources_ifdef(CONFIG_SMBUS_STM32 smbus_stm32.c)

zephyr_library_sources_ifdef(CONFIG_USERSPACE		smbus_handlers.c)
