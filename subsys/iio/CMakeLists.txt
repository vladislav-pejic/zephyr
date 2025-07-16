# Copyright (c) 2025 Analog Devices Inc.
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_IIO)
	zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/iio/iio.h)

	zephyr_library()
	zephyr_library_sources(iio_hal.c)
	zephyr_include_directories(${ZEPHYR_BASE}/subsys/iio)
endif()
