/*
 * Copyright (c) 2025 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_IIO_IIO_BACKEND_H_
#define ZEPHYR_INCLUDE_IIO_IIO_BACKEND_H_

int iio_backend_read(const struct iio_dev *iio_dev, struct iio_dev_attr *attr,
				char *buf);

int iio_backend_write(const struct iio_dev *iio_dev, struct iio_dev_attr *attr,
				const char *buf, size_t count);

#endif /* ZEPHYR_INCLUDE_IIO_IIO_BACKEND_H_ */