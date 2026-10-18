/*
 * IMU on SPI5. See imu.h.
 *
 * SPI read: first byte is the register address with bit 7 set, the
 * register value comes back in the second byte. Mode 3 (CPOL=1, CPHA=1).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "imu.h"

#include <errno.h>
#include <stdint.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define REG_WHO_AM_I 0x0f
#define WHO_AM_I_LSM6DSOX 0x6c
#define SPI_READ 0x80

static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(
	DT_NODELABEL(imu),
	SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
		SPI_MODE_CPOL | SPI_MODE_CPHA,
	0);

static int read_reg(uint8_t reg, uint8_t *val)
{
	uint8_t tx[2] = {reg | SPI_READ, 0};
	uint8_t rx[2];
	const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
	const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
	const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
	int ret;

	ret = spi_transceive_dt(&spi, &tx_set, &rx_set);
	if (ret) {
		return ret;
	}
	*val = rx[1];
	return 0;
}

int imu_init(void)
{
	uint8_t id;
	int ret;

	if (!spi_is_ready_dt(&spi)) {
		printk("imu: spi5 not ready\n");
		return -ENODEV;
	}

	ret = read_reg(REG_WHO_AM_I, &id);
	if (ret) {
		printk("imu: spi read failed: %d\n", ret);
		return ret;
	}

	printk("imu: WHO_AM_I 0x%02x (expected 0x%02x)\n", id, WHO_AM_I_LSM6DSOX);
	return id == WHO_AM_I_LSM6DSOX ? 0 : -EIO;
}
