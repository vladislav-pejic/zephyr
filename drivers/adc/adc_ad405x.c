/*
 * Copyright
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(adc_ad405x, CONFIG_ADC_LOG_LEVEL);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define DT_DRV_COMPAT adi_ad405x_adc

#define GPIO_PORT "GPIO2"
#define GPIO_PIN 7
#define GPIO_FLAGS (GPIO_OUTPUT | GPIO_ACTIVE_LOW)

#define AD4050_CHIP_ID		0
#define AD4052_CHIP_ID		1

#define AD4050_ADC_RESOLUTION 12U
#define AD4052_ADC_RESOLUTION 16U

#define AD405X_REG_INTERFACE_CONFIG_A                 0x00U
#define AD405X_REG_INTERFACE_CONFIG_B                 0x01U
#define AD405X_REG_DEVICE_CONFIG                      0x02U
#define AD405X_REG_DEVICE_TYPE                        0x03U
#define AD405X_REG_PRODUCT_ID_L                       0x04U
#define AD405X_REG_PRODUCT_ID_H                       0x05U
#define AD405X_REG_DEVICE_GRADE                       0x06U
#define AD405X_REG_SCRATCH_PAD                        0x0AU
#define AD405X_REG_VENDOR_L                           0x0CU
#define AD405X_REG_VENDOR_H                           0x0DU
#define AD405X_REG_MODE_SET                           0x20U
#define AD405X_REG_ADC_MODES                          0x21U
#define AD405X_REG_GP_PIN_CONF                        0x24U

#define AD405X_REG_INTERFACE_CONFIG_A_VAL             0x10U
#define AD405X_REG_INTERFACE_CONFIG_B_VAL             0x08U
#define AD405X_REG_DEVICE_CONFIG_VAL                  0xF0U
#define AD405X_REG_DEVICE_TYPE_VAL                    0x07U
#define AD405X_REG_PRODUCT_ID_L_VAL                   0x70U
#define AD405X_REG_PRODUCT_ID_H_VAL                   0x00U
#define AD405X_REG_PRODUCT_ID_VAL                   0x0072U
#define AD405X_REG_DEVICE_GRADE_VAL                   0x00U
#define AD405X_REG_VENDOR_L_VAL                       0x56U
#define AD405X_REG_VENDOR_H_VAL                       0x04U
#define AD405X_REG_VENDOR_VAL                       0x0456U
#define AD405X_REG_INTERFACE_CONFIG_A_RESET_VAL       0x81U
#define AD405X_REG_INTERFACE_CONFIG_15BIT_ADDR_VAL    0x00U
#define AD405X_REG_INTERFACE_CONFIG_7BIT_ADDR_VAL     0x01U
#define AD405X_REG_DEVICE_CONFIG_ACTIVE_MODE_VAL      0x00U
#define AD405X_ENTER_ADC_MODE_VAL                     0x01U

/* AD405X_STATUS_WORD */
#define AD405X_STATUS_WORD_MIN_FLAG_MSK        BIT(0)
//#define AD405X_INT_MAP_OVERRUN_MODE(x)    (((x) & 0x1) << 0)
#define AD405X_STATUS_WORD_MAX_FLAG_MSK        BIT(1)
//#define AD405X_INT_MAP_WATERMARK_MODE(x)  (((x) & 0x1) << 1)
#define AD405X_STATUS_WORD_THRESH_OVERRUN_MSK  BIT(2)
//#define AD405X_INT_MAP_FREE_FALL_MODE(x)  (((x) & 0x1) << 2)
#define AD405X_STATUS_WORD_FUSE_CRC_ERR_MSK    BIT(5)
//#define AD405X_INT_MAP_INACT_MODE(x)      (((x) & 0x1) << 3)
#define AD405X_STATUS_WORD_INTERFACE_ERR_MSK   BIT(6)
//#define AD405X_INT_MAP_ACT_MODE(x)        (((x) & 0x1) << 4)
#define AD405X_STATUS_WORD_DEVICE_READY_MSK    BIT(7)
//#define AD405X_INT_MAP_DOUBLE_TAP_MODE(x) (((x) & 0x1) << 5)
#define AD405X_STATUS_WORD_ADDR_INVALID_MSK    BIT(8)
//#define AD405X_INT_MAP_SINGLE_TAP_MODE(x) (((x) & 0x1) << 6)
#define AD405X_STATUS_WORD_MB_ERR_MSK          BIT(9)
#define AD405X_STATUS_WORD_WR_INVALID_MSK      BIT(10)
#define AD405X_STATUS_WORD_SPI_CRC_ERR_MSK     BIT(11)
#define AD405X_STATUS_WORD_SCLK_ERR_MSK        BIT(12)
#define AD405X_STATUS_WORD_NOT_RDY_ERR_MSK     BIT(15)

/** AD405X_REG_ADC_MODES Bit definition */
#define AD405X_ADC_MODES_MSK			GENMASK(2, 0)
#define AD405X_BURST_AVERAGING_MODE		BIT(0)
#define AD405X_AVERAGING_MODE      		BIT(1)
#define AD405X_AUTONOMOUS_MODE			GENMASK(1, 0)

/** AD405X_REG_MODE_SET Bit definition */
#define AD405X_ENTER_ADC_MODE_MSK		BIT(0)
#define AD405X_ENTER_ADC_MODE     		BIT(0)
#define AD405X_ENTER_SLEEP_MODE     	BIT(1) | BIT(0)
#define AD405X_ENTER_ACTIVE_MODE		0x0U

#define AD405X_ADC_DATA_FORMAT_MSK		BIT(7)

#define AD405X_WRITE_CMD  0x0U
#define AD405X_READ_CMD   0x80U

#define AD405X_DIFFERENTIAL_BIT 0x7U

#define AD405X_ADC_RESOLUTION 16U
#define AD405X_SW_RESET_MSK			BIT(7) | BIT(0)

#define AD405X_GP1_MODE_MSK			GENMASK(6, 4)
#define AD405X_GP0_MODE_MSK			GENMASK(2, 0)
#define AD405X_GP1					0x1U
#define AD405X_GP0					0x0U

#define AD405X_DEV_EN_POL_HIGH					0x1U
#define AD405X_DEV_EN_POL_LOW					0x0U

union ad405x_bus {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif
};

enum ad405x_gpx_mode {
	AD405X_DISABLED = 0,
	AD405X_GP0_1_INTR = 1,
	AD405X_DATA_READY = 2,
	AD405X_DEV_ENABLE = 3,
	AD405X_CHOP = 4,
	AD405X_LOGIC_LOW = 5,
	AD405X_LOGIC_HIGH = 6,
	AD405X_DEV_READY = 7
};

/** AD405X modes of operations */
enum ad405x_operation_mode {
	AD405X_SAMPLE_MODE_OP = 0,
	AD405X_BURST_AVERAGING_MODE_OP = 1,
	AD405X_AVERAGING_MODE_OP = 2,
	AD405X_MONITOR_AUTO_MODE_OP = 3,
	AD405X_CONFIG_MODE_OP = 4,
	AD405X_SLEEP_MODE_OP = 5,
	AD405X_TRIGGER_AUTO_MODE_OP = 7
};

typedef bool (*ad405x_bus_is_ready_fn)(const union ad405x_bus *bus);
typedef int (*aad405x_reg_access_fn)(const struct device *dev, uint8_t cmd,
				     uint8_t reg_addr, uint8_t *data, size_t length);

struct adc_ad405x_config {
	const union ad405x_bus bus;
	ad405x_bus_is_ready_fn bus_is_ready;
	aad405x_reg_access_fn reg_access;
	enum ad405x_operation_mode active_mode;
#ifdef CONFIG_AD405X_TRIGGER
	struct gpio_dt_spec gp1_interrupt;
	struct gpio_dt_spec gp0_interrupt;
#endif
	struct gpio_dt_spec conversion;
	uint8_t chip_id;
	uint16_t resolution;
};

struct adc_ad405x_data {
	struct adc_context ctx;
	const struct device *dev;
	uint8_t adc_conf;
	uint8_t diff;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint8_t channels;
	enum ad405x_operation_mode operation_mode;
	struct k_sem sem_devrdy;
	const struct device *gpio_dev;
	enum ad405x_gpx_mode gp1_mode;
	enum ad405x_gpx_mode gp0_mode;
	uint8_t dev_en_pol;
#ifdef CONFIG_AD405X_TRIGGER
	struct gpio_callback gpio1_cb;
	struct gpio_callback gpio0_cb;
	struct k_sem sem_drdy;
#endif
};

void delay_320ns(void)
{
    __asm__ volatile (
        "NOP\n\t"  // NOP 1
        "NOP\n\t"  // NOP 2
        "NOP\n\t"  // NOP 3
        "NOP\n\t"  // NOP 4
        "NOP\n\t"  // NOP 5
        "NOP\n\t"  // NOP 6
        "NOP\n\t"  // NOP 7
        "NOP\n\t"  // NOP 8
        "NOP\n\t"  // NOP 9
        "NOP\n\t"  // NOP 10
        "NOP\n\t"  // NOP 11
        "NOP\n\t"  // NOP 12
        "NOP\n\t"  // NOP 13
        "NOP\n\t"  // NOP 14
        "NOP\n\t"  // NOP 15
        "NOP\n\t"  // NOP 16
        "NOP\n\t"  // NOP 17
        "NOP\n\t"  // NOP 18
        "NOP\n\t"  // NOP 19
        "NOP\n\t"  // NOP 20
        "NOP\n\t"  // NOP 21
        "NOP\n\t"  // NOP 22
        "NOP\n\t"  // NOP 23
        "NOP\n\t"  // NOP 24
        "NOP\n\t"  // NOP 25
        "NOP\n\t"  // NOP 26
        "NOP\n\t"  // NOP 27
        "NOP\n\t"  // NOP 28
        "NOP\n\t"  // NOP 29
        "NOP\n\t"  // NOP 30
        "NOP\n\t"  // NOP 31
        "NOP\n\t"  // NOP 32
        "NOP\n\t"  // NOP 33
        "NOP\n\t"  // NOP 34
        "NOP\n\t"  // NOP 35
        "NOP\n\t"  // NOP 36
        "NOP\n\t"  // NOP 37
        "NOP\n\t"  // NOP 38
    );
}

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
static bool ad405x_bus_is_ready_spi(const union ad405x_bus *bus)
{
	bool ret = spi_is_ready_dt(&bus->spi);

	return ret;
}

int ad405x_reg_access_spi(const struct device *dev, uint8_t cmd, uint8_t reg_addr,
				  uint8_t *data, size_t length)
{
	const struct adc_ad405x_config *cfg = dev->config;
	uint8_t access = reg_addr | cmd;
	const struct spi_buf buf[2] = {{.buf = &access, .len = 1}, {.buf = data, .len = length}};
	const struct spi_buf_set rx = {.buffers = buf, .count = ARRAY_SIZE(buf)};
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};
	int ret;

	if (cmd == AD405X_READ_CMD) {
		tx.count = 1;
		ret = spi_transceive_dt(&cfg->bus.spi, &tx, &rx);
		return ret;
	} else {
		ret = spi_write_dt(&cfg->bus.spi, &tx);
		return ret;
	}
}
int ad405x_reset_pattern_cmd(const struct device *dev)
{
	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *data = dev->data;
	uint8_t access[18] ={0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};
	const struct spi_buf buf[1] = {{.buf = &access, .len = ARRAY_SIZE(access)}};
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 1,
	};

	int ret = spi_write_dt(&cfg->bus.spi, &tx);

	if(ret < 0)
	{
		return ret;
	}
if (IS_ENABLED(CONFIG_AD405X_TRIGGER)) {
	k_sem_take(&data->sem_devrdy, K_FOREVER);
} else {
	k_msleep(5);
}
	return ret;

}

int ad405x_read_raw(const struct device *dev, uint8_t *data, size_t len)
{
	const struct adc_ad405x_config *cfg = dev->config;
	const struct spi_buf buf[1] = {{.buf = data, .len = len}};
	const struct spi_buf_set rx = {.buffers = buf, .count = ARRAY_SIZE(buf)};
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};

	return spi_transceive_dt(&cfg->bus.spi, NULL, &rx);
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

int ad405x_reg_access(const struct device *dev, uint8_t cmd, uint8_t addr,
				     uint8_t *data, size_t len)
{
	const struct adc_ad405x_config *cfg = dev->config;

	return cfg->reg_access(dev, cmd, addr, data, len);
}

int ad405x_reg_write(const struct device *dev, uint8_t addr, uint8_t *data,
				    uint8_t len)
{

	return ad405x_reg_access(dev, AD405X_WRITE_CMD, addr, data, len);
}

int ad405x_reg_read(const struct device *dev, uint8_t addr, uint8_t *data,
				   uint8_t len)
{

	return ad405x_reg_access(dev, AD405X_READ_CMD, addr, data, len);
}

int ad405x_reg_write_byte(const struct device *dev, uint8_t addr, uint8_t val)
{
	return ad405x_reg_write(dev, addr, &val, 1);
}

int ad405x_reg_read_byte(const struct device *dev, uint8_t addr, uint8_t *buf)

{
	return ad405x_reg_read(dev, addr, buf, 1);
}

int ad405x_reg_update_bits(const struct device *dev, uint8_t addr, uint8_t mask, uint8_t val)
{
	int ret;
	uint8_t byte = 0;
	ret = ad405x_reg_read_byte(dev, addr, &byte);
	if (ret < 0)
	{
		return ret;
	}

	byte &= ~mask;
	byte |= val;

	return ad405x_reg_write_byte(dev, addr, byte);
}

#if defined(CONFIG_AD405X_TRIGGER)
static void ad405x_gpio1_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct adc_ad405x_data *drv_data =
		CONTAINER_OF(cb, struct adc_ad405x_data, gpio1_cb);
	const struct adc_ad405x_config *cfg = drv_data->dev->config;
	gpio_flags_t gpio_flag = GPIO_INT_EDGE_TO_ACTIVE;

	gpio_pin_interrupt_configure_dt(&cfg->gp1_interrupt, GPIO_INT_DISABLE);

	switch(drv_data->gp1_mode)
	{
		case AD405X_DEV_READY:
			k_sem_give(&drv_data->sem_devrdy);
		break;

		case AD405X_DATA_READY:
			k_sem_give(&drv_data->sem_drdy);
			gpio_flag = GPIO_INT_EDGE_TO_INACTIVE;
		break;
	}
	
	gpio_pin_interrupt_configure_dt(&cfg->gp1_interrupt, gpio_flag);
}

static void ad405x_gpio0_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct adc_ad405x_data *drv_data =
		CONTAINER_OF(cb, struct adc_ad405x_data, gpio0_cb);
	const struct adc_ad405x_config *cfg = drv_data->dev->config;
	gpio_flags_t gpio_flag = GPIO_INT_EDGE_TO_ACTIVE;

	gpio_pin_interrupt_configure_dt(&cfg->gp0_interrupt, GPIO_INT_DISABLE);

	switch(drv_data->gp0_mode)
	{
		case AD405X_DATA_READY:
			k_sem_give(&drv_data->sem_drdy);
			//printk("DRDY\n");
			gpio_flag = GPIO_INT_EDGE_TO_INACTIVE;
		break;
	}
	
	gpio_pin_interrupt_configure_dt(&cfg->gp0_interrupt, gpio_flag);

}
#endif

int ad405x_init_conv(const struct device *dev)
{
	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *drv_data = dev->data;
	int ret;

	// if (!gpio_is_ready_dt(&cfg->conversion)) {
	// 	LOG_ERR("GPIO port %s not ready", cfg->conversion.port->name);
	// 	return -EINVAL;
	// }

	ret = gpio_pin_configure_dt(&cfg->conversion, GPIO_OUTPUT_INACTIVE);
	if (ret != 0) {
		return ret;
	}

	ret = gpio_pin_set_dt(&cfg->conversion, 0);
	drv_data->dev = dev;
	return ret;
}

int ad405x_init_interrupt(const struct device *dev)
{
	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *drv_data = dev->data;
	int ret;

	if (!gpio_is_ready_dt(&cfg->gp1_interrupt)) {
		LOG_ERR("GPIO port %s not ready", cfg->gp1_interrupt.port->name);
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&cfg->gp1_interrupt, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	gpio_init_callback(&drv_data->gpio1_cb,
			   ad405x_gpio1_callback,
			   BIT(cfg->gp1_interrupt.pin));

	ret = gpio_add_callback(cfg->gp1_interrupt.port, &drv_data->gpio1_cb);
	if (ret != 0) {
		LOG_ERR("Failed to set gpio callback!");
		return ret;
	}

	drv_data->dev = dev;

	gpio_pin_interrupt_configure_dt(&cfg->gp1_interrupt, GPIO_INT_EDGE_TO_ACTIVE);

	if (!gpio_is_ready_dt(&cfg->gp0_interrupt)) {
		LOG_ERR("GPIO port %s not ready", cfg->gp0_interrupt.port->name);
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&cfg->gp0_interrupt, GPIO_INPUT);
	if (ret != 0) {
		return ret;
	}

	gpio_init_callback(&drv_data->gpio0_cb,
			   ad405x_gpio0_callback,
			   BIT(cfg->gp0_interrupt.pin));

	ret = gpio_add_callback(cfg->gp0_interrupt.port, &drv_data->gpio0_cb);
	if (ret != 0) {
		LOG_ERR("Failed to set gpio callback!");
		return ret;
	}

	drv_data->dev = dev;

	return 0;
}

static int ad405x_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	//const struct adc_ad405x_config *config = dev->config;
	struct adc_ad405x_data *data = dev->data;

	if (channel_cfg->channel_id != 0) {
		LOG_ERR("invalid channel id %d", channel_cfg->channel_id);
		return -EINVAL;
	}

	/* When differential is used data format is signed */
	// return ad405x_reg_update_bits(dev,
	// 				AD405X_REG_ADC_MODES,
	// 				AD405X_ADC_DATA_FORMAT_MSK,
	// 				channel_cfg->differential);

}

static int adc_ad405x_validate_buffer_size(const struct device *dev,
					   const struct adc_sequence *sequence)
{
	uint8_t channels;
	size_t needed;

	channels = POPCOUNT(sequence->channels);
	needed = channels * sizeof(uint16_t);

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

int ad405x_conv_start(const struct device *dev)
{
	const struct adc_ad405x_config *cfg = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&cfg->conversion, 1);
	// if (ret != 0) {
	// 	return ret;
	// }

	/** CNV High Time min 10 ns */
	// k_busy_wait(1);
	ret = gpio_pin_set_dt(&cfg->conversion, 0);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	
	struct adc_ad405x_data *data = CONTAINER_OF(ctx, struct adc_ad405x_data, ctx);
	const struct adc_ad405x_config *cfg = (struct adc_ad405x_config *)data->dev->config;
	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	ad405x_conv_start(data->dev);
// #if CONFIG_AD405X_TRIGGER
// 	k_sem_take(&data->sem_drdy, K_FOREVER);

// 	ad405x_read_raw(data->dev, reg_val_x, 2);
// 	//printk("READ RAW - %x\n", *((uint16_t*)reg_val_x));
// 	adc_context_on_sampling_done(&data->ctx, data->dev);
// #else
	/*320 ns*/
	uint8_t reg_val_x[2];

	int ret = ad405x_read_raw(data->dev, reg_val_x, 2);
	if (ret != 0)
		return ret;
	adc_context_on_sampling_done(&data->ctx, data->dev);
// #endif
}

int ad405x_exit_command(const struct device *dev)
{
	int ret;

	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *data = dev->data;
	uint8_t access[1] = { 0xA8 };
	const struct spi_buf buf[1] = {{.buf = &access, .len = ARRAY_SIZE(access)}};
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 1,
	};

	ret = spi_write_dt(&cfg->bus.spi, &tx);
	if (!ret)
		data->operation_mode = AD405X_CONFIG_MODE_OP;

	return ret;
}

int ad405x_set_operation_mode(const struct device *dev,
			       enum ad405x_operation_mode operation_mode)
{
	struct adc_ad405x_data *data = dev->data;
	int ret;
	
	if(data->operation_mode == AD405X_SLEEP_MODE_OP)
	{
		if(operation_mode != AD405X_SLEEP_MODE_OP){
			ret = ad405x_reg_write_byte(dev,
					AD405X_REG_DEVICE_CONFIG,
					AD405X_ENTER_ACTIVE_MODE);
			if (ret != 0) {
				return ret;
			}
			if(operation_mode != AD405X_CONFIG_MODE_OP){
				/* Set Operation mode. */
				ret = ad405x_reg_update_bits(dev,
							AD405X_REG_ADC_MODES,
							AD405X_ADC_MODES_MSK,
							operation_mode);
				if (ret != 0) {
					return ret;
				}
					/* Enter ADC_MODE. */
				ret = ad405x_reg_update_bits(dev,
							AD405X_REG_MODE_SET,
							AD405X_ENTER_ADC_MODE_MSK,
							AD405X_ENTER_ADC_MODE);
				if (ret != 0) {
					return ret;
				}			
			}
		}
	}
	else if(data->operation_mode == AD405X_CONFIG_MODE_OP)
	{
		if(operation_mode == AD405X_SLEEP_MODE_OP){
			ret = ad405x_reg_write_byte(dev,
					AD405X_REG_DEVICE_CONFIG,
					AD405X_ENTER_SLEEP_MODE);
			if (ret != 0) {
				return ret;
			}
		} else {
			/* Set Operation mode. */
			ret = ad405x_reg_update_bits(dev,
						AD405X_REG_ADC_MODES,
						AD405X_ADC_MODES_MSK,
						operation_mode);
			if (ret != 0) {
				return ret;
			}

			uint8_t red_reg;
			ret = ad405x_reg_read_byte(dev, AD405X_REG_ADC_MODES, &red_reg);
			if (ret < 0) {
				return ret;
			}

			ret = ad405x_reg_read_byte(dev, 0x22, &red_reg);
			if (ret < 0) {
				return ret;
			}

			ret = ad405x_reg_read_byte(dev, 0x24, &red_reg);
			if (ret < 0) {
				return ret;
			}

			ret = ad405x_reg_read_byte(dev, 0x25, &red_reg);
			if (ret < 0) {
				return ret;
			}

			ret = ad405x_reg_write_byte(dev, 0x25, 0);
			if (ret < 0) {
				return ret;
			}

			ret = ad405x_reg_read_byte(dev, 0x40, &red_reg);
			if (ret < 0) {
				return ret;
			}

			ret = ad405x_reg_read_byte(dev, 0x41, &red_reg);
			if (ret < 0) {
				return ret;
			}

				/* Enter ADC_MODE. */
			ret = ad405x_reg_update_bits(dev,
						AD405X_REG_MODE_SET,
						AD405X_ENTER_ADC_MODE_MSK,
						AD405X_ENTER_ADC_MODE);
			if (ret != 0) {
				return ret;
			}			
		}
	}
	else
	{
		ret = ad405x_exit_command(dev);
			if (ret != 0) {
				return ret;
			};
		if(operation_mode == AD405X_SLEEP_MODE_OP){
			ret = ad405x_reg_write_byte(dev,
					AD405X_REG_DEVICE_CONFIG,
					AD405X_ENTER_SLEEP_MODE);
			if (ret != 0) {
				return ret;
			}
		} else if (operation_mode != AD405X_CONFIG_MODE_OP)
		{
			ret = ad405x_reg_update_bits(dev,
						AD405X_REG_ADC_MODES,
						AD405X_ADC_MODES_MSK,
						operation_mode);
			if (ret != 0) {
				return ret;
			}
				/* Enter ADC_MODE. */
			ret = ad405x_reg_update_bits(dev,
						AD405X_REG_MODE_SET,
						AD405X_ENTER_ADC_MODE_MSK,
						AD405X_ENTER_ADC_MODE);
			if (ret != 0) {
				return ret;
			}
		}
	}
	data->operation_mode = operation_mode;
	return 0;
}

int ad405x_set_gpx_mode(const struct device *dev, uint8_t gp0_1,
			       enum ad405x_gpx_mode gpx_mode)
{
	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *data = dev->data;
	uint8_t mask = AD405X_GP1_MODE_MSK;
	int ret;
	enum ad405x_gpx_mode gpx_mode_tmp = gpx_mode;

	if(gp0_1 == AD405X_GP0)
	{
		mask = AD405X_GP0_MODE_MSK;
		if(gpx_mode == AD405X_DEV_READY)
		{
			return -EINVAL;
		}
	}
	else
	{
		gpx_mode_tmp = gpx_mode_tmp << 4;
	}

	//TODO polarity of data en 
	ret = ad405x_reg_update_bits(dev,
				AD405X_REG_GP_PIN_CONF,
				mask,
				gpx_mode_tmp);

	if(ret == 0){
		if(gp0_1 == AD405X_GP0)
		{
			if(gpx_mode == AD405X_DATA_READY) {
				gpio_pin_interrupt_configure_dt(&cfg->gp0_interrupt, GPIO_INT_EDGE_TO_INACTIVE);
			}
			data->gp0_mode = gpx_mode;
		} else {
			if(gpx_mode == AD405X_DATA_READY) {
				gpio_pin_interrupt_configure_dt(&cfg->gp1_interrupt, GPIO_INT_EDGE_TO_INACTIVE);
			}
			data->gp1_mode = gpx_mode;
		}
	}
	return ret;
}
static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_ad405x_data *data = CONTAINER_OF(ctx, struct adc_ad405x_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int adc_ad405x_start_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *data = dev->data;
	int ret;

	if (sequence->resolution != AD405X_ADC_RESOLUTION) {
		LOG_ERR("invalid resolution %d", sequence->resolution);
		return -EINVAL;
	}

	ret = adc_ad405x_validate_buffer_size(dev, sequence);
	if (ret < 0) {
		LOG_ERR("insufficient buffer size");
		return ret;
	}

	ad405x_set_operation_mode(dev, cfg->active_mode);

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}

static int adc_ad405x_read_async(const struct device *dev, const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct adc_ad405x_data *data = dev->data;
	int ret;

	adc_context_lock(&data->ctx, async ? true : false, async);
	ret = adc_ad405x_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);

	return ret;
}

static int ad405x_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return adc_ad405x_read_async(dev, sequence, NULL);
}

static inline bool adc_ad405x_bus_is_ready(const struct device *dev)
{
	const struct adc_ad405x_config *cfg = dev->config;

	return cfg->bus_is_ready(&cfg->bus);
}

int ad405x_soft_reset(const struct device *dev)
{
	int ret;

	ret = ad405x_reg_update_bits(dev, AD405X_REG_INTERFACE_CONFIG_A,
		AD405X_SW_RESET_MSK, AD405X_REG_INTERFACE_CONFIG_A_RESET_VAL);
	if (ret < 0) {
		return ret;
	}

	return ad405x_reg_update_bits(dev, AD405X_REG_INTERFACE_CONFIG_A,
        AD405X_SW_RESET_MSK, 0);
}

static int adc_ad405x_init(const struct device *dev)
{
	const struct adc_ad405x_config *cfg = dev->config;
	struct adc_ad405x_data *data = dev->data;
	int ret;
	uint8_t reg_val;
    uint8_t reg_val_hi;
	uint16_t reg_val_res;
	
	if (!adc_ad405x_bus_is_ready(dev)) {
		LOG_ERR("bus not ready");
		return -ENODEV;
	}


if (IS_ENABLED(CONFIG_AD405X_TRIGGER)) {
	ret = ad405x_init_interrupt(dev);
	if (ret != 0) {
		LOG_ERR("Failed to initialize interrupt!");
		return -EIO;
	}
} else {
	data->gp0_mode = AD405X_DISABLED;
}
	ad405x_init_conv(dev);
	data->gp1_mode = AD405X_DEV_READY;
	data->operation_mode = AD405X_CONFIG_MODE_OP;
	k_sem_init(&data->sem_devrdy, 0, 1);
	adc_context_init(&data->ctx);

    /* SW reset - delay? */
	ret = ad405x_reset_pattern_cmd(dev);
	if (ret < 0) {
		return ret;
	}

	ret = ad405x_reg_read_byte(dev, AD405X_REG_PRODUCT_ID_L, &reg_val);
	if (ret < 0) {
		return ret;
	}

	ret = ad405x_reg_read_byte(dev, AD405X_REG_PRODUCT_ID_H, &reg_val_hi);
	if (ret < 0) {
		return ret;
	}

    reg_val_res = (reg_val_hi << 8) | reg_val;
    if (reg_val_res != AD405X_REG_PRODUCT_ID_VAL) {
        LOG_ERR("Invalid product id");
		return -ENODEV;
    }

	ret = ad405x_reg_read_byte(dev, AD405X_REG_DEVICE_TYPE, &reg_val);
	if (ret < 0) {
		return ret;
	}

    if (reg_val != AD405X_REG_DEVICE_TYPE_VAL) {
        LOG_ERR("Invalid device type");
		return -ENODEV;
    }

	ret = ad405x_reg_read_byte(dev, AD405X_REG_VENDOR_L, &reg_val);
	if (ret < 0) {
		return ret;
	}

	ret = ad405x_reg_read_byte(dev, AD405X_REG_VENDOR_H, &reg_val_hi);
	if (ret < 0) {
		return ret;
	}

    reg_val_res = (reg_val_hi << 8) | reg_val;
    if (reg_val_res != AD405X_REG_VENDOR_VAL) {
        LOG_ERR("Invalid vendor value");
		return -ENODEV;
    }

	if (IS_ENABLED(CONFIG_AD405X_TRIGGER)) {
		ad405x_set_gpx_mode(dev, AD405X_GP1, AD405X_DATA_READY);
		k_sem_init(&data->sem_drdy, 0, 1);
	}

	adc_context_unlock_unconditionally(&data->ctx);
	return ret;
}

static DEVICE_API(adc, ad405x_api_funcs) = {
	.channel_setup = ad405x_channel_setup,
	.read = ad405x_read,
	.ref_internal = 2048,
#ifdef CONFIG_ADC_ASYNC
	.read_async = ad405x_adc_read_async,
#endif
};

#ifdef CONFIG_AD405X_TRIGGER
#define AD405X_CFG_IRQ(inst) \
		.gp1_interrupt = GPIO_DT_SPEC_INST_GET(inst, gp1_gpios), \
		.gp0_interrupt = GPIO_DT_SPEC_INST_GET(inst, gp0_gpios),
#else
#define AD405X_CFG_IRQ(inst)
#endif /* CONFIG_AD405X_TRIGGER */

#define AD405X_CONFIG(inst, chipid)                        \
		.chip_id = chipid,                                  \
		.conversion = GPIO_DT_SPEC_INST_GET(inst, conversion_gpios), \
		.active_mode = AD405X_SAMPLE_MODE_OP,                            \
		COND_CODE_1(AD4050_CHIP_ID, (.resolution = AD4050_ADC_RESOLUTION,), \
				(.resolution = AD4052_ADC_RESOLUTION,)) \

#define ADC_AD405X_CONFIG_SPI(inst, chipid)               \
	{                                             \
		.bus = {.spi = SPI_DT_SPEC_INST_GET(inst, \
							SPI_WORD_SET(8) |     \
						    SPI_TRANSFER_MSB,      \
						    0)},                  \
		AD405X_CONFIG(inst, chipid)              \
		.bus_is_ready = ad405x_bus_is_ready_spi, \
		.reg_access = ad405x_reg_access_spi,     \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, gp1_gpios), \
		(COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, gp0_gpios),	\
		(AD405X_CFG_IRQ(inst)), ())), ())				\
	}

#define ADC_AD405X_DEFINE(inst, chipid)								\
	static struct adc_ad405x_data ad405x_data_##inst##chipid = {			\
	};     \
	static const struct adc_ad405x_config ad405x_config_##inst##chipid =                  \
		COND_CODE_1(DT_INST_ON_BUS(inst, spi), (ADC_AD405X_CONFIG_SPI(inst, chipid)), \
			    ());                                \
                                                                                        \
	DEVICE_DT_INST_DEFINE(inst, adc_ad405x_init, NULL,				\
			      &ad405x_data_##inst##chipid, &ad405x_config_##inst##chipid, POST_KERNEL,\
			      CONFIG_ADC_INIT_PRIORITY, &ad405x_api_funcs);		\

DT_INST_FOREACH_STATUS_OKAY_VARGS(ADC_AD405X_DEFINE, AD4050_CHIP_ID)

#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT adi_adxl366
DT_INST_FOREACH_STATUS_OKAY_VARGS(ADC_AD405X_DEFINE, AD4052_CHIP_ID)
#undef DT_DRV_COMPAT