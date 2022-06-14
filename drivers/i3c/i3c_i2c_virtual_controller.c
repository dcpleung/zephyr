/*
 * Copyright 2020 Google LLC
 * Copyright (c) 2020 Nordic Semiconductor ASA
 * Copyright 2022 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT virtual_i3c_i2c_controller

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i3c_i2c_virtual_controller, CONFIG_I3C_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/drivers/i3c.h>

struct i2c_virt_ctrl_config {
	const struct device *bus;
};

struct i2c_virt_ctrl_data {
	/* Pointer to the array of I2C devices */
	struct i3c_i2c_device_desc *i2c_devs;

	/* Number of I2C devices */
	const size_t num_i2c_devs;
};

static struct i3c_i2c_device_desc *find_i2c_dev(const struct i2c_virt_ctrl_data *data,
						uint8_t addr)
{
	int i;

	for (i = 0; i < data->num_i2c_devs; i++) {
		if (data->i2c_devs[i].addr == addr) {
			return &data->i2c_devs[i];
		}
	}

	return NULL;
}

static int i2c_virt_ctrl_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_virt_ctrl_config *config = dev->config;
	struct i3c_config_controller *ctrl;
	int ret;

	/*
	 * Check for invalid configuration.
	 */
	if ((dev_config & I2C_MODE_CONTROLLER) != I2C_MODE_CONTROLLER) {
		ret = -EINVAL;
		goto out_config;
	}

	ret = i3c_config_get(config->bus, I3C_CONFIG_CONTROLLER, &ctrl);
	if (ret < 0) {
		goto out_config;
	}

	/*
	 * Well... officially the only I2C speed supported on I3C bus
	 * are fast (400 kHz) and fast plus (1 MHz). But this doesn't
	 * prevent the actual hardware to operate at other speed
	 * in I2C specification. So let the I3C controller to decide
	 * whether to accept or reject.
	 */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		ctrl->scl.i2c = 100000;
		break;

	case I2C_SPEED_FAST:
		ctrl->scl.i2c = 400000;
		break;

	case I2C_SPEED_FAST_PLUS:
		ctrl->scl.i2c = 1000000;
		break;

	case I2C_SPEED_HIGH:
		ctrl->scl.i2c = 3400000;
		break;

	case I2C_SPEED_ULTRA:
		ctrl->scl.i2c = 5000000;
		break;

	default:
		ret = -EINVAL;
		goto out_config;
	};

	ret = i3c_configure(config->bus, I3C_CONFIG_CONTROLLER, &ctrl);

out_config:
	return ret;
}

static int i2c_virt_ctrl_config_get(const struct device *dev, uint32_t *dev_config)
{
	const struct i2c_virt_ctrl_config *config = dev->config;
	struct i3c_config_controller *ctrl;
	uint32_t cfg;
	int ret;

	ret = i3c_config_get(config->bus, I3C_CONFIG_CONTROLLER, &ctrl);
	if (ret < 0) {
		goto out_get_config;
	}

	cfg = I2C_MODE_CONTROLLER;

	switch (ctrl->scl.i2c) {
	case 100000:
		cfg |= I2C_SPEED_STANDARD;
		break;

	case 400000:
		cfg |= I2C_SPEED_FAST;
		break;

	case 1000000:
		cfg |= I2C_SPEED_FAST_PLUS;
		break;

	case 3400000:
		cfg |= I2C_SPEED_HIGH;
		break;

	case 5000000:
		cfg |= I2C_SPEED_ULTRA;
		break;

	default:
		ret = -EINVAL;
		goto out_get_config;
	};

out_get_config:
	return ret;
}

static int i2c_virt_ctrl_transfer(const struct device *dev, struct i2c_msg *msgs,
				  uint8_t num_msgs, uint16_t addr)
{
	const struct i2c_virt_ctrl_data *data = dev->data;
	struct i3c_i2c_device_desc *i2c_dev;

	i2c_dev = find_i2c_dev(data, addr);

	if (i2c_dev == NULL) {
		LOG_ERR("Cannot find I2C device 0x%02x", addr);
		return -ENODEV;
	}

	return i3c_i2c_transfer(i2c_dev, msgs, num_msgs);
}

static int i2c_virt_ctrl_recover_bus(const struct device *dev)
{
	const struct i2c_virt_ctrl_config *config = dev->config;

	return i3c_recover_bus(config->bus);
}

/**
 * Set up a new emulator and add it to the list
 *
 * @param dev I2C emulation controller device
 */
static int i2c_virt_ctrl_init(const struct device *dev)
{
	const struct i2c_virt_ctrl_data *data = dev->data;

	int i;

	for (i = 0; i < data->num_i2c_devs; i++) {
		i3c_i2c_device_register(&data->i2c_devs[i]);
	}

	return 0;
}

static struct i2c_driver_api i2c_virt_ctrl_api = {
	.configure = i2c_virt_ctrl_configure,
	.get_config = i2c_virt_ctrl_config_get,
	.transfer = i2c_virt_ctrl_transfer,
	.recover_bus = i2c_virt_ctrl_recover_bus,
};

#define I2C_DEVICE_DESC(node_id)					\
	{								\
		.bus = DEVICE_DT_GET(DT_BUS(DT_PARENT(node_id))),	\
		.addr = DT_PROP_BY_IDX(node_id, reg, 0),		\
		.lvr = DT_PROP(DT_PARENT(node_id), lvr),		\
	}

#define I2C_VIRT_CTRL_INIT(n)						\
	static struct i3c_i2c_device_desc i2c_dev_list_##n[] = {	\
		DT_FOREACH_CHILD(DT_DRV_INST(n), I2C_DEVICE_DESC)	\
	};								\
	static struct i2c_virt_ctrl_config i2c_virt_ctrl_config_##n = {	\
		.bus = DEVICE_DT_GET(DT_INST_BUS(n)),			\
	};								\
	static struct i2c_virt_ctrl_data i2c_virt_ctrl_data_##n = {	\
		.i2c_devs = i2c_dev_list_##n,				\
		.num_i2c_devs = ARRAY_SIZE(i2c_dev_list_##n),		\
	};								\
	I2C_DEVICE_DT_INST_DEFINE(n,					\
			    i2c_virt_ctrl_init,				\
			    NULL,					\
			    &i2c_virt_ctrl_data_##n,			\
			    &i2c_virt_ctrl_config_##n,			\
			    POST_KERNEL,				\
			    CONFIG_I3C_DEV_REGISTER_INIT_PRIORITY,	\
			    &i2c_virt_ctrl_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_VIRT_CTRL_INIT)
