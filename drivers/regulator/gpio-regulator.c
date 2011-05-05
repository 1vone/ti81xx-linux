/*
 * gpio-regulator.c
 *
 * GPIO controlled voltage regulator driver
 *
 * Copyright (C) 2011 Texas Instrument Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/gpio-regulator.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#define GPIO_REG_DCDC_1	0

/* Regulator specific details */
struct gpio_vr_info {
	unsigned	min_uV;
	unsigned	max_uV;
	u8		table_len;
	struct gpio_vr_data *table;
	int		nr_gpio_pins;
	struct gpio	*gpios;
};

/* PMIC details */
struct gpio_vr_pmic {
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	struct gpio_vr_info info;
	spinlock_t lock;
};

static int gpio_vr_dcdc_enable(struct regulator_dev *dev)
{
	int dcdc = rdev_get_id(dev);

	if (dcdc != GPIO_REG_DCDC_1)
		return -EINVAL;

	/* Add your PMIC specific enable code */
	return 0;
}

static int gpio_vr_dcdc_disable(struct regulator_dev *dev)
{
	int dcdc = rdev_get_id(dev);

	if (dcdc != GPIO_REG_DCDC_1)
		return -EINVAL;

	/* Add your PMIC specific disable code */
	return 0;
}

static u8 gpio_read_value(struct gpio *gpios, int num_of_bits)
{
	int i = 0;
	int gpio_val;
	u8 cur_gpio_val = 0;

	while (i < num_of_bits) {
		gpio_val = gpio_get_value(gpios[i].gpio);
		cur_gpio_val = cur_gpio_val | (gpio_val << i);
		i++;
	}

	return cur_gpio_val;
}

static int gpio_write_value(struct gpio *gpios, int num_of_bits,
					int gpio_val, spinlock_t *lock)
{
	int i = 0;
	unsigned long flags;

	spin_lock_irqsave(lock, flags);
	while (i < num_of_bits) {
		gpio_direction_output(gpios[i].gpio, (gpio_val & (1 << i)));
		i++;
	}
	spin_unlock_irqrestore(lock, flags);

	return 0;
}

static int get_nearest_voltage(struct gpio_vr_pmic *tps, int uV)
{
	int vsel;
	s32 temp[tps->info.table_len];

	/* Nearest GPIO val, nothing but picking the minimum diff
	 * gpio values from a group of values
	 */
	for (vsel = 0; vsel < tps->info.table_len; vsel++) {
		temp[vsel] = tps->info.table[vsel].uV - uV;
		if (temp[vsel] >= 0)
			break;
	}

	if (vsel == tps->info.table_len)
		return -EINVAL;

	return tps->info.table[vsel].uV;
}

/*
 * The routine gets the current operating voltage
 * gpio voltage value
 */
static int gpio_vr_get_uv(struct regulator_dev *dev)
{
	struct gpio_vr_pmic *tps = rdev_get_drvdata(dev);
	u8 gpio_val;
	int uV, vsel;

	gpio_val = gpio_read_value(tps->info.gpios, tps->info.nr_gpio_pins);

	/* Get the voltage from the table based on gpio_val */
	for (vsel = 0; vsel < tps->info.table_len; vsel++) {
		if (tps->info.table[vsel].gpio_value == gpio_val) {
			uV = tps->info.table[vsel].uV;
			break;
		}
	}

	return uV;
}

static int gpio_vr_dcdc_get_voltage(struct regulator_dev *dev)
{
	struct gpio_vr_pmic *tps = rdev_get_drvdata(dev);
	int uV, dcdc = rdev_get_id(dev);

	if (dcdc == GPIO_REG_DCDC_1) {
		uV = gpio_vr_get_uv(dev);
		return uV;
	} else
		return tps->info.min_uV;
}

static int gpio_vr_dcdc_set_voltage(struct regulator_dev *dev,
				int min_uV, int max_uV)
{
	struct gpio_vr_pmic *tps = rdev_get_drvdata(dev);
	int vsel, uV = 0;
	int dcdc = rdev_get_id(dev);

	if (dcdc != GPIO_REG_DCDC_1)
		return -EINVAL;

	if (min_uV < tps->info.min_uV ||
			min_uV > tps->info.max_uV)
		return -EINVAL;
	if (max_uV < tps->info.min_uV ||
			max_uV > tps->info.max_uV)
		return -EINVAL;

	uV = get_nearest_voltage(tps, min_uV);

	for (vsel = 0; vsel < tps->info.table_len; vsel++) {
		if (tps->info.table[vsel].uV == uV)
			break;
	}

	if (vsel == tps->info.table_len)
		return -EINVAL;

	return gpio_write_value(tps->info.gpios, tps->info.nr_gpio_pins,
				tps->info.table[vsel].gpio_value, &tps->lock);
}

static int gpio_vr_dcdc_list_voltage(struct regulator_dev *dev,
					unsigned selector)
{
	struct gpio_vr_pmic *tps = rdev_get_drvdata(dev);
	int dcdc = rdev_get_id(dev);

	if (dcdc == GPIO_REG_DCDC_1) {
		if (selector >= tps->info.table_len)
			return -EINVAL;
		else
			return tps->info.table[selector].uV;
	} else
		return tps->info.min_uV;
}

/* Operations permitted on VDCDCx */
static struct regulator_ops gpio_vr_dcdc_ops = {
	.enable = gpio_vr_dcdc_enable,
	.disable = gpio_vr_dcdc_disable,
	.get_voltage = gpio_vr_dcdc_get_voltage,
	.set_voltage = gpio_vr_dcdc_set_voltage,
	.list_voltage = gpio_vr_dcdc_list_voltage,
};

static int __devinit gpio_tps_probe(struct platform_device *pdev)
{
	static int desc_id;
	struct gpio_reg_platform_data *tps_board;
	struct regulator_init_data *init_data;
	struct regulator_dev *rdev;
	struct gpio_vr_pmic *tps;
	int error;

	/**
	 * init_data points to array of regulator_init structures
	 * coming from the board-evm file.
	 */
	tps_board = pdev->dev.platform_data;
	if (!tps_board)
		return -EINVAL;

	/**
	 * init_data points to array of regulator_init structures
	 * coming from the board-evm file.
	 */
	init_data = tps_board->pmic_init_data;
	if (!init_data)
		return -EINVAL;

	tps = kzalloc(sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	/* init spinlock for workqueue */
	spin_lock_init(&tps->lock);

	/* Store regulator specific information */
	tps->desc.name = tps_board->name;
	tps->desc.id = desc_id++;
	tps->desc.n_voltages = tps_board->num_voltages;
	tps->desc.ops = &gpio_vr_dcdc_ops;
	tps->desc.type = REGULATOR_VOLTAGE;
	tps->desc.owner = THIS_MODULE;

	tps->info.min_uV = init_data->constraints.min_uV;
	tps->info.max_uV = init_data->constraints.max_uV;
	tps->info.table = tps_board->gpio_vsel_table;
	tps->info.table_len = tps_board->num_voltages;
	tps->info.gpios = tps_board->gpios;
	tps->info.nr_gpio_pins = tps_board->num_gpio_pins;

	error = gpio_request_array(tps->info.gpios, tps->info.nr_gpio_pins);
	if (error) {
		dev_err(&pdev->dev, "failed to get the gpios");
		return -EINVAL;
	}

	/* Register the regulators */
	rdev = regulator_register(&tps->desc, &pdev->dev,
					init_data, tps);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register %s\n",
			tps_board->name);
		error = PTR_ERR(rdev);
		goto fail;
	}

	/* Save regulator for cleanup */
	tps->rdev = rdev;

	platform_set_drvdata(pdev, tps);

	return 0;

 fail:
	regulator_unregister(tps->rdev);
	kfree(tps);

	return error;
}

/**
 * gpio_vr_remove - GPIO volt reg driver remove handler
 * @client: gpio driver client device structure
 *
 * Unregister TPS driver as an gpio client device driver
 */
static int __devexit gpio_vr_remove(struct platform_device *pdev)
{
	struct gpio_vr_pmic *tps = dev_get_drvdata(&pdev->dev);

	gpio_free_array(tps->info.gpios, tps->info.nr_gpio_pins);

	regulator_unregister(tps->rdev);
	kfree(tps);

	return 0;
}

static struct platform_driver gpio_vr_driver = {
	.driver = {
		.name = "gpio_vr",
		.owner = THIS_MODULE,
	},
	.probe = gpio_tps_probe,
	.remove = __devexit_p(gpio_vr_remove),
};

/**
 * gpio_vr_init
 *
 * GPIO based voltage regulator Module init function
 */
int __init gpio_vr_init(void)
{
	return platform_driver_register(&gpio_vr_driver);
}
subsys_initcall(gpio_vr_init);

/**
 * gpio_vr_exit
 *
 * Module exit function
 */
static void __exit gpio_vr_exit(void)
{
	platform_driver_unregister(&gpio_vr_driver);
}
module_exit(gpio_vr_exit);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("GPIO voltage regulator driver");
MODULE_LICENSE("GPL v2");
