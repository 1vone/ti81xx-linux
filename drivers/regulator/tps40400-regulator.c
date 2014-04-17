/*
 * tps40400-regulator.c
 *
 * Support for Intersil ISL6271A voltage regulator
 *
 * Copyright (C) 2010 Marek Vasut <marek.vasut@gmail.com>
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
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "pmbus.h"

#if 0
#define dprintk(x...) printk(" [reg] " x)
#else
#define dprintk(x...)
#endif

#define CHANGE_FREQ		1
#define AVERAGE_POINTS  100

/* PMIC details */
struct tps40400_vr_pmic_info_pmic {
	struct i2c_client	*client;
	struct regulator_dev	*rdev;
	unsigned		min_uV;
	unsigned		max_uV;
	int 			exponent;
	struct mutex		mtx;
	unsigned 		nominal_uV;
	unsigned		lastSetuV;
};

static int get_data_linear(struct tps40400_vr_pmic_info_pmic *pmic, int data)
{
	s16 exponent;
	s32 mantissa;
	long val;

	exponent = pmic->exponent;
	mantissa = (u16)data;

	val = mantissa;

	/* scale result to micro-units for all sensors except fans */
	val = val * 1000000L;

	if (exponent >= 0)
	{
		val <<= exponent;
	}
	else
	{
		val >>= -exponent;
	}

	return (int)val;
}

static int tps40400_get_voltage(struct regulator_dev *dev)
{
	struct tps40400_vr_pmic_info_pmic *pmic = rdev_get_drvdata(dev);
	int data;

	mutex_lock(&pmic->mtx);

	data = pmic->lastSetuV;
    dprintk("%d:%s Current voltage :%d\n", __LINE__,__func__, data);

	mutex_unlock(&pmic->mtx);
	return data;
}

static u16 set_data2_linear(struct tps40400_vr_pmic_info_pmic *pmic, int data)
{
	/* simple case */
	if (data == 0)
		return 0;

	if (pmic->exponent < 0)
		data <<= -pmic->exponent;
	else
		data >>= pmic->exponent;

	data = DIV_ROUND_CLOSEST(data, 1000000);

	return data & 0xffff;
}

static int tps40400_set_voltage(struct regulator_dev *dev, int minuV, int maxuV)
{
	struct tps40400_vr_pmic_info_pmic *pmic = rdev_get_drvdata(dev);
	struct i2c_client *client = pmic->client;
	int ret, regval;
	int trim_offset;

	dprintk("%d:%s Smartreflex trying to set minuV:%d, maxuV:%d\n", __LINE__,__func__,minuV,maxuV);

	mutex_lock(&pmic->mtx);

	/** trim_offset = minuV - data; **/
	trim_offset = minuV - pmic->nominal_uV; //Nominal voltage is
	dprintk("%d:%s Trim Output voltage by %duV\n", __LINE__,__func__,trim_offset);

	/** Set VOUT_TRIM(22h) **/
	regval = set_data2_linear(rdev_get_drvdata(dev),trim_offset);
	dprintk("%d:%s trying to Set Output voltage: 0x%x\n", __LINE__,__func__,regval);


	ret = i2c_smbus_write_word_data(client, PMBUS_VOUT_TRIM, regval);
	if (ret < 0)
		dev_err(&pmic->client->dev, "Error setting voltage\n");

	pmic->lastSetuV = minuV;
	/* After Each Set_Voltage,wait for rise time = (trim amount)*(TON_RISE)/(Vnom)
											- max time required is
											= 200mV * 2.6 ms/1000mV
											= 0.52 ms (let's wait for 1ms )*/
	udelay(1000);

	mutex_unlock(&pmic->mtx);

	return ret;
}

static int tps40400_list_voltage(struct regulator_dev *dev, unsigned selector)
{
	dprintk("%d:%s\n", __LINE__,__func__);
	return 0;
}

static int tps40400_enable(struct regulator_dev *rdev)
{
	struct tps40400_vr_pmic_info_pmic *pmic = rdev_get_drvdata(rdev);
	struct i2c_client *client = pmic->client;
	int ret = 0;

	mutex_lock(&pmic->mtx);
	ret = i2c_smbus_read_byte_data(client, PMBUS_OPERATION);
	//dprintk("%d:%s Read PMBUS_OPERATION: 0x%x\n", __LINE__,__func__,ret);

	/*10000000(No Margin) - OPERATION(01h)*/
	ret = i2c_smbus_write_byte_data(client, PMBUS_OPERATION, 0x80);
	//dprintk("%d:%s Set: Enable PMBUS_OPERATION: %x\n", __LINE__,__func__,ret);

	mutex_unlock(&pmic->mtx);

	return ret;
}

static int tps40400_disable(struct regulator_dev *rdev)
{
	struct tps40400_vr_pmic_info_pmic *pmic = rdev_get_drvdata(rdev);
	struct i2c_client *client = pmic->client;
	int ret = 0;

	mutex_lock(&pmic->mtx);
	ret = i2c_smbus_read_byte_data(client, PMBUS_OPERATION);
	dprintk("%d:%s Read PMBUS_OPERATION: 0x%x\n", __LINE__,__func__,ret);

	/*00000000(Output Switching Disabled) - OPERATION(01h)*/
	ret = i2c_smbus_write_byte_data(client, PMBUS_OPERATION, 0x00);
	dprintk("%d:%s Disable PMBUS_OPERATION: %x\n", __LINE__,__func__,ret);

	mutex_unlock(&pmic->mtx);

	return ret;
}

static struct regulator_ops tps40400_vr_pmic_info_core_ops = {
	.get_voltage	= tps40400_get_voltage,
	.set_voltage	= tps40400_set_voltage,
	.enable	= tps40400_enable,
	.disable	= tps40400_disable,
	.list_voltage	= tps40400_list_voltage,
};

static struct regulator_desc tps40400_vr_pmic_info_rd = {
		.name		= "pmbus_vr",
		.id			= 0,
		.ops		= &tps40400_vr_pmic_info_core_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
};

static int __devinit tps40400_probe(struct i2c_client *i2c,
				     const struct i2c_device_id *id)
{
	struct regulator_init_data *init_data	= i2c->dev.platform_data;
	struct tps40400_vr_pmic_info_pmic *pmic;
	int err;
	int ret, read_data,regval;
	int read_vout_mode = -1, exponent;
	int i,arr_sum = 0;
	int avgdata_arr[AVERAGE_POINTS] = {0};

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_WRITE_BYTE
				     | I2C_FUNC_SMBUS_BYTE_DATA
				     | I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	if (!init_data) {
		dev_err(&i2c->dev, "no platform data supplied\n");
		return -EIO;
	}

	pmic = kzalloc(sizeof(struct tps40400_vr_pmic_info_pmic), GFP_KERNEL);
	if (!pmic)
		return -ENOMEM;

	pmic->exponent = -10; /*Set this default value for TPS40400*/
	pmic->client = i2c;
	pmic->nominal_uV = 1100000; /* For DM8168 - nominal voltage is 1.1V)*/
	pmic->lastSetuV = pmic->nominal_uV; /* set initial voltage to nominal */

	mutex_init(&pmic->mtx);

	pmic->rdev = regulator_register(&tps40400_vr_pmic_info_rd, &i2c->dev,
						init_data, pmic);
	if (IS_ERR(pmic->rdev)) {
		dev_err(&i2c->dev, "failed to register %s\n", id->name);
		err = PTR_ERR(pmic->rdev);
		goto error;
	}

	i2c_set_clientdata(i2c, pmic);

	/****   VOUT_MODE(20h) */
	read_vout_mode = i2c_smbus_read_byte_data(i2c, PMBUS_VOUT_MODE);
	if (read_vout_mode >= 0 && read_vout_mode != 0xff) {
		exponent = read_vout_mode & 0x1f;
		/* and sign-extend it */
		if (exponent & 0x10)
			exponent |= ~0x1f;
		pmic->exponent = exponent;
		dprintk("%d:%s TPS40400 exponent:%d \n", __LINE__,__func__, pmic->exponent);
	}
	/****/

	mutex_lock(&pmic->mtx);
	/**CLEAR_FAULTS(03h)**/
	ret = i2c_smbus_write_byte(i2c, PMBUS_CLEAR_FAULTS);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed writing 0x80 to 0x%02x\n", 0x03);
	}

	/**PMBUS_REVISION(0x98)**/
	ret = i2c_smbus_read_byte_data(i2c, PMBUS_REVISION);
	dprintk("%d:%s PMBUS_REVISION: %d\n", __LINE__,__func__,ret);

	ret = i2c_smbus_read_byte_data(i2c, PMBUS_VOUT_CAL_OFFSET);
	dprintk("%d:%s VOUT_CAL_OFFSET: %d\n", __LINE__,__func__,ret);

	ret = i2c_smbus_read_byte_data(i2c, PMBUS_VOUT_CAL_GAIN);
	dprintk("%d:%s VOUT_CAL_GAIN: %d\n", __LINE__,__func__,ret);

	ret = i2c_smbus_read_word_data(i2c, PMBUS_VOUT_SCALE_LOOP);
	dprintk("%d:%s PMBUS_VOUT_SCALE_LOOP: 0x%x\n", __LINE__,__func__,ret);
	/* For Nominal voltage 1.0V => 600mV/1000mV x (2^9) = 307.2 => 100110011 => b933*/
	ret = i2c_smbus_write_word_data(i2c, PMBUS_VOUT_SCALE_LOOP, 0xb917); /* Setting it for DM8168 DVR RDK */
	dprintk("%d:%s Set PMBUS_VOUT_SCALE_LOOP: 0x%x\n", __LINE__,__func__,ret);

	ret = i2c_smbus_read_word_data(i2c, PMBUS_VOUT_SCALE_LOOP);
	dprintk("%d:%s PMBUS_VOUT_SCALE_LOOP: 0x%x\n", __LINE__,__func__,ret);

	//# change PMBUS_FREQUENCY - dvr_rdk -----------------------------
	#if CHANGE_FREQ
	ret = i2c_smbus_write_word_data(i2c, PMBUS_FREQUENCY_SWITCH, FREQ_680K);	//# FREQ_1200K
	if (ret < 0)
		dev_err(&pmic->client->dev, "Error setting frequency\n");
	#endif

	/**VOUT_MARGIN_HIGH(25h)**/
	ret = i2c_smbus_read_word_data(i2c, PMBUS_VOUT_MARGIN_HIGH);
	dprintk("%d:%s PMBUS_VOUT_MARGIN_HIGH: 0x%x\n", __LINE__,__func__,ret);

	/**VOUT_MARGIN_LOW(26h)**/
	ret = i2c_smbus_read_word_data(i2c, PMBUS_VOUT_MARGIN_LOW);
	dprintk("%d:%s PMBUS_VOUT_MARGIN_LOW: 0x%x\n", __LINE__,__func__,ret);

	/**VOUT_TRIM(22h) ***/
	ret = i2c_smbus_read_word_data(i2c, PMBUS_VOUT_TRIM);
	dprintk("%d:%s Read VOUT_TRIM: 0x%x\n", __LINE__,__func__,ret);
	if (ret < 0) {
		dev_err(&pmic->client->dev, "Error getting voltage\n");
	}

#if 1
	for(i = 0; i < AVERAGE_POINTS; i++)
	{
		/**READ_VOUT**/
		ret = i2c_smbus_read_word_data(i2c, PMBUS_READ_VOUT);
		/*dprintk("%d:%s PMBUS_READ_VOUT: 0x%x\n", __LINE__,__func__,ret);*/

		/* Convert the data from pmic to microvolts */
		avgdata_arr[i] = get_data_linear(pmic,ret);
		/*dprintk("%d:%s---> Current Output voltage[%d] %duV\n", __LINE__,__func__,i,avgdata_arr[i]);*/
		/*Wait for 250us before consecutive reads*/
		udelay(250);
	}
	for(i = 0; i < AVERAGE_POINTS; i++)
	{
		arr_sum += avgdata_arr[i];
	}
	read_data = arr_sum/AVERAGE_POINTS;
#endif
	dprintk("%d:%s Avg Current Output voltage %duV\n", __LINE__,__func__,read_data);

	regval = set_data2_linear(pmic,(pmic->nominal_uV - read_data));

	/**Setting PMBUS_VOUT_CAL_OFFSET **/
	ret = i2c_smbus_write_word_data(i2c, PMBUS_VOUT_CAL_OFFSET, regval);
	if (ret < 0)
		dev_err(&pmic->client->dev, "Error setting CAL OFFSET \n");

	ret = i2c_smbus_read_byte_data(i2c, PMBUS_VOUT_CAL_OFFSET);
	dprintk("%d:%s VOUT_CAL_OFFSET: %d\n", __LINE__,__func__,ret);

	#if 1
//	regval = 0.006348 * 32768; //# DCR is 6.348:  6.348mOhms/2^-15 = 208.011 is 0xD0
	regval = 0.009 * 32768; //# DCR is 9:  6.mOhms/2^-15
	ret = i2c_smbus_write_word_data(i2c, PMBUS_IOUT_CAL_GAIN, regval);
	if (ret < 0)
		dev_err(&pmic->client->dev, "Error setting out current limit fault\n");
	#endif
	ret = i2c_smbus_read_word_data(i2c, PMBUS_IOUT_CAL_GAIN);
	dprintk("%d:%s PMBUS_IOUT_CAL_GAIN: 0x%x\n", __LINE__,__func__,ret);

	//# Setting PMBUS_IOUT_OC_FAULT_LIMIT(46h) default 10A
	#if 1
	regval = 28 * 2; //# 28A
	ret = i2c_smbus_write_word_data(i2c, PMBUS_IOUT_OC_FAULT_LIMIT, regval);
	if (ret < 0)
		dev_err(&pmic->client->dev, "Error setting out current limit fault\n");
	#endif
	ret = i2c_smbus_read_word_data(i2c, PMBUS_IOUT_OC_FAULT_LIMIT);
	dprintk("%d:%s IOUT_OC_FAULT_LIMIT: 0x%x\n", __LINE__,__func__,ret);

	#if 1
	regval = 25 * 2; //# 25A
	ret = i2c_smbus_write_word_data(i2c, PMBUS_IOUT_OC_WARN_LIMIT, regval);
	if (ret < 0)
		dev_err(&pmic->client->dev, "Error setting over current warning fault\n");
	#endif
	ret = i2c_smbus_read_word_data(i2c, PMBUS_IOUT_OC_WARN_LIMIT);
	dprintk("%d:%s PMBUS_IOUT_OC_WARN_LIMIT: 0x%x\n", __LINE__,__func__,ret);

	mutex_unlock(&pmic->mtx);

	printk("regulator: tps40400 probe done.\n");

	return 0;

error:
	regulator_unregister(pmic->rdev);

	kfree(pmic);
	return err;
}

static int __devexit tps40400_remove(struct i2c_client *i2c)
{
	struct tps40400_vr_pmic_info_pmic *pmic = i2c_get_clientdata(i2c);

	regulator_unregister(pmic->rdev);

	printk("[regulator] tps40400 remove done.\n");

	kfree(pmic);

	return 0;
}

static const struct i2c_device_id tps40400_id[] = {
	{.name = "pmbus", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tps40400_id);

static struct i2c_driver tps40400_i2c_driver = {
	.driver = {
		.name = "pmbus",
		.owner = THIS_MODULE,
	},
	.probe = tps40400_probe,
	.remove = __devexit_p(tps40400_remove),
	.id_table = tps40400_id,
};

static int __init tps40400_init(void)
{
	return i2c_add_driver(&tps40400_i2c_driver);
}

static void __exit tps40400_cleanup(void)
{
	i2c_del_driver(&tps40400_i2c_driver);
}

subsys_initcall(tps40400_init);
module_exit(tps40400_cleanup);

MODULE_AUTHOR("Vivek");
MODULE_DESCRIPTION("tps40400 voltage regulator driver");
MODULE_LICENSE("GPL v2");
