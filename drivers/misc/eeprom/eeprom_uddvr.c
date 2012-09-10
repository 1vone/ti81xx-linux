/*
 * Copyright (C) 2009, Texas Instruments, Incorporated
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/*
 * ATMEL AT93CX6 eeprom supported
 * bus with: x8,
 * note: bus with x16 not supported.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
#include <linux/init.h>
#include <linux/smp_lock.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/eeprom_uddvr.h>

#include <plat/mux.h>
#include <plat/gpio.h>

#define EEPROM_CS_ACTIVE		1
#define EEPROM_CS_INACTIVE		0
#define EEPROM_DELAY			4

#define AT93C_WRITE_ON			0x180
#define AT93C_WRITE_OFF			0x0
#define AT93C_ER_ALL			0x100

#define AT93C_CODE				0
#define AT93C_WRITE_CODE		0x1
#define AT93C_READ_CODE			0x2
#define AT93C_ERASE_CODE		0x3

static int uddvr_hwver = 0xff; //# set the default value

//#define mdelay(n)				udelay((n)*1000)
struct eeprom_priv {
	u32 mem_size;
};

static void eeprom_clk(void)
{
	gpio_set_value(EEPROM_SCLK, 1);
	udelay(EEPROM_DELAY);
	
	gpio_set_value(EEPROM_SCLK, 0);
	udelay(EEPROM_DELAY);
}

static u8 spi_r(void)
{
	int i;
	u8 temp = 0;
	
	for (i = 7; i >= 0; i--) {
		gpio_set_value(EEPROM_SCLK, 1);
		udelay(EEPROM_DELAY);

		temp |= (gpio_get_value(EEPROM_SDI)) ? 1 << i : 0;

		gpio_set_value(EEPROM_SCLK, 0);
		udelay(EEPROM_DELAY);
	}
	
	return temp;
}

static void spi_w(u8 data)
{
	int i;

	for (i = 7; i >= 0; i--) {
		(data & (0x01 << i)) ? gpio_set_value(EEPROM_SDO, 1) :
			gpio_set_value(EEPROM_SDO, 0);
		eeprom_clk();
	}
}

static void spi_reg(u8 data, u16 reg)
{
	int i;

	/* Initial start bit */
	gpio_set_value(EEPROM_SDO, 1);
	eeprom_clk();

	/* AT93C operation
	 *(2-bit op code, msb first) 
	 */
	for (i = 1; i >= 0; i--) {
		(data & (0x01 << i)) ? gpio_set_value(EEPROM_SDO, 1) :
			gpio_set_value(EEPROM_SDO, 0);
		eeprom_clk();
	}

	/* Address location (9-bit) */
	for (i = 8; i >= 0; i--) {
		(reg & (0x01 << i)) ? gpio_set_value(EEPROM_SDO, 1) :
			gpio_set_value(EEPROM_SDO, 0);
		eeprom_clk();
	}
}

/**
 * eeprom_read - read from AT93CX6 EEPROM
 * @reg:	The register offset.
 *
 * This function reads a word from the AT93CX6 EEPROM.
 *
 * Return the data value.
 */
static u8 eeprom_read_word(u16 reg)
{
	u8 data;

	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	spi_reg(AT93C_READ_CODE, (reg&0x1ff));
	data = spi_r();
	
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);

	return data;
}

/**
 * eeprom_write - write to AT93CX6 EEPROM
 * @reg:	The register offset.
 * @data:	The data value.
 *
 * This procedure writes a word(8bit) to the AT93CX6 EEPROM.
 */
static void eeprom_write_word(u16 reg, u8 data)
{
	int timeout = 0;
	
	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	/* Enable write. */
	spi_reg(AT93C_CODE, AT93C_WRITE_ON);
	/* inactive cs */
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);
	udelay(1);
	
	/* Erase the register. */
	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	spi_reg(AT93C_ERASE_CODE, reg);
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);
	udelay(1);
	
	/* Check operation complete. */
	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	timeout = 8;
	mdelay(2);
	
	do {
		mdelay(1);
	} while (!gpio_get_value(EEPROM_SDI) && --timeout);
	
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);
	udelay(1);
	
	/* Write the register. */
	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	spi_reg(AT93C_WRITE_CODE, reg);
	spi_w(data);
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);
	udelay(1);
	
	/* Check operation complete. (max 10ms) */
	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	timeout = 8;
	mdelay(2);
	
	do {
		mdelay(1);
	} while (!gpio_get_value(EEPROM_SDI) && --timeout);
	
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);
	udelay(1);
	
	/* Disable write. */
	gpio_set_value(EEPROM_CS, EEPROM_CS_ACTIVE);
	spi_reg(AT93C_CODE, AT93C_WRITE_OFF);
	gpio_set_value(EEPROM_CS, EEPROM_CS_INACTIVE);
}

int eeprom_uddvr_read(u32 offset, size_t len, void *buf)
{
	u8 *p, *tmp;
	unsigned int i;

	p = tmp = (u8 *)buf;
	
	if (offset >= EEPROM_SIZE)
		return 0;
	
	if ((offset + len) > EEPROM_SIZE) {
		len = (EEPROM_SIZE - offset);
	}
	
	for (i = offset; len > 0; i++, --len) 
		*p++ = eeprom_read_word(i);

	return (int)(p-tmp);
}
EXPORT_SYMBOL_GPL(eeprom_uddvr_read);

int eeprom_uddvr_write(u32 offset, size_t len, const void *buf)
{
	u32 i;
	u8 *p, *tmp;

	p = tmp = (u8 *)buf;

	if (offset >= EEPROM_SIZE)
		return 0;
	
	if ((offset + len) > EEPROM_SIZE) {
		len = (EEPROM_SIZE - offset);
	}
	
	for (i = offset; len > 0; ++i, --len) {
		eeprom_write_word((i&0x1ff), *p++);
	}

	return (int)(p-tmp);
}
EXPORT_SYMBOL_GPL(eeprom_uddvr_write);

int eeprom_uddvr_get_hwver(void)
{
	return uddvr_hwver;
}
EXPORT_SYMBOL_GPL(eeprom_uddvr_get_hwver);

int eeprom_uddvr_set_hwver(void)
{
	u8 ver[2];
	
	eeprom_uddvr_read(16, 2, &ver[0]);
	uddvr_hwver = (ver[0] << 8) + ver[1];
	
	return 0;
}
EXPORT_SYMBOL_GPL(eeprom_uddvr_set_hwver);

static loff_t eeprom_llseek(struct file *file, loff_t offset, int start)
{
	struct eeprom_priv *dev = (struct eeprom_priv *)file->private_data;
	unsigned int size;
	
	size = dev->mem_size;

	lock_kernel();
	switch (start) {
	case 1:
		offset += file->f_pos;
		break;
	case 2:
		offset += size;
		break;
	}
	
	if (offset < 0) {
		unlock_kernel();
		return -EINVAL;
	}
	
	file->f_pos = offset;
	unlock_kernel();
	return file->f_pos;
}

static ssize_t eeprom_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct eeprom_priv *dev = (struct eeprom_priv *)file->private_data;
	char __user *p = buf;
	unsigned int i, size;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	size = dev->mem_size;
	if (*ppos >= size)
		return 0;

	for (i = *ppos; count > 0 && i < size; ++i, ++p, --count)
		if (__put_user(eeprom_read_word(i), p))
			return -EFAULT;

	*ppos = i;
	return p - buf;
}

static ssize_t eeprom_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct eeprom_priv *dev = (struct eeprom_priv *)file->private_data;
	const char __user *p = buf;
	
	unsigned int i, size;
	char c;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	size = dev->mem_size;
	if (*ppos >= size)
		return 0;

	for (i = *ppos; count > 0 && i < size; ++i, ++p, --count) {
		if (__get_user(c, p))
			return -EFAULT;
			
		eeprom_write_word(i, c);
	}

	*ppos = i;
	return p - buf;
}

static int eeprom_open(struct inode *inode, struct file *file)
{
	struct eeprom_priv *dev;

	/* allocate memory for subchannel data */
	dev = kzalloc(sizeof (struct eeprom_priv), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "%s: couldn't allocate device data\n",
		       __FUNCTION__);
		return -ENOMEM;
	}

	dev->mem_size = 128;

	/* Initialization device */
	file->private_data = dev;
	return 0;
}

static int eeprom_relase(struct inode *inode, struct file *file)
{
	struct eeprom_priv *dev = (struct eeprom_priv *)file->private_data;

	kfree(dev);
	return 0;
}

struct file_operations eeprom_fops = {
	.owner		= THIS_MODULE,
	.llseek		= eeprom_llseek,
	.read		= eeprom_read,
	.write		= NULL,
	.open		= eeprom_open,
	.release	= eeprom_relase,
};

static struct miscdevice eeprom_device = {
	NVRAM_MINOR,
	"eeprom",
	&eeprom_fops
};

int __init eeprom_init(void)
{
	return misc_register(&eeprom_device);
}

void __exit eeprom_cleanup(void)
{
	misc_deregister(&eeprom_device);
}

module_init(eeprom_init);
module_exit(eeprom_cleanup);
MODULE_LICENSE("GPL");
