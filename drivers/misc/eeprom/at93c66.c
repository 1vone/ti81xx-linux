/*
 * Copyright (C) 2009 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
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
#include <linux/delay.h>
#include <asm/uaccess.h>

struct at93c66_device {
	unsigned int dev_size;
};

extern u8 gpio_eeprom_read(int addr);
extern void gpio_eeprom_write(int addr, u8 data);

/*
 * AT93C66 Maximum Clock Speed -> 1MHz
 */
// ------ SPI over GPIO routines ---------------------------------------------
static loff_t at93c66_eeprom_llseek(struct file *file, loff_t offset, int origin)
{
	unsigned int size;

	struct at93c66_device *device = (struct at93c66_device *)file->private_data;
	size = device->dev_size;

	lock_kernel();
	switch (origin) {
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

static ssize_t at93c66_eeprom_read(struct file *file, char __user *buf,
			  size_t count, loff_t *ppos)
{
	unsigned int i,size;
	struct at93c66_device *device = (struct at93c66_device *)file->private_data;
	char __user *p = buf;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	size = device->dev_size;
	if (*ppos >= size)
		return 0;

	for (i = *ppos; count > 0 && i < size; ++i, ++p, --count)
		if (__put_user(gpio_eeprom_read(i), p))
			return -EFAULT;

	*ppos = i;
	return p - buf;
}

static ssize_t at93c66_eeprom_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	unsigned int i,size;
	char c;
	struct at93c66_device *device = (struct at93c66_device *)file->private_data;
	const char __user *p = buf;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	size = device->dev_size;
	if (*ppos >= size)
		return 0;

	device = (struct at93c66_device *)file->private_data;
	for (i = *ppos; count > 0 && i < size; ++i, ++p, --count) {
		if (__get_user(c, p))
			return -EFAULT;
		gpio_eeprom_write(c, i);
	}

	*ppos = i;
	return p - buf;
}

static int at93c66_eeprom_open(struct inode *inode, struct file *file)
{
	struct at93c66_device *dev;

	/* allocate memory for subchannel data */
	dev = kzalloc(sizeof (struct at93c66_device), GFP_KERNEL);
	if (dev == NULL) {
		printk(KERN_ERR "%s: couldn't allocate device data\n",
		       __FUNCTION__);
		return -ENOMEM;
	}

	dev->dev_size = 512;

	/* Initialization device */
	file->private_data = dev;

	return 0;
}

static int at93c66_eeprom_relase(struct inode *inode, struct file *file)
{
	struct at93c66_device *dev = (struct at93c66_device *) file->private_data;

	kfree(dev);

	return 0;
}

struct file_operations at93c66_eeprom_fops = {
	.owner		= THIS_MODULE,
	.llseek		= at93c66_eeprom_llseek,
	.read		= at93c66_eeprom_read,
	.write		= NULL,
	.open		= at93c66_eeprom_open,
	.release	= at93c66_eeprom_relase,
};

static struct miscdevice at93c66_eeprom_dev = {
	NVRAM_MINOR,
	"eeprom",
	&at93c66_eeprom_fops
};

int __init at93c66_eeprom_init(void)
{
	return misc_register(&at93c66_eeprom_dev);
}

void __exit at93c66_eeprom_cleanup(void)
{
	misc_deregister(&at93c66_eeprom_dev);
}

module_init(at93c66_eeprom_init);
module_exit(at93c66_eeprom_cleanup);
MODULE_LICENSE("GPL");

