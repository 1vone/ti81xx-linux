/******************************************************************************
 * TI8168 DVR Board
 * Copyright by UDWorks, Incoporated. All Rights Reserved.
 *-----------------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *---------------------------------------------------------------------------*/
 /**
 * @file	dm8168_dvr_io.c
 * @brief
 * @author
 * @section	MODIFY history
 *     - 2011.05.25 : First Created
 */
/*****************************************************************************/

/*----------------------------------------------------------------------------
 Defines referenced header files
-----------------------------------------------------------------------------*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>			//# fops
#include <linux/miscdevice.h>	//# misc_register
#include <linux/uaccess.h>		//# copy_from_user
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/sched.h>		//# waitqueue
#include <linux/proc_fs.h>		//# kalloc

#include <mach/gpio.h>

/*----------------------------------------------------------------------------
 Definitions and macro
-----------------------------------------------------------------------------*/
#define DVRIO_MINOR		255
#define MAX_REG_REQ		32
#define DBOUNCE_TIME	30		//# mS

typedef struct {
	unsigned int	gpio;
	unsigned int 	dir;		//# 0:input, 1:output
	unsigned int	trigger;	//# 1:falling, 2:rising
	unsigned int 	val;		//# default value in case output
} dvrio_t;

typedef struct {
	unsigned int	irq_cnt;
	unsigned char	irq_gpio[MAX_REG_REQ];
	unsigned char	evt_val;
	unsigned int	debounce_interval;
	struct timer_list	timer;
	struct mutex	mutex;
	wait_queue_head_t	wait;
} dev_dvrio_t;

//# ioctl command
#define DVRIO_CMD_INIT		_IOW('g', 1, dvrio_t)
#define DVRIO_CMD_CHG_IRQ	_IOW('g', 2, dvrio_t)
#define DVRIO_CMD_RD		_IOR('g', 3, dvrio_t)

/*----------------------------------------------------------------------------
 Declares variables
-----------------------------------------------------------------------------*/
static dev_dvrio_t *dvrio;

/*----------------------------------------------------------------------------
 Declares a function prototype
-----------------------------------------------------------------------------*/
#define eprintk(x...) printk("dvrio-ERR: " x);
//#define dprintk(x...) printk("dvrio: " x);
#define dprintk(x...)

/*----------------------------------------------------------------------------
 Local function
-----------------------------------------------------------------------------*/
static void dvrio_check_io(unsigned int data)
{
	dprintk("dvrio_check_io\n");

	wake_up_interruptible(&dvrio->wait);
}

static irqreturn_t dvrio_isr(int irq, void *dev_id)
{
	unsigned char *gpio = (unsigned char *)dev_id;

	dvrio->evt_val = *gpio;
	dprintk("dvrio_isr irq 0x%x, gpio %d\n", irq, dvrio->evt_val);

	if (dvrio->debounce_interval)
		mod_timer(&dvrio->timer,
			jiffies + msecs_to_jiffies(dvrio->debounce_interval));
	else
		wake_up_interruptible(&dvrio->wait);

	return IRQ_HANDLED;
}

/*****************************************************************************
* @brief	dvrio function
* @section	DESC Description
*	- desc
*****************************************************************************/
static ssize_t dvrio_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	dvrio_t io;

	if (copy_from_user(&io, (void __user *)buf, sizeof(dvrio_t)))
		return -EFAULT;

	gpio_set_value(io.gpio, io.val);

	return 1;
}

static ssize_t dvrio_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret=0;

	if ((file->f_flags & O_NONBLOCK))
		return -EAGAIN;

	//interruptible_sleep_on_timeout(&dvrio->wait, 0);
	interruptible_sleep_on(&dvrio->wait);

	if (mutex_lock_interruptible(&dvrio->mutex))
		return -EINTR;

	if (copy_to_user(buf, &dvrio->evt_val, 1)) {
		mutex_unlock(&dvrio->mutex);
		return -EFAULT;
	}

	mutex_unlock(&dvrio->mutex);

	dprintk("dvrio_read\n");

	return ret;
}

static int dvrio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	dvrio_t io;
	int irq, i;

	if (copy_from_user(&io, (void __user *)arg, sizeof(dvrio_t)))
		return -EFAULT;

	dprintk("dvrio_ioctl cmd(%d), gpio(%d)\n", cmd, io.gpio);

	switch (cmd)
	{
		case DVRIO_CMD_INIT:
		{
			gpio_request(io.gpio, "dvrio");
			if(io.dir) {
				gpio_direction_output(io.gpio, io.val);
			} else {
				gpio_direction_input(io.gpio);

				irq = gpio_to_irq(io.gpio);
				if (irq < 0) {
					eprintk("Unable to get irq number %d\n", io.gpio);
				}
				dprintk("io.gpio %d(%d)\n", io.gpio, irq);

				ret = request_irq(irq,
						dvrio_isr,
						io.trigger,
						"dvr_io", (void *)&dvrio->irq_gpio[dvrio->irq_cnt]);
				if(ret < 0) {
					eprintk("gpio request irq failed %d\n", ret);
				} else {
					dvrio->irq_gpio[dvrio->irq_cnt++] = io.gpio;
				}
			}
			break;
		}
		case DVRIO_CMD_CHG_IRQ:
		{
			for(i=0; i<dvrio->irq_cnt; i++)
			{
				if(io.gpio == dvrio->irq_gpio[i])
				{
					irq = gpio_to_irq(io.gpio);
					if (irq < 0) {
						eprintk("Unable to get irq number %d\n", io.gpio);
					}
					dprintk("io.gpio %d(%d)\n", io.gpio, irq);

					free_irq(irq, &dvrio->irq_gpio[i]);

					ret = request_irq(irq,
							dvrio_isr,
							io.trigger,
							"dvr_io", (void *)&dvrio->irq_gpio[i]);
					if(ret < 0) {
						eprintk("gpio request irq failed %d\n", ret);
					}
					break;
				}
			}
			break;
		}
		case DVRIO_CMD_RD:
		{
			dprintk("DVRIO_CMD_RD");
			io.val = gpio_get_value(io.gpio);
			if (copy_to_user((void __user *)arg, &io, sizeof(dvrio_t))) {
				return -EFAULT;
			}
			break;
		}
		default:
			break;
	}

	return 0;
}

static int dvrio_open(struct inode *inode, struct file *file)
{
	/* allocate memory for device data */
	dvrio = kzalloc(sizeof(dev_dvrio_t), GFP_KERNEL);
	if (dvrio == NULL) {
		printk(KERN_ERR "%s: couldn't allocate device data\n", __FUNCTION__);
		return -ENOMEM;
	}

	//# init device struct
	dvrio->irq_cnt = 0;
	memset(dvrio->irq_gpio, 0x0, MAX_REG_REQ);

	dvrio->debounce_interval = DBOUNCE_TIME;

	setup_timer(&dvrio->timer, dvrio_check_io, 0);

	mutex_init(&dvrio->mutex);
	init_waitqueue_head(&dvrio->wait);

	dprintk("dvrio_open\n");

	return 0;
}

static int dvrio_release(struct inode *inode, struct file *file)
{
	int i;

	if(dvrio->irq_cnt) {
		for(i=0; i<dvrio->irq_cnt; i++)
			free_irq((gpio_to_irq(dvrio->irq_gpio[i])), &dvrio->irq_gpio[i]);
	}

	if(dvrio != NULL)
		kfree(dvrio);

	dprintk("dvrio_release\n");

	return 0;
}

static struct file_operations dvrio_fops = {
	.owner		= THIS_MODULE,
	.open		= dvrio_open,
	.release	= dvrio_release,
	.write		= dvrio_write,
	.read		= dvrio_read,
	.unlocked_ioctl	= dvrio_ioctl,
};

static struct miscdevice dvrio_misc = {
	.fops   = &dvrio_fops,
	.minor  = DVRIO_MINOR,
	.name   = "dvr_io",
};

/*****************************************************************************
* @brief	dvr_netra gpio module init & exit function
* @section	DESC Description
*	- desc
*****************************************************************************/
static int __init dvr_io_init(void)
{
	return misc_register(&dvrio_misc);
}

static void __exit dvr_io_exit(void)
{
	misc_deregister(&dvrio_misc);
}

module_init(dvr_io_init)
module_exit(dvr_io_exit)

MODULE_AUTHOR("UDWorks");
MODULE_DESCRIPTION("DVR_NETRA GPIO Driver");
MODULE_LICENSE("GPL");
