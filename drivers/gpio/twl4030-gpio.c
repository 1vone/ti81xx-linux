/*
 * twl4030_gpio.c -- access to GPIOs on TWL4030/TPS659x0 chips
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Copyright (C) 2006 MontaVista Software, Inc.
 *
 * Code re-arranged and cleaned up by:
 *	Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * Initial Code:
 *	Andy Lowe / Nishanth Menon
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/i2c/twl4030.h>


/*
 * The GPIO "subchip" supports 18 GPIOs which can be configured as
 * inputs or outputs, with pullups or pulldowns on each pin.  Each
 * GPIO can trigger interrupts on either or both edges.
 *
 * GPIO interrupts can be fed to either of two IRQ lines; this is
 * intended to support multiple hosts.
 *
 * There are also two LED pins used sometimes as output-only GPIOs.
 *
 * FIXME code currently only handles the first IRQ line.
 */


static inline void activate_irq(int irq)
{
#ifdef CONFIG_ARM
	/* ARM requires an extra step to clear IRQ_NOREQUEST, which it
	 * sets on behalf of every irq_chip.  Also sets IRQ_NOPROBE.
	 */
	set_irq_flags(irq, IRQF_VALID);
#else
	/* same effect on other architectures */
	set_irq_noprobe(irq);
#endif
}

static struct gpio_chip twl_gpiochip;
static int twl4030_gpio_irq_base;
static int twl4030_gpio_irq_end;

/* genirq interfaces are not available to modules */
#ifdef MODULE
#define is_module()	true
#else
#define is_module()	false
#endif

/* GPIO_CTRL Fields */
#define MASK_GPIO_CTRL_GPIO0CD1		BIT(0)
#define MASK_GPIO_CTRL_GPIO1CD2		BIT(1)
#define MASK_GPIO_CTRL_GPIO_ON		BIT(2)

/* Mask for GPIO registers when aggregated into a 32-bit integer */
#define GPIO_32_MASK			0x0003ffff

/* Data structures */
static DEFINE_MUTEX(gpio_lock);

/* store usage of each GPIO. - each bit represents one GPIO */
static unsigned int gpio_usage_count;

/* protect what irq_chip modifies through its helper task */
static DEFINE_SPINLOCK(gpio_spinlock);

/* shadow the imr register; updates are delayed */
static u32 gpio_imr_shadow;
static bool gpio_pending_mask;

/* bitmask of requests to set gpio irq trigger type */
static unsigned int gpio_pending_trigger;

/* pointer to helper thread */
static struct task_struct *gpio_helper_thread;

/*
 * Helper functions to read and write the GPIO ISR and IMR registers as
 * 32-bit integers. Functions return 0 on success, non-zero otherwise.
 * The caller must hold gpio_lock.
 */

/* NOTE:  only the IRQ dispatcher may read ISR; reading it has
 * side effects, because of the clear-on-read mechanism (COR=1).
 */
static int gpio_read_isr(unsigned int *isr)
{
	int ret;

	*isr = 0;
	ret = twl4030_i2c_read(TWL4030_MODULE_GPIO, (u8 *) isr,
			REG_GPIO_ISR1A, 3);
	le32_to_cpup(isr);
	*isr &= GPIO_32_MASK;

	return ret;
}

/* IMR is written only during initialization and
 * by request of the irq_chip code.
 */
static int gpio_write_imr(unsigned int imr)
{
	imr &= GPIO_32_MASK;
	/*
	 * The buffer passed to the twl4030_i2c_write() routine must have an
	 * extra byte at the beginning reserved for its internal use.
	 */
	imr <<= 8;
	imr = cpu_to_le32(imr);
	return twl4030_i2c_write(TWL4030_MODULE_GPIO, (u8 *) &imr,
				REG_GPIO_IMR1A, 3);
}

/*
 * These are the irqchip methods for the TWL4030 GPIO interrupts.
 * Our IRQ handle method doesn't call these, but they will be called by
 * other routines such as setup_irq() and enable_irq().  They are called
 * with cpu interrupts disabled and with a lock on the irq_desc.lock
 * spinlock.  This complicates matters, because accessing the TWL4030 GPIO
 * interrupt controller requires I2C bus transactions that can't be initiated
 * in this context.  Our solution is to defer accessing the interrupt
 * controller to a kernel thread.
 */

static void twl4030_gpio_irq_ack(unsigned int irq)
{
	/* NOP -- dispatcher acks when it reads ISR */
}

static void twl4030_gpio_irq_mask(unsigned int irq)
{
	int gpio = irq - twl4030_gpio_irq_base;
	u32 mask = 1 << gpio;
	unsigned long flags;

	spin_lock_irqsave(&gpio_spinlock, flags);
	gpio_pending_mask = true;
	gpio_imr_shadow |= mask;
	if (gpio_helper_thread && gpio_helper_thread->state != TASK_RUNNING)
		wake_up_process(gpio_helper_thread);
	spin_unlock_irqrestore(&gpio_spinlock, flags);
}

static void twl4030_gpio_irq_unmask(unsigned int irq)
{
	int gpio = irq - twl4030_gpio_irq_base;
	u32 mask = 1 << gpio;
	unsigned long flags;

	spin_lock_irqsave(&gpio_spinlock, flags);
	gpio_pending_mask = true;
	gpio_imr_shadow &= ~mask;
	if (gpio_helper_thread && gpio_helper_thread->state != TASK_RUNNING)
		wake_up_process(gpio_helper_thread);
	spin_unlock_irqrestore(&gpio_spinlock, flags);
}

static int twl4030_gpio_irq_set_type(unsigned int irq, unsigned trigger)
{
	struct irq_desc *desc = irq_desc + irq;
	int gpio = irq - twl4030_gpio_irq_base;
	unsigned long flags;

	trigger &= IRQ_TYPE_SENSE_MASK;
	if (trigger & ~IRQ_TYPE_EDGE_BOTH)
		return -EINVAL;
	if ((desc->status & IRQ_TYPE_SENSE_MASK) == trigger)
		return 0;

	desc->status &= ~IRQ_TYPE_SENSE_MASK;
	desc->status |= trigger;

	spin_lock_irqsave(&gpio_spinlock, flags);
	gpio_pending_trigger |= (1 << gpio);
	if (gpio_helper_thread && gpio_helper_thread->state != TASK_RUNNING)
		wake_up_process(gpio_helper_thread);
	spin_unlock_irqrestore(&gpio_spinlock, flags);

	return 0;
}

static struct irq_chip twl4030_gpio_irq_chip = {
	.name		= "twl4030",
	.ack		= twl4030_gpio_irq_ack,
	.mask		= twl4030_gpio_irq_mask,
	.unmask		= twl4030_gpio_irq_unmask,
	.set_type	= twl4030_gpio_irq_set_type,
};


/*
 * To configure TWL4030 GPIO module registers
 */
static inline int gpio_twl4030_write(u8 address, u8 data)
{
	return twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, data, address);
}

/*
 * To read a TWL4030 GPIO module register
 */
static inline int gpio_twl4030_read(u8 address)
{
	u8 data;
	int ret = 0;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_GPIO, &data, address);
	return (ret < 0) ? ret : data;
}

/*
 * twl4030 GPIO request function
 */
int twl4030_request_gpio(int gpio)
{
	int ret = 0;

	if (unlikely(gpio >= TWL4030_GPIO_MAX))
		return -EPERM;

	ret = gpio_request(twl_gpiochip.base + gpio, NULL);
	if (ret < 0)
		return ret;

	mutex_lock(&gpio_lock);
	if (gpio_usage_count & BIT(gpio)) {
		ret = -EBUSY;
	} else {
		/* First time usage? - switch on GPIO module */
		if (!gpio_usage_count) {
			ret = gpio_twl4030_write(REG_GPIO_CTRL,
					MASK_GPIO_CTRL_GPIO_ON);

		}
		if (!ret)
			gpio_usage_count |= BIT(gpio);
		else
			gpio_free(twl_gpiochip.base + gpio);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_request_gpio);

/*
 * TWL4030 GPIO free module
 */
int twl4030_free_gpio(int gpio)
{
	int ret = 0;

	if (unlikely(gpio >= TWL4030_GPIO_MAX))
		return -EPERM;

	mutex_lock(&gpio_lock);

	if ((gpio_usage_count & BIT(gpio)) == 0) {
		ret = -EPERM;
	} else {
		gpio_usage_count &= ~BIT(gpio);
		gpio_free(twl_gpiochip.base + gpio);
	}

	/* Last time usage? - switch off GPIO module */
	if (ret == 0 && !gpio_usage_count)
		ret = gpio_twl4030_write(REG_GPIO_CTRL, 0x0);

	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_free_gpio);

int twl4030_set_gpio_direction(int gpio, int is_input)
{
	u8 d_bnk = gpio >> 3;
	u8 d_msk = BIT(gpio & 0x7);
	u8 reg = 0;
	u8 base = REG_GPIODATADIR1 + d_bnk;
	int ret = 0;

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		if (is_input)
			reg = ret & ~d_msk;
		else
			reg = ret | d_msk;

		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_set_gpio_direction);

int twl4030_set_gpio_dataout(int gpio, int enable)
{
	u8 d_bnk = gpio >> 3;
	u8 d_msk = BIT(gpio & 0x7);
	u8 base = 0;

	if (enable)
		base = REG_SETGPIODATAOUT1 + d_bnk;
	else
		base = REG_CLEARGPIODATAOUT1 + d_bnk;

	return gpio_twl4030_write(base, d_msk);
}

int twl4030_get_gpio_datain(int gpio)
{
	u8 d_bnk = gpio >> 3;
	u8 d_off = gpio & 0x7;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & BIT(gpio))))
		return -EPERM;

	base = REG_GPIODATAIN1 + d_bnk;
	ret = gpio_twl4030_read(base);
	if (ret > 0)
		ret = (ret >> d_off) & 0x1;

	return ret;
}
EXPORT_SYMBOL(twl4030_get_gpio_datain);

static int twl4030_set_gpio_edge_ctrl(int gpio, int edge)
{
	u8 c_bnk = gpio >> 2;
	u8 c_off = (gpio & 0x3) * 2;
	u8 c_msk = 0;
	u8 reg = 0;
	u8 base = 0;
	int ret = 0;

	base = REG_GPIO_EDR1 + c_bnk;

	if (edge & IRQ_TYPE_EDGE_RISING)
		c_msk |= BIT(c_off + 1);
	if (edge & IRQ_TYPE_EDGE_FALLING)
		c_msk |= BIT(c_off);

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		/* clear the previous rising/falling values */
		reg = (u8) ret;
		reg &= ~(0x3 << c_off);
		reg |= c_msk;
		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}

/*
 * Configure debounce timing value for a GPIO pin on TWL4030
 */
int twl4030_set_gpio_debounce(int gpio, int enable)
{
	u8 d_bnk = gpio >> 3;
	u8 d_msk = BIT(gpio & 0x7);
	u8 reg = 0;
	u8 base = 0;
	int ret = 0;

	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & BIT(gpio))))
		return -EPERM;

	base = REG_GPIO_DEBEN1 + d_bnk;
	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(base);
	if (ret >= 0) {
		if (enable)
			reg = ret | d_msk;
		else
			reg = ret & ~d_msk;

		ret = gpio_twl4030_write(base, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
EXPORT_SYMBOL(twl4030_set_gpio_debounce);

#if 0
/*
 * Configure Card detect for GPIO pin on TWL4030
 *
 * This means:  VMMC1 or VMMC2 is enabled or disabled based
 * on the status of GPIO-0 or GPIO-1 pins (respectively).
 */
int twl4030_set_gpio_card_detect(int gpio, int enable)
{
	u8 reg = 0;
	u8 msk = (1 << gpio);
	int ret = 0;

	/* Only GPIO 0 or 1 can be used for CD feature.. */
	if (unlikely((gpio >= TWL4030_GPIO_MAX)
		|| !(gpio_usage_count & BIT(gpio))
		|| (gpio >= TWL4030_GPIO_MAX_CD))) {
		return -EPERM;
	}

	mutex_lock(&gpio_lock);
	ret = gpio_twl4030_read(REG_GPIO_CTRL);
	if (ret >= 0) {
		if (enable)
			reg = (u8) (ret | msk);
		else
			reg = (u8) (ret & ~msk);

		ret = gpio_twl4030_write(REG_GPIO_CTRL, reg);
	}
	mutex_unlock(&gpio_lock);
	return ret;
}
#endif

/* MODULE FUNCTIONS */

/*
 * gpio_helper_thread() is a kernel thread that is awakened by irq_chip
 * methods to update IRQ masks and trigger modes.
 *
 * We must do this in a thread rather than in irq_chip methods because of the
 * need to access the TWL4030 via the I2C bus.  Note that we don't need to be
 * concerned about race conditions where the request to unmask a GPIO interrupt
 * has already been cancelled before this thread does the unmasking.  If a GPIO
 * interrupt is improperly unmasked, then the IRQ handler for it will mask it
 * when an interrupt occurs.
 */
static int twl4030_gpio_helper_thread(void *dev)
{
	current->flags |= PF_NOFREEZE;

	while (!kthread_should_stop()) {
		int irq;
		bool do_mask;
		u32 local_imr;
		unsigned int gpio_trigger;

		spin_lock_irq(&gpio_spinlock);

		do_mask = gpio_pending_mask;
		gpio_pending_mask = false;
		local_imr = gpio_imr_shadow;

		gpio_trigger = gpio_pending_trigger;
		gpio_pending_trigger = 0;

		spin_unlock_irq(&gpio_spinlock);

		if (do_mask) {
			int status = gpio_write_imr(local_imr);

			if (status)
				dev_err(dev, "write IMR err %d\n", status);
		}

		for (irq = twl4030_gpio_irq_base;
				gpio_trigger;
				gpio_trigger >>= 1, irq++) {
			struct irq_desc *desc;
			unsigned type;

			if (!(gpio_trigger & 0x1))
				continue;

			desc = irq_desc + irq;
			spin_lock_irq(&desc->lock);
			type = desc->status & IRQ_TYPE_SENSE_MASK;
			spin_unlock_irq(&desc->lock);

			twl4030_set_gpio_edge_ctrl(irq - twl4030_gpio_irq_base,
					type);
		}

		spin_lock_irq(&gpio_spinlock);
		if (!gpio_pending_mask && !gpio_pending_trigger)
			set_current_state(TASK_INTERRUPTIBLE);
		spin_unlock_irq(&gpio_spinlock);

		schedule();
	}
	set_current_state(TASK_RUNNING);
	return 0;
}

/*
 * do_twl4030_gpio_module_irq() is the desc->handle method for the twl4030 gpio
 * module interrupt.  It executes in kernel thread context.
 * This is a chained interrupt, so there is no desc->action method for it.
 * We query the gpio module interrupt controller in the twl4030 to determine
 * which gpio lines are generating interrupt requests, and then call the
 * desc->handle method for each gpio that needs service.
 * On entry, cpu interrupts are disabled.
 */
static void do_twl4030_gpio_module_irq(unsigned int irq, irq_desc_t *desc)
{
	int gpio_irq;
	unsigned int gpio_isr;

	/* PIH irqs like this can't be individually masked or acked ...
	 * so we don't even call the PIH irq_chip stub methods.
	 */
	local_irq_enable();
	if (gpio_read_isr(&gpio_isr))
		gpio_isr = 0;
	local_irq_disable();

	for (gpio_irq = twl4030_gpio_irq_base; 0 != gpio_isr;
		gpio_isr >>= 1, gpio_irq++) {
		if (gpio_isr & 0x1)
			generic_handle_irq(gpio_irq);
	}
}

/*----------------------------------------------------------------------*/

static int twl_direction_in(struct gpio_chip *chip, unsigned offset)
{
	return twl4030_set_gpio_direction(offset, 1);
}

static int twl_get(struct gpio_chip *chip, unsigned offset)
{
	int status = twl4030_get_gpio_datain(offset);

	return (status < 0) ? 0 : status;
}

static int twl_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	twl4030_set_gpio_dataout(offset, value);
	return twl4030_set_gpio_direction(offset, 0);
}

static void twl_set(struct gpio_chip *chip, unsigned offset, int value)
{
	twl4030_set_gpio_dataout(offset, value);
}

static int twl_to_irq(struct gpio_chip *chip, unsigned offset)
{
	/* NOTE: assumes IRQs are set up ... */
	return twl4030_gpio_irq_base + offset;
}

static struct gpio_chip twl_gpiochip = {
	.label			= "twl4030",
	.owner			= THIS_MODULE,
	.direction_input	= twl_direction_in,
	.get			= twl_get,
	.direction_output	= twl_direction_out,
	.set			= twl_set,
	.to_irq			= twl_to_irq,
	.can_sleep		= 1,
};

/*----------------------------------------------------------------------*/

static int __devinit gpio_twl4030_pulls(u32 ups, u32 downs)
{
	u8		message[6];
	unsigned	i, gpio_bit;

	/* For most pins, a pulldown was enabled by default.
	 * We should have data that's specific to this board.
	 */
	for (gpio_bit = 1, i = 1; i < 6; i++) {
		u8		bit_mask;
		unsigned	j;

		for (bit_mask = 0, j = 0; j < 8; j += 2, gpio_bit <<= 1) {
			if (ups & gpio_bit)
				bit_mask |= 1 << (j + 1);
			else if (downs & gpio_bit)
				bit_mask |= 1 << (j + 0);
		}
		message[i] = bit_mask;
	}

	return twl4030_i2c_write(TWL4030_MODULE_GPIO, message,
				REG_GPIOPUPDCTR1, 5);
}

static int gpio_twl4030_remove(struct platform_device *pdev);

static int __devinit gpio_twl4030_probe(struct platform_device *pdev)
{
	struct twl4030_gpio_platform_data *pdata = pdev->dev.platform_data;
	int ret;
	int irq = 0;

	/* All GPIO interrupts are initially masked */
	gpio_pending_mask = false;
	gpio_imr_shadow = GPIO_32_MASK;
	ret = gpio_write_imr(gpio_imr_shadow);

	twl4030_gpio_irq_base = pdata->irq_base;
	twl4030_gpio_irq_end = pdata->irq_end;

	if ((twl4030_gpio_irq_end - twl4030_gpio_irq_base) > 0) {
		if (is_module()) {
			dev_err(&pdev->dev,
				"can't dispatch IRQs from modules\n");
			goto no_irqs;
		}
		if (twl4030_gpio_irq_end > NR_IRQS) {
			dev_err(&pdev->dev,
				"last IRQ is too large: %d\n",
				twl4030_gpio_irq_end);
			return -EINVAL;
		}
	} else {
		dev_notice(&pdev->dev,
			"no IRQs being dispatched\n");
		goto no_irqs;
	}

	if (!ret) {
		/*
		 * Create a kernel thread to handle deferred operations on gpio
		 * interrupts.
		 */
		gpio_helper_thread = kthread_create(twl4030_gpio_helper_thread,
				&pdev->dev, "twl4030 gpio");
		if (!gpio_helper_thread) {
			dev_err(&pdev->dev,
				"could not create helper thread!\n");
			ret = -ENOMEM;
		}
	}

	if (!ret) {
		/* install an irq handler for each of the gpio interrupts */
		for (irq = twl4030_gpio_irq_base; irq < twl4030_gpio_irq_end;
				irq++) {
			set_irq_chip_and_handler(irq, &twl4030_gpio_irq_chip,
					handle_edge_irq);
			activate_irq(irq);
		}

		/* gpio module IRQ */
		irq = platform_get_irq(pdev, 0);

		/* COR=1 means irqs are acked by reading ISR
		 * PENDDIS=0 means pending events enabled
		 * EXCLEN=0 means route to both irq lines
		 */
		gpio_twl4030_write(REG_GPIO_SIH_CTRL,
				TWL4030_SIH_CTRL_COR_MASK);

		/*
		 * Install an irq handler to demultiplex the gpio module
		 * interrupt.
		 */
		set_irq_chained_handler(irq, do_twl4030_gpio_module_irq);
		wake_up_process(gpio_helper_thread);

		dev_info(&pdev->dev, "IRQ %d chains IRQs %d..%d\n", irq,
			twl4030_gpio_irq_base, twl4030_gpio_irq_end - 1);
	}

no_irqs:
	if (!ret) {
		/*
		 * NOTE:  boards may waste power if they don't set pullups
		 * and pulldowns correctly ... default for non-ULPI pins is
		 * pulldown, and some other pins may have external pullups
		 * or pulldowns.  Careful!
		 */
		ret = gpio_twl4030_pulls(pdata->pullups, pdata->pulldowns);
		if (ret)
			dev_dbg(&pdev->dev, "pullups %.05x %.05x --> %d\n",
					pdata->pullups, pdata->pulldowns,
					ret);

		twl_gpiochip.base = pdata->gpio_base;
		twl_gpiochip.ngpio = TWL4030_GPIO_MAX;
		twl_gpiochip.dev = &pdev->dev;

		ret = gpiochip_add(&twl_gpiochip);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"could not register gpiochip, %d\n",
					ret);
			twl_gpiochip.ngpio = 0;
			gpio_twl4030_remove(pdev);
		} else if (pdata->setup) {
			int status;

			status = pdata->setup(&pdev->dev,
					pdata->gpio_base, TWL4030_GPIO_MAX);
			if (status)
				dev_dbg(&pdev->dev, "setup --> %d\n", status);
		}
	}

	return ret;
}

static int __devexit gpio_twl4030_remove(struct platform_device *pdev)
{
	struct twl4030_gpio_platform_data *pdata = pdev->dev.platform_data;
	int status;
	int irq;

	if (pdata->teardown) {
		status = pdata->teardown(&pdev->dev,
				pdata->gpio_base, TWL4030_GPIO_MAX);
		if (status) {
			dev_dbg(&pdev->dev, "teardown --> %d\n", status);
			return status;
		}
	}

	status = gpiochip_remove(&twl_gpiochip);
	if (status < 0)
		return status;

	if (is_module() || (twl4030_gpio_irq_end - twl4030_gpio_irq_base) <= 0)
		return 0;

	/* uninstall the gpio demultiplexing interrupt handler */
	irq = platform_get_irq(pdev, 0);
	set_irq_handler(irq, NULL);

	/* uninstall the irq handler for each of the gpio interrupts */
	for (irq = twl4030_gpio_irq_base; irq < twl4030_gpio_irq_end; irq++)
		set_irq_handler(irq, NULL);

	/* stop the gpio helper kernel thread */
	if (gpio_helper_thread) {
		kthread_stop(gpio_helper_thread);
		gpio_helper_thread = NULL;
	}

	return 0;
}

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:twl4030_gpio");

static struct platform_driver gpio_twl4030_driver = {
	.driver.name	= "twl4030_gpio",
	.driver.owner	= THIS_MODULE,
	.probe		= gpio_twl4030_probe,
	.remove		= __devexit_p(gpio_twl4030_remove),
};

static int __init gpio_twl4030_init(void)
{
	return platform_driver_register(&gpio_twl4030_driver);
}
subsys_initcall(gpio_twl4030_init);

static void __exit gpio_twl4030_exit(void)
{
	platform_driver_unregister(&gpio_twl4030_driver);
}
module_exit(gpio_twl4030_exit);

MODULE_AUTHOR("Texas Instruments, Inc.");
MODULE_DESCRIPTION("GPIO interface for TWL4030");
MODULE_LICENSE("GPL");
