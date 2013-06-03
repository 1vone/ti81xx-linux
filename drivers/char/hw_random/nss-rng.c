/*
 * drivers/char/hw_random/nss-rng.c
 *
 * Copyright (c) 2011 Texas Instruments
 * TRNG driver for NSS - Herman Schuurman <herman@ti.com>
 *
 * derived from omap-rng.c.
 *
 * Author: Deepak Saxena <dsaxena@plexity.net>
 *
 * Copyright 2005 (c) MontaVista Software, Inc.
 *
 * Mostly based on original driver:
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/random.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/hw_random.h>
#include <linux/delay.h>

#include <mach/hardware.h>
#include <asm/io.h>

#define NSS_SEC_TRNG_BASE	(TI81XX_SEC_BASE + 0x00130000)
/* ==================================================================== */
/** RNG module layout.
 */
/* ==================================================================== */
#define	RNG_REG_OUTPUT_L		0x00
#define	RNG_REG_OUTPUT_H		0x04

#define	RNG_REG_STATUS			0x08
#define	RNG_REG_STATUS_NEED_CLK		(1 << 31)
#define	RNG_REG_STATUS_SHUTDOWN_OFLO	(1 << 1)
#define	RNG_REG_STATUS_RDY		(1 << 0)

#define	RNG_REG_IMASK			0x0C
#define	RNG_REG_IMASK_SHUTDOWN_OFLO	(1 << 1)
#define	RNG_REG_IMASK_RDY		(1 << 0)

#define	RNG_REG_INTACK			0x10
#define	RNG_REG_INTACK_SHUTDOWN_OFLO	(1 << 1)
#define	RNG_REG_INTACK_RDY		(1 << 0)

#define	RNG_REG_CONTROL			0x14
#define	RNG_REG_CONTROL_STARTUP_MASK	0xFFFF0000
#define	RNG_REG_CONTROL_ENABLE_TRNG	(1 << 10)
#define	RNG_REG_CONTROL_NO_LFSR_FB	(1 << 2)

#define	RNG_REG_CONFIG			0x18
#define	RNG_REG_CONFIG_MAX_REFILL_MASK	0xFFFF0000
#define	RNG_REG_CONFIG_SAMPLE_DIV	0x00000F00
#define	RNG_REG_CONFIG_MIN_REFILL_MASK	0x000000FF

#define	RNG_REG_ALARMCNT		0x1C
#define	RNG_REG_ALARMCNT_SHTDWN_MASK	0x3F000000
#define	RNG_REG_ALARMCNT_SD_THLD_MASK	0x001F0000
#define	RNG_REG_ALARMCNT_ALM_THLD_MASK	0x000000FF

#define	RNG_REG_FROENABLE		0x20
#define	RNG_REG_FRODETUNE		0x24
#define	RNG_REG_ALARMMASK		0x28
#define	RNG_REG_ALARMSTOP		0x2C
#define	RNG_REG_LFSR_L			0x30
#define	RNG_REG_LFSR_M			0x34
#define	RNG_REG_LFSR_H			0x38
#define	RNG_REG_COUNT			0x3C
#define	RNG_REG_TEST			0x40

#define	RNG_REG_OPTIONS			0x78
#define	RNG_REG_OPTIONS_NUM_FROS_MASK	0x00000FC0

#define	RNG_REG_EIP_REV			0x7C
#define	RNG_REG_STATUS_EN		0x1FD8
#define	RNG_REG_STATUS_EN_SHUTDOWN_OFLO	(1 << 1)
#define	RNG_REG_STATUS_EN_RDY		(1 << 0)

#define	RNG_REG_REV			0x1FE0
#define	 RNG_REG_REV_X_MAJOR_MASK	(0x0F << 4)
#define	 RNG_REG_REV_Y_MINOR_MASK	(0x0F << 0)

#define	RNG_REG_SYSCFG			0x1FE4
#define	RNG_REG_SYSCFG_SIDLEMODE_MASK	(3 << 3)
#define	 RNG_REG_SYSCFG_SIDLEMODE_FORCE	(0 << 3)
#define	 RNG_REG_SYSCFG_SIDLEMODE_NO	(1 << 3)
#define	 RNG_REG_SYSCFG_SIDLEMODE_SMART	(2 << 3)
#define	RNG_REG_SYSCFG_AUTOIDLE		(1 << 0)

#define	RNG_REG_STATUS_SET		0x1FEC
#define	RNG_REG_STATUS_SET_SHUTDOWN_OFLO (1 << 1)
#define	RNG_REG_STATUS_SET_RDY		(1 << 0)

#define	RNG_REG_SOFT_RESET		0x1FF0
#define	RNG_REG_SOFTRESET		(1 << 0)

#define	RNG_REG_IRQ_EOI			0x1FF4
#define	RNG_REG_IRQ_EOI_PULSE_INT_CLEAR	(1 << 0)

#define	RNG_REG_IRQSTATUS		0x1FF8
#define	RNG_REG_IRQSTATUS_IRQ_EN	(1 << 0)


static void __iomem *rng_base;
static struct platform_device *rng_dev;

#define trng_read(reg)						\
({								\
	u32 __val;						\
	__val = __raw_readl(rng_base + RNG_REG_##reg);		\
})

#define trng_write(val, reg)					\
({								\
	__raw_writel((val), rng_base + RNG_REG_##reg);		\
})

static int nss_rng_data_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	int res, i;

	for (i = 0; i < 20; i++) {
		res = trng_read(STATUS) & RNG_REG_STATUS_RDY;
		if (res || !wait)
			break;
		/* RNG produces data fast enough (2+ MBit/sec, even
		 * during "rngtest" loads, that these delays don't
		 * seem to trigger.  We *could* use the RNG IRQ, but
		 * that'd be higher overhead ... so why bother?
		 */
		udelay(10);
	}

	/* If we have data waiting, collect it... */
	if (res) {
		*(u32 *)buf = trng_read(OUTPUT_L);
		buf += sizeof(u32);
		*(u32 *)buf = trng_read(OUTPUT_H);

		trng_write(RNG_REG_INTACK_RDY, INTACK);

		res = 2  * sizeof(u32);
	}
	return res;
}

static struct hwrng nss_rng_ops = {
	.name		= "nss",
	.read		= nss_rng_data_read,
};

static int __devinit nss_rng_probe(struct platform_device *pdev)
{
	unsigned long base;
	int ret;
	u32 reg;

	/*
	 * A bit ugly, and it will never actually happen but there can
	 * be only one RNG and this catches any bork
	 */
	if (rng_dev)
		return -EBUSY;

	/* This should be handled as a resource, but for the time
	 * being let's just wing it.... */
	base = NSS_SEC_TRNG_BASE;
	rng_base = ioremap(base, SZ_8K);
	if (!rng_base) {
		ret = -ENOMEM;
		goto err_ioremap;
	}

	ret = hwrng_register(&nss_rng_ops);
	if (ret)
		goto err_register;

	reg = trng_read(REV);
	dev_info(&pdev->dev, "NSS Random Number Generator ver. %u.%u\n",
		 ((reg & RNG_REG_REV_X_MAJOR_MASK) >> 4),
		 (reg & RNG_REG_REV_Y_MINOR_MASK));

	rng_dev = pdev;

	/* start TRNG if not running yet */
	if (!(trng_read(CONTROL) & RNG_REG_CONTROL_ENABLE_TRNG)) {
		trng_write(0x00220021, CONFIG);
		trng_write(0x00210400, CONTROL);
	}

	return 0;

err_register:
	iounmap(rng_base);
	rng_base = NULL;
err_ioremap:
	return ret;
}

static int __exit nss_rng_remove(struct platform_device *pdev)
{
	hwrng_unregister(&nss_rng_ops);

	trng_write(trng_read(CONTROL) & ~RNG_REG_CONTROL_ENABLE_TRNG, CONTROL);

	iounmap(rng_base);

	rng_base = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int nss_rng_suspend(struct platform_device *pdev, pm_message_t message)
{
	trng_write(trng_read(CONTROL) & ~RNG_REG_CONTROL_ENABLE_TRNG, CONTROL);

	return 0;
}

static int nss_rng_resume(struct platform_device *pdev)
{
	trng_write(trng_read(CONTROL) | RNG_REG_CONTROL_ENABLE_TRNG, CONTROL);

	return 0;
}

#else

#define	nss_rng_suspend		NULL
#define	nss_rng_resume		NULL

#endif

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:nss_rng");

static struct platform_driver nss_rng_driver = {
	.driver = {
		.name		= "nss_rng",
		.owner		= THIS_MODULE,
	},
	.probe		= nss_rng_probe,
	.remove		= __exit_p(nss_rng_remove),
	.suspend	= nss_rng_suspend,
	.resume		= nss_rng_resume
};

static int __init nss_rng_init(void)
{
	if (!cpu_is_ti81xx())
		return -ENODEV;

	return platform_driver_register(&nss_rng_driver);
}

static void __exit nss_rng_exit(void)
{
	platform_driver_unregister(&nss_rng_driver);
}

module_init(nss_rng_init);
module_exit(nss_rng_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NSS TRNG driver");
