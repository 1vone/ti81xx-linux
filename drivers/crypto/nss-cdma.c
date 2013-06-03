/*
 * linux/arch/arm/plat-omap/dma.c
 *
 * Copyright (C) 2003 - 2008 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
 * DMA channel linking for 1610 by Samuel Ortiz <samuel.ortiz@nokia.com>
 * Graphics DMA and LCD DMA graphics tranformations
 * by Imre Deak <imre.deak@nokia.com>
 * OMAP2/3 support Copyright (C) 2004-2007 Texas Instruments, Inc.
 * Merged to support both OMAP1 and OMAP2 by Tony Lindgren <tony@atomide.com>
 * Some functions based on earlier dma-omap.c Copyright (C) 2001 RidgeRun, Inc.
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * Support functions for the OMAP internal DMA channels.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
/*
 * Copyright © 2011 Texas Instruments Incorporated
 * Author: Herman Schuurman
 * Change: July 2011 - Adapted the OMAP dma.c driver to support Netra
 *	security subsystem Crypto-DMA, which is similar to the OMAP SDMA.
 *	We can't directly use the SDMA driver, due to a shim layer tying
 *	SDMA to EDMA for Netra devices.
 *	Also added interrupt redirector to reflect subsystem interrupts
 *	back to the main interrupt system.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <mach/irqs.h>
#include "nss-cdma.h"

#undef DEBUG

enum { DMA_CH_ALLOC_DONE, DMA_CH_PARAMS_SET_DONE, DMA_CH_STARTED,
	DMA_CH_QUEUED, DMA_CH_NOTSTARTED, DMA_CH_PAUSED, DMA_CH_LINK_ENABLED
};

enum { DMA_CHAIN_STARTED, DMA_CHAIN_NOTSTARTED };

#define NSS_CDMA_ACTIVE		0x01
#define NSS_CDMA_CSR_CLEAR_MASK	0xffe

struct nss_cdma_lch {
	int dev_id;
	u16 saved_csr;
	u16 enabled_irqs;
	const char *dev_name;
	void (*callback)(int lch, u16 ch_status, void *data);
	void *data;

	long flags;
};

static u32 nss_irqs = 1 << NSS_IRQ_CDMA_INT_REQ_0;
static struct {
	void (*func)(int irq, void *dev_id);
	void *dev_id;
} irq_handler[32];

static int dma_lch_count;
static int dma_chan_count;

static spinlock_t dma_chan_lock;
static struct nss_cdma_lch *dma_chan;
static void __iomem *nss_cdma_base;
static void __iomem *ctrl_base;

static inline void nss_enable_channel_irq(int lch);

#define cdma_read(reg)						\
({								\
	u32 __val;						\
	__val = __raw_readl(nss_cdma_base + NSS_CDMA_##reg);	\
})

#define cdma_write(val, reg)					\
({								\
	__raw_writel((val), nss_cdma_base + NSS_CDMA_##reg);	\
})

#define ctrl_read(reg)						\
({								\
	u32 __val;						\
	__val = __raw_readl(ctrl_base + NSS_PUB_##reg);		\
})

#define ctrl_write(val, reg)					\
({								\
	__raw_writel((val), ctrl_base + NSS_PUB_##reg);		\
})


void nss_set_cdma_transfer_params(int lch, int data_type, int elem_count,
				  int frame_count, int sync_mode,
				  int dma_trigger, int src_or_dst_synch)
{
	u32 l;

	l = cdma_read(CSDP(lch));
	l &= ~0x03;
	l |= data_type;
	cdma_write(l, CSDP(lch));

	if (dma_trigger) {
		u32 val;

		val = cdma_read(CCR(lch));

		/* DMA_SYNCHRO_CONTROL_UPPER depends on the channel number */
		val &= ~((3 << 19) | 0x1f);
		val |= (dma_trigger & ~0x1f) << 14;
		val |= dma_trigger & 0x1f;

		if (sync_mode & NSS_CDMA_SYNC_FRAME)
			val |= 1 << 5;
		else
			val &= ~(1 << 5);

		if (sync_mode & NSS_CDMA_SYNC_BLOCK)
			val |= 1 << 18;
		else
			val &= ~(1 << 18);

		if (src_or_dst_synch == NSS_CDMA_DST_SYNC_PREFETCH) {
			val &= ~(1 << 24);	/* dest synch */
			val |= (1 << 23);	/* Prefetch */
		} else if (src_or_dst_synch)
			val |= 1 << 24;		/* source synch */
		else
			val &= ~(1 << 24);	/* dest synch */

		cdma_write(val, CCR(lch));
	}

	cdma_write(elem_count, CEN(lch));
	cdma_write(frame_count, CFN(lch));
}
EXPORT_SYMBOL(nss_set_cdma_transfer_params);

/* Note that src_port is only for omap1 */
void nss_set_cdma_src_params(int lch, int src_port, int src_amode,
			     unsigned long src_start,
			     int src_ei, int src_fi)
{
	u32 l;

	l = cdma_read(CCR(lch));
	l &= ~(0x03 << 12);
	l |= src_amode << 12;
	cdma_write(l, CCR(lch));
	cdma_write(src_start, CSSA(lch));
	cdma_write(src_ei, CSEI(lch));
	cdma_write(src_fi, CSFI(lch));
}
EXPORT_SYMBOL(nss_set_cdma_src_params);

void nss_set_cdma_src_burst_mode(int lch, enum nss_cdma_burst_mode burst_mode)
{
	unsigned int burst = 0;
	u32 l;

	l = cdma_read(CSDP(lch));
	l &= ~(0x03 << 7);

	switch (burst_mode) {
	case NSS_CDMA_DATA_BURST_DIS:
		break;
	case NSS_CDMA_DATA_BURST_4:
		burst = 0x1;
	case NSS_CDMA_DATA_BURST_8:
		burst = 0x2;
		break;
	case NSS_CDMA_DATA_BURST_16:
		burst = 0x3;
		break;
	default:
		BUG();
	}

	l |= (burst << 7);
	cdma_write(l, CSDP(lch));
}
EXPORT_SYMBOL(nss_set_cdma_src_burst_mode);

/* Note that dest_port is only for OMAP1 */
void nss_set_cdma_dest_params(int lch, int dest_port, int dest_amode,
			      unsigned long dest_start,
			      int dst_ei, int dst_fi)
{
	u32 l;

	l = cdma_read(CCR(lch));
	l &= ~(0x03 << 14);
	l |= dest_amode << 14;
	cdma_write(l, CCR(lch));
	cdma_write(dest_start, CDSA(lch));
	cdma_write(dst_ei, CDEI(lch));
	cdma_write(dst_fi, CDFI(lch));
}
EXPORT_SYMBOL(nss_set_cdma_dest_params);

void nss_set_cdma_dest_burst_mode(int lch, enum nss_cdma_burst_mode burst_mode)
{
	unsigned int burst = 0;
	u32 l;

	l = cdma_read(CSDP(lch));
	l &= ~(0x03 << 14);

	switch (burst_mode) {
	case NSS_CDMA_DATA_BURST_DIS:
		break;
	case NSS_CDMA_DATA_BURST_4:
		burst = 0x1;
	case NSS_CDMA_DATA_BURST_8:
		burst = 0x2;
	case NSS_CDMA_DATA_BURST_16:
		burst = 0x3;
		break;
	default:
		printk(KERN_ERR "Invalid DMA burst mode\n");
		BUG();
		return;
	}
	l |= (burst << 14);
	cdma_write(l, CSDP(lch));
}
EXPORT_SYMBOL(nss_set_cdma_dest_burst_mode);

static inline void nss_enable_channel_irq(int lch)
{
	/* Clear CSR */
	cdma_write(NSS_CDMA_CSR_CLEAR_MASK, CSR(lch));

	/* Enable some nice interrupts. */
	cdma_write(dma_chan[lch].enabled_irqs, CICR(lch));
}

static inline void nss_enable_irq_lch(int lch)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&dma_chan_lock, flags);
	val = cdma_read(IRQENABLE_L0);
	val |= 1 << lch;
	cdma_write(val, IRQENABLE_L0);
	spin_unlock_irqrestore(&dma_chan_lock, flags);
}

static inline void nss_disable_irq_lch(int lch)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(&dma_chan_lock, flags);
	val = cdma_read(IRQENABLE_L0);
	val &= ~(1 << lch);
	cdma_write(val, IRQENABLE_L0);
	spin_unlock_irqrestore(&dma_chan_lock, flags);
}

int nss_request_cdma(int dev_id, const char *dev_name,
		     void (*callback)(int lch, u16 ch_status, void *data),
		     void *data, int *dma_ch_out)
{
	int ch, free_ch = -1;
	unsigned long flags;
	struct nss_cdma_lch *chan;

	spin_lock_irqsave(&dma_chan_lock, flags);
	for (ch = 0; ch < dma_chan_count; ch++) {
		if (free_ch == -1 && dma_chan[ch].dev_id == -1) {
			free_ch = ch;
			if (dev_id == 0)
				break;
		}
	}
	if (free_ch == -1) {
		spin_unlock_irqrestore(&dma_chan_lock, flags);
		return -EBUSY;
	}
	chan = dma_chan + free_ch;
	chan->dev_id = dev_id;

	nss_clear_cdma(free_ch);

	spin_unlock_irqrestore(&dma_chan_lock, flags);

	chan->dev_name = dev_name;
	chan->callback = callback;
	chan->data = data;
	chan->flags = 0;

	chan->enabled_irqs = NSS_CDMA_DROP_IRQ | NSS_CDMA_BLOCK_IRQ;

	chan->enabled_irqs |= NSS_CDMA_MISALIGNED_ERR_IRQ |
		NSS_CDMA_TRANS_ERR_IRQ;

	nss_enable_irq_lch(free_ch);
	nss_enable_channel_irq(free_ch);
	/* Clear the CSR register and IRQ status register */
	cdma_write(NSS_CDMA_CSR_CLEAR_MASK, CSR(free_ch));
	cdma_write(1 << free_ch, IRQSTATUS_L0);

	*dma_ch_out = free_ch;

	return 0;
}
EXPORT_SYMBOL(nss_request_cdma);

void nss_free_cdma(int lch)
{
	unsigned long flags;

	if (dma_chan[lch].dev_id == -1) {
		pr_err("nss_cdma: trying to free unallocated DMA channel %d\n",
		       lch);
		return;
	}

	nss_disable_irq_lch(lch);

	/* Clear the CSR register and IRQ status register */
	cdma_write(NSS_CDMA_CSR_CLEAR_MASK, CSR(lch));
	cdma_write(1 << lch, IRQSTATUS_L0);

	/* Disable all DMA interrupts for the channel. */
	cdma_write(0, CICR(lch));

	/* Make sure the DMA transfer is stopped. */
	cdma_write(0, CCR(lch));
	nss_clear_cdma(lch);

	spin_lock_irqsave(&dma_chan_lock, flags);
	dma_chan[lch].dev_id = -1;
	dma_chan[lch].callback = NULL;
	spin_unlock_irqrestore(&dma_chan_lock, flags);
}
EXPORT_SYMBOL(nss_free_cdma);

/**
 * @brief nss_cdma_set_global_params : Set global priority settings for dma
 *
 * @param arb_rate
 * @param max_fifo_depth
 * @param tparams - Number of threads to reserve : DMA_THREAD_RESERVE_NORM
 * 						   DMA_THREAD_RESERVE_ONET
 * 						   DMA_THREAD_RESERVE_TWOT
 * 						   DMA_THREAD_RESERVE_THREET
 */
void
nss_cdma_set_global_params(int arb_rate, int max_fifo_depth, int tparams)
{
	u32 reg;

	if (max_fifo_depth == 0)
		max_fifo_depth = 1;
	if (arb_rate == 0)
		arb_rate = 1;

	reg = 0xff & max_fifo_depth;
	reg |= (0x3 & tparams) << 12;
	reg |= (arb_rate & 0xff) << 16;

	cdma_write(reg, GCR);
}
EXPORT_SYMBOL(nss_cdma_set_global_params);

/*
 * Clears any DMA state so the DMA engine is ready to restart with new buffers
 * through nss_start_cdma(). Any buffers in flight are discarded.
 */
void nss_clear_cdma(int lch)
{
	unsigned long flags;
	int i;
	void __iomem *lch_base;

	local_irq_save(flags);

	lch_base = nss_cdma_base + NSS_CDMA_CH_BASE(lch);
	for (i = 0; i < 0x44; i += 4)
		__raw_writel(0, lch_base + i);

	local_irq_restore(flags);
}
EXPORT_SYMBOL(nss_clear_cdma);

void nss_start_cdma(int lch)
{
	u32 l;

	/*
	 * The CPC/CDAC register needs to be initialized to zero
	 * before starting dma transfer.
	 */
	cdma_write(0, CDAC(lch));

	nss_enable_channel_irq(lch);

	l = cdma_read(CCR(lch));
	l |= NSS_CDMA_CCR_EN;
	cdma_write(l, CCR(lch));

	dma_chan[lch].flags |= NSS_CDMA_ACTIVE;
}
EXPORT_SYMBOL(nss_start_cdma);

void nss_stop_cdma(int lch)
{
	u32 l;

	/* Disable all interrupts on the channel */
	l = cdma_read(CCR(lch));
	l &= ~NSS_CDMA_CCR_EN;
	cdma_write(l, CCR(lch));

	dma_chan[lch].flags &= ~NSS_CDMA_ACTIVE;
}
EXPORT_SYMBOL(nss_stop_cdma);

/*----------------------------------------------------------------------------*/

static int nss_cdma_handle_ch(int ch)
{
	u32 status = cdma_read(CSR(ch));

	if (!status) {
		if (printk_ratelimit())
			printk(KERN_WARNING "Spurious DMA IRQ for lch %d\n",
				ch);
		cdma_write(1 << ch, IRQSTATUS_L0);
		return 0;
	}
	if (unlikely(dma_chan[ch].dev_id == -1)) {
		if (printk_ratelimit())
			printk(KERN_WARNING "IRQ %04x for non-allocated DMA"
					"channel %d\n", status, ch);
		return 0;
	}
	if (unlikely(status & NSS_CDMA_DROP_IRQ))
		printk(KERN_INFO
		       "DMA synchronization event drop occurred with device "
		       "%d\n", dma_chan[ch].dev_id);
	if (unlikely(status & NSS_CDMA_TRANS_ERR_IRQ)) {
		u32 ccr;

		printk(KERN_INFO "DMA transaction error with device %d\n",
		       dma_chan[ch].dev_id);
		/*
		 * Errata: sDMA Channel is not disabled
		 * after a transaction error. So we explicitely
		 * disable the channel
		 */
		ccr = cdma_read(CCR(ch));
		ccr &= ~NSS_CDMA_CCR_EN;
		cdma_write(ccr, CCR(ch));
		dma_chan[ch].flags &= ~NSS_CDMA_ACTIVE;
	}
	if (unlikely(status & NSS_CDMA_SECURE_ERR_IRQ))
		printk(KERN_INFO "DMA secure error with device %d\n",
		       dma_chan[ch].dev_id);
	if (unlikely(status & NSS_CDMA_MISALIGNED_ERR_IRQ))
		printk(KERN_INFO "DMA misaligned error with device %d\n",
		       dma_chan[ch].dev_id);

	cdma_write(NSS_CDMA_CSR_CLEAR_MASK, CSR(ch));
	cdma_write(1 << ch, IRQSTATUS_L0);

	cdma_write(status, CSR(ch));

	if (likely(dma_chan[ch].callback != NULL))
		dma_chan[ch].callback(ch, status, dma_chan[ch].data);

	return 0;
}

int nss_attach_irq(int irq, void (*handler)(int irq, void *dev_id), void *dev_id)
{
	if (irq < 0 || irq > 31) {
		printk(KERN_INFO "NSS IRQ %d unknown\n", irq);
		goto out_fail;
	}
	irq_handler[irq].func = handler;
	irq_handler[irq].dev_id = dev_id;
	nss_irqs |= (1 << irq);
	ctrl_write(nss_irqs, HWI_STATUS_ENABLE_SET);
	ctrl_write(nss_irqs, INT_EN);
	return 0;

 out_fail:
	return -1;
}
EXPORT_SYMBOL(nss_attach_irq);

int nss_free_irq(int irq)
{
	if (irq < 0 || irq > 31) {
		printk(KERN_INFO "NSS IRQ %d unknown\n", irq);
		goto out_fail;
	}
	nss_irqs &= ~(1 << irq);
	ctrl_write(nss_irqs, HWI_STATUS_ENABLE_SET);
	ctrl_write(nss_irqs, INT_EN);
	irq_handler[irq].func = NULL;
	return 0;

 out_fail:
	return -1;
}
EXPORT_SYMBOL(nss_free_irq);

static irqreturn_t ti81xx_nss_irq_handler(int irq, void *dev_id)
{
	u32 val, ack;
	int i;

	ack = val = ctrl_read(HWI_STATUS);

	for (i = 0; i < 32 && val != 0; i++) {
		if (val & 1)
			irq_handler[i].func(i, irq_handler[i].dev_id);
		val >>= 1;
	}
	
	ctrl_write(ack, HWI_STATUS_CLR);
	return IRQ_HANDLED;
}

/* STATUS register count is from 1-16 while ours is 0-15 */
void ti81xx_secpint_irq_handler(int irq, void *dev_id)
{
	u32 val, enable_reg;
	int i;

	val = cdma_read(IRQSTATUS_L0);
	if (val == 0) {
		if (printk_ratelimit())
			printk(KERN_WARNING "Spurious DMA IRQ\n");
		goto done;
	}
	enable_reg = cdma_read(IRQENABLE_L0);
	val &= enable_reg; /* Dispatch only relevant interrupts */
	for (i = 0; i < dma_lch_count && val != 0; i++) {
		if (val & 1)
			nss_cdma_handle_ch(i);
		val >>= 1;
	}

 done:
	return;
}

static struct irqaction ti81xx_secpint_irq = {
	.name = "CDMA",
	.handler = ti81xx_nss_irq_handler,
	.flags = IRQF_DISABLED
};

/*----------------------------------------------------------------------------*/

static int __init nss_init_cdma(void)
{
	unsigned long base;
	int ch, r;
	u32 revision;
	int irq;
	u32 v;

	base = NSS_CDMA_BASE;
	dma_lch_count = NSS_CDMA_LOGICAL_DMA_CH_COUNT;

	nss_cdma_base = ioremap(base, SZ_4K);
	BUG_ON(!nss_cdma_base);

	ctrl_base = ioremap(NSS_SEC_REGS_BASE, SZ_16K);
	BUG_ON(!ctrl_base);

	/* enable DMA interrupt feed-through from Security SS */
	ctrl_write(nss_irqs, INT_EN);
	ctrl_write(nss_irqs, HWI_STATUS_ENABLE_SET);

	revision = cdma_read(REVISION);

	dma_chan = kzalloc(sizeof(struct nss_cdma_lch) * dma_lch_count,
				GFP_KERNEL);
	if (!dma_chan) {
		r = -ENOMEM;
		goto out_unmap;
	}

	irq = TI81XX_IRQ_SECPUBINT;
	printk(KERN_INFO "NSS Crypto DMA hardware revision %d.%d @ IRQ %d\n",
	       revision >> 16, (revision >> 8) & 0xff, irq);
	dma_chan_count = dma_lch_count;

	spin_lock_init(&dma_chan_lock);

	for (ch = 0; ch < dma_chan_count; ch++) {
		nss_clear_cdma(ch);
		nss_disable_irq_lch(ch);

		dma_chan[ch].dev_id = -1;
	}

	nss_cdma_set_global_params(DMA_DEFAULT_ARB_RATE,
			DMA_DEFAULT_FIFO_DEPTH, 0);

	for (ch = 0; ch < 32; ch++) {
		irq_handler[ch].func = NULL;
	}

	irq_handler[NSS_IRQ_CDMA_INT_REQ_0].func = ti81xx_secpint_irq_handler;
	irq_handler[NSS_IRQ_CDMA_INT_REQ_0].dev_id = NULL;

	r = setup_irq(irq, &ti81xx_secpint_irq);
	if (r) {
		printk(KERN_INFO "set_up failed for IRQ %d for DMA (error %d)\n", irq, r);
		goto out_free;
	}

	/* Enable smartidle idlemodes and autoidle */
	v = cdma_read(OCP_SYSCONFIG);
	v &= ~(DMA_SYSCONFIG_MIDLEMODE_MASK |
			DMA_SYSCONFIG_SIDLEMODE_MASK |
			DMA_SYSCONFIG_AUTOIDLE);
	v |= (DMA_SYSCONFIG_MIDLEMODE(DMA_IDLEMODE_SMARTIDLE) |
		DMA_SYSCONFIG_SIDLEMODE(DMA_IDLEMODE_SMARTIDLE) |
		DMA_SYSCONFIG_AUTOIDLE);
	cdma_write(v , OCP_SYSCONFIG);

	return 0;

out_free:
	kfree(dma_chan);

out_unmap:
	iounmap(ctrl_base);
	iounmap(nss_cdma_base);

	return r;
}

arch_initcall(nss_init_cdma);

