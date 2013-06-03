/*
 * arch/arm/plat-omap/include/mach/dma.h.
 *
 *  Copyright (C) 2003 Nokia Corporation
 *  Author: Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
/*
 * Copyright © 2011 Texas Instruments Incorporated
 * Author: Herman Schuurman
 * Change: July 2011 - Adapted the OMAP dma.h include file to support Netra
 *	security subsystem Crypto-DMA, which is similar to the OMAP SDMA.
 *	We can't directly use the SDMA driver, due to a shim layer tying
 *	SDMA to EDMA for Netra devices.
 */
#ifndef __DRIVER_CRYPTO_NSS_CDMA_H
#define __DRIVER_CRYPTO_NSS_CDMA_H

/* Hardware register base for NSS */
#define	NSS_SEC_REGS_BASE	(TI81XX_SEC_BASE + 0x001F0000)

/* ==================================================================== */
/** Security register layout (interrupt routing fragment).
 */
/* ==================================================================== */
#define	NSS_PUB_HWI_EOI				0x0214
#define	NSS_PUB_HWI_STATUS_SET			0x0218
#define	NSS_PUB_HWI_STATUS_CLR			0x021C
#define	NSS_PUB_HWI_STATUS			0x0220
#define	NSS_PUB_HWI_STATUS_ENABLE_SET		0x0224
#define	NSS_PUB_HWI_STATUS_ENABLE_CLR		0x0228
#define	NSS_PUB_HWI_STATUS_ENABLE		0x022C
#define	NSS_PUB_HWI_STATUS_OUT			0x0230
#define	NSS_PUB_HWI_STATUS_SWAKEUP_ENA		0x0234

#ifdef CONFIG_ARCH_TI816X
#define	NSS_SEC_INT_EN				0x3014
#define	NSS_PUB_INT_EN				0x3018
#endif
#ifdef CONFIG_ARCH_TI814X
#define	NSS_SEC_INT_EN				0x2014
#define	NSS_PUB_INT_EN				0x2018
#endif

/* ==================================================================== */
/** Security subsystem interrupt layout.
 */
/* ==================================================================== */
#define	NSS_IRQ_AES1_S		(0)
#define	NSS_IRQ_AES1_P		(1)
#define	NSS_IRQ_SHA1_S		(2)
#define	NSS_IRQ_SHA1_PART_S	(3)
#define	NSS_IRQ_SHA1_P		(4)
#define	NSS_IRQ_SHA1_PART_P	(5)
#define	NSS_IRQ_AES2_S		(6)
#define	NSS_IRQ_AES2_P		(7)
#define	NSS_IRQ_SHA2_S		(8)
#define	NSS_IRQ_SHA2_PART_S	(9)
#define	NSS_IRQ_SHA2_P		(10)
#define	NSS_IRQ_SHA2_PART_P	(11)
#define	NSS_IRQ_FPKA		(12)
#define	NSS_IRQ_DES_S		(13)
#define	NSS_IRQ_DES_P		(14)
#define	NSS_IRQ_RNG		(15)
#define	NSS_IRQ_CDMA_INT_REQ_0	(16)
#define	NSS_IRQ_CDMA_INT_REQ_1	(17)
#define	NSS_IRQ_CDMA_INT_REQ_2	(18)
#define	NSS_IRQ_CDMA_INT_REQ_3	(19)
#define	NSS_IRQ_CDMA_SECURE_ERR	(20)
#define	NSS_IRQ_FW_ERR_DWRT_FNC	(21)
#define	NSS_IRQ_FW_ERR_DRD_FNC	(22)
#define	NSS_IRQ_FW_ERR_L3_FNC	(23)
#define	NSS_IRQ_FW_ERR_DWRT_DBG	(24)
#define	NSS_IRQ_FW_ERR_DRD_DBG	(25)
#define	NSS_IRQ_FW_ERR_L3_DBG	(26)
#define	NSS_IRQ_WDT		(27)
#define	NSS_IRQ_TIMER_INT_0	(28)
#define	NSS_IRQ_TIMER_INT_1	(29)
#define	NSS_IRQ_TIMER_INT_2	(30)
#define	NSS_IRQ_TIMER_INT_3	(31)

extern int nss_attach_irq(int irq, void (*handler)(int irq, void *dev_id), void *dev_id);
extern int nss_free_irq(int irq);

/* ==================================================================== */

#define	NSS_CDMA_BASE			(TI81XX_SEC_BASE + 0x1E0000)

#define NSS_CDMA_REVISION		0x00
#define NSS_CDMA_IRQSTATUS_L0		0x08
#define NSS_CDMA_IRQSTATUS_L1		0x0c
#define NSS_CDMA_IRQSTATUS_L2		0x10
#define NSS_CDMA_IRQSTATUS_L3		0x14
#define NSS_CDMA_IRQENABLE_L0		0x18
#define NSS_CDMA_IRQENABLE_L1		0x1c
#define NSS_CDMA_IRQENABLE_L2		0x20
#define NSS_CDMA_IRQENABLE_L3		0x24
#define NSS_CDMA_SYSSTATUS		0x28
#define NSS_CDMA_OCP_SYSCONFIG		0x2c
#define NSS_CDMA_CAPS_0			0x64
#define NSS_CDMA_CAPS_2			0x6c
#define NSS_CDMA_CAPS_3			0x70
#define NSS_CDMA_CAPS_4			0x74
#define NSS_CDMA_GCR			0x78

#define NSS_CDMA_LOGICAL_DMA_CH_COUNT	16

/* Common channel specific registers */
#define NSS_CDMA_CH_BASE(n)		(0x60 * (n) + 0x80)
#define NSS_CDMA_CCR(n)			(0x60 * (n) + 0x80)
#define NSS_CDMA_CLNK_CTRL(n)		(0x60 * (n) + 0x84)
#define NSS_CDMA_CICR(n)		(0x60 * (n) + 0x88)
#define NSS_CDMA_CSR(n)			(0x60 * (n) + 0x8c)
#define NSS_CDMA_CSDP(n)		(0x60 * (n) + 0x90)
#define NSS_CDMA_CEN(n)			(0x60 * (n) + 0x94)
#define NSS_CDMA_CFN(n)			(0x60 * (n) + 0x98)
#define NSS_CDMA_CSSA(n)		(0x60 * (n) + 0x9c)
#define NSS_CDMA_CDSA(n)		(0x60 * (n) + 0xa0)
#define NSS_CDMA_CSEI(n)		(0x60 * (n) + 0xa4)
#define NSS_CDMA_CSFI(n)		(0x60 * (n) + 0xa8)
#define NSS_CDMA_CDEI(n)		(0x60 * (n) + 0xac)
#define NSS_CDMA_CDFI(n)		(0x60 * (n) + 0xb0)
#define NSS_CDMA_CSAC(n)		(0x60 * (n) + 0xb4)
#define NSS_CDMA_CDAC(n)		(0x60 * (n) + 0xb8)
#define NSS_CDMA_CCEN(n)		(0x60 * (n) + 0xbc)
#define NSS_CDMA_CCFN(n)		(0x60 * (n) + 0xc0)
#define NSS_CDMA_COLOR(n)		(0x60 * (n) + 0xc4)
#define NSS_CDMA_CDP(n)			(0x60 * (n) + 0xd0)
#define NSS_CDMA_CNDP(n)		(0x60 * (n) + 0xd4)
#define NSS_CDMA_CCDN(n)		(0x60 * (n) + 0xd8)

/* DMA channel for NSS */
#define	NSS_CDMA_SHA1_P_CTX_OUT_REQ	1	/* CDMA_0 */
#define	NSS_CDMA_SHA1_P_DATA_IN_REQ	2	/* CDMA_1 */
#define	NSS_CDMA_SHA1_P_CTX_IN_REQ	3	/* CDMA_2 */
#define	NSS_CDMA_SHA1_S_CTX_OUT_REQ	4	/* CDMA_3 */
#define	NSS_CDMA_SHA1_S_DATA_IN_REQ	5	/* CDMA_4 */
#define	NSS_CDMA_SHA1_S_CTX_IN_REQ	6	/* CDMA_5 */
#define	NSS_CDMA_SHA2_P_CTX_OUT_REQ	7	/* CDMA_6 */
#define	NSS_CDMA_SHA2_P_DATA_IN_REQ	8	/* CDMA_7 */
#define	NSS_CDMA_SHA2_P_CTX_IN_REQ	9	/* CDMA_8 */
#define	NSS_CDMA_SHA2_S_DATA_IN_REQ	10	/* CDMA_9 */
#define	NSS_CDMA_SHA2_S_CTX_IN_REQ	11	/* CDMA_10 */
#define	NSS_CDMA_SHA2_S_CTX_OUT_REQ	12	/* CDMA_11 */
#define	NSS_CDMA_DES_P_CTX_IN_REQ	13	/* CDMA_12 */
#define	NSS_CDMA_DES_P_DATA_OUT_REQ	14	/* CDMA_13 */
#define	NSS_CDMA_DES_P_DATA_IN_REQ	15	/* CDMA_14 */
#define	NSS_CDMA_DES_S_CTX_IN_REQ	16	/* CDMA_15 */
#define	NSS_CDMA_DES_S_DATA_OUT_REQ	17	/* CDMA_16 */
#define	NSS_CDMA_DES_S_DATA_IN_REQ	18	/* CDMA_17 */
#define	NSS_CDMA_AES1_P_CTX_OUT_REQ	19	/* CDMA_18 */
#define	NSS_CDMA_AES1_P_DATA_OUT_REQ	20	/* CDMA_19 */
#define	NSS_CDMA_AES1_P_DATA_IN_REQ	21	/* CDMA_20 */
#define	NSS_CDMA_AES1_P_CTX_IN_REQ	22	/* CDMA_21 */
#define	NSS_CDMA_AES1_S_CTX_OUT_REQ	23	/* CDMA_22 */
#define	NSS_CDMA_AES1_S_DATA_OUT_REQ	24	/* CDMA_23 */
#define	NSS_CDMA_AES1_S_DATA_IN_REQ	25	/* CDMA_24 */
#define	NSS_CDMA_AES1_S_CTX_IN_REQ	26	/* CDMA_25 */
#define	NSS_CDMA_AES2_S_CTX_OUT_REQ	27	/* CDMA_26 */
#define	NSS_CDMA_AES2_S_DATA_OUT_REQ	28	/* CDMA_27 */
#define	NSS_CDMA_AES2_S_DATA_IN_REQ	29	/* CDMA_28 */
#define	NSS_CDMA_AES2_S_CTX_IN_REQ	30	/* CDMA_29 */
#define	NSS_CDMA_AES2_P_DATA_OUT_REQ	31	/* CDMA_30 */
#define	NSS_CDMA_AES2_P_DATA_IN_REQ	32	/* CDMA_31 */
#define	NSS_CDMA_AES2_P_CTX_IN_REQ	33	/* CDMA_32 */
#define	NSS_CDMA_AES2_P_CTX_OUT_REQ	34	/* CDMA_33 */

/*----------------------------------------------------------------------------*/
/** CICR bit definitions
 */
#define NSS_CDMA_DROP_IRQ		(1 << 1)
#define NSS_CDMA_HALF_IRQ		(1 << 2)
#define NSS_CDMA_FRAME_IRQ		(1 << 3)
#define NSS_CDMA_LAST_IRQ		(1 << 4)
#define NSS_CDMA_BLOCK_IRQ		(1 << 5)
#define NSS_CDMA_SYNC_IRQ		(1 << 6)
#define NSS_CDMA_PKT_IRQ		(1 << 7)
#define NSS_CDMA_TRANS_ERR_IRQ		(1 << 8)
#define NSS_CDMA_SECURE_ERR_IRQ		(1 << 9)
#define NSS_CDMA_SUPERVISOR_ERR_IRQ	(1 << 10)
#define NSS_CDMA_MISALIGNED_ERR_IRQ	(1 << 11)
#define	NSS_CDMA_DRAIN_IRQ		(1 << 12)
#define	NSS_CDMA_DOMAIN_ERR_IRQ		(1 << 13)
#define	NSS_CDMA_SUPER_BLOCK_IRQ	(1 << 14)

/** CCR bit definitions
 */
#define NSS_CDMA_CCR_EN			(1 << 7)
#define	NSS_CDMA_CCR_READ_PRIORITY	(1 << 6)

#define NSS_CDMA_DATA_TYPE_S8		0x00
#define NSS_CDMA_DATA_TYPE_S16		0x01
#define NSS_CDMA_DATA_TYPE_S32		0x02

#define NSS_CDMA_SYNC_ELEMENT		0x00
#define NSS_CDMA_SYNC_FRAME		0x01
#define NSS_CDMA_SYNC_BLOCK		0x02
#define NSS_CDMA_SYNC_PACKET		0x03

#define NSS_CDMA_DST_SYNC_PREFETCH	0x02
#define NSS_CDMA_SRC_SYNC		0x01
#define NSS_CDMA_DST_SYNC		0x00

#define NSS_CDMA_PORT_EMIFF		0x00
#define NSS_CDMA_PORT_EMIFS		0x01
#define NSS_CDMA_PORT_OCP_T1		0x02
#define NSS_CDMA_PORT_TIPB		0x03
#define NSS_CDMA_PORT_OCP_T2		0x04
#define NSS_CDMA_PORT_MPUI		0x05

#define NSS_CDMA_AMODE_CONSTANT		0x00
#define NSS_CDMA_AMODE_POST_INC		0x01
#define NSS_CDMA_AMODE_SINGLE_IDX	0x02
#define NSS_CDMA_AMODE_DOUBLE_IDX	0x03

#define DMA_DEFAULT_FIFO_DEPTH		0x10
#define DMA_DEFAULT_ARB_RATE		0x01
/* Pass THREAD_RESERVE ORed with THREAD_FIFO for tparams */
#define DMA_THREAD_RESERVE_NORM		(0x00 << 12) /* Def */
#define DMA_THREAD_RESERVE_ONET		(0x01 << 12)
#define DMA_THREAD_RESERVE_TWOT		(0x02 << 12)
#define DMA_THREAD_RESERVE_THREET	(0x03 << 12)
#define DMA_THREAD_FIFO_NONE		(0x00 << 14) /* Def */
#define DMA_THREAD_FIFO_75		(0x01 << 14)
#define DMA_THREAD_FIFO_25		(0x02 << 14)
#define DMA_THREAD_FIFO_50		(0x03 << 14)

/** OCP SysConfig bit definitions
 */
#define DMA_SYSCONFIG_MIDLEMODE_MASK		(3 << 12)
#define DMA_SYSCONFIG_CLOCKACTIVITY_MASK	(3 << 8)
#define DMA_SYSCONFIG_EMUFREE			(1 << 5)
#define DMA_SYSCONFIG_SIDLEMODE_MASK		(3 << 3)
#define DMA_SYSCONFIG_SOFTRESET			(1 << 2)
#define DMA_SYSCONFIG_AUTOIDLE			(1 << 0)

#define DMA_SYSCONFIG_MIDLEMODE(n)		((n) << 12)
#define DMA_SYSCONFIG_SIDLEMODE(n)		((n) << 3)

#define DMA_IDLEMODE_SMARTIDLE			0x2
#define DMA_IDLEMODE_NO_IDLE			0x1
#define DMA_IDLEMODE_FORCE_IDLE			0x0

/* Chaining modes*/
#define NSS_CDMA_STATIC_CHAIN		0x1
#define NSS_CDMA_DYNAMIC_CHAIN		0x2
#define NSS_CDMA_CHAIN_ACTIVE		0x1
#define NSS_CDMA_CHAIN_INACTIVE		0x0

#define DMA_CH_PRIO_HIGH		0x1
#define DMA_CH_PRIO_LOW			0x0 /* Def */

enum nss_cdma_burst_mode {
	NSS_CDMA_DATA_BURST_DIS = 0,
	NSS_CDMA_DATA_BURST_4,
	NSS_CDMA_DATA_BURST_8,
	NSS_CDMA_DATA_BURST_16,
};

enum end_type {
	NSS_CDMA_LITTLE_ENDIAN = 0,
	NSS_CDMA_BIG_ENDIAN
};

enum nss_cdma_color_mode {
	NSS_CDMA_COLOR_DIS = 0,
	NSS_CDMA_CONSTANT_FILL,
	NSS_CDMA_TRANSPARENT_COPY
};

enum nss_cdma_write_mode {
	NSS_CDMA_WRITE_NON_POSTED = 0,
	NSS_CDMA_WRITE_POSTED,
	NSS_CDMA_WRITE_LAST_NON_POSTED
};

enum nss_cdma_channel_mode {
	NSS_CDMA_LCH_2D = 0,
	NSS_CDMA_LCH_G,
	NSS_CDMA_LCH_P,
	NSS_CDMA_LCH_PD
};

struct nss_cdma_channel_params {
	int data_type;		/* data type 8,16,32 */
	int elem_count;		/* number of elements in a frame */
	int frame_count;	/* number of frames in a element */

	int src_port;
	int src_amode;		/* constant, post increment, indexed,
					double indexed */
	unsigned long src_start; /* source address : physical */
	int src_ei;		/* source element index */
	int src_fi;		/* source frame index */

	int dst_port;
	int dst_amode;		/* constant, post increment, indexed,
					double indexed */
	unsigned long dst_start; /* source address : physical */
	int dst_ei;		/* source element index */
	int dst_fi;		/* source frame index */

	int trigger;		/* trigger attached if the channel is
					synchronized */
	int sync_mode;		/* sync on element, frame , block or packet */
	int src_or_dst_synch;	/* source synch(1) or destination synch(0) */

	int ie;			/* interrupt enabled */

	unsigned char read_prio;/* read priority */
	unsigned char write_prio;/* write priority */

	enum nss_cdma_burst_mode burst_mode; /* Burst mode 4/8/16 words */
};


extern int nss_request_cdma(int dev_id, const char *dev_name,
			void (*callback)(int lch, u16 ch_status, void *data),
			void *data, int *dma_ch);
extern void nss_free_cdma(int ch);
extern void nss_start_cdma(int lch);
extern void nss_stop_cdma(int lch);
extern void nss_set_cdma_transfer_params(int lch, int data_type,
					 int elem_count, int frame_count,
					 int sync_mode,
					 int dma_trigger, int src_or_dst_synch);

extern void nss_set_cdma_src_params(int lch, int src_port, int src_amode,
				    unsigned long src_start,
				    int src_ei, int src_fi);
extern void nss_set_cdma_src_burst_mode(int lch,
					enum nss_cdma_burst_mode burst_mode);

extern void nss_set_cdma_dest_params(int lch, int dest_port, int dest_amode,
				     unsigned long dest_start,
				     int dst_ei, int dst_fi);
extern void nss_set_cdma_dest_burst_mode(int lch,
					 enum nss_cdma_burst_mode burst_mode);

extern void nss_clear_cdma(int lch);
extern void nss_cdma_set_global_params(int arb_rate, int max_fifo_depth,
				       int tparams);

void nss_cdma_global_context_save(void);
void nss_cdma_global_context_restore(void);

#endif /* __DRIVER_CRYPTO_NSS_CDMA_H */
