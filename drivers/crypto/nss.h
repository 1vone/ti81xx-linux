/*
 * drivers/crypto/nss.h
 *
 * Copyright © 2011 Texas Instruments Incorporated
 * Author: Herman Schuurman
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
#ifndef __DRIVERS_CRYPTO_NSS_H
#define __DRIVERS_CRYPTO_NSS_H

/* ==================================================================== */
/** Crypto subsystem module layout
 */
/* ==================================================================== */
#define NSS_SEC_AES_1S_BASE	(TI81XX_SEC_BASE + 0x00140000)
#define NSS_SEC_AES_1P_BASE	(TI81XX_SEC_BASE + 0x00141000)
#define NSS_SEC_AES_2S_BASE	(TI81XX_SEC_BASE + 0x001A0000)
#define NSS_SEC_AES_2P_BASE	(TI81XX_SEC_BASE + 0x001A1000)
#define NSS_SEC_SHA_1S_BASE	(TI81XX_SEC_BASE + 0x00100000)
#define NSS_SEC_SHA_1P_BASE	(TI81XX_SEC_BASE + 0x00101000)
#define NSS_SEC_SHA_2S_BASE	(TI81XX_SEC_BASE + 0x001C0000)
#define NSS_SEC_SHA_2P_BASE	(TI81XX_SEC_BASE + 0x001C1000)
#define NSS_SEC_DES_1S_BASE	(TI81XX_SEC_BASE + 0x00160000)
#define NSS_SEC_DES_1P_BASE	(TI81XX_SEC_BASE + 0x00161000)
#define NSS_SEC_TRNG_BASE	(TI81XX_SEC_BASE + 0x00130000)
#define NSS_SEC_PKA_BASE	(TI81XX_SEC_BASE + 0x00120000)
#define NSS_SEC_CDMA_BASE	(TI81XX_SEC_BASE + 0x001E0000)
#define	NSS_SEC_REGS_BASE	(TI81XX_SEC_BASE + 0x001F0000)

#define FLD_MASK(start, end)		(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end)	(((val) << (end)) & FLD_MASK(start, end))

/* ==================================================================== */
/** AES module layout
 */
/* ==================================================================== */

#define	AES_REG_KEY2(x)			(0x1C - ((x ^ 0x01) * 0x04))
#define	AES_REG_KEY1(x)			(0x3C - ((x ^ 0x01) * 0x04))
#define	AES_REG_IV(x)			(0x40 + ((x) * 0x04))

#define	AES_REG_CTRL			0x50
#define	AES_REG_CTRL_CTX_RDY		(1 << 31)
#define	AES_REG_CTRL_SAVE_CTX_RDY	(1 << 30)
#define	AES_REG_CTRL_SAVE_CTX		(1 << 29)
#define	AES_REG_CTRL_CCM_M_MASK		(7 << 22)
#define	AES_REG_CTRL_CCM_M_SHFT		22
#define	AES_REG_CTRL_CCM_L_MASK		(7 << 19)
#define	AES_REG_CTRL_CCM_L_SHFT		19
#define	AES_REG_CTRL_CCM		(1 << 18)
#define	AES_REG_CTRL_GCM		(3 << 16)
#define	AES_REG_CTRL_CBCMAC		(1 << 15)
#define	AES_REG_CTRL_F9			(1 << 14)
#define	AES_REG_CTRL_F8			(1 << 13)
#define	AES_REG_CTRL_XTS_MASK		(3 << 11)
#define	 AES_REG_CTRL_XTS_01		(1 << 11)
#define	 AES_REG_CTRL_XTS_10		(2 << 11)
#define	 AES_REG_CTRL_XTS_11		(3 << 11)
#define	AES_REG_CTRL_CFB		(1 << 10)
#define	AES_REG_CTRL_ICM		(1 << 9)
#define	AES_REG_CTRL_CTR_WIDTH_MASK	(3 << 7)
#define	 AES_REG_CTRL_CTR_WIDTH_32	(0 << 7)
#define	 AES_REG_CTRL_CTR_WIDTH_64	(1 << 7)
#define	 AES_REG_CTRL_CTR_WIDTH_96	(2 << 7)
#define	 AES_REG_CTRL_CTR_WIDTH_128	(3 << 7)
#define	AES_REG_CTRL_CTR		(1 << 6)
#define	AES_REG_CTRL_CBC		(1 << 5)
#define	AES_REG_CTRL_KEY_SIZE_MASK	(3 << 3)
#define	 AES_REG_CTRL_KEY_SIZE_128	(1 << 3)
#define	 AES_REG_CTRL_KEY_SIZE_192	(2 << 3)
#define	 AES_REG_CTRL_KEY_SIZE_256	(3 << 3)
#define	AES_REG_CTRL_DIRECTION		(1 << 2)
#define	AES_REG_CTRL_INPUT_RDY		(1 << 1)
#define	AES_REG_CTRL_OUTPUT_RDY		(1 << 0)

#define	AES_REG_LENGTH_N(x)		(0x54 + ((x) * 0x04))
#define	AES_REG_AUTH_LENGTH		0x5C
#define	AES_REG_DATA			0x60
#define	AES_REG_DATA_N(x)		(0x60 + ((x) * 0x04))
#define	AES_REG_TAG			0x70
#define	AES_REG_TAG_N(x)		(0x70 + ((x) * 0x04))

#define AES_REG_REV			0x80
#define	 AES_REG_REV_SCHEME_MASK	(3 << 30)
#define	 AES_REG_REV_FUNC_MASK		(0xFFF << 16)
#define	 AES_REG_REV_R_RTL_MASK		(0x1F << 11)
#define	 AES_REG_REV_X_MAJOR_MASK	(7 << 8)
#define	 AES_REG_REV_CUSTOM_MASK	(3 << 6)
#define	 AES_REG_REV_Y_MINOR_MASK	(0x3F << 0)

#define	AES_REG_SYSCFG			0x84
#define	AES_REG_SYSCFG_K3		(1 << 12)
#define	AES_REG_SYSCFG_KEY_ENC		(1 << 11)
#define	AES_REG_SYSCFG_KEK_MODE		(1 << 10)
#define	AES_REG_SYSCFG_MAP_CTX_OUT	(1 << 9)
#define	AES_REG_SYSCFG_DREQ_MASK	(15 << 5)
#define	 AES_REG_SYSCFG_DREQ_CTX_OUT_EN	(1 << 8)
#define	 AES_REG_SYSCFG_DREQ_CTX_IN_EN	(1 << 7)
#define	 AES_REG_SYSCFG_DREQ_DATA_OUT_EN (1 << 6)
#define	 AES_REG_SYSCFG_DREQ_DATA_IN_EN	(1 << 5)
#define	AES_REG_SYSCFG_DIRECTBUSEN	(1 << 4)
#define	AES_REG_SYSCFG_SIDLE_MASK	(3 << 2)
#define	 AES_REG_SYSCFG_SIDLE_FORCEIDLE	(0 << 2)
#define	 AES_REG_SYSCFG_SIDLE_NOIDLE	(1 << 2)
#define	 AES_REG_SYSCFG_SIDLE_SMARTIDLE	(2 << 2)
#define	AES_REG_SYSCFG_SOFTRESET	(1 << 1)
#define	AES_REG_SYSCFG_AUTOIDLE		(1 << 0)

#define	AES_REG_SYSSTATUS		0x88
#define	AES_REG_SYSSTATUS_RESETDONE	(1 << 0)

#define	AES_REG_IRQSTATUS		0x8C
#define	AES_REG_IRQSTATUS_CTX_OUT	(1 << 3)
#define	AES_REG_IRQSTATUS_DATA_OUT	(1 << 2)
#define	AES_REG_IRQSTATUS_DATA_IN	(1 << 1)
#define	AES_REG_IRQSTATUS_CTX_IN	(1 << 0)

#define	AES_REG_IRQENA			0x90
#define	AES_REG_IRQENA_CTX_OUT		(1 << 3)
#define	AES_REG_IRQENA_DATA_OUT		(1 << 2)
#define	AES_REG_IRQENA_DATA_IN		(1 << 1)
#define	AES_REG_IRQENA_CTX_IN		(1 << 0)

#define	AES_REG_DBITS			0x94
#define	AES_REG_DBITS_P_DIRTY		(1 << 3)
#define	AES_REG_DBITS_P_ACCESS		(1 << 2)
#define	AES_REG_DBITS_S_DIRTY		(1 << 1)
#define	AES_REG_DBITS_S_ACCESS		(1 << 0)

#define	AES_REG_LOCK			0x98
#define	AES_REG_LOCK_LENGTH		(1 << 5)
#define	AES_REG_LOCK_CONTROL		(1 << 4)
#define	AES_REG_LOCK_IV			(1 << 3)
#define	AES_REG_LOCK_KEY3		(1 << 2)
#define	AES_REG_LOCK_KEY2		(1 << 1)
#define	AES_REG_LOCK_KEY		(1 << 0)

/* ==================================================================== */
/** DES/3DES module layout.
 */
/* ==================================================================== */
#define DES_REG_KEY3_L			0x00
#define DES_REG_KEY3_H			0x04
#define DES_REG_KEY2_L			0x08
#define DES_REG_KEY2_H			0x0C
#define DES_REG_KEY1_L			0x10
#define DES_REG_KEY1_H			0x14
#define DES_REG_IV_L			0x18
#define DES_REG_IV_H			0x1C
#define	DES_REG_KEYS(x)			(0x14 - ((x ^ 0x01) * 0x04))

#define DES_REG_CTRL			0x20
#define DES_REG_CTRL_CTX		(1 << 31)
#define DES_REG_CTRL_MODE_MASK		(3 << 4)
#define  DES_REG_CTRL_MODE_ECB		(0 << 4)
#define  DES_REG_CTRL_MODE_CBC		(1 << 4)
#define  DES_REG_CTRL_MODE_CFB		(2 << 4)
#define DES_REG_CTRL_TDES		(1 << 3)
#define DES_REG_CTRL_DIRECTION		(1 << 2)
#define DES_REG_CTRL_INPUT_RDY		(1 << 1)
#define DES_REG_CTRL_OUTPUT_RDY		(1 << 0)

#define DES_REG_LENGTH			0x24
#define DES_REG_DATA_L			0x28
#define DES_REG_DATA_H			0x2C
#define DES_REG_REV			0x30
#define	 DES_REG_REV_SCHEME_MASK	(3 << 30)
#define	 DES_REG_REV_FUNC_MASK		(0xFFF << 16)
#define	 DES_REG_REV_R_RTL_MASK		(0x1F << 11)
#define	 DES_REG_REV_X_MAJOR_MASK	(7 << 8)
#define	 DES_REG_REV_CUSTOM_MASK	(3 << 6)
#define	 DES_REG_REV_Y_MINOR_MASK	(0x3F << 0)

#define DES_REG_SYSCFG			0x34
#define	DES_REG_SYSCFG_DREQ_MASK	(7 << 5)
#define  DES_REG_SYSCFG_DREQ_CTX_IN_EN	(1 << 7)
#define  DES_REG_SYSCFG_DREQ_DATA_OUT_EN (1 << 6)
#define  DES_REG_SYSCFG_DREQ_DATA_IN_EN	(1 << 5)
#define DES_REG_SYSCFG_DIRECTBUSEN	(1 << 4)
#define DES_REG_SYSCFG_SIDLE_MASK	(3 << 2)
#define  DES_REG_SYSCFG_SIDLE_FORCEIDLE	(0 << 2)
#define  DES_REG_SYSCFG_SIDLE_NOIDLE	(1 << 2)
#define  DES_REG_SYSCFG_SIDLE_SMARTIDLE	(2 << 2)
#define DES_REG_SYSCFG_SOFTRESET	(1 << 1)
#define DES_REG_SYSCFG_AUTOIDLE		(1 << 0)

#define DES_REG_SYSSTATUS		0x38
#define DES_REG_SYSSTATUS_RESETDONE	(1 << 0)

#define DES_REG_IRQSTATUS		0x3C
#define DES_REG_IRQSTATUS_DATA_OUT	(1 << 2)
#define DES_REG_IRQSTATUS_DATA_IN	(1 << 1)
#define DES_REG_IRQSTATUS_CTX_IN	(1 << 0)

#define DES_REG_IRQENA			0x40
#define DES_REG_IRQENA_DATA_OUT		(1 << 2)
#define DES_REG_IRQENA_DATA_IN		(1 << 1)
#define DES_REG_IRQENA_CTX_IN		(1 << 0)

#define DES_REG_DBITS			0x44
#define DES_REG_DBITS_P_DIRTY		(1 << 3)
#define DES_REG_DBITS_P_ACCESS		(1 << 2)
#define DES_REG_DBITS_S_DIRTY		(1 << 1)
#define DES_REG_DBITS_S_ACCESS		(1 << 0)

#define DES_REG_LOCK			0x48
#define DES_REG_LOCK_LENGTH		(1 << 3)
#define DES_REG_LOCK_CONTROL		(1 << 2)
#define DES_REG_LOCK_IV			(1 << 1)
#define DES_REG_LOCK_KEY		(1 << 0)

/* ==================================================================== */
/** SHA / MD5 module layout.
 */
/* ==================================================================== */

#define	SHA_REG_ODIGEST			0x00
#define	SHA_REG_ODIGEST_N(x)		(0x00 + ((x) * 0x04))
#define	SHA_REG_IDIGEST			0x20
#define	SHA_REG_IDIGEST_N(x)		(0x20 + ((x) * 0x04))

#define SHA_REG_DIGEST_COUNT		0x40
#define SHA_REG_MODE			0x44
#define SHA_REG_MODE_HMAC_OUTER_HASH	(1 << 7)
#define SHA_REG_MODE_HMAC_KEY_PROC	(1 << 5)
#define SHA_REG_MODE_CLOSE_HASH		(1 << 4)
#define SHA_REG_MODE_ALGO_CONSTANT	(1 << 3)
#define SHA_REG_MODE_ALGO_MASK		(3 << 1)
#define  SHA_REG_MODE_ALGO_MD5_128	(0 << 1)
#define  SHA_REG_MODE_ALGO_SHA1_160	(1 << 1)
#define  SHA_REG_MODE_ALGO_SHA2_224	(2 << 1)
#define  SHA_REG_MODE_ALGO_SHA2_256	(3 << 1)

#define SHA_REG_LENGTH			0x48

#define	SHA_REG_DATA			0x80
#define	SHA_REG_DATA_N(x)		(0x80 + ((x) * 0x04))

#define SHA_REG_REV			0x100
#define	 SHA_REG_REV_SCHEME_MASK	(3 << 30)
#define	 SHA_REG_REV_FUNC_MASK		(0xFFF << 16)
#define	 SHA_REG_REV_R_RTL_MASK		(0x1F << 11)
#define	 SHA_REG_REV_X_MAJOR_MASK	(7 << 8)
#define	 SHA_REG_REV_CUSTOM_MASK	(3 << 6)
#define	 SHA_REG_REV_Y_MINOR_MASK	(0x3F << 0)

#define	SHA_REG_SYSCFG			0x110
#define	SHA_REG_SYSCFG_SADVANCED	(1 << 7)
#define	SHA_REG_SYSCFG_SCONT_SWT	(1 << 6)
#define	SHA_REG_SYSCFG_SIDLE_MASK	(3 << 4)
#define	 SHA_REG_SYSCFG_SIDLE_FORCEIDLE	(0 << 4)
#define	 SHA_REG_SYSCFG_SIDLE_NOIDLE	(1 << 4)
#define	 SHA_REG_SYSCFG_SIDLE_SMARTIDLE	(2 << 4)
#define	SHA_REG_SYSCFG_SDMA_EN		(1 << 3)
#define	SHA_REG_SYSCFG_SIT_EN		(1 << 2)
#define	SHA_REG_SYSCFG_SOFTRESET	(1 << 1)
#define	SHA_REG_SYSCFG_AUTOIDLE		(1 << 0)

#define SHA_REG_SYSSTATUS		0x114
#define SHA_REG_SYSSTATUS_RESETDONE	(1 << 0)

#define SHA_REG_IRQSTATUS		0x118
#define SHA_REG_IRQSTATUS_CTX_RDY	(1 << 3)
#define SHA_REG_IRQSTATUS_PARTHASH_RDY (1 << 2)
#define SHA_REG_IRQSTATUS_INPUT_RDY	(1 << 1)
#define SHA_REG_IRQSTATUS_OUTPUT_RDY	(1 << 0)

#define SHA_REG_IRQENA			0x11C
#define SHA_REG_IRQENA_CTX_RDY		(1 << 3)
#define SHA_REG_IRQENA_PARTHASH_RDY	(1 << 2)
#define SHA_REG_IRQENA_INPUT_RDY	(1 << 1)
#define SHA_REG_IRQENA_OUTPUT_RDY	(1 << 0)

#define SHA_REG_XSSTATUS		0x140
#define SHA_REG_XSSTATUS_PDIRTY		(1 << 3)
#define SHA_REG_XSSTATUS_PACCESSED	(1 << 2)
#define SHA_REG_XSSTATUS_SDIRTY		(1 << 1)
#define SHA_REG_XSSTATUS_SACCESSED	(1 << 0)

#define SHA_REG_LOCK			0x144
#define SHA_REG_LOCK_CLOSE8		(1 << 28)
#define SHA_REG_LOCK_CLOSE7		(1 << 27)
#define SHA_REG_LOCK_CLOSE6		(1 << 26)
#define SHA_REG_LOCK_CLOSE5		(1 << 25)
#define SHA_REG_LOCK_CLOSE4		(1 << 24)
#define SHA_REG_LOCK_CLOSE3		(1 << 7)
#define SHA_REG_LOCK_CLOSE2		(1 << 6)
#define SHA_REG_LOCK_CLOSE1		(1 << 5)
#define SHA_REG_LOCK_CLOSE		(1 << 4)
#define SHA_REG_LOCK_ALGO_CST		(1 << 3)
#define SHA_REG_LOCK_ALGO_MASK		(3 << 1)

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
#define	RNG_REG_REV_MASK		0xFF

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


#endif /* __DRIVERS_CRYPTO_NSS_H */
