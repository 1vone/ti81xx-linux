/*
 * irqs-ti816x.h
 *  TI816X family interrupts
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_TI816X_IRQS_H
#define __ARCH_ARM_MACH_OMAP2_TI816X_IRQS_H

/*
 * Netra Interrupts
 */
#define TI816X_IRQ_EMU		0
#define TI816X_IRQ_COMMTX	1
#define TI816X_IRQ_COMMRX	2
#define TI816X_IRQ_BENCH	3
#define TI816X_IRQ_ELM		4
#define TI816X_IRQ_SSM_WFI	5
#define TI816X_IRQ_SSM		6
#define TI816X_IRQ_NMI		7
#define TI816X_IRQ_SEC_EVNT	8
#define TI816X_IRQ_L3_DEBUG	9
#define TI816X_IRQ_L3_APP	10
#define TI816X_IRQ_EDMA_COMP	12
#define TI816X_IRQ_EDMA_MPERR	13
#define TI816X_IRQ_EDMA_ERR	14
#define TI816X_IRQ_SATA		16
#define TI816X_IRQ_USBSS	17
#define TI816X_IRQ_USB0		18
#define TI816X_IRQ_USB1		19
#define TI816X_IRQ_TPPSS_ERR	20
#define TI816X_IRQ_TPPSS_MBOX	21
#define TI816X_IRQ_TPPSS_STC0	22
#define TI816X_IRQ_TPPSS_STC1	23
#define TI816X_IRQ_TPPSS_DMAPC0	24
#define TI816X_IRQ_TPPSS_DMABS0	25
#define TI816X_IRQ_TPPSS_ERR0	26
#define TI816X_IRQ_TPPSS_ERR1	27
#define TI816X_IRQ_TPPSS_ERR2	28
#define TI816X_IRQ_TPPSS_ERR3	29
#define TI816X_IRQ_MCARD_TX	30
#define TI816X_IRQ_MCARD_RX	31
#define TI816X_IRQ_USB_WKUP	34
#define TI816X_IRQ_PCIE_WKUP	35
#define TI816X_IRQ_DSSINT	36
#define TI816X_IRQ_GFXINT	37
#define TI816X_IRQ_HDMIINT	38
#define TI816X_IRQ_VLYNQ	39
#define TI816X_IRQ_MACRXTHR0	40
#define TI816X_IRQ_MACRXINT0	41
#define TI816X_IRQ_MACTXINT0	42
#define TI816X_IRQ_MACMISC0	43
#define TI816X_IRQ_MACRXTHR1	44
#define TI816X_IRQ_MACRXINT1	45
#define TI816X_IRQ_MACTXINT1	46
#define TI816X_IRQ_MACMISC1	47
#define TI816X_IRQ_PCIINT0	48
#define TI816X_IRQ_PCIINT1	49
#define TI816X_IRQ_PCIINT2	50
#define TI816X_IRQ_PCIINT3	51
#define TI816X_IRQ_SD		64
#define TI816X_IRQ_SPI		65
#define TI816X_IRQ_GPT1		66
#define TI816X_IRQ_GPT2		67
#define TI816X_IRQ_GPT3		68
#define TI816X_IRQ_GPT4		69
#define TI816X_IRQ_I2C0		70
#define TI816X_IRQ_I2C1		71
#define TI816X_IRQ_UART0	72
#define TI816X_IRQ_UART1	73
#define TI816X_IRQ_UART2	74
#define TI816X_IRQ_RTC		75
#define TI816X_IRQ_RTC_ALARM	76
#define TI816X_IRQ_MBOX		77
#define TI816X_IRQ_MCASP0_TX	80
#define TI816X_IRQ_MCASP0_RX	81
#define TI816X_IRQ_MCASP1_TX	82
#define TI816X_IRQ_MCASP1_RX	83
#define TI816X_IRQ_MCASP2_TX	84
#define TI816X_IRQ_MCASP2_RX	85
#define TI816X_IRQ_MCBSP	86
#define TI816X_IRQ_SMCD0	87
#define TI816X_IRQ_SMCD1	88
#define TI816X_IRQ_WDT1		91
#define TI816X_IRQ_GPT5		92
#define TI816X_IRQ_GPT6		93
#define TI816X_IRQ_GPT7		94
#define TI816X_IRQ_GPT8		95
#define TI816X_IRQ_GPIO_0A	96
#define TI816X_IRQ_GPIO_0B	97
#define TI816X_IRQ_GPIO_1A	98
#define TI816X_IRQ_GPIO_1B	99
#define TI816X_IRQ_GPMC		100
#define TI816X_IRQ_DDR_ERR0	101
#define TI816X_IRQ_DDR_ERR1	102
#define TI816X_IRQ_IVA0CONT1SYNC	103
#define TI816X_IRQ_IVA0CONT2SYNC	104
#define TI816X_IRQ_IVA1CONT1SYNC	105
#define TI816X_IRQ_IVA1CONT2SYNC	106
#define TI816X_IRQ_IVA0MBOX	107
#define TI816X_IRQ_IVA1MBOX	108
#define TI816X_IRQ_IVA2MBOX	109
#define TI816X_IRQ_IVA2CONT1SYNC	110
#define TI816X_IRQ_IVA2CONT2SYNC	111
#define TI816X_IRQ_TPTC0	112
#define TI816X_IRQ_TPTC1	113
#define TI816X_IRQ_TPTC2	114
#define TI816X_IRQ_TPTC3	115
#define TI816X_IRQ_SECPUBINT	116
#define TI816X_IRQ_SECSECINT	117
#define TI816X_IRQ_SECPUBSWINT	118
#define TI816X_IRQ_SECSECSWINT	119
#define TI816X_IRQ_SMRFLX0	120
#define TI816X_IRQ_SMRFLX1	121
#define TI816X_IRQ_SYS_MMU	122
#define TI816X_IRQ_MC_MMU	123
#define TI816X_IRQ_DMM		124


#endif