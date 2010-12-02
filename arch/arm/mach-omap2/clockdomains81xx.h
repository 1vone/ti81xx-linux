/*
 * arch/arm/mach-omap2/clockdomains81xx.h
 *
 * TI81XX Clock Domain data.
 *
 * Copyright (C) 2010 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCKDOMAINS81XX_H
#define __ARCH_ARM_MACH_OMAP2_CLOCKDOMAINS81XX_H

#include <plat/clockdomain.h>

#include "cm.h"
#include "cm-regbits-81xx.h"

/*
 * TODO:
 * - Add other domains as required
 * - Fill up associated powerdomans (especially ALWON powerdomains are NULL at
 *   the moment
 * - Consider dependencies across domains (probably not applicable till now)
 */

#ifdef CONFIG_ARCH_TI81XX

/* Common fo TI816X and TI814X */
static struct clockdomain alwon_l3_slow_81xx_clkdm = {
	.name		  = "alwon_l3_slow_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI81XX_CM_ALWON_L3_SLOW_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X | CHIP_IS_TI814X),
};

static struct clockdomain alwon_ethernet_81xx_clkdm = {
	.name		  = "alwon_ethernet_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI81XX_CM_ETHERNET_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X | CHIP_IS_TI814X),
};

static struct clockdomain mmu_81xx_clkdm = {
	.name		  = "mmu_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI81XX_CM_MMU_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X | CHIP_IS_TI814X),
};

static struct clockdomain mmu_cfg_81xx_clkdm = {
	.name		  = "mmu_cfg_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI81XX_CM_MMUCFG_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X | CHIP_IS_TI814X),
};

/* TI814X specific */
static struct clockdomain dsp_814x_clkdm= {
	.name		= "dsp_clkdm",
	.pwrdm		= { .name = "dsp_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_DSP_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

static struct clockdomain hdvicp_814x_clkdm= {
	.name		= "hdvicp_clkdm",
	.pwrdm		= { .name = "hdvicp_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_HDVICP_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* ISS is inside ISP */
static struct clockdomain isp_814x_clkdm= {
	.name		= "isp_clkdm",
	.pwrdm		= { .name = "isp_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_ALWON2_MC_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* GFX clock domain */
static struct clockdomain gfx_814x_clkdm= {
	.name		= "gfx_clkdm",
	.pwrdm		= { .name = "gfx_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_GFX_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* DSS clock domain */
static struct clockdomain hdvpss_814x_clkdm= {
	.name		= "hdvpss_clkdm",
	.pwrdm		= { .name = "hdvpss_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_HDVPSS_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* L3 med clock domain */
static struct clockdomain alwon2_l3_med_814x_clkdm= {
	.name		= "alwon2_l3_med_clkdm",
	.pwrdm		= { .name = "alwon2_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_ALWON2_L3_MED_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* PCIe clock domain */
static struct clockdomain alwon2_pcie_814x_clkdm= {
	.name		= "alwon2_pcie_clkdm",
	.pwrdm		= { .name = "alwon2_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_ALWON2_PCI_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* USB clock domain */
static struct clockdomain alwon2_usb_814x_clkdm= {
	.name		= "alwon2_usb_clkdm",
	.pwrdm		= { .name = "alwon2_pwrdm" },
	.clkstctrl_reg	= TI814X_CM_ALWON2_L3_SLOW_CLKSTCTRL,
	.clktrctrl_mask	= TI81XX_CLKTRCTRL_MASK,
	.flags		= CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_TI814X),
};

/* TI816X specific */
static struct clockdomain alwon_mpu_816x_clkdm = {
	.name		  = "alwon_mpu_clkdm",
	.pwrdm		  = { .name = "alwon_pwrdm" },
	.clkstctrl_reg	  = TI81XX_CM_ALWON_MPU_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain active_gem_816x_clkdm = {
	.name		  = "active_gem_clkdm",
	.pwrdm		  = { .name = "active_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_ACTIVE_GEM_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain ivahd0_816x_clkdm = {
	.name		  = "ivahd0_clkdm",
	.pwrdm		  = { .name = "ivahd0_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_IVAHD0_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain ivahd1_816x_clkdm = {
	.name		  = "ivahd1_clkdm",
	.pwrdm		  = { .name = "ivahd1_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_IVAHD1_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain ivahd2_816x_clkdm = {
	.name		  = "ivahd2_clkdm",
	.pwrdm		  = { .name = "ivahd2_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_IVAHD2_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain sgx_816x_clkdm = {
	.name		  = "sgx_clkdm",
	.pwrdm		  = { .name = "sgx_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_SGX_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_l3_med_816x_clkdm = {
	.name		  = "default_l3_med_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_L3_MED_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_ducati_816x_clkdm = {
	.name		  = "default_ducati_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_DUCATI_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_pcie_816x_clkdm = {
	.name		  = "default_pcie_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_PCI_CLKSTCTRL,
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

static struct clockdomain default_usb_816x_clkdm = {
	.name		  = "default_usb_clkdm",
	.pwrdm		  = { .name = "default_pwrdm" },
	.clkstctrl_reg	  = TI816X_CM_DEFAULT_L3_SLOW_CLKSTCTRL, /* FIXME */
	.clktrctrl_mask	  = TI81XX_CLKTRCTRL_MASK,
	.flags		  = CLKDM_CAN_HWSUP_SWSUP,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_TI816X),
};

#endif /* CONFIG_ARCH_TI81XX */

#endif
