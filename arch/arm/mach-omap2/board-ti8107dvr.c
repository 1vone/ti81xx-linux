/*
 * Code for TI8107 DVR.
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/regulator/machine.h>
#include <linux/eeprom_uddvr.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/mcspi.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/asp.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/hdmi_lib.h>

#include "board-flash.h"
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"
#include "control.h"

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux     NULL
#endif

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL, /* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

static struct i2c_board_info __initdata ti810x_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("ds1337", 0x68),
	},
};

static void __init ti810x_dvr_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, ti810x_i2c_boardinfo,
				ARRAY_SIZE(ti810x_i2c_boardinfo));
}

/* NAND flash information */
static struct mtd_partition ti810x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "U-Boot-min",
		.offset         = 0,    				/* Offset = 0x0 */
		.size           = SZ_128K,
		.mask_flags     = MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 20000 */
		.size           = 17 * SZ_128K,			/* 0x220000 */
		.mask_flags     = MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x240000 */
		.size           = 2 * SZ_128K,			/* 0x40000 */
	},
	{
		.name			= "U-Boot Logo",
		.offset			= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size			= 24 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x580000 */
		.size           = 34 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x9C0000 */
		.size           = 1601 * SZ_128K,
	},
	{
		.name           = "Reserved",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0xCEE0000 */
		.size           = MTDPART_SIZ_FULL,
	},
};


static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode           = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode           = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode           = MUSB_PERIPHERAL,
#endif
	.power		= 500,
	.instances	= 1,
};

static void __init ti810x_dvr_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

#ifdef CONFIG_SND_SOC_TI81XX_HDMI
static struct snd_hdmi_platform_data ti81xx_snd_hdmi_pdata = {
	.dma_addr = TI81xx_HDMI_WP + HDMI_WP_AUDIO_DATA,
	.channel = 53,
	.data_type = 4,
	.acnt = 4,
	.fifo_level = 0x20,
};

static struct platform_device ti81xx_hdmi_audio_device = {
	.name   = "hdmi-dai",
	.id     = -1,
    .num_resources = 0,
	.dev = {
		.platform_data = &ti81xx_snd_hdmi_pdata,
	}
};

static struct platform_device ti81xx_hdmi_codec_device = {
	.name   = "hdmi-dummy-codec",
	.id     = -1,
};

static struct platform_device *ti81xx_hdmi_devices[] __initdata = {
	&ti81xx_hdmi_audio_device,
	&ti81xx_hdmi_codec_device,
};
#endif

#ifdef CONFIG_SND_SOC_TVP5158_AUDIO
static struct platform_device tvp5158_audio_device = {
	.name	= "tvp5158-audio",
	.id	= -1,
};

static u8 tvp5158_iis_serializer_direction[] = {
	RX_MODE, INACTIVE_MODE,	 INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

/* TVP5158 <-> McASP0 */
static struct snd_platform_data tvp5158_snd_data = {
	.tx_dma_offset	= 0x46000000,
	.rx_dma_offset	= 0x46000000,
	.asp_chan_q	    = EVENTQ_2,
	.clk_input_pin  = MCASP_AHCLKX_OUT,
	.tdm_slots	    = 16, /* number of channels */
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(tvp5158_iis_serializer_direction),
	.serial_dir	= tvp5158_iis_serializer_direction,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

static struct resource ti81xx_mcasp0_resource[] = {
    {
        .name = "mcasp0",
        .start = TI81XX_ASP0_BASE,
        .end = TI81XX_ASP0_BASE + (SZ_1K * 12) - 1,
        .flags = IORESOURCE_MEM,
    },
    /* TX event */
    {
        .start = TI81XX_DMA_MCASP0_AXEVT,
        .end = TI81XX_DMA_MCASP0_AXEVT,
        .flags = IORESOURCE_DMA,
    },
    /* RX event */
    {
        .start = TI81XX_DMA_MCASP0_AREVT,
        .end = TI81XX_DMA_MCASP0_AREVT,
        .flags = IORESOURCE_DMA,
    },
};

static struct platform_device ti81xx_mcasp0_device = {
    .name = "davinci-mcasp",
    .id = 0,
    .dev ={
		.platform_data = &tvp5158_snd_data,
	},
    .num_resources = ARRAY_SIZE(ti81xx_mcasp0_resource),
    .resource = ti81xx_mcasp0_resource,
};
#endif

#ifdef CONFIG_SND_SOC_TLV320AIC3X
static u8 aic3x_iis_serializer_direction[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

/* McASP1 <-> AIC3101 */
static struct snd_platform_data aic3x_snd_data = {
	.tx_dma_offset	= 0x46400000,
	.rx_dma_offset	= 0x46400000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(aic3x_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= aic3x_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

static struct resource ti81xx_mcasp1_resource[] = {
    {
        .name = "mcasp1",
        .start = TI81XX_ASP1_BASE,
        .end = TI81XX_ASP1_BASE + (SZ_1K * 12) - 1,
        .flags = IORESOURCE_MEM,
    },
    /* TX event */
    {
        .start = TI81XX_DMA_MCASP1_AXEVT,
        .end = TI81XX_DMA_MCASP1_AXEVT,
        .flags = IORESOURCE_DMA,
    },
    /* RX event */
    {
        .start = TI81XX_DMA_MCASP1_AREVT,
        .end = TI81XX_DMA_MCASP1_AREVT,
        .flags = IORESOURCE_DMA,
    },
};

static struct platform_device ti81xx_mcasp1_device = {
    .name = "davinci-mcasp",
    .id = 1,
    .dev ={
		.platform_data = &aic3x_snd_data,
	},
    .num_resources = ARRAY_SIZE(ti81xx_mcasp1_resource),
    .resource = ti81xx_mcasp1_resource,
};
#endif

#ifdef CONFIG_EEPROM_UDDVR
static int __init ti810x_dvr_eeprom_init(void)
{
	int ret = 0;
	
	ret = gpio_request(EEPROM_CS, "eeprom cs");
	if (ret) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_CS);
		return ret;
	}
	/* default-inactive */
	gpio_direction_output(EEPROM_CS, 0);

	ret = gpio_request(EEPROM_SCLK, "eeprom clk");
	if (ret) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_CS);
		return ret;
	}
	
	gpio_direction_output(EEPROM_SCLK, 0);

	ret = gpio_request(EEPROM_SDO, "eeprom sdo");
	if (ret) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_SDO);
		return ret;
	}
	
	gpio_direction_output(EEPROM_SDO, 0);
	
	ret = gpio_request(EEPROM_SDI, "eeprom sdi");
	if (ret) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_SDI);
		return ret;
	}
	gpio_direction_input(EEPROM_SDI);
	
	/* 
	 * Get dvr hardware version 
	 * off:[0] ~ [7]->MAC0 Address
	 * off:[8] ~ [15]->MAC1 Address
	 * off:[16] ~ [17]->H/W Version
	 */
	eeprom_uddvr_set_hwver();
	
	printk("3-wired eeprom init done. (H/W ver:%02x)\n", eeprom_uddvr_get_hwver());
	
	return 0;
}
#endif

static void __init ti810x_dvr_init(void)
{
	ti814x_mux_init(board_mux);
	omap_serial_init();
	
	ti810x_dvr_eeprom_init();
	ti810x_dvr_i2c_init();
	omap2_hsmmc_init(mmc);

	/* nand initialisation */
	board_nand_init(ti810x_nand_partitions, 
				ARRAY_SIZE(ti810x_nand_partitions), 0, 0);

	/* initialize usb */
	usb_musb_init(&musb_board_data);
	
	#ifdef CONFIG_SND_SOC_TVP5158_AUDIO
	platform_device_register(&tvp5158_audio_device);
	#endif
	
	platform_device_register(&ti81xx_mcasp0_device);
	
	#ifdef CONFIG_SND_SOC_TLV320AIC3X
	platform_device_register(&ti81xx_mcasp1_device);
	#endif
	
	#ifdef CONFIG_SND_SOC_TI81XX_HDMI
	platform_add_devices(ti81xx_hdmi_devices, ARRAY_SIZE(ti81xx_hdmi_devices));
	#endif
	
	regulator_use_dummy_regulator();
}

static void __init ti810x_dvr_map_io(void)
{
	omap2_set_globals_ti816x();
	ti81xx_map_common_io();
}

MACHINE_START(DM385EVM, "ud8107_dvr")
	/* Maintainer: Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io			= ti810x_dvr_map_io,
	.reserve        = ti81xx_reserve,
	.init_irq		= ti810x_dvr_init_irq,
	.init_machine	= ti810x_dvr_init,
	.timer			= &omap_timer,
MACHINE_END
