/*
 * Code for TI8168 EVM.
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
#include <linux/platform_device.h>
#include <linux/i2c/at24.h>
#include <linux/device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/device.h>
#include <linux/mtd/nand.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/mtd/physmap.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>
#include <linux/eeprom_uddvr.h>

#ifdef CONFIG_REGULATOR_GPIO
#include <linux/regulator/gpio-regulator.h>
#endif
#ifdef CONFIG_REGULATOR_TPS40400
#include <linux/regulator/pmbus-regulator.h>
#endif
#include <linux/sii9022a.h>
#include <linux/clk.h>

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

#include "control.h"
#include "clock.h"
#include "mux.h"
#include "hsmmc.h"
#include "board-flash.h"

#include <mach/board-ti816x.h>

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,/* Dedicated pins for CD and WP */
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

static struct mtd_partition ti816x_nand_partitions_bs128[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "U-Boot",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 18 * SZ_128K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x240000 */
		.size		= 2 * SZ_128K,
	},
	{
		.name		= "U-Boot Logo",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 24 * SZ_128K,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x580000 */
		.size		= 34 * SZ_128K,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x6C0000 */
		.size		= 1601 * SZ_128K,
	},
	{
		.name		= "Reserved",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0xCEE0000 */
		.size		= MTDPART_SIZ_FULL,
	},

};

static struct mtd_partition ti816x_nand_partitions_bs256[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "U-Boot",
		.offset		= 0,	/* Offset = 0x0 */
		.size		= 9 * SZ_256K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x240000 */
		.size		= 1 * SZ_256K,
	},
	{
		.name		= "U-Boot Logo",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 12 * SZ_256K,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x580000 */
		.size		= 17 * SZ_256K,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x6C0000 */
		.size		= 1024 * SZ_256K,	//1024(256M), 4000*SZ_256K,
	},
	{
		.name		= "Reserved",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x3EEC0000 */
		.size		= MTDPART_SIZ_FULL,
	},

};

#ifdef CONFIG_REGULATOR_TPS40400
/* Macro for PMBUS voltage regulator */
#define VR_PMBUS_INSTANCE	0

static struct regulator_consumer_supply ti816x_pmbus_dcdc_supply[] = {
	{
		.supply = "vdd_avs",
	},
};

static struct regulator_init_data pmbus_pmic_init_data = {
	.constraints = {
		.min_uV		= 800000,
		.max_uV		= 1050000,
		.always_on	= 1,
		.valid_ops_mask	= (REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS),
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= ti816x_pmbus_dcdc_supply,
};
#endif


#ifdef CONFIG_REGULATOR_GPIO
/* Macro for GPIO voltage regulator */
#define VR_GPIO_INSTANCE0	0
#define VR_GPIO_INSTANCE1	1

static struct regulator_consumer_supply ti816x_gpio_dcdc_supply[] = {
	{
		.supply = "vdd_avs",
	},
};

static struct regulator_init_data gpio_pmic_init_data = {
	.constraints = {
		.min_uV		= 800000,
		.max_uV		= 1025000,
		.valid_ops_mask	= (REGULATOR_CHANGE_VOLTAGE |
			REGULATOR_CHANGE_STATUS),
	},
	.num_consumer_supplies	= ARRAY_SIZE(ti816x_gpio_dcdc_supply),
	.consumer_supplies	= ti816x_gpio_dcdc_supply,
};

/* Supported voltage values for regulators */
static struct gpio_vr_data ti816x_vsel_table[] = {
	{0x0, 800000}, {0x8, 815000}, {0x4, 830000}, {0xC, 845000},
	{0x2, 860000}, {0xA, 875000}, {0x6, 890000}, {0xE, 905000},
	{0x1, 920000}, {0x9, 935000}, {0x5, 950000}, {0xD, 965000},
	{0x3, 980000}, {0xB, 995000}, {0x7, 1010000}, {0xF, 1025000},
};

static struct gpio vcore_gpios[] = {
	{ (VR_GPIO_INSTANCE1 * 32) + 0,  GPIOF_IN, "vgpio 0"},
	{ (VR_GPIO_INSTANCE1 * 32) + 29, GPIOF_IN, "vgpio 1"},
	{ (VR_GPIO_INSTANCE1 * 32) + 20, GPIOF_IN, "vgpio 2"},
	{ (VR_GPIO_INSTANCE0 * 32) + 1,  GPIOF_IN, "vgpio 3"},
};

/* GPIO regulator platform data */
static struct gpio_reg_platform_data gpio_vr_init_data = {
	.name			= "VFB",
	.pmic_init_data		= &gpio_pmic_init_data,
	.gpio_vsel_table	= ti816x_vsel_table,
	.num_voltages		= ARRAY_SIZE(ti816x_vsel_table),
	.gpios			= vcore_gpios,
	.gpio_single_bank	= true,
	.gpio_arr_mask		= 0xF,
	.num_gpio_pins		= ARRAY_SIZE(vcore_gpios),
	.pmic_vout		= 600000,
};

/* VCORE for SR regulator init */
static struct platform_device ti816x_gpio_vr_device = {
	.name		= "gpio_vr",
	.id		= -1,
	.dev = {
		.platform_data = &gpio_vr_init_data,
	},
};

static void __init ti816x_gpio_vr_init(void)
{
	if (platform_device_register(&ti816x_gpio_vr_device))
		printk(KERN_ERR "failed to register ti816x_gpio_vr device\n");
	else
		printk(KERN_INFO "registered ti816x_gpio_vr device\n");
}
#else
static inline void ti816x_gpio_vr_init(void) {}
#endif

static struct sii9022a_platform_data sii9022a_pdata = {
	.hdmi_hot_plug_gpio_intr_line   = 0,
	.sync_mode                      = 0,
	.clk_edge                       = 0,
};

static struct i2c_board_info __initdata ti816x_i2c_boardinfo0[] = {
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x18),
	},
	{
		I2C_BOARD_INFO("ds1337", 0x68),
	},
	#ifdef CONFIG_REGULATOR_TPS40400
	{
		I2C_BOARD_INFO("pmbus", 0x38),
		.platform_data	= &pmbus_pmic_init_data,
	},
	#endif
};

static struct i2c_board_info __initdata ti816x_i2c_boardinfo1[] = {
	{
		I2C_BOARD_INFO("sii9022a", 0x39),
		.platform_data	= &sii9022a_pdata,
	},
};

static int __init ti816x_dvr_i2c_init(void)
{
	#ifdef CONFIG_REGULATOR_TPS40400
	int hwver = eeprom_uddvr_get_hwver();		//# must get after eeprom_init()
	if(hwver < 0x50)					//# use not tps40400
		ti816x_i2c_boardinfo0[2].platform_data = NULL;
	#endif

	omap_register_i2c_bus(1, 100, ti816x_i2c_boardinfo0,
		ARRAY_SIZE(ti816x_i2c_boardinfo0));
	omap_register_i2c_bus(2, 100, ti816x_i2c_boardinfo1,
		ARRAY_SIZE(ti816x_i2c_boardinfo1));
	return 0;
}

static void __init ti8168_dvr_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
	gpmc_init();
}

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

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

int __init ti_ahci_register(u8 num_inst);

#ifdef CONFIG_SND_SOC_TI81XX_HDMI
static struct snd_hdmi_platform_data ti8168_snd_hdmi_pdata = {
	.dma_addr = TI81xx_HDMI_WP + HDMI_WP_AUDIO_DATA,
	.channel = 53,
	.data_type = 4,
	.acnt = 4,
	.fifo_level = 0x20,
};

static struct platform_device ti8168_hdmi_audio_device = {
	.name	= "hdmi-dai",
	.id	= -1,
        .dev = {
		.platform_data = &ti8168_snd_hdmi_pdata,
        }
};

static struct platform_device ti8168_hdmi_codec_device = {
	.name	= "hdmi-dummy-codec",
	.id	= -1,
};

static struct platform_device *ti8168_devices[] __initdata = {
	&ti8168_hdmi_audio_device,
	&ti8168_hdmi_codec_device,
};
#endif

/*
 * TI8168 PG2.0 support Auto CTS which needs MCLK .
 * McBSP clk is used as MCLK in PG2.0 which have 4 parent clks
 * sysclk20, sysclk21, sysclk21 and CLKS(external)
 * Currently we are using sysclk22 as the parent for McBSP clk
 * ToDo:
*/
void __init ti8168_hdmi_mclk_init(void)
{
	int ret = 0;
	struct clk *parent, *child;

	/* modify the clk name from list to use different clk source */
	parent = clk_get(NULL, "sysclk22_ck");
	if (IS_ERR(parent))
		pr_err("Unable to get [sysclk22_ck] clk\n");

	/* get HDMI dev clk*/
	child = clk_get(NULL, "hdmi_i2s_ck");
	if (IS_ERR(child))
		pr_err("Unable to get [hdmi_i2s_ck] clk\n");

	ret = clk_set_parent(child, parent);
	if (ret < 0)
		pr_err("Unable to set parent [hdmi_ck] clk\n");

	pr_debug("HDMI: Audio MCLK setup complete!\n");

}

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
	.tx_dma_offset	= 0x46800000,
	.rx_dma_offset	= 0x46800000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer = ARRAY_SIZE(aic3x_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= aic3x_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_2,
	.txnumevt	= 1,
	.rxnumevt	= 1,
};

static struct resource ti81xx_mcasp2_resource[] = {
   {
		.name = "mcasp2",
		.start = TI81XX_ASP2_BASE,
		.end = TI81XX_ASP2_BASE + (SZ_1K * 12) - 1,
		.flags = IORESOURCE_MEM,
	},
	/* TX event */
	{
		.start = TI81XX_DMA_MCASP2_AXEVT,
		.end = TI81XX_DMA_MCASP2_AXEVT,
		.flags = IORESOURCE_DMA,
	},
	/* RX event */
	{
		.start = TI81XX_DMA_MCASP2_AREVT,
		.end = TI81XX_DMA_MCASP2_AREVT,
		.flags = IORESOURCE_DMA,
	},
};

static struct platform_device ti81xx_mcasp2_device = {
    .name = "davinci-mcasp",
    .id = 2,
    .dev ={
		.platform_data = &aic3x_snd_data,
	},
    .num_resources = ARRAY_SIZE(ti81xx_mcasp2_resource),
    .resource = ti81xx_mcasp2_resource,
};
#endif

#ifdef CONFIG_EEPROM_UDDVR
static int __init ti8168_dvr_eeprom_init(void)
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

static void __init ti8168_dvr_init(void)
{
	static struct mtd_partition *nand_partitions;
	u8 nand_array_size;

	ti81xx_mux_init(board_mux);
	omap_serial_init();
	
	ti8168_dvr_eeprom_init();
	ti816x_dvr_i2c_init();

	#ifdef CONFIG_SND_SOC_TVP5158_AUDIO
	platform_device_register(&tvp5158_audio_device);
	#endif

	platform_device_register(&ti81xx_mcasp0_device);
	
	#ifdef CONFIG_SND_SOC_TLV320AIC3X
	platform_device_register(&ti81xx_mcasp2_device);
	#endif
	
	/* initialize usb */
	usb_musb_init(&musb_board_data);
	
	if (eeprom_uddvr_get_hwver() > 0x30) {
		nand_partitions = (struct mtd_partition *)&ti816x_nand_partitions_bs128;
		nand_array_size = ARRAY_SIZE(ti816x_nand_partitions_bs128);
	}
	else {
		nand_partitions = (struct mtd_partition *)&ti816x_nand_partitions_bs256;
		nand_array_size = ARRAY_SIZE(ti816x_nand_partitions_bs256);
	}
	
	board_nand_init(nand_partitions, nand_array_size, 0, 0);

	omap2_hsmmc_init(mmc);

	#ifdef CONFIG_SND_SOC_TI81XX_HDMI
	ti8168_hdmi_mclk_init();
	platform_add_devices(ti8168_devices, ARRAY_SIZE(ti8168_devices));
	#endif

	regulator_has_full_constraints();
	regulator_use_dummy_regulator();
}

static int __init ti8168_dvr_gpio_setup(void)
{
	/* GPIO-20 should be low for NOR access beyond 4KiB */
	/*
	gpio_request(20, "nor");
	gpio_direction_output(20, 0x0);
	*/
	return 0;
}
/* GPIO setup should be as subsys_initcall() as gpio driver
 * is registered in arch_initcall()
 */
subsys_initcall(ti8168_dvr_gpio_setup);

static void __init ti8168_dvr_map_io(void)
{
	omap2_set_globals_ti816x();
	ti81xx_map_common_io();
}

MACHINE_START(TI8168EVM, "ud8168_dvr")
	/* Maintainer: Texas Instruments */
	.boot_params	= 0x80000100,
	.map_io		= ti8168_dvr_map_io,
	.reserve         = ti81xx_reserve,
	.init_irq	= ti8168_dvr_init_irq,
	.init_machine	= ti8168_dvr_init,
	.timer		= &omap_timer,
MACHINE_END
