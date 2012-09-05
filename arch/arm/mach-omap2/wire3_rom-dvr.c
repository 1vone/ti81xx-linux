/*
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option)any later version.
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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <plat/mux.h>
#include <plat/gpio.h>

#include "mux.h"

#define EEPROM_CS_GPIO				19
#define EEPROM_CLK_GPIO				18
#define EEPROM_SDI_GPIO				17
#define EEPROM_SDO_GPIO				16

#define EEPROM_SIZE					512

#define CS_ACTIVE					1
#define CS_INACTIVE					0
#define CMD_BITS					12
#define DATA_BITS					8

/*
 *  EEPROM op-codes
 */
#define EEPROM_READ				0xC000	/* read */
#define EEPROM_WRITE			0xA000	/* write */
#define EEPROM_WREN				0x9FF0	/* write enable */
#define EEPROM_ERASE			0xE000	/* location erase */

//# EEPROM Info address
#define ROM_MAC_SYNC	0x4E		//# 'N' for mac is valid
#define ROM_MAC_ADDR0	0
#define ROM_MAC_SIZE	8
#define ROM_MAC_ADDR1	(ROM_MAC_ADDR0+ROM_MAC_SIZE)

#define ROM_HW_ADDR		(ROM_MAC_ADDR1+ROM_MAC_SIZE)
#define ROM_HW_SIZE		2

static int dvr_hwver=0;

// ------ SPI over GPIO routines ---------------------------------------------
static inline void setsck(int val)
{
    gpio_set_value(EEPROM_CLK_GPIO, val ? 1 : 0);
}

static inline void setmosi(int val)
{
    gpio_set_value(EEPROM_SDO_GPIO, val ? 1 : 0);
}

static inline u32 getmiso(void)
{
    return gpio_get_value(EEPROM_SDI_GPIO) ? 1 : 0;
}

static inline void do_spidelay(unsigned nsecs)
{
	ndelay(nsecs);
}

static void setcs(int on)
{
    gpio_set_value(EEPROM_CS_GPIO, on ? 1 : 0);
}

static void eeprom_cmd(int addr, int cmd)
{
	int i;
	unsigned short word;

	if(addr)
		word = (addr & 0x1FF) << 4 | cmd;
	else
		word = cmd;

	setcs(CS_ACTIVE);
	do_spidelay(500); 	/* min. setup time 200ns */

	for(i = 0; i < CMD_BITS; i++) {
		/* setup MSB (to slave) on leading edge */
		setsck(0);
		setmosi(word & (1 << 15));
		do_spidelay(2000); 	/* T(setup) */

		word <<=1;
		setsck(1);
		do_spidelay(2000);
	}

	setcs(CS_INACTIVE);

	// wait tcs(min 1us)
	do_spidelay(2000);
}

u8 gpio_eeprom_read(int addr)
{
	unsigned short word;
	int i;
	unsigned char data = 0;

	word = (addr & 0x1FF) << 4;
	word |= EEPROM_READ;

	setcs( CS_ACTIVE);
	do_spidelay(500); 	/* min. setup time 200ns */

	for(i = 0; i < CMD_BITS; i++) {
		/* setup MSB (to slave) on leading edge */
		setsck(0);
		setmosi(word & (1 << 15));
		do_spidelay(2000); 	/* T (setup) 100ns */

		word <<=1;
		setsck(1);
		do_spidelay(2000);
	}

	// Note. Dummy bit discard the 8 bit data outputs.
	setsck(0);
	getmiso();
	do_spidelay(2000);
	setsck(1);
	do_spidelay(2000);

	for(i = 0; i < DATA_BITS; i++) {
		data <<= 1;
		/* sample MSB (from slave) on trailing edge */
		setsck(0);
		data |= getmiso();
		do_spidelay(2000); 	/* T(setup) */

		setsck(1);
		do_spidelay(2000);
	}

	setcs(CS_INACTIVE);

	return data;
}
EXPORT_SYMBOL(gpio_eeprom_read);

void gpio_eeprom_write(int addr, u8 data)
{
	unsigned short word;
	int i;

	// write enable command issue.
	eeprom_cmd(0, EEPROM_WREN);

	// page erase command issue.
	eeprom_cmd(addr, EEPROM_ERASE);
	// wait at least 20ms.
	mdelay(20);

	word = (addr & 0x1FF) << 4;
	word |= EEPROM_WRITE;

	setcs(CS_ACTIVE);
	do_spidelay(500); 	/* min. setup time 200ns */

	for(i = 0; i < CMD_BITS; i++) {
		/* setup MSB (to slave) on leading edge */
		setsck(0);
		setmosi(word & (1 << 15));
		do_spidelay(2000); 	/* T(setup) */

		word <<=1;
		setsck(1);
		do_spidelay(2000);
	}

	for(i = 0; i < DATA_BITS; i++) {
		/* sample MSB (from slave) on trailing edge */
		setsck(0);
		setmosi(data & 0x80);
		do_spidelay(2000); 	/* T(setup) */

		data <<= 1;
		setsck(1);
		do_spidelay(2000);
	}

	setcs(CS_INACTIVE);

	// wait tcs(min 1us)
	do_spidelay(2000);

	// wait max 10ms..
	mdelay(20);
}
EXPORT_SYMBOL(gpio_eeprom_write);

int get_dvr_hwver(void)
{
	return dvr_hwver;
}
EXPORT_SYMBOL(get_dvr_hwver);

static void set_dvr_hwver(void)
{
	unsigned char ver[ROM_HW_SIZE];
	int i;

	for(i=0; i<ROM_HW_SIZE; i++)
		ver[i] = gpio_eeprom_read(ROM_HW_ADDR+i);

	dvr_hwver = (ver[0]<<8) + ver[1];
}

int __init eeprom_init(void)
{
	int status;

	/* enable CS GPIO 19 */
	//omap_mux_init_gpio(EEPROM_CS_GPIO, OMAP_PIN_OUTPUT);
	status = gpio_request(EEPROM_CS_GPIO, "eeprom cs");
	if (status) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_CS_GPIO);
		return status;
	}

	gpio_direction_output(EEPROM_CS_GPIO, 1);

	/* enable CK GPIO 18 */
	//omap_mux_init_gpio(EEPROM_CLK_GPIO, OMAP_PIN_OUTPUT);
	status = gpio_request(EEPROM_CLK_GPIO, "eeprom clk");
	if (status) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_CLK_GPIO);
		return status;
	}

	gpio_direction_output(EEPROM_CLK_GPIO, 1);

	/* enable SDI GPIO 16 */
	//omap_mux_init_gpio(EEPROM_SDI_GPIO, OMAP_PIN_INPUT);
	status = gpio_request(EEPROM_SDI_GPIO, "eeprom sdi");
	if (status) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_CLK_GPIO);
		return status;
	}

	gpio_direction_input(EEPROM_SDI_GPIO);

	/* enable SDO GPIO 17 */
	//omap_mux_init_gpio(EEPROM_SDO_GPIO, OMAP_PIN_OUTPUT);
	status = gpio_request(EEPROM_SDO_GPIO, "eeprom sd0");
	if (status) {
		printk(KERN_ERR "Cannot request GPIO %d\n", EEPROM_CLK_GPIO);
		return status;
	}

	gpio_direction_output(EEPROM_SDO_GPIO, 1);

	set_dvr_hwver();

	printk("3-wired eeprom init done. (H/W ver:%02x)\n", dvr_hwver);

	return 0;
}
EXPORT_SYMBOL(eeprom_init);

static void __exit eeprom_exit(void)
{
	gpio_free(EEPROM_CS_GPIO);
	gpio_free(EEPROM_CLK_GPIO);
	gpio_free(EEPROM_SDI_GPIO);
	gpio_free(EEPROM_SDO_GPIO);
}

//subsys_initcall(eeprom_init);
//module_exit(eeprom_exit);

//MODULE_LICENSE("GPL");

