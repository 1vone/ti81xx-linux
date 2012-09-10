/*
 * Copyright (C) 2012, UDWORKs
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
 
#ifndef __EEPROM_UDDVR_H__
#define __EEPROM_UDDVR_H__

#include <asm/mach-types.h>

#ifdef CONFIG_MACH_UD8168_DVR
#	define EEPROM_SCLK			18 //# output
#	define EEPROM_CS			19 //# output
#	define EEPROM_SDO			16 //# output
#	define EEPROM_SDI			17 //# input
#else
#	define EEPROM_SCLK			1 //# output
#	define EEPROM_CS			2 //# output
#	define EEPROM_SDO			3 //# output
#	define EEPROM_SDI			4 //# input
#endif

#define EEPROM_SIZE				128 //# bytes

int eeprom_uddvr_read(u32 offset, size_t len, void *buf);
int eeprom_uddvr_write(u32 offset, size_t len, const void *buf);
int eeprom_uddvr_set_hwver(void);
int eeprom_uddvr_get_hwver(void);

#endif //# end of __EEPROM_UDDVR_H__
