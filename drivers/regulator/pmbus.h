/*
 * pmbus.h - Common defines for PMBus devices
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
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef PMBUS_H
#define PMBUS_H

/*
 * Registers
 */
#define PMBUS_OPERATION				0x01
#define PMBUS_ON_OFF_CONFIG			0x02
#define PMBUS_CLEAR_FAULTS			0x03
#define PMBUS_VOUT_MODE				0x20
#define PMBUS_VOUT_TRIM				0x22
#define PMBUS_VOUT_CAL_OFFSET		0xD4
#define PMBUS_VOUT_CAL_GAIN			0xD5
#define PMBUS_VOUT_MAX				0x24
#define PMBUS_VOUT_MARGIN_HIGH		0x25
#define PMBUS_VOUT_MARGIN_LOW		0x26
#define PMBUS_VOUT_SCALE_LOOP		0x29
#define PMBUS_FREQUENCY_SWITCH		0x33
#define PMBUS_IOUT_CAL_GAIN         0x38
#define PMBUS_IOUT_OC_FAULT_LIMIT	0x46
#define PMBUS_IOUT_OC_WARN_LIMIT	0x4A

//# freq value = <freq KH>/32(2^5)
#define FREQ_400K					13
#define FREQ_600K					19
#define FREQ_680K					21	//# 672K
#define FREQ_1200K					38

#define PMBUS_READ_VOUT				0x8B

#define PMBUS_REVISION				0x98

#endif /* PMBUS_H */
