/*
 * pmbus-regulator.h  --  PMBus Voltage regulation
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef REGULATOR_PMBUS
#define REGULATOR_PMBUS

#include <linux/regulator/machine.h>

/**
 * pmbus_reg_platform_data - platform data for pmbus regulator
 * @name:		name of the pmic output
 * @pmic_init_data:	regulator platform inititialization data
 * @pmic_vout:		pmic output voltage value
 */
struct pmbus_reg_platform_data {
	const	char			*name;
	struct	regulator_init_data	*pmic_init_data;
	int				pmic_vout;
	/*this should be the place to have the pmbus device handle*/
};

#endif
