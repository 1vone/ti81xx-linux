/*
 * SmartReflex Voltage Control driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 * Author: AnilKumar Ch <anilkumar@ti.com>
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
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>

#include <plat/common.h>
#include <plat/smartreflex.h>

#include "control.h"

#define MARGIN_PERCEN_ERROR	0
#define CLK_NAME_LEN		40
#define MAX_SENSORS_PER_VD	2

/* Restrict SR to Class1 operation */
#if 0
#define SRCLASS1
#endif

/* Enable calibration of Ntarget during SR init */
#if 1
#define SRCALIBRATE
#endif

/* Enable debug prints */
#if 1
#define dprintk(x...) printk("[DBG]" x)
#else
#define dprintk(x...)
#endif

struct ti816x_sr_sensor {
	u32			irq;
	u32			irq_status;
	u32			efuse_offs;
	u32			nvalue;
	u32			e2v_gain;
	u32			err_weight;
	u32			err_minlimit;
	u32			err_maxlimit;
	u32			senn_en;
	u32			senp_en;
	char			*name;
	void __iomem		*base;
	struct clk		*fck;
	struct timer_list	timer;
};

struct ti816x_sr {
	u32				autocomp_active;
	u32				sens_per_vd;
	u32				irq_delay;
	u32				ip_type;
	int				init_volt_mv;
	int				uvoltage_step_size;
	struct regulator		*reg;
	struct work_struct		work;
#ifdef SRCLASS1
	struct work_struct		opwork;
#endif
	struct sr_platform_data		*sr_data;
	struct ti816x_sr_sensor		sen[MAX_SENSORS_PER_VD];
	struct platform_device		*pdev;
};

#ifdef SRCLASS1
static struct timer_list optimer;
static int opdelay, stopsrop, load_volt_mv;
static void irq_sr_optimer(unsigned long data);
static void sr_stop_vddautocomp(struct ti816x_sr *sr);
#endif

#ifdef SRCALIBRATE
#if !defined(CONFIG_MACH_UD8168_DVR)
#define PMIC_MAX_VOLT		1025000 /* uV - Max voltage capacity of PMIC */
#define PMIC_RESOLUTION		15000	/* uV - Step size of PMIC */
#else
#define PMIC_MAX_VOLT		1050000 /* uV - Max voltage capacity of PMIC */
#define PMIC_RESOLUTION		7812	/* uV - Step size of PMIC */
#endif
#define REGRESSION_RANGE	3	/* int - No of error values used in regression */
#define MAX_LOOPCNT		100	/* size - Error counts, estimates Array Size */
#define ERRAVG_LOOPCNT		5	/* int - No of AvgErr readings used for ncounts calc */
#define GUARDBAND_PERCENTAGE	5	/* % - Customer must decide this value based on their test, % of supply voltage */
#define FAILSAFE_SRVMIN		(((PMIC_MAX_VOLT)*100) / (100 + (GUARDBAND_PERCENTAGE)))	/* uV - This should be PMIC_MAX / ( 1 + GB%)*/
#define TEST_GUARDBAND		25000	/* uV - Extra guardband left over during test */
static int calib_start, calib_step;
static int pmic_setpnt_arr[MAX_LOOPCNT];
static int srvmin_hvtn_arr[MAX_LOOPCNT], srvmin_svtn_arr[MAX_LOOPCNT];
#endif

static inline void sr_write_reg(struct ti816x_sr *sr, int offset, u32 value,
					u32 srid)
{
	__raw_writel(value, sr->sen[srid].base + offset);
}

static inline void sr_modify_reg(struct ti816x_sr *sr, int offset, u32 mask,
				u32 value, u32 srid)
{
	u32 reg_val;

	reg_val = __raw_readl(sr->sen[srid].base + offset);
	reg_val &= ~mask;
	reg_val |= (value&mask);

	__raw_writel(reg_val, sr->sen[srid].base + offset);
}

static inline u32 sr_read_reg(struct ti816x_sr *sr, int offset, u32 srid)
{
	return __raw_readl(sr->sen[srid].base + offset);
}

/* get_errvolt - get error voltage from SR error register
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is HVT or SVT
 *
 * Read the error from SENSOR error register and then convert
 * to voltage delta, return value is the voltage delta in micro
 * volt.
 */
static int get_errvolt(struct ti816x_sr *sr, s32 srid)
{
	u32 senerror_reg;
	int uvoltage;
	s8 terror;

	senerror_reg = sr_read_reg(sr, SENERROR_V2, srid);
	senerror_reg = (senerror_reg & 0x0000FF00);
	terror = (s8)(senerror_reg >> 8);

	/* converting binary to percentage error */
	uvoltage = (int)((terror * 25) >> 5);

	uvoltage = (uvoltage + MARGIN_PERCEN_ERROR) *
				sr->uvoltage_step_size;
	uvoltage = uvoltage * sr->sen[srid].e2v_gain;

	/* converting percentage to value by dividing 100 */
	uvoltage = uvoltage/100;

	return uvoltage;
}

#ifdef SRCLASS1
/* Set PMIC out put to final voltage
 * final voltage = curr_voltage + guard-band voltage
 * Disable the regulator as we enabled it during init
*/
static void set_final_voltage(struct work_struct *work)
{
	struct ti816x_sr *sr;
	int final_volt_mv;

	sr = container_of(work, struct ti816x_sr, opwork);

	final_volt_mv	= regulator_get_voltage(sr->reg);
	dprintk("smartreflex: voltage stabilized at %dmV\n",
						final_volt_mv);
	final_volt_mv	= (final_volt_mv*100/95) + load_volt_mv;
	dprintk("smartreflex: setting final voltage to %dmV\n",
						final_volt_mv);
	regulator_set_voltage(sr->reg, final_volt_mv, final_volt_mv);
	regulator_disable(sr->reg);

}
#endif

/* set_voltage - Schedule task for setting the voltage
 * @work:	pointer to the work structure
 *
 * Voltage is set based on previous voltage and corresponding
 * voltage changes from two sensors. Make sure that the device
 * voltage is such that it satisfies the sensor requesting for
 * the higher voltage.
 *
 * Generic voltage regulator set voltage is used for changing
 * the voltage to new value, both minimum and maximum voltage
 * values are same in this case
 *
 * Disabling the module before changing the voltage, this is
 * needed for not generating interrupt during voltage change,
 * enabling after voltage change. This will also take care of
 * resetting the nCount registers.
 */
static void set_voltage(struct work_struct *work)
{
	struct ti816x_sr *sr;
	int prev_volt, new_volt;
	s32 hvt_dvolt, svt_dvolt;

	sr = container_of(work, struct ti816x_sr, work);

	/* Get the current voltage from GPIO */
	prev_volt = regulator_get_voltage(sr->reg);

	hvt_dvolt = get_errvolt(sr, SRHVT);
	svt_dvolt = get_errvolt(sr, SRSVT);

	if (hvt_dvolt > svt_dvolt)
		new_volt = prev_volt + hvt_dvolt;
	else
		new_volt = prev_volt + svt_dvolt;

	/* Clear the counter, SR module disable */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
			~SRCONFIG_SRENABLE, SRHVT);
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
			~SRCONFIG_SRENABLE, SRSVT);

	printk(KERN_INFO "smartreflex: prev volt %d new_volt %d\n",
						prev_volt, new_volt);

	regulator_set_voltage(sr->reg, new_volt, new_volt);

	/* Restart the module after voltage set */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
			SRCONFIG_SRENABLE, SRHVT);
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
			SRCONFIG_SRENABLE, SRSVT);
#ifdef SRCLASS1
	/* Load and start the timer here */

	if (timer_pending(&optimer)) {
		dprintk("smartreflex: Received another interrupt"
			"before Timer expired\n");
		dprintk("smartreflex: Resetting the Timer...\n");
		/* Received interrupt before timer expired */
		del_timer(&optimer);
	} else if (work_pending(&sr->opwork))
		cancel_work_sync(&sr->opwork);

	optimer.data = (unsigned long)sr;
		optimer.function = irq_sr_optimer;
	optimer.expires = jiffies +
			msecs_to_jiffies(opdelay);
	dprintk("smartreflex: starting operational Timer for"
			"%d secs\n", opdelay/1000);
	add_timer(&optimer);
#endif

}

/* irq_sr_htimer - sr HVT timer callback
 * @data:	data contains the SR driver structure
 *
 * While servicing the HVT IRQ, the timer gets added by HVT IRQ
 * with some time delay and this is called once the timer elapses.
 * This will re-enable the HVT interrupt
 */
static void irq_sr_htimer(unsigned long data)
{
	struct ti816x_sr *sr;

	sr = (struct ti816x_sr *)data;

	/* Enable the interrupt */
	sr_modify_reg(sr, IRQENABLE_SET, IRQENABLE_MCUBOUNDSINT,
			IRQENABLE_MCUBOUNDSINT, SRHVT);
}

/* irq_sr_stimer - sr SVT timer callback
 * @data:	data contains the SR driver structure
 *
 * While servicing the SVT IRQ, the timer gets added by SVT IRQ
 * with some time delay and this is called once the timer elapses.
 * This will re-enable the SVT interrupt
 */
static void irq_sr_stimer(unsigned long data)
{
	struct ti816x_sr *sr;

	sr = (struct ti816x_sr *)data;

	/* Enable the interrupt */
	sr_modify_reg(sr, IRQENABLE_SET, IRQENABLE_MCUBOUNDSINT,
			IRQENABLE_MCUBOUNDSINT, SRSVT);
}

#ifdef SRCLASS1
/* SR Operational timer callback */
static void irq_sr_optimer(unsigned long data)
{
	struct ti816x_sr *sr;
	sr = (struct ti816x_sr *)data;
	stopsrop = 1;

	dprintk("smartreflex: Timer expired ->"
					"Disabling SmartReflex\n");

	sr_stop_vddautocomp(sr);

	dprintk("smartreflex: SmartReflex module disabled\n");

}
#endif

/* sr_class2_irq - sr irq handling
 * @irq:	Number of the irq serviced
 * @data:	data contains the SR driver structure
 *
 * Smartreflex IRQ handling for class2 IP, once the IRQ handler
 * is here then disable the interrupt and re-enable after some
 * time. This is the work around for handling both interrupts,
 * while one got satisfied with the voltage change but not the
 * other. The same logic helps the case where PMIC cannot set
 * the exact voltage requested by SR IP
 *
 * Schedule work only if both interrupts are serviced
 *
 * Note that same irq handler is used for both the interrupts,
 * needed for decision making for voltage change
 */
static irqreturn_t sr_class2_irq(int irq, void *data)
{
	u32 srid;
	struct ti816x_sr *sr = (struct ti816x_sr *)data;

	if (irq == TI81XX_IRQ_SMRFLX0)
		srid = SRHVT;
	else
		srid = SRSVT;

	sr->sen[srid].irq_status = 1;

	/* Clear MCUBounds Interrupt */
	sr_modify_reg(sr, IRQSTATUS, IRQSTATUS_MCBOUNDSINT,
			IRQSTATUS_MCBOUNDSINT, srid);

	/* Disable the interrupt and enable after timer expires */
	sr_modify_reg(sr, IRQENABLE_CLR, IRQENABLE_MCUBOUNDSINT,
			IRQENABLE_MCUBOUNDSINT, srid);

	if (!timer_pending(&sr->sen[srid].timer)) {
		sr->sen[srid].timer.data = (unsigned long)sr;
		if (srid == SRHVT)
			sr->sen[srid].timer.function = irq_sr_htimer;
		else
			sr->sen[srid].timer.function = irq_sr_stimer;

		sr->sen[srid].timer.expires = jiffies +
				msecs_to_jiffies(sr->irq_delay);
		add_timer(&sr->sen[srid].timer);
	} else {
		/* Timer of this interrupt should not be pending
		 * while system is running
		 */
		BUG();
	}

	if ((sr->sen[SRHVT].irq_status == 1) &&
			(sr->sen[SRSVT].irq_status == 1)) {
		schedule_work(&sr->work);
		sr->sen[SRHVT].irq_status = 0;
		sr->sen[SRSVT].irq_status = 0;
	}

	return IRQ_HANDLED;
}

static int sr_clk_enable(struct ti816x_sr *sr, u32 srid)
{
	if (clk_enable(sr->sen[srid].fck) != 0) {
		dev_err(&sr->pdev->dev, "%s: Could not enable sr_fck\n",
					__func__);
		return -EINVAL;
	}

	return 0;
}

static int sr_clk_disable(struct ti816x_sr *sr, u32 srid)
{
	clk_disable(sr->sen[srid].fck);

	return 0;
}

static inline int sr_set_nvalues(struct ti816x_sr *sr, u32 srid)
{
	/* Read nTarget value form EFUSE register*/
	sr->sen[srid].nvalue = __raw_readl(TI81XX_CTRL_REGADDR
			(sr->sen[srid].efuse_offs)) & 0xFFFFFF;

	return 0;
}

/* sr_configure - Configure SR module to work in Error generator mode
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is HVT or SVT
 *
 * Configure the corresponding values to SR module registers for
 * operating SR module in Error Generator mode.
 */
static void sr_configure(struct ti816x_sr *sr, u32 srid)
{
	/* Configuring the SR module with clock length, enabling the
	 * error generator, enable SR module, enable individual N and P
	 * sensors
	 */
	sr_write_reg(sr, SRCONFIG, (SRCLKLENGTH_125MHZ_SYSCLK |
		SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
		(sr->sen[srid].senn_en << SRCONFIG_SENNENABLE_V2_SHIFT) |
		(sr->sen[srid].senp_en << SRCONFIG_SENPENABLE_V2_SHIFT)),
		srid);

	/* Configuring the Error Generator */
	sr_modify_reg(sr, ERRCONFIG_V2, (SR_ERRWEIGHT_MASK |
		SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
		((sr->sen[srid].err_weight << ERRCONFIG_ERRWEIGHT_SHIFT) |
		(sr->sen[srid].err_maxlimit << ERRCONFIG_ERRMAXLIMIT_SHIFT) |
		(sr->sen[srid].err_minlimit << ERRCONFIG_ERRMINLIMIT_SHIFT)),
		srid);
}

/* sr_enable - Enable SR module
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is HVT or SVT
 *
 * Enable SR module by writing nTarget values to corresponding SR
 * NVALUERECIPROCAL register, enable the interrupt and enable SR
 */
static void sr_enable(struct ti816x_sr *sr, u32 srid)
{
	/* Check if SR is already enabled. If yes do nothing */
	if (sr_read_reg(sr, SRCONFIG, srid) & SRCONFIG_SRENABLE)
		return;

	if (sr->sen[srid].nvalue == 0)
		dev_err(&sr->pdev->dev, "%s: OPP doesn't support SmartReflex\n",
				__func__);

	/* Writing the nReciprocal value to the register */
	sr_write_reg(sr, NVALUERECIPROCAL, sr->sen[srid].nvalue, srid);

	/* Enable the interrupt */
	sr_modify_reg(sr, IRQENABLE_SET, IRQENABLE_MCUBOUNDSINT,
				IRQENABLE_MCUBOUNDSINT, srid);

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
				SRCONFIG_SRENABLE, srid);
}

/* sr_disable - Disable SR module
 * @sr:		contains SR driver data
 * @srid:	contains the srid, specify whether it is HVT or SVT
 *
 * Disable SR module by disabling the interrupt and Smartrefelx module
 */
static void sr_disable(struct ti816x_sr *sr, u32 srid)
{
	/* Disable the interrupt */
	sr_modify_reg(sr, IRQENABLE_CLR, IRQENABLE_MCUBOUNDSINT,
				IRQENABLE_MCUBOUNDSINT, srid);

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE,
				~SRCONFIG_SRENABLE, srid);
}

/* sr_start_vddautocomp - Start VDD auto compensation
 * @sr:		contains SR driver data
 *
 * This is the starting point for AVS enable from user space.
 */
static void sr_start_vddautocomp(struct ti816x_sr *sr)
{
	int i;
	int ret;

	if ((sr->sen[SRHVT].nvalue == 0) || (sr->sen[SRSVT].nvalue == 0)) {
		dev_err(&sr->pdev->dev, "SR module not enabled, nTarget"
					" values are not found\n");
		return;
	}

	if (sr->autocomp_active == 1) {
		dev_warn(&sr->pdev->dev, "SR VDD autocomp is already active\n");
		return;
	}

	ret = regulator_enable(sr->reg);
	if (ret) {
		dev_err(&sr->pdev->dev, "Failed to enable supply: %d\n", ret);
		return;
	}

	for (i = 0; i < sr->sens_per_vd; i++) {
		init_timer(&sr->sen[i].timer);
		sr_clk_enable(sr, i);
		sr_configure(sr, i);
		sr_enable(sr, i);
	}
#ifdef SRCLASS1
	/* Init operational timer */
	init_timer(&optimer);
	/* Time delay for voltage stabilization */
	opdelay = 60000; /*msec*/
	stopsrop = 0;
	load_volt_mv = 0; /* volt diff between max and min load condtitions */

	INIT_WORK(&sr->opwork, set_final_voltage);
#endif
	sr->autocomp_active = 1;
}

/* sr_stop_vddautocomp - Stop VDD auto compensation
 * @sr:		contains SR driver data
 *
 * This is the ending point during SR disable from user space.
 */
static void sr_stop_vddautocomp(struct ti816x_sr *sr)
{
	int i;

	if (sr->autocomp_active == 0) {
		dev_warn(&sr->pdev->dev, "SR VDD autocomp is not active\n");
		return;
	}

	cancel_work_sync(&sr->work);

	for (i = 0; i < sr->sens_per_vd; i++) {
		sr_disable(sr, i);
		del_timer_sync(&sr->sen[i].timer);
		sr_clk_disable(sr, i);
	}
#ifdef SRCLASS1
	if (!stopsrop) {
#endif
		regulator_set_voltage(sr->reg, sr->init_volt_mv,
						sr->init_volt_mv);
		regulator_disable(sr->reg);
#ifdef SRCLASS1
	} else
		schedule_work(&sr->opwork);
#endif
	sr->autocomp_active = 0;
}

/* ti816x_sr_autocomp_show - Store user input value and stop SR
 * @data:		contains SR driver data
 * @val:		pointer to store autocomp_active status
 *
 * This is the Debug Fs enteries to show whether SR is enabled
 * or disabled
 */
static int ti816x_sr_autocomp_show(void *data, u64 *val)
{
	struct ti816x_sr *sr_info = (struct ti816x_sr *) data;

	*val = (u64) sr_info->autocomp_active;

	return 0;
}

/* ti816x_sr_autocomp_store - Store user input and start SR
 * @data:		contains SR driver data
 * @val:		contains the value pased by user
 *
 * This is the Debug Fs enteries to store user input and
 * enable smartreflex.
 */
static int ti816x_sr_autocomp_store(void *data, u64 val)
{
	struct ti816x_sr *sr_info = (struct ti816x_sr *) data;

	/* Sanity check */
	if (val && (val != 1)) {
		dev_warn(&sr_info->pdev->dev, "%s: Invalid argument %llu\n",
				__func__, val);
		return -EINVAL;
	}

	if (!val)
		sr_stop_vddautocomp(sr_info);
	else
		sr_start_vddautocomp(sr_info);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sr_fops, ti816x_sr_autocomp_show,
		ti816x_sr_autocomp_store, "%llu\n");

/* sr_curr_volt_show - Show current voltage value
 * @data:		contains SR driver data
 * @val:		pointer to store current voltage value
 *
 * Read the current voltage value and display the same on console
 * This is used in debugfs entries
 */
static int sr_curr_volt_show(void *data, u64 *val)
{
	struct ti816x_sr *sr_info = (struct ti816x_sr *) data;

	*val = (u64) regulator_get_voltage(sr_info->reg);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(curr_volt_fops, sr_curr_volt_show,
		NULL, "%llu\n");

#ifdef CONFIG_DEBUG_FS
/* sr_debugfs_entries - Create debugfs entries
 * @sr_info:		contains SR driver data
 *
 * Create debugfs entries, which is exposed to user for knowing
 * the current status. Some of the parameters can change during
 * run time
 */
static int sr_debugfs_entries(struct ti816x_sr *sr_info)
{
	struct dentry *dbg_dir, *sen_dir;
	int i;

	dbg_dir = debugfs_create_dir("smartreflex", NULL);
	if (IS_ERR(dbg_dir)) {
		dev_err(&sr_info->pdev->dev, "%s: Unable to create debugfs directory\n",
								__func__);
		return PTR_ERR(dbg_dir);
	}

	(void) debugfs_create_file("autocomp", S_IRUGO | S_IWUGO, dbg_dir,
				(void *)sr_info, &sr_fops);
	(void) debugfs_create_u32("initial_voltage", S_IRUGO, dbg_dir,
				&sr_info->init_volt_mv);
	(void) debugfs_create_file("current_voltage", S_IRUGO, dbg_dir,
				(void *)sr_info, &curr_volt_fops);
	(void) debugfs_create_u32("interrupt_delay", S_IRUGO | S_IWUGO,
				dbg_dir, &sr_info->irq_delay);

	for (i = 0; i < sr_info->sens_per_vd; i++) {

		sen_dir = debugfs_create_dir(sr_info->sen[i].name, dbg_dir);
		if (IS_ERR(sen_dir)) {
			dev_err(&sr_info->pdev->dev, "%s: Unable to create"
				" debugfs directory\n", __func__);
			return PTR_ERR(sen_dir);
		}

		(void) debugfs_create_x32("err2voltgain", S_IRUGO,
					sen_dir, &sr_info->sen[i].e2v_gain);
		(void) debugfs_create_x32("nvalue", S_IRUGO,
					sen_dir, &sr_info->sen[i].nvalue);
	}
	return 0;
}
#else
static int sr_debugfs_entries(struct ti816x_sr *sr_info)
{
	return 0;
}
#endif

#ifdef SRCALIBRATE

/* Gives you the Ntargets loaded into sr registers from efuses */
static int sr_get_ncounts(struct ti816x_sr *sr_info, int srid, int *ntarg,
									int *ga)
{
	int ntarget_reg, nreciprocal, gain;

	if (NULL == ntarg)
		return -EINVAL;
	ntarget_reg = sr_read_reg(sr_info, NVALUERECIPROCAL, srid);
	nreciprocal = ntarget_reg & 0xff;
	gain = (ntarget_reg&0xf0000) >> 16;
	if (NULL != ga)
		*ga = gain;
	*ntarg = ((int)(0x1 << (gain + 8)) / nreciprocal);
	return 0;
}

/* Gives you the difference between efused ntargets and ncounts
 * at the timer of the error */
static void sr_get_counts_from_error(struct ti816x_sr *sr_info, int srid,
								int *buff)
{
	int ntarget, senerror_reg, avgerror, i, ncounts_avg = 0;

	sr_get_ncounts(sr_info, srid, &ntarget, NULL);
	dprintk("%s ntarget %d\n", __func__, ntarget);
	/* Read sr sensor avgerr register and calculate ncounts
	 * Use the average of 5 readings to account for
	 * momentary changes in the average due to volt ripples */
	for(i = 0; i < ERRAVG_LOOPCNT; i++) {
		senerror_reg = sr_read_reg(sr_info, SENERROR_V2, srid);
		senerror_reg = (senerror_reg & 0x0000FF00);
		avgerror = (s8)(senerror_reg >> 8);
		dprintk("%s error rdng no %d =  %d\n", __func__, i, avgerror);
		ncounts_avg += (-(int)(ntarget*avgerror)/100);
		/* wait for a couple of msecs */
		usleep_range(1000, 5000);
	}
	*buff = ncounts_avg / ERRAVG_LOOPCNT;
	dprintk("%s Ncounts offset %d\n", __func__, *buff);
}

/* runs a regression to for given x_for_eval and returns estimate of y */
static int sr_run_regression(int x_for_eval, int range, int *x, int *y,
								int count)
{
	int i, num, denom, slope, intercept;
	int xsum, ysum, xavg, yavg;

	i = num = denom = slope = intercept = 0;
	xsum = ysum = xavg = yavg =0;

	if (NULL == x || NULL == y)
		return -EINVAL;

	dprintk("%s count %d\n", __func__, count);
	dprintk("%s range %d\n", __func__, range);
	for (i = count - range +1; i <= count; i++) {
		xsum += x[i];
		ysum += y[i];
	}
	xavg = xsum / range;
	yavg = ysum / range;
	dprintk("%s xavg %d\n", __func__, xavg);
	dprintk("%s yavg %d\n", __func__, yavg);

	/* Calculate slope */
	for (i=count - range + 1; i <= count; i++) {
		num += (x[i]- xavg)*(y[i]- yavg);
		denom += (x[i]- xavg)*(x[i]-xavg);
	}

	/* Check for denominator != 0 */
	if (denom == 0)	{
		printk(KERN_ERR "%s Error: Denominator is zero\n",
								__func__);
		BUG();
	}
	dprintk("%s num %d denom %d\n", __func__, num,denom);
	slope = num / denom;

	dprintk("slope = %d\n", slope);
	/* Calculate intercept */
	intercept = yavg - slope * xavg;

	dprintk("intercept = %d\n", intercept);
	/* Return prediciton */
	dprintk("value = %d\n", (intercept + slope * x_for_eval));
	return (intercept + slope * x_for_eval);

}

/* Write the new target values to registers */
static int sr_set_new_ntarget(struct ti816x_sr *sr_info, int srid,
								int offset_n)
{
	int ntarget, gain, nreciprocal, val;
	/* As per SR spec only n and p type sensors are exclusive
	 * we are using n type
	 * if p type sensor is enabled then get p counts and rewrite reg
	 */
	/* CHANGE#6 Failsafe : Do not lower Ntargets
	 * Program new ntargets only if the offset is +ve */
	if(offset_n > 0) {
		sr_get_ncounts(sr_info, srid, &ntarget, &gain);
		dprintk("Old Ntarget value for %s was %d\n",srid==SRHVT?"HVT":"SVT",ntarget);
		ntarget = ntarget + offset_n;
		nreciprocal = (0x1 << (gain+8)) / ntarget;
		val = sr_read_reg(sr_info, NVALUERECIPROCAL, srid);
		val &= 0xffffff00;
		val |= (0xff & nreciprocal);
		sr_write_reg(sr_info, NVALUERECIPROCAL, val, srid);
		/* update in software */
		sr_info->sen[srid].nvalue = val;
	}
	return 0;
}

/* Hey.. who is Max? */
int sr_max_of(int a, int b)
{
	return (a>b ? a : b);
}

/* Calibration routine */
static int sr_ntarget_calibration(struct ti816x_sr *sr)
{
	int pmic_set_volt, srvmin_estimate, gbsrvmin_estimate, srvmin_gb;
	int gb_count_estimate_hvtn = 0, gb_count_estimate_svtn = 0, new_ntarget;
	int srvmin_estimate_hvtn, srvmin_estimate_svtn, loopcnt = 0;
	int i, srstatus_reg, supp_settling_time_us = 1000; /* usecs - T = CV/I */
	struct ti816x_sr *sr_info = (struct ti816x_sr *) sr;
	/* Initialize test parameters */
	pmic_set_volt	= calib_start = PMIC_MAX_VOLT; /* uV */
	calib_step	= PMIC_RESOLUTION;
	srvmin_gb	= GUARDBAND_PERCENTAGE;
	gbsrvmin_estimate = -1;

	/* Disable SR interrupts */

	for(i = 0; i <= 1; i++) {
		/* Clear MCUBounds Interrupt */
		sr_modify_reg(sr, IRQSTATUS, IRQSTATUS_MCBOUNDSINT,
				IRQSTATUS_MCBOUNDSINT, i);

		/* Disable the interrupt and enable after timer expires */
		sr_modify_reg(sr, IRQENABLE_CLR, IRQENABLE_MCUBOUNDSINT,
				IRQENABLE_MCUBOUNDSINT, i);
	}

	/* XXX: Make the loop termination independent of guardband
	 * currently we use gbsrvmin_estimate for looptermination which
	 * is deopendent on guardband(srvmin_gb) fixed @5%
	 * Stretch goal: we need find a way to make srvmin_gb a configurable
	 * value and derive it runtime as per the drop/ripple on the board
	 */
	while (((loopcnt <= REGRESSION_RANGE) ||
					(pmic_set_volt >= gbsrvmin_estimate)) && loopcnt < MAX_LOOPCNT) {
		dprintk("\tloopcnt/iteration %d\n", loopcnt);
		pmic_set_volt = calib_start - loopcnt * calib_step;
		/* Clear the counter, SR module disable */
		sr_modify_reg(sr_info, SRCONFIG, SRCONFIG_SRENABLE,
						~SRCONFIG_SRENABLE, SRHVT);
		sr_modify_reg(sr_info, SRCONFIG, SRCONFIG_SRENABLE,
						~SRCONFIG_SRENABLE, SRSVT);

		dprintk("%s old voltage %d new voltage %d\n",
			__func__, regulator_get_voltage(sr_info->reg),
								pmic_set_volt);

		regulator_set_voltage(sr_info->reg, pmic_set_volt,
								pmic_set_volt);
		/*  Wait for the pmic voltage to stabilze  T = CV/I*/
		udelay(supp_settling_time_us);

		/* Restart the module after voltage set */
		sr_modify_reg(sr_info, SRCONFIG, SRCONFIG_SRENABLE,
						SRCONFIG_SRENABLE, SRHVT);
		sr_modify_reg(sr_info, SRCONFIG, SRCONFIG_SRENABLE,
						SRCONFIG_SRENABLE, SRSVT);
		pmic_setpnt_arr[loopcnt] = pmic_set_volt;

		/* Sleep for sometime let the error accumulate*/
		usleep_range(10000, 20000);

		/* wait till the AvgError is valid for HVT*/
		srstatus_reg = sr_read_reg(sr_info, SRSTATUS, SRHVT);
		while((srstatus_reg & SRSTATUS_AVGERRVALID) == 0) {
			usleep_range(10000, 20000);
			dprintk("\tWaiting for AvgErr to be valid for HVT\n");
		}
		sr_get_counts_from_error(sr_info, SRHVT,
						&srvmin_hvtn_arr[loopcnt]);
		dprintk("\tSRErrorCounts N, HVT %d\n",
						srvmin_hvtn_arr[loopcnt]);

		/* wait till the AvgError is valid for SVT*/
		srstatus_reg = sr_read_reg(sr_info, SRSTATUS, SRSVT);
		while((srstatus_reg & SRSTATUS_AVGERRVALID) == 0) {
			usleep_range(10000, 20000);
			dprintk("\tWaiting for AvgErr to be valid for SVT\n");
		}

		sr_get_counts_from_error(sr_info, SRSVT,
						&srvmin_svtn_arr[loopcnt]);
		dprintk("\tSRErrorCounts N, SVT %d\n",
						srvmin_svtn_arr[loopcnt]);
		loopcnt++;
		if (loopcnt >= REGRESSION_RANGE) {
			srvmin_estimate_hvtn =
			sr_run_regression(0, REGRESSION_RANGE, srvmin_hvtn_arr,
						pmic_setpnt_arr, loopcnt-1);
			srvmin_estimate_svtn =
			sr_run_regression(0, REGRESSION_RANGE, srvmin_svtn_arr,
						pmic_setpnt_arr, loopcnt-1);

			dprintk("\tsrvmin_estimate_hvtn %d\n",
							srvmin_estimate_hvtn);
			dprintk("\tsrvmin_estimate_svtn %d\n",
							srvmin_estimate_svtn);

			srvmin_estimate = sr_max_of(srvmin_estimate_hvtn,
							srvmin_estimate_svtn);
			dprintk("srvmin_estimate = %d\n",
							srvmin_estimate);
			/* CHANGE#2:  If PMIC current voltage gone below srvmin_estimate
			 * then there is a risk of system crashing
			 */
			WARN((pmic_set_volt <  srvmin_estimate), "PMIC voltage has gone below estimated SRVmin value - system reliablity is at risk");

			/* CHANGE#4: USe the left over guardband during test */
			gbsrvmin_estimate =
			(srvmin_estimate + (srvmin_estimate * srvmin_gb) / 100) - TEST_GUARDBAND;

			dprintk("gbsrvmin_estimate = %d\n",
							gbsrvmin_estimate);

			gb_count_estimate_hvtn =
			sr_run_regression(gbsrvmin_estimate, REGRESSION_RANGE,
					pmic_setpnt_arr, srvmin_hvtn_arr,
								loopcnt-1);

			gb_count_estimate_svtn =
			sr_run_regression(gbsrvmin_estimate, REGRESSION_RANGE,
					pmic_setpnt_arr, srvmin_svtn_arr,
								loopcnt-1);

			dprintk("\tgb_count_estimate_hvtn %d\n",
							gb_count_estimate_hvtn);
			dprintk("\tgb_count_estimate_svtn %d\n",
							gb_count_estimate_svtn);
			dprintk("\t search end state pmic_set_volt <= gbsrvmin_estimate %d\n",
							gbsrvmin_estimate);
		}
	}

	/* XXX: If we have maxed out on arrays go ahead with available data instead of bugging out */
	WARN((loopcnt == MAX_LOOPCNT - 1), "Err_Counts Array size was not enough for calibration loop, using acquired data for calibration\n");

	/* CHANGE#1: Need to handle the case where gbsrvmin_estimate is greater than the
	 * maximum regulator output
	 */
	if (gbsrvmin_estimate > PMIC_MAX_VOLT || gbsrvmin_estimate > FAILSAFE_SRVMIN) {
	/* CHANGE#3: if gbsrvmin estimated is greater than our failsafe voltage then fall back */
		if(gbsrvmin_estimate > FAILSAFE_SRVMIN) {
			printk(KERN_ERR "WARN: Guardbanded SRVmin %d is greater than failsafe volt %d\n",
									gbsrvmin_estimate, FAILSAFE_SRVMIN);
			gbsrvmin_estimate = FAILSAFE_SRVMIN;
		} else {
			printk(KERN_ERR "Error: Guardbanded SR VMIN (srvmin_estimate * srvmin_gb = gbsrvmin_estimate)"
				"greater than PMIC maximum voltage ($PMICMaxVoltage)\n");
			printk(KERN_ERR "This device can not be supported with the curent PMIC settings\n");
		}
		printk(KERN_ERR "Adjusting Ntargets to a failsafe voltage of %d\n", FAILSAFE_SRVMIN);
		gb_count_estimate_hvtn =
			sr_run_regression(FAILSAFE_SRVMIN, REGRESSION_RANGE,
					pmic_setpnt_arr, srvmin_hvtn_arr,
								loopcnt-1);

		gb_count_estimate_svtn =
			sr_run_regression(FAILSAFE_SRVMIN, REGRESSION_RANGE,
					pmic_setpnt_arr, srvmin_svtn_arr,
								loopcnt-1);
	}
	/* Upon exit, the last values of the SRError_* registers will contain
	 * the target error.  Offset the efuse value by this error
	 * This should be +
	 */
	sr_set_new_ntarget(sr_info, SRHVT, gb_count_estimate_hvtn);
	sr_get_ncounts(sr_info, SRHVT, &new_ntarget, NULL);
	dprintk("\t***** New Target value for HVT %d\n",
								new_ntarget);

	sr_set_new_ntarget(sr_info, SRSVT, gb_count_estimate_svtn);
	sr_get_ncounts(sr_info, SRSVT, &new_ntarget, NULL);
	dprintk("\t ***** New Target value for SVT %d\n",
								new_ntarget);

	dprintk("\tVoltage boosted by %d\n",
					gbsrvmin_estimate - srvmin_estimate);
	for(i = 0; i <= 1; i++) {
		/* Enable the interrupt */
		sr_modify_reg(sr, IRQENABLE_SET, IRQENABLE_MCUBOUNDSINT,
				IRQENABLE_MCUBOUNDSINT, i);
	}
	return 0;
}
#endif

static int __init ti816x_sr_probe(struct platform_device *pdev)
{
	struct ti816x_sr *sr_info;
	struct ti816x_sr_platform_data *pdata;
	struct resource *res[MAX_SENSORS_PER_VD];
	char *name[MAX_SENSORS_PER_VD];
	int irq;
	int ret;
	int i;

	sr_info = kzalloc(sizeof(struct ti816x_sr), GFP_KERNEL);
	if (!sr_info) {
		dev_err(&pdev->dev, "%s: unable to allocate sr_info\n",
					__func__);
		return -ENOMEM;
	}

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "%s: platform data missing\n", __func__);
		ret = -EINVAL;
		goto err_free_sr_info;
	}

	sr_info->pdev = pdev;
	sr_info->sen[SRHVT].name = "sr_hvt";
	sr_info->sen[SRSVT].name = "sr_svt";
	sr_info->ip_type = pdata->ip_type;
	sr_info->irq_delay = pdata->irq_delay;
	sr_info->sens_per_vd = pdata->no_of_sens/pdata->no_of_vds;
	sr_info->uvoltage_step_size = pdata->vstep_size_uv;
	sr_info->autocomp_active = false;

	/* Reading nTarget Values */
	for (i = 0; i < sr_info->sens_per_vd; i++) {
		sr_info->sen[i].efuse_offs = pdata->sr_sdata[i].efuse_offs;
		sr_set_nvalues(sr_info, i);
	}

	if ((sr_info->sen[SRHVT].nvalue == 0) ||
			(sr_info->sen[SRSVT].nvalue == 0)) {
		dev_err(&pdev->dev, "Driver is not initialized,"
				" nTarget values are not found\n");
		ret = 0;
		goto err_free_sr_info;
	}

	INIT_WORK(&sr_info->work, set_voltage);

	for (i = 0; i < sr_info->sens_per_vd; i++) {

		name[i] = kzalloc(CLK_NAME_LEN + 1, GFP_KERNEL);
		/* resources */
		res[i] = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					sr_info->sen[i].name);
		if (!res[i]) {
			dev_err(&pdev->dev, "%s: no mem resource\n", __func__);
			ret = -ENOENT;
			goto err_free_mem;
		}

		irq = platform_get_irq_byname(pdev, sr_info->sen[i].name);
		if (irq < 0) {
			dev_err(&pdev->dev, "Can't get interrupt resource\n");
			ret = irq;
			goto err_free_mem;
		}
		sr_info->sen[i].irq = irq;

		res[i] = request_mem_region(res[i]->start,
				resource_size(res[i]), pdev->name);
		if (!res[i]) {
			dev_err(&pdev->dev, "can't request mem region\n");
			ret = -EBUSY;
			goto err_free_reg;
		}

		sr_info->sen[i].base = ioremap(res[i]->start,
				resource_size(res[i]));
		if (!sr_info->sen[i].base) {
			dev_err(&pdev->dev, "%s: ioremap fail\n", __func__);
			ret = -ENOMEM;
			goto err_release_mem;
		}

		strcat(name[i], sr_info->sen[i].name);
		strcat(name[i], "_fck");

		sr_info->sen[i].fck = clk_get(NULL, name[i]);
		if (IS_ERR(sr_info->sen[i].fck)) {
			dev_err(&pdev->dev, "%s: Could not get sr fck\n",
						__func__);
			ret = PTR_ERR(sr_info->sen[i].fck);
			goto err_unmap;
		}

		ret = request_irq(sr_info->sen[i].irq, sr_class2_irq,
			IRQF_DISABLED, sr_info->sen[i].name, (void *)sr_info);
		if (ret) {
			dev_err(&pdev->dev, "%s: Could not install SR ISR\n",
						__func__);
			goto err_put_clock;
		}

		sr_info->sen[i].e2v_gain = pdata->sr_sdata[i].e2v_gain;
		sr_info->sen[i].err_weight = pdata->sr_sdata[i].err_weight;
		sr_info->sen[i].err_minlimit = pdata->sr_sdata[i].err_minlimit;
		sr_info->sen[i].err_maxlimit = pdata->sr_sdata[i].err_maxlimit;
		sr_info->sen[i].senn_en = pdata->sr_sdata[i].senn_mod;
		sr_info->sen[i].senp_en = pdata->sr_sdata[i].senp_mod;
	}

	/* debugfs entries */
	ret = sr_debugfs_entries(sr_info);
	if (ret)
		dev_warn(&pdev->dev, "%s: Debugfs entries are not created\n",
						__func__);

	sr_info->reg = regulator_get(NULL, pdata->vd_name);
	if (IS_ERR(sr_info->reg))
		goto err_free_irq;

	/* Read current GPIO value and voltage */
	sr_info->init_volt_mv = regulator_get_voltage(sr_info->reg);

	platform_set_drvdata(pdev, sr_info);

	dev_info(&pdev->dev, "Driver initialized\n");


	if (pdata->enable_on_init)
		sr_start_vddautocomp(sr_info);
#ifdef SRCALIBRATE
	/* Calibrate Ntarget offsets */
	sr_ntarget_calibration (sr_info);
#endif
	return ret;

err_free_irq:
	for (i = 0; i < sr_info->sens_per_vd; i++)
		free_irq(sr_info->sen[i].irq, (void *)sr_info);

err_put_clock:
	if (i == 1)
		free_irq(sr_info->sen[i-1].irq, pdev);
	for (i = 0; i < sr_info->sens_per_vd; i++)
		clk_put(sr_info->sen[i].fck);

err_unmap:
	if (i == 1)
		clk_put(sr_info->sen[i-1].fck);
	for (i = 0; i < sr_info->sens_per_vd; i++)
		iounmap(sr_info->sen[i].base);

err_release_mem:
	if (i == 1)
		iounmap(sr_info->sen[i-1].base);
	for (i = 0; i < sr_info->sens_per_vd; i++)
		release_mem_region(res[i]->start, resource_size(res[i]));

err_free_reg:
	if (i == 1)
		release_mem_region(res[i-1]->start, resource_size(res[i-1]));

err_free_mem:
	if (i == 1)
		kfree(name[i-1]);
	for (i = 0; i < sr_info->sens_per_vd; i++)
		kfree(name[i]);

err_free_sr_info:
	kfree(sr_info);
	return ret;
}

static int __devexit ti816x_sr_remove(struct platform_device *pdev)
{
	struct ti816x_sr *sr_info;
	struct resource *res[MAX_SENSORS_PER_VD];
	int irq;
	int i;

	sr_info = dev_get_drvdata(&pdev->dev);
	if (!sr_info) {
		dev_err(&pdev->dev, "%s: sr_info missing\n", __func__);
		return -EINVAL;
	}

	if (sr_info->autocomp_active)
		sr_stop_vddautocomp(sr_info);

	for (i = 0; i < sr_info->sens_per_vd; i++) {
		clk_put(sr_info->sen[i].fck);

		iounmap(sr_info->sen[i].base);
		res[i] = platform_get_resource_byname(pdev,
				IORESOURCE_MEM, sr_info->sen[i].name);

		release_mem_region(res[i]->start, resource_size(res[i]));

		irq = platform_get_irq_byname(pdev, sr_info->sen[i].name);
		free_irq(irq, (void *)sr_info);
	}

	kfree(sr_info);

	return 0;
}

static struct platform_driver smartreflex_driver = {
	.driver		= {
		.name	= "smartreflex",
		.owner	= THIS_MODULE,
	},
	.remove		= ti816x_sr_remove,
};

static int __init sr_init(void)
{
	int ret;

	ret = platform_driver_probe(&smartreflex_driver, ti816x_sr_probe);
	if (ret) {
		pr_err("%s: platform driver register failed\n", __func__);
		return ret;
	}

	return 0;
}

static void __exit sr_exit(void)
{
	platform_driver_unregister(&smartreflex_driver);
}
late_initcall(sr_init);
module_exit(sr_exit);

MODULE_DESCRIPTION("TI816X Smartreflex Class2 Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc");
