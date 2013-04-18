/*
 * OMAP Adaptive Body-Bias core
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Mike Turquette <mturquette@ti.com>
 *
 * Copyright (C) 2013 Texas Instruments, Inc.
 * Andrii Tseglytskyi <andrii.tseglytskyi@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/opp.h>
#include <linux/debugfs.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

/* NOMINAL_OPP bypasses the ABB ldo, FAST_OPP sets it to Forward Body-Bias */
#define OMAP_ABB_NOMINAL_OPP	0
#define OMAP_ABB_FAST_OPP	1
#define OMAP_ABB_SLOW_OPP	3
#define OMAP_ABB_NO_LDO		(~0)

/* Time for the ABB ldo to settle after transition (in micro-seconds) */
#define ABB_TRANXDONE_TIMEOUT	50

#define ABB_OPP_SEL_MASK		(0x3 << 0)
#define ABB_OPP_CHANGE_MASK		(0x1 << 2)
#define ABB_SR2EN_MASK			(0x1 << 0)
#define ABB_ACTIVE_FBB_SEL_MASK		(0x1 << 2)
#define ABB_ACTIVE_RBB_SEL_MASK		(0x1 << 1)
#define ABB_SR2_WTCNT_VALUE_MASK	(0xff << 8)

/*
 * struct omap_abb_data - common data for each instance of ABB ldo
 *
 * @opp_sel_mask:	selects Fast/Nominal/Slow OPP for ABB
 * @opp_change_mask:	selects OPP_CHANGE bit value
 * @sr2_wtcnt_value_mask:	LDO settling time for active-mode OPP change
 * @sr2en_mask:			enables/disables ABB
 * @fbb_sel_mask:		selects FBB mode
 * @rbb_sel_mask:		selects RBB mode
 * @settling_time:	IRQ handle used to resolve IRQSTATUS offset & masks
 * @clock_cycles:	value needed for LDO setting time calculation
 * @setup_offs:		PRM_LDO_ABB_XXX_SETUP register offset
 * @control_offs:	PRM_LDO_ABB_IVA_CTRL register offset
 */
struct omap_abb_data {
	u32 opp_sel_mask;
	u32 opp_change_mask;
	u32 sr2_wtcnt_value_mask;
	u32 sr2en_mask;
	u32 fbb_sel_mask;
	u32 rbb_sel_mask;
	unsigned long settling_time;
	unsigned long clock_cycles;
	u8 setup_offs;
	u8 control_offs;
};

/*
 * struct omap_abb - ABB ldo instance
 *
 * @control:	memory mapped ABB registers
 * @txdone:	memory mapped IRQSTATUS register
 * @dev:	device, for which ABB is created
 * @txdone_mask:	ABB mode change done bit
 * @opp_sel:		current ABB status - Fast/Nominal/Slow
 * @notify_clk:		clock, which rate changes are handled by ABB
 * @data:		common data
 * @abb_clk_nb:		clock rate change notifier block
 */
struct omap_abb {
	void __iomem	*control;
	void __iomem	*txdone;
	struct device	*dev;
	u32		txdone_mask;
	u32		opp_sel;
	u32		current_volt;
	struct omap_abb_data	data;
	struct regulator	*supply_reg;
	struct regulator_desc   rdesc;
};

static const struct omap_abb_data __initdata omap36xx_abb_data = {
	.opp_sel_mask		= ABB_OPP_SEL_MASK,
	.opp_change_mask	= ABB_OPP_CHANGE_MASK,
	.sr2en_mask		= ABB_SR2EN_MASK,
	.fbb_sel_mask		= ABB_ACTIVE_FBB_SEL_MASK,
	.sr2_wtcnt_value_mask	= ABB_SR2_WTCNT_VALUE_MASK,
	.setup_offs		= 0,
	.control_offs		= 0x4,
	.settling_time		= 30,
	.clock_cycles		= 8,
};

static const struct omap_abb_data __initdata omap4_abb_data = {
	.opp_sel_mask		= ABB_OPP_SEL_MASK,
	.opp_change_mask	= ABB_OPP_CHANGE_MASK,
	.sr2en_mask		= ABB_SR2EN_MASK,
	.fbb_sel_mask		= ABB_ACTIVE_FBB_SEL_MASK,
	.rbb_sel_mask		= ABB_ACTIVE_RBB_SEL_MASK,
	.sr2_wtcnt_value_mask	= ABB_SR2_WTCNT_VALUE_MASK,
	.setup_offs		= 0,
	.control_offs		= 0x4,
	.settling_time		= 50,
	.clock_cycles		= 16,
};

static const struct omap_abb_data __initdata omap5_abb_data = {
	.opp_sel_mask		= ABB_OPP_SEL_MASK,
	.opp_change_mask	= ABB_OPP_CHANGE_MASK,
	.sr2en_mask		= ABB_SR2EN_MASK,
	.fbb_sel_mask		= ABB_ACTIVE_FBB_SEL_MASK,
	.sr2_wtcnt_value_mask	= ABB_SR2_WTCNT_VALUE_MASK,
	.setup_offs		= 0,
	.control_offs		= 0x4,
	.settling_time		= 50,
	.clock_cycles		= 16,
};

/**
 * omap_abb_readl() - reads ABB control memory
 * @abb:	pointer to the abb instance
 * @offs:	offset to read
 *
 * Returns @offs value
 */
static u32 omap_abb_readl(const struct omap_abb *abb, u32 offs)
{
	return readl(abb->control + offs);
}

/**
 * omap_abb_rmw() - modifies ABB control memory
 * @abb:	pointer to the abb instance
 * @mask:	mask to modify
 * @bits:	bits to store
 * @offs:	offset to modify
 */
static void omap_abb_rmw(const struct omap_abb *abb,
			 u32 mask, u32 bits, u32 offs)
{
	u32 val;

	val = readl(abb->control + offs);
	val &= ~mask;
	val |= bits;
	writel(val, abb->control + offs);
}

/**
 * omap_abb_check_txdone() - checks ABB tranxdone status
 * @abb:	pointer to the abb instance
 *
 * Returns true or false
 */
static bool omap_abb_check_txdone(const struct omap_abb *abb)
{
	return !!(readl(abb->txdone) & abb->txdone_mask);
}

/**
 * omap_abb_clear_txdone() - clears ABB tranxdone status
 * @abb:	pointer to the abb instance
 */
static void omap_abb_clear_txdone(const struct omap_abb *abb)
{
	writel(abb->txdone_mask, abb->txdone);
};

/**
 * omap_abb_wait_tranx() - waits for ABB tranxdone event
 * @abb:	pointer to the abb instance
 *
 * Returns 0 on success or -ETIMEDOUT if the event
 * is not set on time.
 */
static int omap_abb_wait_tranx(const struct omap_abb *abb)
{
	int timeout;
	bool status;

	timeout = 0;
	while (timeout++ < ABB_TRANXDONE_TIMEOUT) {
		status = omap_abb_check_txdone(abb);
		if (status)
			break;

		udelay(1);
	}

	if (timeout >= ABB_TRANXDONE_TIMEOUT) {
		dev_warn(abb->dev, "%s: ABB TRANXDONE timeout=(%d)\n",
			 __func__, timeout);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * omap_abb_clear_tranx() - clears ABB tranxdone event
 * @abb:	pointer to the abb instance
 *
 * Returns 0 on success or -ETIMEDOUT if the event
 * is not cleared on time.
 */
static int omap_abb_clear_tranx(const struct omap_abb *abb)
{
	int timeout;
	bool status;

	/* clear interrupt status */
	timeout = 0;
	while (timeout++ < ABB_TRANXDONE_TIMEOUT) {
		omap_abb_clear_txdone(abb);

		status = omap_abb_check_txdone(abb);
		if (!status)
			break;

		udelay(1);
	}

	if (timeout >= ABB_TRANXDONE_TIMEOUT) {
		dev_warn(abb->dev, "%s: ABB TRANXDONE timeout=(%d)\n",
			 __func__, timeout);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * omap_abb_set_opp() - program ABB ldo
 * @abb:	pointer to the abb instance
 * @volt:	target voltage
 *
 * Program the ABB ldo to the new state (if necessary), clearing the
 * PRM_IRQSTATUS bit before and after the transition.  Returns 0 on
 * success, -ETIMEDOUT otherwise.
 */
static int omap_abb_set_opp(struct omap_abb *abb, int volt)
{
	u32 opp_sel = 0;
	struct opp *opp = NULL;
	const struct omap_abb_data *data = &abb->data;
	int ret = 0;

	/* Use common OPP API to retrieve ABB sel */
	rcu_read_lock();
	opp = opp_find_freq_exact(abb->dev, volt * 1000, true);
	rcu_read_unlock();

	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		goto out;
	}

	opp_sel = opp_get_voltage(opp);

	/* bail early if no transition is necessary */
	if (opp_sel == abb->opp_sel)
		goto out;

	/* clear interrupt status */
	ret = omap_abb_clear_tranx(abb);
	if (ret)
		goto out;

	/* program the setup register */
	switch (opp_sel) {
	case OMAP_ABB_NOMINAL_OPP:
		omap_abb_rmw(abb,
			     data->fbb_sel_mask | data->rbb_sel_mask,
			     0x0,
			     data->setup_offs);
		break;
	case OMAP_ABB_SLOW_OPP:
		omap_abb_rmw(abb,
			     data->fbb_sel_mask | data->rbb_sel_mask,
			     data->rbb_sel_mask,
			     data->setup_offs);
		break;
	case OMAP_ABB_FAST_OPP:
		omap_abb_rmw(abb,
			     data->fbb_sel_mask | data->rbb_sel_mask,
			     data->fbb_sel_mask,
			     data->setup_offs);
		break;
	default:
		/* Should have never been here! */
		WARN_ONCE(1, "%s: opp_sel %d!!!\n",
			  __func__, opp_sel);
		ret = -EINVAL;
		goto out;
	}

	/* program next state of ABB ldo */
	omap_abb_rmw(abb, data->opp_sel_mask,
		     opp_sel << __ffs(data->opp_sel_mask),
		     data->control_offs);

	/* initiate ABB ldo change */
	omap_abb_rmw(abb, data->opp_change_mask,
		     data->opp_change_mask,
		     data->control_offs);

	/* Wait for conversion completion */
	ret = omap_abb_wait_tranx(abb);
	if (ret)
		goto out;

	/* clear interrupt status */
	ret = omap_abb_clear_tranx(abb);
	if (ret)
		goto out;

	/* track internal state */
	abb->opp_sel = opp_sel;

out:
	if (ret)
		dev_warn(abb->dev, "%s: failed to scale: opp_sel=%d (%d)\n",
			 __func__, opp_sel, ret);

	return ret;
}

/**
 * omap_abb_set_supplier_voltage() - scales supplier voltage
 * @abb:	pointer to the ABB instance
 * @min_uv:	target minimum voltage
 * @max_uv:	target maximum voltage
 *
 * Returns 0 on success, or error code otherwise.
 */
static int omap_abb_set_supplier_voltage(const struct omap_abb *abb,
					 int min_uv, int max_uv)
{
	int ret = 0;

	if (!abb->supply_reg)
		return ret;

	ret = regulator_set_voltage(abb->supply_reg, min_uv, max_uv);
	if (ret)
		dev_err(abb->dev,
			"%s: failed to scale supply ret (%d)\n",
			__func__, ret);
	else
		dev_err(abb->dev, "\n\n[:) :) :)] %s supply SCALE SUCCESS\n\n", __func__);

	return ret;
}

/**
 * omap_abb_reg_set_voltage() - ABB regulator "set_voltage" callback
 * @rdev:	ABB regulator device
 * @min_uv:	target minimum voltage
 * @max_uv:	target maximum voltage
 * @selector:	unused
 *
 * Program the ABB ldo according to new target voltage
 * Returns 0 on success, or error code otherwise.
 */
static int omap_abb_reg_set_voltage(struct regulator_dev *rdev, int min_uv,
				    int max_uv, unsigned *selector)
{
	int ret = 0;
	struct omap_abb *abb = rdev_get_drvdata(rdev);

	if (abb->current_volt == min_uv)
		return 0;

	/*
	 * handle chain regulator
	 *
	 * 1) if OPP scales from low to high order of scaling is the following:
	 * - scale voltage
	 * - set ABB mode
	 * - scale rate
	 *
	 * For this function it means, if regulator is linked to chain,
	 * supplier (which is normally AVS) need to be scaled first.
	 * Sequence will look like
	 *
	 * AVS -> Voltage scale -> ABB set mode -> Freq scale
	 *
	 * 2) if OPP scales from high to low order is opposite:
	 *
	 * Freq scale -> ABB set mode -> AVS -> Voltage scale
	 */
	if (min_uv > abb->current_volt) {
		ret |= omap_abb_set_supplier_voltage(abb, min_uv, max_uv);
		ret |= omap_abb_set_opp(abb, min_uv);
	} else {
		ret |= omap_abb_set_opp(abb, min_uv);
		ret |= omap_abb_set_supplier_voltage(abb, min_uv, max_uv);
	}

	if (ret)
		dev_err(rdev_get_dev(rdev),
			"%s: error (%d) min_uv (%d) max_uv (%d)\n",
			__func__, ret, min_uv, max_uv);
	else
		abb->current_volt = min_uv;

	return ret;
}

/**
 * omap_abb_reg_get_voltage() - ABB regulator "get_voltage" callback
 * @rdev:	ABB regulator device
 *
 * Returns current voltage of ABB regulator
 */
static int omap_abb_reg_get_voltage(struct regulator_dev *rdev)
{
	struct omap_abb *abb = rdev_get_drvdata(rdev);
	return 1;
}

static const struct of_device_id __initdata omap_abb_of_match[] = {
	{ .compatible = "ti,omap36xx-abb", .data = &omap36xx_abb_data},
	{ .compatible = "ti,omap4-abb", .data = &omap4_abb_data},
	{ .compatible = "ti,omap5-abb", .data = &omap5_abb_data},
	{},
};
MODULE_DEVICE_TABLE(of, omap_abb_of_match);

static unsigned int omap_abb_reg_get_optimum_mode (
					struct regulator_dev *rdev,
					int input_uv,
					int output_uv, int load_uA)
{
	return REGULATOR_MODE_NORMAL;
}

static int omap_abb_reg_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
	return 0;
}

static struct regulator_ops omap_abb_reg_ops = {
	.set_voltage	= omap_abb_reg_set_voltage,
	.get_voltage	= omap_abb_reg_get_voltage,
	.get_optimum_mode	= omap_abb_reg_get_optimum_mode,
	.set_mode = omap_abb_reg_set_mode,
};

/*
 * omap_abb_info_dump() - ABB debug dump
 *
 * Returns 0
 */
static int omap_abb_info_dump(struct seq_file *sf, void *unused)
{
	const struct omap_abb *abb = (struct omap_abb *)sf->private;
	struct opp *opp;
	u32 abb_ctrl, abb_setup;
	unsigned long freq = 0;

	if (!abb) {
		seq_printf(sf, "No ABB defined\n");
		goto err;
	}

	abb_ctrl =  omap_abb_readl(abb, abb->data.control_offs);
	abb_setup = omap_abb_readl(abb, abb->data.setup_offs);
	seq_printf(sf, "Enabled\t->\t%d\n"
		   "opp_sel\t->\t%u\n"
		   "FBB mode\t->\t%u\n"
		   "RBB mode\t->\t%u\n"
		   "PRM_LDO_ABB_XXX_SETUP\t->\t0x%08x\n"
		   "PRM_LDO_ABB_XXX_CTRL\t->\t0x%08x\n",
		   !!(abb_setup & abb->data.sr2en_mask),
		   abb->opp_sel,
		   !!(abb_setup & abb->data.fbb_sel_mask),
		   !!(abb_setup & abb->data.rbb_sel_mask),
		   abb_setup,
		   abb_ctrl);

	seq_printf(sf, "OPP table\n");
	rcu_read_lock();
	do {
		opp = opp_find_freq_ceil(abb->dev, &freq);
		if (IS_ERR(opp))
			break;

		seq_printf(sf, "Voltage (%lu) ABB (%lu)\n",
			   opp_get_freq(opp) / 1000,
			   opp_get_voltage(opp));

		freq++;
	} while (1);
	rcu_read_unlock();

err:
	return 0;
}

/*
 * omap_abb_fops_open() - debugfs "open" callback
 *
 * Returns 0 on success or error code otherwise
 */
static int omap_abb_fops_open(struct inode *inode, struct file *file)
{
	return single_open(file, omap_abb_info_dump, inode->i_private);
}

static const struct file_operations omap_abb_debug_fops = {
	.open = omap_abb_fops_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*
 * omap_abb_init_timings() - Initialize ABB timings
 * @abb:	pointer to the ABB instance
 *
 * Returns 0 on success or error code otherwise
 */
static int __init omap_abb_init_timings(struct omap_abb *abb)
{
	struct clk *sys_clk = NULL;
	u32 sys_clk_rate, sr2_wt_cnt_val, clock_cycles, abb_sel;

	/*
	 * SR2_WTCNT_VALUE is the settling time for the ABB ldo after a
	 * transition and must be programmed with the correct time at boot.
	 * The value programmed into the register is the number of SYS_CLK
	 * clock cycles that match a given wall time profiled for the ldo.
	 * This value depends on:
	 * settling time of ldo in micro-seconds (varies per OMAP family)
	 * # of clock cycles per SYS_CLK period (varies per OMAP family)
	 * the SYS_CLK frequency in MHz (varies per board)
	 * The formula is:
	 *
	 *                      ldo settling time (in micro-seconds)
	 * SR2_WTCNT_VALUE = ------------------------------------------
	 *                   (# system clock cycles) * (sys_clk period)
	 *
	 * Put another way:
	 *
	 * SR2_WTCNT_VALUE = settling time / (# SYS_CLK cycles / SYS_CLK rate))
	 *
	 * To avoid dividing by zero multiply both "# clock cycles" and
	 * "settling time" by 10 such that the final result is the one we want.
	 */

	sys_clk = clk_get(abb->dev, "abb_sys_ck");
	if (IS_ERR_OR_NULL(sys_clk))
		return -ENODEV;

	/* convert SYS_CLK rate to MHz & prevent divide by zero */
	sys_clk_rate = DIV_ROUND_CLOSEST(clk_get_rate(sys_clk), 1000000);

	/* calculate cycle rate */
	clock_cycles = DIV_ROUND_CLOSEST((abb->data.clock_cycles * 10),
					 sys_clk_rate);

	/* calulate SR2_WTCNT_VALUE */
	sr2_wt_cnt_val = DIV_ROUND_CLOSEST((abb->data.settling_time * 10),
					   clock_cycles);

	omap_abb_rmw(abb, abb->data.sr2_wtcnt_value_mask,
		     (sr2_wt_cnt_val << __ffs(abb->data.sr2_wtcnt_value_mask)),
		     abb->data.setup_offs);

	/* did bootloader set OPP_SEL? */
	abb_sel = omap_abb_readl(abb, abb->data.control_offs);
	abb_sel &= abb->data.opp_sel_mask;
	abb->opp_sel = abb_sel >> __ffs(abb->data.opp_sel_mask);

	/* enable the ldo if not done by bootloader */
	abb_sel = omap_abb_readl(abb, abb->data.setup_offs);
	abb_sel &= abb->data.sr2en_mask;
	if (!abb_sel)
		omap_abb_rmw(abb, abb->data.sr2en_mask,
			     abb->data.sr2en_mask, abb->data.setup_offs);

	clk_put(sys_clk);
	return 0;
}
/*
 * omap_abb_probe() - Initialize an ABB ldo instance
 * @pdev: ABB platform device
 *
 * Initializes an individual ABB ldo for Forward Body-Bias.  FBB is used to
 * insure stability at higher voltages.  Note that some older OMAP chips have a
 * Reverse Body-Bias mode meant to save power at low voltage, but that mode is
 * unsupported and phased out on newer chips.
 */
static int __init omap_abb_probe(struct platform_device *pdev)
{
	const struct of_device_id *match = NULL;
	struct omap_abb *abb = NULL;
	struct resource *mem = NULL;
	struct regulator_init_data *initdata = NULL;
	struct regulator_dev *rdev = NULL;
	struct regulator_config	config = { };
	int ret = 0;

	match = of_match_device(omap_abb_of_match, &pdev->dev);
	if (!match) {
		ret = -ENODEV;
		goto err;
	}

	abb = devm_kzalloc(&pdev->dev,
			   sizeof(struct omap_abb),
			   GFP_KERNEL);
	if (!abb) {
		ret = -ENOMEM;
		goto err;
	}

	abb->data = *((struct omap_abb_data *)match->data);
	abb->dev = &pdev->dev;

	/* map ABB resources */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		ret = -ENODEV;
		goto err;
	}

	abb->control = devm_request_and_ioremap(&pdev->dev, mem);
	if (!abb->control) {
		ret = -ENOMEM;
		goto err;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!mem) {
		ret = -ENODEV;
		goto err;
	}

	abb->txdone = devm_ioremap_nocache(&pdev->dev, mem->start,
					   resource_size(mem));
	if (!abb->txdone) {
		ret = -ENOMEM;
		goto err;
	}

	/* read device tree properties */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "ti,tranxdone_status_mask",
				   &abb->txdone_mask);
	if (ret)
		goto err;

	/* init own OPP table for ABB */
	ret = of_init_opp_table(&pdev->dev);
	if (ret)
		goto err;

	initdata = of_get_regulator_init_data(&pdev->dev, pdev->dev.of_node);
	if (!initdata)
		goto err;

	/* create ABB regulator */
	abb->rdesc.supply_name = "avs";
	abb->rdesc.name = dev_name(&pdev->dev);
	abb->rdesc.type = REGULATOR_VOLTAGE;
	abb->rdesc.ops = &omap_abb_reg_ops;
	abb->rdesc.owner = THIS_MODULE;

	initdata->constraints.valid_ops_mask |= REGULATOR_CHANGE_DRMS;
	config.init_data = initdata;
	config.dev = &pdev->dev;
	config.driver_data = abb;
	config.of_node = pdev->dev.of_node;

	rdev = regulator_register(&abb->rdesc, &config);
	if (IS_ERR(rdev)) {
		dev_err(&pdev->dev, "failed to register regulator %s\n",
			abb->rdesc.name);
		ret = PTR_ERR(rdev);
		goto err;
	}

	/* init ABB time cycles */
	ret = omap_abb_init_timings(abb);
	if (ret)
		goto err;

	/* get supply regulator */
	abb->supply_reg = regulator_get(&pdev->dev, "avs");
	if (IS_ERR(abb->supply_reg)) {
		dev_err(&pdev->dev, "\n\n[XXXXXXXXXX] %s supply get failed\n\n", __func__);
		abb->supply_reg = NULL;
	} else
		dev_err(&pdev->dev, "\n\n[:) :) :)] %s supply get SUCCESS \n\n", __func__);

	/* create debugfs entry */
	debugfs_create_file(dev_name(&pdev->dev), S_IRUGO, NULL,
			    abb, &omap_abb_debug_fops);

	return 0;

err:
	dev_err(&pdev->dev, "%s: error on init (%d)\n",
		__func__, ret);

	return ret;
}

static struct platform_driver omap_abb_driver = {
	.driver		= {
		.name	= "omap_abb",
		.of_match_table = of_match_ptr(omap_abb_of_match),
	},
};

static int __init omap_abb_driver_init(void)
{
	return platform_driver_probe(&omap_abb_driver, omap_abb_probe);
}
late_initcall(omap_abb_driver_init);
