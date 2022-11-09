// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>

static const struct of_device_id scp_power_early_id_table[] = {
	{ .compatible = "mediatek,scp-power-early" },
	{ },
};
MODULE_DEVICE_TABLE(of, scp_power_early_id_table);

static int scp_power_early_on(struct device *dev)
{
	int ret;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pr_err("%s: get power fail, ret=%d\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int scp_power_early_off(struct device *dev)
{
	int ret;

	ret = pm_runtime_put_sync(dev);
	if (ret) {
		pr_err("%s: put power fail, ret=%d\n", __func__, ret);
		return ret;
	}
	pm_runtime_disable(dev);

	return 0;
}

static int scp_power_early_probe(struct platform_device *pdev)
{
	if (!pdev->dev.pm_domain) {
		pr_err("%s: power domains are not ready\n", __func__);
		return -EPROBE_DEFER;
	}

	return scp_power_early_on(&pdev->dev);
}

static int scp_power_early_remove(struct platform_device *pdev)
{
	return scp_power_early_off(&pdev->dev);
}

#ifdef CONFIG_MTK_TINYSYS_SUSPEND_SUPPORT
static int scp_power_early_suspend(struct device *dev)
{
	return scp_power_early_off(dev);
}

static int scp_power_early_resume(struct device *dev)
{
	return scp_power_early_on(dev);
}
#endif

static const struct dev_pm_ops scp_power_early_pm_ops = {
#ifdef CONFIG_MTK_TINYSYS_SUSPEND_SUPPORT
	.suspend = scp_power_early_suspend,
	.resume = scp_power_early_resume,
#endif
};

static struct platform_driver scp_power_early = {
	.probe          = scp_power_early_probe,
	.remove         = scp_power_early_remove,
	.driver         = {
		.name   = "scp-power-early",
		.of_match_table = scp_power_early_id_table,
		.pm = &scp_power_early_pm_ops,
	},
};

static int __init scp_power_early_init(void)
{
	return platform_driver_register(&scp_power_early);
}

arch_initcall(scp_power_early_init);
