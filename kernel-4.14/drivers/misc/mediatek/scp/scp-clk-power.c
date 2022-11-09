// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

#ifdef CONFIG_MTK_TINYSYS_SUSPEND_SUPPORT
#include <linux/arm-smccc.h>

#define MTK_SIP_KERNEL_SCP_RESET_AARCH32        0x8200020B
#define MTK_SIP_KERNEL_SCP_RESET_AARCH64        0xC200020B

#define MTK_SCP_SHUT_DOWN     0
#define MTK_SCP_RESET         1
#endif

static const struct of_device_id scp_clk_power_id_table[] = {
	{ .compatible = "mediatek,scp-clk-power" },
	{ },
};
MODULE_DEVICE_TABLE(of, scp_clk_power_id_table);

struct clk_data {
	struct device *dev;
	struct clk **clk_list;
	int clk_list_con;
};

static int scp_clk_power_on(struct device *dev)
{
	int i, ret;
	struct clk_data *clk_dev = dev_get_drvdata(dev);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);

	if (ret < 0) {
		pr_err("scp: get power fail, ret=%d\n", ret);
		return ret;
	}

	for (i = 0; i < clk_dev->clk_list_con; i++) {
		ret = clk_prepare_enable(clk_dev->clk_list[i]);
		if (ret) {
			pr_err("scp: enable clock fail, ret=%d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int scp_clk_power_off(struct device *dev)
{
	int i, ret;
	struct clk_data *clk_dev = dev_get_drvdata(dev);

	for (i = 0; i < clk_dev->clk_list_con; i++)
		clk_disable_unprepare(clk_dev->clk_list[i]);

	ret = pm_runtime_put_sync(dev);
	if (ret) {
		pr_err("scp: put power fail, ret=%d\n", ret);
		return ret;
	}
	pm_runtime_disable(dev);

	return 0;
}

static int scp_clk_power_probe(struct platform_device *pdev)
{
	int i;
	struct device *dev = &pdev->dev;
	struct clk_data *clk_dev;

	if (!pdev->dev.pm_domain) {
		pr_err("scp: power domains are not ready\n");
		return -EPROBE_DEFER;
	}

	clk_dev = devm_kzalloc(dev, sizeof(*clk_dev), GFP_KERNEL);
	if (!clk_dev)
		return -ENOMEM;

	clk_dev->dev = dev;
	clk_dev->clk_list_con =
		of_count_phandle_with_args(pdev->dev.of_node,
						"clocks", "#clock-cells");

	clk_dev->clk_list =
		devm_kcalloc(dev, clk_dev->clk_list_con,
				sizeof(*clk_dev->clk_list), GFP_KERNEL);
	if (!clk_dev->clk_list)
		return PTR_ERR(clk_dev->clk_list);

	for (i = 0; i < clk_dev->clk_list_con; i++) {
		clk_dev->clk_list[i] = of_clk_get(pdev->dev.of_node, i);
		if (IS_ERR(clk_dev->clk_list[i])) {
			long ret = PTR_ERR(clk_dev->clk_list[i]);

			if (ret == -EPROBE_DEFER)
				pr_err("scp: clk %d is not ready\n", i);
			else
				pr_err("scp: get clk %d fail, ret=%d\n",
					i, (int)ret);

			devm_kfree(clk_dev->dev, clk_dev->clk_list);

			return (int) ret;
		}
	}
	platform_set_drvdata(pdev, clk_dev);

	return scp_clk_power_on(clk_dev->dev);
}

static int scp_clk_power_remove(struct platform_device *pdev)
{
	int i, ret;
	struct clk_data *clk_dev = dev_get_drvdata(&pdev->dev);

	ret = scp_clk_power_off(clk_dev->dev);
	if (ret)
		return ret;

	for (i = 0; i < clk_dev->clk_list_con; i++)
		devm_clk_put(clk_dev->dev, clk_dev->clk_list[i]);
	devm_kfree(clk_dev->dev, clk_dev->clk_list);

	return 0;
}

#ifdef CONFIG_MTK_TINYSYS_SUSPEND_SUPPORT
static unsigned long invoke_scp_smc(unsigned long id, unsigned long arg)
{
	struct arm_smccc_res res;

	arm_smccc_smc(id, arg, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static unsigned long mtk_scp_shut_down(void)
{
	return invoke_scp_smc(MTK_SIP_KERNEL_SCP_RESET_AARCH64,
			      MTK_SCP_SHUT_DOWN);
}

static unsigned long mtk_scp_reset(void)
{
	return invoke_scp_smc(MTK_SIP_KERNEL_SCP_RESET_AARCH64,
			      MTK_SCP_RESET);
}

static int scp_clk_power_suspend(struct device *dev)
{
	unsigned long err;

	err = mtk_scp_shut_down();
	if (err) {
		pr_err("scp: cm4 shut down fail, err = %ld\n", err);
		return -1;
	}

	return scp_clk_power_off(dev);
}

static int scp_clk_power_resume(struct device *dev)
{
	int ret;
	unsigned long err;

	ret = scp_clk_power_on(dev);
	if (ret)
		return ret;

	err = mtk_scp_reset();
	if (err) {
		pr_err("scp: cm4 reset fail, err = %ld\n", err);
		return -1;
	}

	return 0;
}
#endif

static const struct dev_pm_ops scp_clk_power_pm_ops = {
#ifdef CONFIG_MTK_TINYSYS_SUSPEND_SUPPORT
	.suspend = scp_clk_power_suspend,
	.resume = scp_clk_power_resume,
#endif
};

static struct platform_driver scp_clk_power = {
	.probe		= scp_clk_power_probe,
	.remove		= scp_clk_power_remove,
	.driver		= {
		.name	= "scp-clk-power",
		.of_match_table = scp_clk_power_id_table,
		.pm = &scp_clk_power_pm_ops,
	},
};

module_platform_driver(scp_clk_power);
