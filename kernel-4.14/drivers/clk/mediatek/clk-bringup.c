// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>

#define CLK_MAX_NUM 160

static const struct of_device_id bring_up_id_table[] = {
	{ .compatible = "mediatek,clk-bring-up",},
	{ .compatible = "mediatek,mt8163-bring-up",},
	{ .compatible = "mediatek,mt8173-bring-up",},
	{ },
};
MODULE_DEVICE_TABLE(of, bring_up_id_table);

static int bring_up_probe(struct platform_device *pdev)
{
	struct clk *clk_list[CLK_MAX_NUM];
	int clk_con, i;
	long ret;

	clk_con = of_count_phandle_with_args(pdev->dev.of_node, "clocks",
			"#clock-cells");

	for (i = 0; i < clk_con; i++) {
		clk_list[i] = of_clk_get(pdev->dev.of_node, i);
		if (IS_ERR(clk_list[i])) {
			ret = PTR_ERR(clk_list[i]);

			if (ret == -EPROBE_DEFER)
				pr_err("clk %d is not ready\n", i);
			else
				pr_err("get clk %d fail, ret=%d, clk_con=%d\n",
				       i, (int)ret, clk_con);
			i--;
			goto failed;
		}
	}

	for (i = 0; i < clk_con; i++)
		clk_prepare_enable(clk_list[i]);

	return 0;

failed:
	while (i > 0) {
		clk_put(clk_list[i]);
		i--;
	}
	return ret;
}

static int bring_up_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver bring_up = {
	.probe		= bring_up_probe,
	.remove		= bring_up_remove,
	.driver		= {
		.name	= "bring_up",
		.owner	= THIS_MODULE,
		.of_match_table = bring_up_id_table,
	},
};

module_platform_driver(bring_up);
