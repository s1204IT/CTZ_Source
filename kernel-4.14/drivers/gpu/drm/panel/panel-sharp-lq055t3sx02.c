/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/backlight.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */

/* option function to read data from some panel address */
/* #define PANEL_SUPPORT_READBACK */

struct sharp {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vio18_gpio;
	struct gpio_desc *bias_gpio;

	bool prepared;
	bool enabled;

	int error;
};

#define sharp_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64, "DCS sequence too big for stack");\
	sharp_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define sharp_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	sharp_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

static inline struct sharp *panel_to_sharp(struct drm_panel *panel)
{
	return container_of(panel, struct sharp, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int sharp_dcs_read(struct sharp *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret, cmd);
		ctx->error = ret;
	}

	return ret;
}

static void sharp_panel_get_data(struct sharp *ctx)
{
	u8 buffer[3] = {0};
	static int ret;

	if (1)/*ret == 0) */{
		ret = sharp_dcs_read(ctx,  0x0A, buffer, 1);
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			 ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void sharp_dcs_write(struct sharp *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	pr_info("%s+\n", __func__);

	if (ctx->error < 0)
		return;

	ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	if (ret < 0) {
		dev_err(ctx->dev, "error %zd writing dcs seq: %ph\n",
			ret, data);
	}
	pr_info("%s-\n", __func__);
}

static void sharp_panel_init(struct sharp *ctx)
{
	sharp_dcs_write_seq_static(ctx, 0xff, 0x00);
	sharp_dcs_write_seq_static(ctx, 0xFB, 0x01);
	msleep(23);
	sharp_dcs_write_seq_static(ctx, 0xFF, 0x00);
	sharp_dcs_write_seq_static(ctx, 0xD3, 0x08);
	sharp_dcs_write_seq_static(ctx, 0xD4, 0x0E);
	sharp_dcs_write_seq_static(ctx, 0x11);
		msleep(120);
	sharp_dcs_write_seq_static(ctx, 0x29);
	msleep(20);
	sharp_dcs_write_seq_static(ctx, 0x51, 0xFF);
	sharp_dcs_write_seq_static(ctx, 0x53, 0x2C);
	sharp_dcs_write_seq_static(ctx, 0x55, 0x03);
}

static int sharp_disable(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int sharp_unprepare(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);

	if (!ctx->prepared)
		return 0;

	sharp_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	sharp_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	msleep(120);

	gpiod_set_value(ctx->bias_gpio, 0);

	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2000, 4000);

	gpiod_set_value(ctx->vio18_gpio, 0);

	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int sharp_prepare(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);
	int ret;

	pr_info("%s+\n", __func__);

	if (ctx->prepared)
		return 0;

	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(20);

	gpiod_set_value(ctx->reset_gpio, 0);
	msleep(20);

	gpiod_set_value(ctx->vio18_gpio, 1);
	msleep(20);

	gpiod_set_value(ctx->reset_gpio, 1);
	msleep(100);
	gpiod_set_value(ctx->reset_gpio, 0);
	usleep_range(2000, 4000);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(2000, 4000);
	gpiod_set_value(ctx->bias_gpio, 1);
	msleep(20);

	sharp_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		sharp_unprepare(panel);

	ctx->prepared = true;

#ifdef PANEL_SUPPORT_READBACK
	sharp_panel_get_data(ctx);
#endif
	pr_info("%s-\n", __func__);

	return ret;
}

static int sharp_enable(struct drm_panel *panel)
{
	struct sharp *ctx = panel_to_sharp(panel);

	pr_info("%s+\n", __func__);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	pr_info("%s-\n", __func__);

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 135930,
	.hdisplay = 1080,
	.hsync_start = 1080 + 40,
	.hsync_end = 1080 + 40 + 10,
	.htotal = 1080 + 40 + 10 + 20,
	.vdisplay = 1920,
	.vsync_start = 1920 + 40,
	.vsync_end = 1920 + 40 + 2,
	.vtotal = 1920 + 40 + 2 + 8,
	.vrefresh = 60,
};

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *           become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *          display the first valid frame after starting to receive
	 *          video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *           turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *             to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int sharp_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 75;
	panel->connector->display_info.height_mm = 133;

	return 1;
}

static const struct drm_panel_funcs sharp_drm_funcs = {
	.disable = sharp_disable,
	.unprepare = sharp_unprepare,
	.prepare = sharp_prepare,
	.enable = sharp_enable,
	.get_modes = sharp_get_modes,
};

static int sharp_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct sharp *ctx;
	struct device_node *backlight;
	int ret;

	pr_info("%s+\n", __func__);
	ctx = devm_kzalloc(dev, sizeof(struct sharp), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio)) {
		ret = PTR_ERR(ctx->reset_gpio);
		dev_err(dev, "cannot get reset-gpios %d\n", ret);
		return ret;
	}

	ctx->vio18_gpio = devm_gpiod_get(dev, "vio18", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->vio18_gpio)) {
		ret = PTR_ERR(ctx->vio18_gpio);
		dev_err(dev, "cannot get vio18_gpio %d\n", ret);
		return ret;
	}

	ctx->bias_gpio = devm_gpiod_get(dev, "bias", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->bias_gpio)) {
		ret = PTR_ERR(ctx->bias_gpio);
		dev_err(dev, "cannot get bias_gpio %d\n", ret);
		return ret;
	}

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->prepared = false;

	drm_panel_init(&ctx->panel);
	ctx->panel.dev = dev;
	ctx->panel.funcs = &sharp_drm_funcs;

	ret = drm_panel_add(&ctx->panel);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);
	pr_info("%s-\n", __func__);

	return ret;
}

static int sharp_remove(struct mipi_dsi_device *dsi)
{
	struct sharp *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

	return 0;
}

static const struct of_device_id sharp_of_match[] = {
	{ .compatible = "sharp,lq055t3sx02", },
	{ }
};

MODULE_DEVICE_TABLE(of, sharp_of_match);

static struct mipi_dsi_driver sharp_driver = {
	.probe = sharp_probe,
	.remove = sharp_remove,
	.driver = {
		.name = "panel-sharp-lq055t3sx02",
		.owner = THIS_MODULE,
		.of_match_table = sharp_of_match,
	},
};

module_mipi_dsi_driver(sharp_driver);

MODULE_AUTHOR("Jitao Shi <jitao.shi@mediatek.com>");
MODULE_DESCRIPTION("sharp lq055t3sx02 Panel Driver");
MODULE_LICENSE("GPL v2");

