/*
 * ALSA SoC TAS2505 codec driver
 *
 * Author: Hieu Tran Dang <dangtranhieu2012@gmail.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED AS IS AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include "tas2505.h"

static int tas2505_spkdrv_getvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int val;

	snd_soc_component_read(component, TAS2505_SPKVOL1, &val);
	val = (val > mc->max) ? mc->max : val;
	val = mc->invert ? mc->max - val : val;
	ucontrol->value.integer.value[0] = val;

	return 0;
}

static int tas2505_spkdrv_putvol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *component =
		snd_soc_kcontrol_component(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	u8 val;

	val = (ucontrol->value.integer.value[0] & 0x7f);
	val = mc->invert ? mc->max - val : val;
	val = (val < 0) ? 0 : val;
	snd_soc_component_write(component, TAS2505_SPKVOL1, val);

	return 0;
}

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_LINEAR(spk_drv_vol_tlv, TLV_DB_GAIN_MUTE, 0);
static const DECLARE_TLV_DB_SCALE(spk_amp_vol_tlv, 0, 600, 1);

static const struct snd_kcontrol_new tas2505_snd_controls[] = {
	SOC_SINGLE_S8_TLV("DAC Playback Volume", TAS2505_DACVOL,
		-127, 48, dac_vol_tlv),
	SOC_SINGLE_RANGE_EXT_TLV("Speaker Driver Volume", TAS2505_SPKVOL1,
		0, 0, 117, 1,
		tas2505_spkdrv_getvol, tas2505_spkdrv_putvol, spk_drv_vol_tlv),
	SOC_SINGLE_TLV("Speaker Amplifer Volume", TAS2505_SPKVOL2,
		4, 5, 0, spk_amp_vol_tlv),
};

static const struct snd_soc_dapm_widget tas2505_dapm_widgets[] = {
	SND_SOC_DAPM_DAC("DAC Channel", "Playback",
		TAS2505_DACSETUP1, 7, 0),
	SND_SOC_DAPM_OUT_DRV("Speaker Driver", TAS2505_SPKAMPCTRL1,
		1, 0, NULL, 0),
	SND_SOC_DAPM_OUTPUT("Speaker"),
};

static const struct snd_soc_dapm_route tas2505_audio_map[] = {
	{ "Speaker Driver", NULL, "DAC Channel" },
	{ "Speaker", NULL, "Speaker Driver" },
};

static const struct reg_default tas2505_reg_defaults[] = {
	{ TAS2505_CLKMUX, 0x00 },
	{ TAS2505_PLLPR, 0x11 },
	{ TAS2505_PLLJ, 0x04 },
	{ TAS2505_PLLDMSB, 0x00 },
	{ TAS2505_PLLDLSB, 0x00 },
	{ TAS2505_NDAC, 0x01 },
	{ TAS2505_MDAC, 0x01 },
	{ TAS2505_DOSRMSB, 0x00 },
	{ TAS2505_DOSRLSB, 0x80 },
	{ TAS2505_IFACE1, 0x00 },
	{ TAS2505_IFACE3, 0x00 },
	{ TAS2505_DACSETUP1, 0x14 },
	{ TAS2505_DACSETUP2, 0x0c },
	{ TAS2505_DACVOL, 0x00 },
	{ TAS2505_REF_POR_LDO_BGAP_CTRL, 0x00 },
	{ TAS2505_LDO_CTRL, 0x0c },
	{ TAS2505_SPKAMPCTRL1, 0x00 },
	{ TAS2505_SPKVOL1, 0x00 },
	{ TAS2505_SPKVOL2, 0x00 },
	{ TAS2505_DACFLAG1, 0x00 },
	{ TAS2505_DACFLAG2, 0x00 },
	{ TAS2505_STICKYFLAG1, 0x00 },
	{ TAS2505_STICKYFLAG2, 0x00 },
	{ TAS2505_INTFLAG1, 0x00 },
	{ TAS2505_INTFLAG2, 0x00 },
	{ TAS2505_DACANLGAINFLAG, 0x00 },
};

static bool tas2505_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case TAS2505_PAGECTL:
		case TAS2505_RESET:
		case TAS2505_DACFLAG1:
		case TAS2505_DACFLAG2:
		case TAS2505_STICKYFLAG1:
		case TAS2505_STICKYFLAG2:
		case TAS2505_INTFLAG1:
		case TAS2505_INTFLAG2:
		case TAS2505_DACANLGAINFLAG:
			return true;
	}
	return false;
}

static bool tas2505_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case TAS2505_DACFLAG1:
		case TAS2505_DACFLAG2:
		case TAS2505_STICKYFLAG1:
		case TAS2505_STICKYFLAG2:
		case TAS2505_INTFLAG1:
		case TAS2505_INTFLAG2:
		case TAS2505_DACANLGAINFLAG:
			return false;
	}
	return true;
}

static const struct regmap_range_cfg tas2505_ranges[] = {
	{
		.range_min = 0,
		.range_max = 69 * 128,
		.selector_reg = TAS2505_PAGECTL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

static const struct regmap_config tas2505_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2505_writeable,
	.volatile_reg = tas2505_volatile,
	.reg_defaults = tas2505_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tas2505_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.ranges = tas2505_ranges,
	.num_ranges = ARRAY_SIZE(tas2505_ranges),
	.max_register = 69 * 128,
};

struct tas2505_rate_divs {
	u32 mclk_p;
	u32 rate;
	u8 pll_r;
	u8 pll_j;
	u16 pll_d;
	u8 mdac;
	u8 ndac;
	u16 dosr;
};

static const struct tas2505_rate_divs tas2505_divs[] = {
	{ 12288000, 44100, 1, 7, 35, 4, 4, 128 },
	{ 12288000, 48000, 1, 7, 0, 7, 2, 128 },
};

struct tas2505_priv {
	struct snd_soc_codec *codec;
	struct device *dev;
	struct regmap *regmap;
	u32 sysclk;
	u8 p_div;
	int rate_div_line;
};

static int tas2505_setup_pll(struct snd_soc_codec *codec,
	struct snd_pcm_hw_params *params)
{
	struct tas2505_priv *tas2505 = snd_soc_codec_get_drvdata(codec);
	int mclk_p = tas2505->sysclk / tas2505->p_div;
	int match = -1;
	u8 p_div;
	int i;

	for (i = 0; i < ARRAY_SIZE(tas2505_divs); i++) {
		if (
			tas2505_divs[i].rate == params_rate(params) &&
			tas2505_divs[i].mclk_p == mclk_p
		) {
			match = i;
			break;
		}
	}

	if (match == -1) {
		dev_err(codec->dev,
			"Sample rate (%u) and format not supported\n",
			params_rate(params));
		return -EINVAL;
	}

	tas2505->rate_div_line = match;

	p_div = (tas2505->p_div == 8) ? 0 : tas2505->p_div;
	p_div <<= TAS2505_PLLPR_P_SHIFT;

	snd_soc_update_bits(codec, TAS2505_PLLPR, TAS2505_PLLPR_P_MASK,
		p_div);
	snd_soc_update_bits(codec, TAS2505_PLLPR, TAS2505_PLLPR_R_MASK,
		tas2505_divs[match].pll_r);
	snd_soc_write(codec, TAS2505_PLLJ, tas2505_divs[match].pll_j);
	snd_soc_write(codec, TAS2505_PLLDMSB, tas2505_divs[match].pll_d >> 8);
	snd_soc_write(codec, TAS2505_PLLDLSB, tas2505_divs[match].pll_d & 0xff);
	snd_soc_update_bits(codec, TAS2505_NDAC, TAS2505_PLL_DAC_MASK,
		tas2505_divs[match].ndac);
	snd_soc_update_bits(codec, TAS2505_MDAC, TAS2505_PLL_DAC_MASK,
		tas2505_divs[match].mdac);
	snd_soc_write(codec, TAS2505_DOSRMSB, tas2505_divs[match].dosr >> 8);
	snd_soc_write(codec, TAS2505_DOSRLSB, tas2505_divs[match].dosr & 0xff);
	snd_soc_update_bits(codec, TAS2505_BCLKNDIV, TAS2505_BCLKNDIV_MASK,
		(tas2505_divs[match].dosr * tas2505_divs[match].mdac) / snd_soc_params_to_frame_size(params));

	return 0;
}

static int tas2505_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	u8 data = 0;

	switch (params_width(params)) {
		case 16:
			break;

		case 20:
			data = TAS2505_WORD_LEN_20BITS;
			break;

		case 24:
			data = TAS2505_WORD_LEN_24BITS;
			break;

		case 32:
			data = TAS2505_WORD_LEN_32BITS;
			break;

		default:
			dev_err(codec->dev, "Unsupported width %d\n",
				params_width(params));
			return -EINVAL;
	}

	data <<= TAS2505_IFACE1_DATALEN_SHIFT;

	snd_soc_update_bits(codec, TAS2505_IFACE1,
		TAS2505_IFACE1_DATALEN_MASK, data);

	return tas2505_setup_pll(codec, params);
}

static int tas2505_dac_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if (mute) {
		snd_soc_update_bits(codec, TAS2505_DACSETUP2,
			TAS2505_DACSETUP2_MUTE_MASK,
			TAS2505_DACSETUP2_MUTE_MASK);
	} else {
		snd_soc_update_bits(codec, TAS2505_DACSETUP2,
			TAS2505_DACSETUP2_MUTE_MASK, 0x0);
	}

	return 0;
}

static int tas2505_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 iface_reg1 = 0;
	u8 iface_reg3 = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
		case SND_SOC_DAIFMT_CBS_CFS:
			break;

		case SND_SOC_DAIFMT_CBM_CFM:
			iface_reg1 |= TAS2505_IFACE1_BCLKDIR_MASK;
			iface_reg1 |= TAS2505_IFACE1_WCLKDIR_MASK;
			break;

		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			break;

		case SND_SOC_DAIFMT_IB_NF:
			iface_reg3 |= TAS2505_IFACE3_BCLKINV_MASK;
			break;

		default:
			return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			break;

		case SND_SOC_DAIFMT_DSP_A:
		case SND_SOC_DAIFMT_DSP_B:
			iface_reg1 |= (TAS2505_DSP_MODE <<
				TAS2505_IFACE1_INTERFACE_SHIFT);
			break;

		case SND_SOC_DAIFMT_RIGHT_J:
			iface_reg1 |= (TAS2505_RJF_MODE <<
				TAS2505_IFACE1_INTERFACE_SHIFT);
			break;

		case SND_SOC_DAIFMT_LEFT_J:
			iface_reg1 |= (TAS2505_LJF_MODE <<
				TAS2505_IFACE1_INTERFACE_SHIFT);
			break;

		default:
			dev_err(codec->dev, "Invalid DAI interface format\n");
			return -EINVAL;
	}

	snd_soc_write(codec, TAS2505_IFACE1, iface_reg1);
	snd_soc_update_bits(codec, TAS2505_IFACE3,
		TAS2505_IFACE3_BCLKINV_MASK | TAS2505_IFACE3_BDIVCLKIN_MASK,
		iface_reg3);

	return 0;
}

static int tas2505_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id,
	unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct tas2505_priv *tas2505 = snd_soc_codec_get_drvdata(codec);
	int i, x;

	dev_dbg(tas2505->dev, "clk_id: %d, freq: %d\n", clk_id, freq);

	for (i = 0; i < ARRAY_SIZE(tas2505_divs); i++) {
		for (x = 1; x < 9; x++) {
			if ((freq / x) == tas2505_divs[i].mclk_p) {
				tas2505->p_div = x;
				goto found_p_div;
			}
		}
	}

	dev_err(tas2505->dev, "Can't produce required PLL_CLKIN frequency\n");
	return -EINVAL;

found_p_div:
	snd_soc_write(codec, TAS2505_CLKMUX,
		(clk_id << TAS2505_PLL_CLKIN_SHIFT) | TAS2505_CODEC_CLKIN_PLL);
	
	tas2505->sysclk = freq;

	return 0;
}

static void tas2505_clk_on(struct snd_soc_codec *codec)
{
	u8 mask = TAS2505_PM_MASK;
	u8 on = TAS2505_PM_MASK;

	snd_soc_update_bits(codec, TAS2505_PLLPR, mask, on);
	mdelay(15);
	snd_soc_update_bits(codec, TAS2505_NDAC, mask, on);
	snd_soc_update_bits(codec, TAS2505_MDAC, mask, on);
	snd_soc_update_bits(codec, TAS2505_BCLKNDIV, mask, on);
}

static void tas2505_clk_off(struct snd_soc_codec *codec)
{
	u8 mask = TAS2505_PM_MASK;

	snd_soc_update_bits(codec, TAS2505_BCLKNDIV, mask, 0);
	snd_soc_update_bits(codec, TAS2505_MDAC, mask, 0);
	snd_soc_update_bits(codec, TAS2505_NDAC, mask, 0);
	snd_soc_update_bits(codec, TAS2505_PLLPR, mask, 0);
}

static void tas2505_power_on(struct snd_soc_codec *codec)
{
	snd_soc_write(codec, TAS2505_RESET, 1);
	usleep_range(500, 1000);
	snd_soc_update_bits(codec, TAS2505_LDO_CTRL,
		TAS2505_LDO_PLL_HP_LVL_MASK, 0);
	snd_soc_update_bits(codec, TAS2505_REF_POR_LDO_BGAP_CTRL,
		TAS2505_REF_POR_LDO_BGAP_MASTER_REF_MASK,
		TAS2505_REF_POR_LDO_BGAP_MASTER_REF_MASK);
}

static void tas2505_power_off(struct snd_soc_codec *codec)
{
	snd_soc_update_bits(codec, TAS2505_REF_POR_LDO_BGAP_CTRL,
		TAS2505_REF_POR_LDO_BGAP_MASTER_REF_MASK, 0);
	snd_soc_update_bits(codec, TAS2505_LDO_CTRL,
		TAS2505_LDO_PLL_HP_LVL_MASK,
		TAS2505_LDO_PLL_HP_LVL_MASK);
}

static int tas2505_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	int current_lvl = snd_soc_codec_get_bias_level(codec);

	switch (level) {
		case SND_SOC_BIAS_ON:
			break;
		case SND_SOC_BIAS_PREPARE:
			if (current_lvl == SND_SOC_BIAS_STANDBY)
				tas2505_clk_on(codec);
			break;
		case SND_SOC_BIAS_STANDBY:
			if (current_lvl == SND_SOC_BIAS_OFF)
				tas2505_power_on(codec);
			else if (current_lvl == SND_SOC_BIAS_PREPARE)
				tas2505_clk_off(codec);
			else
				BUG();
			break;
		case SND_SOC_BIAS_OFF:
			if (current_lvl == SND_SOC_BIAS_STANDBY)
				tas2505_power_off(codec);
		default:
			dev_err(codec->dev, "Invalid bias level\n");
			return -EINVAL;
	}

	return 0;
}

static int tas2505_codec_probe(struct snd_soc_codec *codec)
{
	struct tas2505_priv *tas2505 = snd_soc_codec_get_drvdata(codec);
	printk("%s called\n", __func__);
	tas2505->codec = codec;
	return 0;
}

static int tas2505_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_driver_tas2505 = {
	.probe			= tas2505_codec_probe,
	.remove			= tas2505_codec_remove,
	.set_bias_level		= tas2505_set_bias_level,
	.suspend_bias_off	= true,

	.component_driver = {
		.controls		= tas2505_snd_controls,
		.num_controls		= ARRAY_SIZE(tas2505_snd_controls),
		.dapm_widgets		= tas2505_dapm_widgets,
		.num_dapm_widgets	= ARRAY_SIZE(tas2505_dapm_widgets),
		.dapm_routes		= tas2505_audio_map,
		.num_dapm_routes	= ARRAY_SIZE(tas2505_audio_map),
	},
};

static const struct snd_soc_dai_ops tas2505_dai_ops = {
	.hw_params	= tas2505_hw_params,
	.set_sysclk	= tas2505_set_dai_sysclk,
	.set_fmt	= tas2505_set_dai_fmt,
	.digital_mute	= tas2505_dac_mute,
};

static struct snd_soc_dai_driver tas2505_dai_driver[] = {
	{
		.name = "tas2505-hifi",
		.playback = {
			.stream_name	= "Playback",
			.channels_min	= 1,
			.channels_max	= 2,
			.rates		= TAS2505_RATES,
			.formats	= TAS2505_FORMATS,
		},
		.ops = &tas2505_dai_ops,
		.symmetric_rates = 1,
	},
};

static int tas2505_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct tas2505_priv *tas2505;
	struct device_node *np = i2c->dev.of_node;
	const struct regmap_config *regmap_config = &tas2505_i2c_regmap;
	int ret;

	printk("%s called\n", __func__);
	tas2505 = devm_kzalloc(&i2c->dev, sizeof(*tas2505), GFP_KERNEL);
	if (tas2505 == NULL)
		return -ENOMEM;

	tas2505->regmap = devm_regmap_init_i2c(i2c, regmap_config);
	if (IS_ERR(tas2505->regmap)) {
		ret = PTR_ERR(tas2505->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = of_get_named_gpio(np, "gpio-reset", 0);
	if ((ret > 0) && gpio_is_valid(ret)) {
		devm_gpio_request_one(&i2c->dev, ret, GPIOF_OUT_INIT_HIGH, "reset");
	}

	usleep_range(1000, 2000);
	/* for i350 */
	{
		struct regmap *rm = tas2505->regmap;

		regmap_write(rm, TAS2505_RESET, 0x01);
		usleep_range(500, 1000);
		regmap_write(rm, TAS2505_LDO_CTRL, 0x00);
		regmap_write(rm, TAS2505_CLKMUX, 0x03);
		regmap_write(rm, TAS2505_PLLPR, 0x91);
		regmap_write(rm, TAS2505_PLLJ, 0x07);
		regmap_write(rm, TAS2505_PLLDMSB, 0x06);
		regmap_write(rm, TAS2505_PLLDLSB, 0x90);
		usleep_range(15, 20);
		regmap_write(rm, TAS2505_NDAC, 0x87);
		regmap_write(rm, TAS2505_MDAC, 0x82);
		regmap_write(rm, TAS2505_DOSRMSB, 0x00);
		regmap_write(rm, TAS2505_DOSRLSB, 0x80);
		regmap_write(rm, TAS2505_IFACE1, 0x00);
		regmap_write(rm, TAS2505_IFACE2, 0x00);
		regmap_write(rm, TAS2505_DACINSTRSET, 0x02);
		regmap_write(rm, TAS2505_REF_POR_LDO_BGAP_CTRL, 0x10);
		regmap_write(rm, TAS2505_CM_CTRL, 0x00);
		regmap_write(rm, TAS2505_HP_RT_SEL, 0x00);
		regmap_write(rm, TAS2505_SPKVOL1, 0x00);
		regmap_write(rm, TAS2505_SPKVOL2, 0x10);
		regmap_write(rm, TAS2505_SPKAMPCTRL1, 0x02);
		regmap_write(rm, TAS2505_DACSETUP1, 0x90);
		regmap_write(rm, TAS2505_DACVOL, 0x00);
		regmap_write(rm, TAS2505_DACSETUP2, 0x04);
	}

	tas2505->dev = &i2c->dev;

	dev_set_drvdata(tas2505->dev, tas2505);

	printk("%s done\n", __func__);
	return snd_soc_register_codec( &i2c->dev, &soc_codec_driver_tas2505,
		tas2505_dai_driver, ARRAY_SIZE(tas2505_dai_driver));
}

static int tas2505_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static const struct of_device_id tas2505_of_match[] = {
	{ .compatible = "ti,tas2505" },
	{},
};
MODULE_DEVICE_TABLE(of, tas2505_of_match);

static const struct i2c_device_id tas2505_i2c_id[] = {
	{ "tas2505", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2ic, tas2505_i2c_id);

static struct i2c_driver tas2505_i2c_driver = {
	.driver = {
		.name		= "tas2505-codec",
		.of_match_table	= of_match_ptr(tas2505_of_match),
	},
	.probe		= tas2505_i2c_probe,
	.remove		= tas2505_i2c_remove,
	.id_table	= tas2505_i2c_id,
};

module_i2c_driver(tas2505_i2c_driver);

MODULE_DESCRIPTION("ASoC TAS2505 codec driver");
MODULE_AUTHOR("Hieu Tran Dang <dangtranhieu2012@gmail.com>");
MODULE_LICENSE("GPL");
