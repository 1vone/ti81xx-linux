/*
 * ASoC driver for TI 81XX DVR platform
 *
 * Based on davinci-evm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <plat/asp.h>
#include <asm/hardware/edma.h>

#include "../codecs/tlv320aic3x.h"

#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"

static int ti81xx_tvp5158_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned fmt = 0;
	int ret = 0;
	
	fmt = (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM |
		SND_SOC_DAIFMT_NB_IF); 
	
	/* DVR RDK used TX_SYNC mode(via CLKX) */
	/* TVP5158 side:
	 * DSP: valid data -> WS(falling), BCLK->(rising), 1clk delay 
	 * I2S: valid data -> WS(falling), BCLK->(rising), 1clk delay 
	 * McASP side:
	 * WS(falling), BCLK->falling, 1clk delay valid. 
	 */
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't set cpu dai format(%d)\n", ret);
		return ret;
	}
	
	return 0;
}

static struct snd_soc_ops ti81xx_tvp5158_ops = {
	.hw_params = ti81xx_tvp5158_hw_params,
};
static int ti81xx_aic3x_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned sysclk, fmt = 0;
	int ret = 0;
	
	/* default */
	#if defined(CONFIG_MACH_UD8168_DVR)
	sysclk = 24576000;
	#else
	sysclk = 20000000;
	#endif
	
	fmt = (SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBM_CFM |
	    SND_SOC_DAIFMT_NB_NF); 
	
	/* AIC3x is valid NB_NF, if audio I2S 
	 * DSP: valid data -> WS(rising), BCLK->(falling), no delay 
	 * I2S: valid data -> WS(falling), BCLK->(rising), 1clk delay 
	 */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't set codec dai format(%d)\n", ret);
		return ret;
	}
	
	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't set cpu dai format(%d)\n", ret);
		return ret;
	}
	
	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk(KERN_ERR "Couldn't set codec dai system clock(%d)\n", ret);
		return ret;
	}
	
	return 0;
}

static struct snd_soc_ops ti81xx_aic3x_ops = {
	.hw_params = ti81xx_aic3x_hw_params,
};

/* davinci-evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},
};

static int ti81xx_dvr_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;

	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(codec, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	/* Set up davinci-evm specific audio path audio_map */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* not connected */
	snd_soc_dapm_disable_pin(codec, "MONO_LOUT");
	snd_soc_dapm_disable_pin(codec, "HPLCOM");
	snd_soc_dapm_disable_pin(codec, "HPRCOM");

	/* always connected */
	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Line Out");
	snd_soc_dapm_enable_pin(codec, "Mic Jack");
	snd_soc_dapm_enable_pin(codec, "Line In");

	snd_soc_dapm_sync(codec);

	return 0;
}

static struct snd_soc_dai_link ti81xx_mcasp_dai[] = {
	{
		.name = "TVP5158AUDIO",
		.stream_name = "TVP-PCM",
		.cpu_dai_name= "davinci-mcasp.0",
		.codec_dai_name = "tvp5158-hifi",
		.platform_name ="davinci-pcm-audio",
		.codec_name = "tvp5158-audio",
		.ops = &ti81xx_tvp5158_ops,
	},
	{
		.name = "TLV320AIC3X",
		.stream_name = "AIC3X",
		#if defined(CONFIG_MACH_UD8168_DVR)
		.cpu_dai_name= "davinci-mcasp.2",
		#else
		/* TI8107 DVR */
		.cpu_dai_name= "davinci-mcasp.1",
		#endif
		.codec_dai_name = "tlv320aic3x-hifi",
		.codec_name = "tlv320aic3x-codec.1-0018",
		.platform_name = "davinci-pcm-audio",
		.init = ti81xx_dvr_aic3x_init,
		.ops = &ti81xx_aic3x_ops,
	}
};

#ifdef CONFIG_SND_SOC_TI81XX_HDMI
static struct snd_soc_dai_link ti81xx_hdmi_dai = {
	.name = "HDMI_SOC_LINK",
	.stream_name = "hdmi",
	.cpu_dai_name = "hdmi-dai",
	.platform_name = "davinci-pcm-audio",
	.codec_dai_name = "HDMI-DAI-CODEC",     /* DAI name */
	.codec_name = "hdmi-dummy-codec",
};
#endif

static struct snd_soc_card ti81xx_dvr_mcasp_card = {
	.name = "TI81XX_DVR_CARD0",
	.dai_link = ti81xx_mcasp_dai,
	.num_links = ARRAY_SIZE(ti81xx_mcasp_dai),
};

#ifdef CONFIG_SND_SOC_TI81XX_HDMI
static struct snd_soc_card ti81xx_dvr_mcasp_card1 = {
	.name = "TI81XX_DVR_CARD1",
	.dai_link = &ti81xx_hdmi_dai,
	.num_links = 1,
};
#endif

static struct platform_device *ti81xx_pdev0;
static struct platform_device *ti81xx_pdev1;

static int __init ti81xx_dvr_soc_init(void)
{
	int ret;

	ti81xx_pdev0 = platform_device_alloc("soc-audio", 0);
	if (!ti81xx_pdev0)
		return -ENOMEM;
	
	platform_set_drvdata(ti81xx_pdev0, &ti81xx_dvr_mcasp_card);
	ret = platform_device_add(ti81xx_pdev0);
	if (ret) {
		printk(KERN_ERR "Can't add soc platform device\n");
		platform_device_put(ti81xx_pdev0);
		return ret;
	}
	
	ti81xx_pdev1 = platform_device_alloc("soc-audio", 1);
	if (!ti81xx_pdev1) {
		platform_device_put(ti81xx_pdev0);
		return -ENOMEM;
	}
	
	platform_set_drvdata(ti81xx_pdev1, &ti81xx_dvr_mcasp_card1);
	ret = platform_device_add(ti81xx_pdev1);
	if (ret) {
		printk(KERN_ERR "Can't add soc platform device\n");
		platform_device_put(ti81xx_pdev0);
		platform_device_put(ti81xx_pdev1);
		return ret;
	}
	
	
	return ret;
}

static void __exit ti81xx_dvr_soc_exit(void)
{
	platform_device_unregister(ti81xx_pdev0);
	platform_device_unregister(ti81xx_pdev1);
}

module_init(ti81xx_dvr_soc_init);
module_exit(ti81xx_dvr_soc_exit);

MODULE_AUTHOR("UDWORKs");
MODULE_DESCRIPTION("TI81XX DVR ASoC driver");
MODULE_LICENSE("GPL");
