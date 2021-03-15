// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for AD9083 and similar mixed signal front end (MxFE®)
 *
 * Copyright 2021 Analog Devices Inc.
 */
//#define DEBUG
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <asm/unaligned.h>


#include <linux/kernel.h>

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/debugfs.h>


#include "cf_axi_adc.h"
#include <linux/jesd204/jesd204.h>

#include "ad9083/adi_ad9083.h"
//	#include "ad9083/uc_settings.h"

struct ad9083_jesd204_priv {
	struct ad9083_phy *phy;
};

struct ad9083_phy {
	adi_ad9083_device_t	adi_ad9083;
	struct axiadc_chip_info chip_info;
	struct jesd204_dev *jdev;
	struct jesd204_link jesd204_link;
	u32 dcm;
	u64 sampling_frequency_hz;
};

static int ad9083_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	struct jesd204_link *link;

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	link = &phy->jesd204_link;

	jesd204_copy_link_params(lnk, link);

	lnk->sample_rate = phy->sampling_frequency_hz;
	lnk->sample_rate_div = phy->dcm;
	lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	//if (phy->sysref_mode == AD9208_SYSREF_CONT)
		lnk->sysref.mode = JESD204_SYSREF_CONTINUOUS;
	//else if (phy->sysref_mode == AD9208_SYSREF_ONESHOT)
//		lnk->sysref.mode = JESD204_SYSREF_ONESHOT;

	return JESD204_STATE_CHANGE_DONE;
}


static int ad9083_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	//struct uc_settings *uc_settings = get_uc_settings();
	// adi_cms_jesd_param_t *jtx_param = &uc_settings->jtx_param[uc];
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	ret = adi_ad9083_jtx_startup(&phy->adi_ad9083, NULL);
	if (ret < 0) {
		dev_err(dev, "Failed to enabled JESD204 link (%d)\n", ret);
		return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_ad9083_init = {
	.state_ops = {
		[JESD204_OP_LINK_INIT] = {
			.per_link = ad9083_jesd204_link_init,
		},
		[JESD204_OP_CLOCKS_ENABLE] = {
			.per_link = ad9083_jesd204_clks_enable,
		},
		[JESD204_OP_LINK_ENABLE] = {
			.per_link = NULL,//ad9083_jesd204_link_enable,
			.post_state_sysref = true,
		},
	},

	.max_num_links = 1,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9083_jesd204_priv),
};

static int ad9083_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
//	struct ad9083_state *st;
	struct jesd204_dev *jdev;
	int ret;

	printk("==============================================ad9083 probed===============================================");

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9083_init);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	return 0;
}


static const struct spi_device_id ad9083_id[] = {
	{ "ad9083", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9083_id);

static const struct of_device_id ad9083_of_match[] = {
	{ .compatible = "adi,ad9083" },
	{},
};
MODULE_DEVICE_TABLE(of, ad9083_of_match);

static struct spi_driver ad9083_driver = {
	.driver = {
			.name = "ad9083",
			.of_match_table = of_match_ptr(ad9083_of_match),
		},
	.probe = ad9083_probe,
	.id_table = ad9083_id,
};
module_spi_driver(ad9083_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9083 ADC");
MODULE_LICENSE("GPL v2");
