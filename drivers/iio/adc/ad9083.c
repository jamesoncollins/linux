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
#include "ad9083/uc_settings.h"

#define IN_OUT_BUFF_SZ 3
#define MAX_REG_ADDR		0x1000

#define CHIPID_AD9083		0x00EA
#define CHIPID_MASK			0xFFFF

struct ad9083_jesd204_priv {
	struct ad9083_phy *phy;
};

struct ad9083_phy {
	adi_ad9083_device_t	adi_ad9083;
	struct axiadc_chip_info	chip_info;
	struct jesd204_dev	*jdev;
	struct jesd204_link	jesd204_link;
	u32 dcm;
	u64 sampling_frequency_hz;
	u32 uc;
};

static int ad9083_udelay(void *user_data, unsigned int us)
{
	usleep_range(us, (us * 110) / 100);
	return 0;
}

static int32_t ad9083_log_write(void *user_data, int32_t log_type, const char *message,
			 va_list argp)
{
	char logMessage[160];

	vsnprintf(logMessage, sizeof(logMessage), message, argp);

	switch (log_type) {
	case ADI_CMS_LOG_NONE:
		break;
	case ADI_CMS_LOG_MSG:
		break;
	case ADI_CMS_LOG_WARN:
		printk("%s\n", logMessage);
		break;
	case ADI_CMS_LOG_ERR:
		printk("%s\n", logMessage);
		break;
	case ADI_CMS_LOG_SPI:
		break;
	case ADI_CMS_LOG_API:
		break;
	case ADI_CMS_LOG_ALL:
		printk(logMessage);
		break;
	}

	return 0;
}

static int32_t ad9083_spi_xfer(void *user_data, uint8_t *wbuf,
			   uint8_t *rbuf, uint32_t len)
{
	struct axiadc_converter *conv = user_data;
	int ret;

	struct spi_transfer t = {
		.tx_buf = wbuf,
		.rx_buf = rbuf,
		.len = len,
	};

	ret = spi_sync_transfer(conv->spi, &t, 1);

	dev_dbg(&conv->spi->dev,"%s: reg=0x%X, val=0x%X",
		(wbuf[0] & 0x80) ? "rd" : "wr",
		(wbuf[0] & 0x7F) << 8 | wbuf[1],
		(wbuf[0] & 0x80) ? rbuf[2] : wbuf[2]);

	return ret;
}

int ad9083_register_write(adi_ad9083_device_t *h,
			  const uint16_t address, const uint8_t data)
{
	int32_t ret;
	uint8_t inData[IN_OUT_BUFF_SZ];
	uint8_t outData[IN_OUT_BUFF_SZ];

	if (address < MAX_REG_ADDR) {
		inData[0] = address >> 8;
		inData[1] = address;
		inData[2] = data;
		ret = h->hal_info.spi_xfer(h->hal_info.user_data, inData, outData, IN_OUT_BUFF_SZ);
		if (ret != 0)
			return ret;
	}

	return 0;
}

 int ad9083_register_read(adi_ad9083_device_t *h,
 			 const uint16_t address, uint8_t *data)
{
	int32_t ret;
	uint8_t inData[IN_OUT_BUFF_SZ];
	uint8_t outData[IN_OUT_BUFF_SZ];

	if (address < MAX_REG_ADDR) {
		inData[0] = (address >> 8) | 0x80;
		inData[1] = address;
		ret = h->hal_info.spi_xfer(h->hal_info.user_data, inData, outData, IN_OUT_BUFF_SZ);
		if (ret == 0)
	 		*data = outData[2];
	}
	
	return 0;
}

static int ad9083_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad9083_phy *phy = conv->phy;
	int ret;
	u8 val;

	if (readval == NULL)
		return ad9083_register_write(&phy->adi_ad9083, reg, writeval);
	
	ret = ad9083_register_read(&phy->adi_ad9083, reg, &val);
	if (ret < 0)
		return ret;
	*readval = val;

	return 0;
}

static int ad9083_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		*val = 1;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = 2;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 3;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9083_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
	{
		return 0;
	}
	default:
		return -EINVAL;
	}
}

static int ad9083_jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	struct ad9083_phy *phy = priv->phy;
	struct jesd204_link *link;
	printk("ad9083_jesd204_link_init1\n");

	switch (reason) {
	case JESD204_STATE_OP_REASON_INIT:
		break;
	default:
		return JESD204_STATE_CHANGE_DONE;
	}

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	link = &phy->jesd204_link;

	lnk->num_lanes = 4;
	lnk->link_id = 0;
	lnk->octets_per_frame = 8;
	lnk->frames_per_multiframe = 32;
	lnk->converter_resolution = 16;
	lnk->bits_per_sample = 16;
	lnk->num_converters = 16;
	lnk->sample_rate = 125000000;
	lnk->subclass = 0;
	lnk->sample_rate_div = 1;
	lnk->jesd_encoder = JESD204_ENCODER_8B10B;
	lnk->jesd_version = JESD204_VERSION_B;



	 //jesd204_copy_link_params(lnk, link);

	
	

	//if (phy->sysref_mode == AD9208_SYSREF_CONT)
		// lnk->sysref.mode = JESD204_SYSREF_CONTINUOUS;
	//else if (phy->sysref_mode == AD9208_SYSREF_ONESHOT)
//		lnk->sysref.mode = JESD204_SYSREF_ONESHOT;
	printk("ad9083_jesd204_link_init2\n");
	return JESD204_STATE_CHANGE_DONE;
}


static int ad9083_jesd204_clks_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	// int uc = 7;
	// struct uc_settings *uc_settings = get_uc_settings();
	// adi_cms_jesd_param_t *jtx_param = &uc_settings->jtx_param[uc];
	struct device *dev = jesd204_dev_to_device(jdev);
	// struct ad9083_jesd204_priv *priv = jesd204_dev_priv(jdev);
	// int ret;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		__LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	// if (ret < 0) {
	// 	dev_err(dev, "Failed to enabled JESD204 link (%d)\n", ret);
	// 	return ret;
	// }
printk("ad9083_jesd204_clks_enable2\n");
	return JESD204_STATE_CHANGE_DONE;
}

static int ad9083_jesd204_link_enable(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__,
		 __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));
	printk("ad9083_jesd204_link_enable2\n");
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
			.per_link = ad9083_jesd204_link_enable,
			.post_state_sysref = true,
		},
	},

	.max_num_links = 1,
	.num_retries = 3,
	.sizeof_priv = sizeof(struct ad9083_jesd204_priv),
};

static int ad9083_request_clks(struct axiadc_converter *conv)
{
	struct ad9083_phy *phy = conv->phy;
	int ret;
	printk("ad9083_request_clks 1\n");
	conv->clk = devm_clk_get(&conv->spi->dev, "adc_ref_clk");
	if (IS_ERR(conv->clk))
		return PTR_ERR(conv->clk);
	
	if (phy->jdev)
		return 0;


	printk("ad9083_request_clks 3\n");
	conv->sysref_clk = devm_clk_get(&conv->spi->dev, "adc_sysref");
	if (IS_ERR(conv->lane_clk))
		return PTR_ERR(conv->lane_clk);	

	printk("ad9083_request_clks 2\n");
	conv->lane_clk = devm_clk_get(&conv->spi->dev, "jesd_adc_clk");
	if (IS_ERR(conv->lane_clk))
		return PTR_ERR(conv->lane_clk);



	printk("ad9083_request_clks 4\n");
	ret = clk_prepare_enable(conv->sysref_clk);
	if (ret < 0)
		return ret;
	
	printk("ad9083_request_clks 5\n");


	// printk("ad9083_request_clks 1\n");
	// conv->clk = devm_clk_get(&conv->spi->dev, "adc_clk1");
	// if (IS_ERR(conv->clk) || PTR_ERR(conv->clk) != -ENOENT)
	// 	return PTR_ERR(conv->clk);

	// if (phy->jdev)
	// 	return 0;
	// printk("ad9083_request_clks 2\n");
	// // conv->lane_clk = devm_clk_get(&conv->spi->dev, "jesd_adc_clk");
	// // if (IS_ERR(conv->lane_clk) && PTR_ERR(conv->lane_clk) != -ENOENT) {
	// // 	pr_err("ad9083_request_clks 3: %d",PTR_ERR(conv->lane_clk));
	// // 	return PTR_ERR(conv->lane_clk);
	// // }

	// printk("ad9083_request_clks 4\n");
	// conv->sysref_clk = devm_clk_get(&conv->spi->dev, "adc_sysref1");
	// if (IS_ERR(conv->sysref_clk)) {
	// 	if (PTR_ERR(conv->sysref_clk) != -ENOENT)
	// 		return PTR_ERR(conv->sysref_clk);
	// 	conv->sysref_clk = NULL;
	// 	printk("ad9083_request_clks 5\n");
	// } else {
	// 	ret = clk_prepare_enable(conv->sysref_clk);
	// 	if (ret < 0)
	// 		return ret;
	// 	printk("ad9083_request_clks 6\n");
	// }
	// printk("ad9083_request_clks 7\n");
	return 0;
}

static int32_t ad9083_setup(struct spi_device *spi , uint8_t uc)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);
	struct ad9083_phy *phy = conv->phy;
	adi_cms_chip_id_t chip_id;
	struct uc_settings *uc_settings = get_uc_settings();
	uint64_t *clk_hz = uc_settings->clk_hz[uc];
	uint32_t vmax = uc_settings->vmax[uc];
	uint32_t fc = uc_settings->fc[uc];
	uint8_t rterm = uc_settings->rterm[uc];
	uint32_t en_hp = uc_settings->en_hp[uc];
	uint32_t backoff = uc_settings->backoff[uc];
	uint32_t finmax = uc_settings->finmax[uc];
	uint64_t *nco_freq_hz = uc_settings->nco_freq_hz[uc];
	uint8_t *decimation = uc_settings->decimation[uc];
	uint8_t nco0_datapath_mode = uc_settings->nco0_datapath_mode[uc];
	adi_cms_jesd_param_t *jtx_param = &uc_settings->jtx_param[uc];
	int32_t ret;

	printk(KERN_INFO"ad9083 ad9083_setup spi->dev.init_name=%s\n", spi->dev.driver->name);
	ret = ad9083_request_clks(conv);
	if (ret)
		return ret;

	printk(KERN_INFO"ad9083 ad9083_setup uc=%d\n", uc);

	ret = adi_ad9083_device_chip_id_get(&phy->adi_ad9083, &chip_id);
	if (ret < 0) {
		printk("ad9083 adi_ad9083_device_chip_id_get error 0\n");
		return ret;
	}
	if ((chip_id.prod_id & CHIPID_MASK) != CHIPID_AD9083) {
		printk("ad9083 adi_ad9083_device_chip_id_get error 1\n");
		return -ENOENT;
	}

	printk("ad9083 adi_ad9083_device_chip_id_get OK!!!\n");

	/* software reset, resistor is not mounted */
	ret = adi_ad9083_device_reset(&phy->adi_ad9083, AD9083_SOFT_RESET);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_device_init(&phy->adi_ad9083);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_device_clock_config_set(&phy->adi_ad9083, clk_hz[2],
			clk_hz[0]);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_rx_adc_config_set(&phy->adi_ad9083, vmax, fc,
					   rterm, en_hp, backoff, finmax);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_rx_datapath_config_set(&phy->adi_ad9083,
						nco0_datapath_mode, decimation, nco_freq_hz);
	if (ret < 0)
		return ret;

	ret = adi_ad9083_jtx_startup(&phy->adi_ad9083, jtx_param);
	if (ret < 0)
		return ret;

	printk("ad9083 ad9083_setup OK!!!\n");
	return 0;
}

static int ad9083_parse_dt(struct ad9083_phy *phy, struct device *dev)
{
	struct device_node *np = dev->of_node;
	// struct device_node *chan_np;
	u32 tmp;//, reg;
	// int ret;

	of_property_read_u32(np, "adi,uc", &tmp);
	phy->uc = tmp;
	printk(KERN_INFO"ad9083 uc=%d\n", tmp);
	/* Pin Config */

	// phy->powerdown_pin_en = of_property_read_bool(np,
	// 				"adi,powerdown-pin-enable");

	// tmp = AD9208_POWERUP;
	// of_property_read_u32(np, "adi,powerdown-mode", &tmp);
	// phy->powerdown_mode = tmp;

	// /* Clock Config */

	// of_property_read_u64(np, "adi,sampling-frequency",
	// 		     &phy->sampling_frequency_hz);

	// tmp = 1;
	// of_property_read_u32(np, "adi,input-clock-divider-ratio", &tmp);
	// phy->input_div = tmp;

	// phy->duty_cycle_stabilizer_en = of_property_read_bool(np,
	// 				"adi,duty-cycle-stabilizer-enable");

	// /* Analog Conifg */

	// phy->analog_input_mode = of_property_read_bool(np,
	// 				"adi,analog-input-dc-coupling-enable");

	// phy->ext_vref_en  = of_property_read_bool(np,
	// 				"adi,external-vref-enable");

	// tmp = AD9208_ADC_BUFF_CURR_500_UA;
	// of_property_read_u32(np, "adi,analog-input-neg-buffer-current", &tmp);
	// phy->buff_curr_n = tmp;

	// tmp = AD9208_ADC_BUFF_CURR_500_UA;
	// of_property_read_u32(np, "adi,analog-input-pos-buffer-current", &tmp);
	// phy->buff_curr_p = tmp;

	// /* SYSREF Config */

	// tmp = 0;
	// of_property_read_u32(np, "adi,sysref-lmfc-offset", &tmp);
	// phy->sysref_lmfc_offset = tmp;

	// phy->sysref_edge_sel = of_property_read_bool(np,
	// 				"adi,sysref-edge-high-low-enable");
	// phy->sysref_clk_edge_sel  = of_property_read_bool(np,
	// 				"adi,sysref-clk-edge-falling-enable");

	// tmp = 0;
	// of_property_read_u32(np, "adi,sysref-neg-window-skew", &tmp);
	// phy->sysref_neg_window_skew = tmp;

	// tmp = 0;
	// of_property_read_u32(np, "adi,sysref-pos-window-skew", &tmp);
	// phy->sysref_pos_window_skew = tmp;

	// tmp = AD9208_SYSREF_CONT;
	// of_property_read_u32(np, "adi,sysref-mode", &tmp);
	// phy->sysref_mode = tmp;

	// tmp = 0;
	// of_property_read_u32(np,  "adi,sysref-nshot-ignore-count", &tmp);
	// phy->sysref_count = tmp;

	// /* DDC Config */

	// tmp = AD9208_FULL_BANDWIDTH_MODE;
	// of_property_read_u32(np, "adi,ddc-channel-number", &tmp);
	// phy->fc_ch = tmp;

	// phy->ddc_output_format_real_en = of_property_read_bool(np,
	// 				"adi,ddc-complex-to-real-enable");
	// phy->ddc_input_format_real_en = of_property_read_bool(np,
	// 				"adi,ddc-mixer-real-enable");

	// for_each_child_of_node(np, chan_np) {
	// 	ret = of_property_read_u32(chan_np, "reg", &reg);
	// 	if (!ret && (reg < ARRAY_SIZE(phy->ddc))) {
	// 		ret = of_property_read_u32(chan_np, "adi,decimation",
	// 					   &phy->ddc[reg].decimation);
	// 		if (ret)
	// 			return ret;
	// 		ret = of_property_read_u32(chan_np,
	// 					"adi,nco-mode-select",
	// 					&phy->ddc[reg].nco_mode);
	// 		if (ret)
	// 			return ret;

	// 		of_property_read_u64(chan_np,
	// 			"adi,nco-channel-carrier-frequency-hz",
	// 			&phy->ddc[reg].carrier_freq_hz);
	// 		of_property_read_u64(chan_np,
	// 			"adi,nco-channel-phase-offset",
	// 			&phy->ddc[reg].po);
	// 		phy->ddc[reg].gain_db = of_property_read_bool(chan_np,
	// 			"adi,ddc-gain-6dB-enable");
	// 		phy->ddc_cnt++;
	// 	}
	// }

	// /* JESD Link Config */

	// JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, &phy->jesd204_link,
	// 				  &phy->jesd_param.jesd_F, 1);

	// JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, &phy->jesd204_link,
	// 				       &phy->jesd_param.jesd_K , 32);

	// JESD204_LNK_READ_HIGH_DENSITY(dev, np, &phy->jesd204_link,
	// 			      &phy->jesd_param.jesd_HD, 0);

	// JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, &phy->jesd204_link,
	// 				      &phy->jesd_param.jesd_N, 16);

	// JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, &phy->jesd204_link,
	// 				 &phy->jesd_param.jesd_NP, 16);

	// JESD204_LNK_READ_NUM_CONVERTERS(dev, np, &phy->jesd204_link,
	// 				&phy->jesd_param.jesd_M, 2);

	// JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(dev, np, &phy->jesd204_link,
	// 				      &phy->jesd_param.jesd_CS, 0);

	// JESD204_LNK_READ_NUM_LANES(dev, np, &phy->jesd204_link,
	// 			   &phy->jesd_param.jesd_L , 8);

	// JESD204_LNK_READ_SUBCLASS(dev, np, &phy->jesd204_link,
	// 			  &phy->jesd_subclass, JESD_SUBCLASS_0);

	return 0;
}
enum {
	ID_AD9083,

};

#define AIM_CHAN(_chan, _mod, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .modified = 1,						\
	  .channel = _chan,						\
	  .channel2 = _mod,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
			BIT(IIO_CHAN_INFO_CALIBPHASE),			\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	/*.ext_info = axiadc_ext_info,*/			\
	  .scan_index = _si,						\
	  .scan_type = {						\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = 0,						\
	  },								\
	}

static struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9083] = {
		.name = "AD9083",
		.max_rate = 245760000,
		.max_testmode = 0,
		.num_channels = 16,
		.channel[0] = AIM_CHAN(0, IIO_MOD_I, 0, 16, 'S'),
		.channel[1] = AIM_CHAN(0, IIO_MOD_Q, 1, 16, 'S'),
		.channel[2] = AIM_CHAN(1, IIO_MOD_I, 2, 16, 'S'),
		.channel[3] = AIM_CHAN(1, IIO_MOD_Q, 3, 16, 'S'),
		.channel[4] = AIM_CHAN(2, IIO_MOD_I, 4, 16, 'S'),
		.channel[5] = AIM_CHAN(2, IIO_MOD_Q, 5, 16, 'S'),
		.channel[6] = AIM_CHAN(3, IIO_MOD_I, 6, 16, 'S'),
		.channel[7] = AIM_CHAN(3, IIO_MOD_Q, 7, 16, 'S'),
		.channel[8] = AIM_CHAN(4, IIO_MOD_I, 8, 16, 'S'),
		.channel[9] = AIM_CHAN(4, IIO_MOD_Q, 9, 16, 'S'),
		.channel[10] = AIM_CHAN(5, IIO_MOD_I, 10, 16, 'S'),
		.channel[11] = AIM_CHAN(5, IIO_MOD_Q, 11, 16, 'S'),
		.channel[12] = AIM_CHAN(6, IIO_MOD_I, 12, 16, 'S'),
		.channel[13] = AIM_CHAN(6, IIO_MOD_Q, 13, 16, 'S'),
		.channel[14] = AIM_CHAN(7, IIO_MOD_I, 14, 16, 'S'),
		.channel[15] = AIM_CHAN(7, IIO_MOD_Q, 15, 16, 'S'),
	},
};


static const struct iio_info ad9083_iio_info = {
	.read_raw = &ad9083_read_raw,
	.write_raw = &ad9083_write_raw,
	.debugfs_reg_access = &ad9083_reg_access,
	// .attrs = &ad9083_phy_attribute_group,
};

static int ad9083_register_iiodev(struct axiadc_converter *conv)
{
	struct iio_dev *indio_dev;
	struct spi_device *spi = conv->spi;
	struct ad9081_phy *phy = conv->phy;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, conv);

	indio_dev->dev.parent = &spi->dev;

	if (spi->dev.of_node)
		indio_dev->name = spi->dev.of_node->name;
	else
		indio_dev->name = spi_get_device_id(spi)->name;

	indio_dev->info = &ad9083_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	// indio_dev->channels = phy->chip_info.channel;
	// indio_dev->num_channels = phy->chip_info.num_channels;

	ret = iio_device_register(indio_dev);
	//ad9081_post_iio_register(indio_dev);

	conv->indio_dev = indio_dev;

	return ret;
}

static int ad9083_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct ad9083_phy *phy;
	struct jesd204_dev *jdev;
	// struct iio_dev *indio_dev;
	struct ad9083_jesd204_priv *priv;
//	struct ad9083_state *st;
	
	
	int ret;

	printk("==============================================ad9083 probed===============================================");
	
	if (!spi) {
		return -ENOENT;
		printk("ad9083_0001 pointer null\n");
	}
	printk("ad9083_001\n");

	jdev = devm_jesd204_dev_register(&spi->dev, &jesd204_ad9083_init);
	printk("ad9083_01\n");
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);
	printk("ad9083_1\n");
	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;
	
	printk("ad9083_2\n");
	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (phy == NULL)
		return -ENOMEM;

	printk("ad9083_3\n");
	conv->adc_clkscale.mult = 1;
	conv->adc_clkscale.div = 1;

	spi_set_drvdata(spi, conv);
	conv->spi = spi;
	conv->phy = phy;
	conv->chip_info = &axiadc_chip_info_tbl[0];

	printk("ad9083_4\n");
	if (jdev) {
		phy->jdev = jdev;
		priv = jesd204_dev_priv(jdev);
		priv->phy = phy;
	}
	phy->adi_ad9083.hal_info.user_data = conv;
	phy->adi_ad9083.hal_info.spi_xfer = ad9083_spi_xfer;
	phy->adi_ad9083.hal_info.delay_us = ad9083_udelay;
	phy->adi_ad9083.hal_info.sdo = SPI_SDIO;
	phy->adi_ad9083.hal_info.msb = SPI_MSB_FIRST;
	phy->adi_ad9083.hal_info.addr_inc = SPI_ADDR_INC_AUTO;
	phy->adi_ad9083.hal_info.log_write = ad9083_log_write;
	
	printk("ad9083_5\n");
	ret = ad9083_parse_dt(phy, &spi->dev);
	if (ret < 0) {
		dev_err(&spi->dev, "Parsing devicetree failed (%d)\n", ret);
		return -ENODEV;
	}

	ret = ad9083_setup(spi, phy->uc);
	if (ret < 0) {
		dev_err(&spi->dev, "ad9083_setup failed(%d)\n", ret);
	 	return -ENODEV;
	}

	conv->reg_access = ad9083_reg_access;
	conv->write_raw = ad9083_write_raw;
	conv->read_raw = ad9083_read_raw;

	ret = ad9083_register_iiodev(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "ad9083_register_iiodev failed (%d)\n", ret);
	 	return -ENODEV;
	}

	printk("ad9083_6\n");
	ret = jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
    if (ret < 0) {
        printk(KERN_INFO"jesd204_fsm_start failed (%d)\n", ret);
        return ret;
    }
	printk("ad9083_7\n");

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
