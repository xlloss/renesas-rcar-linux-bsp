// SPDX-License-Identifier: GPL-2.0+
/*
 * TP2854 Deserializer Driver
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/of_device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "tp2854.h"

/*
 * The sink and source pads are created to match the OF graph port numbers so
 * that their indexes can be used interchangeably.
 */
#define TP2854_CH_NUM 4
#define TP2854_N_SINKS 4
#define TP2854_N_PADS 5
#define TP2854_SRC_PAD 4

#define for_each_source(priv, source) \
	for ((source) = NULL; ((source) = next_source((priv), (source))); )

#define to_index(priv, source) ((source) - &(priv)->sources[0])

enum {
	TP2854A = 0,
	TP2854B,
	TP2854_TYPE_END,
};

int tp2854_typeset[TP2854_TYPE_END] = {TP2854A, TP2854B};

struct tp2854_source {
	struct v4l2_subdev *sd;
	struct fwnode_handle *fwnode;
};

struct tp2854_asd {
	struct v4l2_async_subdev base;
	struct tp2854_source *source;
};

static inline struct tp2854_asd *to_tp2854_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct tp2854_asd, base);
}

struct tp2854_priv {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pads[TP2854_N_PADS];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixelrate;

	struct v4l2_mbus_framefmt fmt[TP2854_N_SINKS];

	/* Protects controls and fmt structures */
	struct mutex mutex;

	struct i2c_mux_core *mux;
	bool mux_open;
	unsigned int mux_channel;
	unsigned int nsources;
	unsigned int source_mask;
	unsigned int route_mask;
	unsigned int bound_sources;
	unsigned int csi2_data_lanes;
	struct tp2854_source sources[TP2854_CH_NUM];
	struct v4l2_async_notifier notifier;

	int nxpl;
	int tvi_clk;
	int hsync;
	int vsync;
	int pixelclk;
};

static DEFINE_MUTEX(io_mutex);

static struct tp2854_source *next_source(struct tp2854_priv *priv,
					  struct tp2854_source *source)
{
	if (!source)
		source = &priv->sources[0];
	else
		source++;

	for (; source < &priv->sources[TP2854_CH_NUM]; source++) {
		if (source->fwnode)
			return source;
	}

	return NULL;
}

static inline struct tp2854_priv *sd_to_tp2854(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tp2854_priv, sd);
}

/* -----------------------------------------------------------------------------
 * I2C IO
 */

int tp2854_read(struct tp2854_priv *priv, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(priv->client, reg);
	if (ret < 0)
		dev_err(&priv->client->dev,
			"%s: register 0x%02x read failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

int tp2854_write(struct tp2854_priv *priv, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(priv->client, reg, val);
	if (ret < 0)
		dev_err(&priv->client->dev,
			"%s: register 0x%02x write failed (%d)\n",
			__func__, reg, ret);

	return ret;
}

static int tp2854_read_id(struct tp2854_priv *priv)
{
	int id, *tp2854_type;

	tp2854_write(priv ,TP2854_PAGE, MIPI_PAGE_DISABLE);

	/* read TP2854_ID_1 */
	id = tp2854_read(priv, TP2854_ID1_REG);
	if (id != TP2854_ID_1) {
		dev_err(&priv->client->dev, "TP2854_ID1 fail %x", id);
		return -ENODEV;
	}
	dev_info(&priv->client->dev, "TP2854_ID1 %x\n", id);

	/* read TP2854_ID_2 */
	id = tp2854_read(priv, TP2854_ID2_REG);
	if (id != TP2854_ID_2) {
		dev_err(&priv->client->dev,  "TP2854_ID2 fail %x", id);
		return -ENODEV;
	}
	dev_info(&priv->client->dev, "TP2854_ID2 %x\n", id);

	tp2854_type = (int *)of_device_get_match_data(&priv->client->dev);

	if (*tp2854_type == 0)
		dev_info(&priv->client->dev, "TP2854A");
	else
		dev_info(&priv->client->dev, "TP2854B");

	return 0;
}

static int tp2854_get_input_status(struct tp2854_priv *priv, unsigned int vin_ch)
{
	#define VIN(a) (a)
	#define VIDEO_VLOCK 6
	#define VIDEO_HLOCK 5
	unsigned int hlock, vlock, tmp;
	int ret = 0;

	tp2854_write(priv, TP2854_PAGE, MIPI_PAGE_DISABLE_AR | VIN(vin_ch));

	tmp = tp2854_read(priv, TP2854_INPUT_STATUS);
	vlock = tmp & (1 << VIDEO_VLOCK);
	hlock = tmp & (1 << VIDEO_HLOCK);

	if (!vlock || !hlock) {
		ret = -EIO;
		ret = 0;
		dev_err(&priv->client->dev, "vertical or horizontal no lock");
	}

	return ret;
}

static void tp2854sub_mipiclk_enable(struct tp2854_priv *priv, u8 enable)
{
	u8 setval;

	setval = enable ? MIPI_CLK_NORMAL : MIPI_CLK_STOP;
	tp2854_write(priv, TP2854_MIPI_STOPCLK, setval);
}

static void tp2854sub_mipiout_enable(struct tp2854_priv *priv,
    u8 enable, u8 mipilan)
{
	u8 setval;

	setval = enable ? (mipilan) : DISABLE_ALL_LANES;
	tp2854_write(priv, TP2854_MIPI_OUTPUT_EN, setval);
}

static int tp2854_enable_power(struct tp2854_priv *priv, int on)
{
	#define TP2854_MIPICLK_ENABLE 1
	#define TP2854_MIPICLK_DISABLE 0
	#define TP2854_MIPI_ENABLE 1
	#define TP2854_MIPI_DISABLE 0
	unsigned int enable_n_lan;
	unsigned int use_n_lanes = priv->csi2_data_lanes;

	if (priv->csi2_data_lanes != 4 && priv->csi2_data_lanes != 2) {
			dev_err(&priv->client->dev, "not support lanes=%d",
				priv->csi2_data_lanes);
			return -EINVAL;
	}

	mutex_lock(&priv->mutex);

	tp2854_write(priv, TP2854_PAGE, MIPI_PAGE_ENABLE);
	if (on) {
		enable_n_lan = use_n_lanes == 4 ? ENABLE_ALL_LANES : ENABLE_LANE12;
		tp2854sub_mipiout_enable(priv, TP2854_MIPI_ENABLE, enable_n_lan);
		udelay(10);
		tp2854sub_mipiclk_enable(priv, TP2854_MIPICLK_ENABLE);
	} else {
		tp2854sub_mipiout_enable(priv, TP2854_MIPI_DISABLE, ENABLE_ALL_LANES);
		tp2854sub_mipiclk_enable(priv, TP2854_MIPICLK_DISABLE);
	}

	mutex_unlock(&priv->mutex);
	return 0;
}

/*
 * tp2854_check_video_links() - Make sure video links are detected and locked
 *
 * Performs safety checks on video link status. Make sure they are detected
 * and all enabled links are locked.
 *
 * Returns 0 for success, -EIO for errors.
 */
static int tp2854_check_video_links(struct tp2854_priv *priv,
	unsigned char video_ch)
{
	return tp2854_get_input_status(priv, video_ch);
}

static int tp2854_notify_bound(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct tp2854_priv *priv = sd_to_tp2854(notifier->sd);
	struct tp2854_source *source = to_tp2854_asd(asd)->source;
	unsigned int index = to_index(priv, source);
	unsigned int src_pad;
	int ret;

	ret = media_entity_get_fwnode_pad(&subdev->entity,
				source->fwnode,
				MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(&priv->client->dev,
			"Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	priv->bound_sources |= BIT(index);
	source->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&source->sd->entity, src_pad,
				&priv->sd.entity, index,
				MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(&priv->client->dev,
			"Unable to link %s:%u -> %s:%u\n",
			source->sd->name, src_pad, priv->sd.name, index);
		return ret;
	}

	dev_info(&priv->client->dev, "Bound %s pad: %u on index %u\n",
		subdev->name, src_pad, index);

	/*
	 * We can only register v4l2_async_notifiers, which do not provide a
	 * means to register a complete callback. bound_sources allows us to
	 * identify when all remote serializers have completed their probe.
	 */
	if (priv->bound_sources != priv->source_mask)
		return 0;

	/*
	 * tp2854 default video channel order,
	 * reverse rcar vin channel number
	 *
	 * -----------------
	 * |tp2854 rcar-vin |
	 * ------------------
	 * |   0      3     |
	 * |   1      2     |
	 * |   2      1     |
	 * |   3      0     |
	 * ------------------
	 */

	/* change to tp2854 video default order */
	ret = tp2854_check_video_links(priv, (TP2854_CH_NUM - 1) - index);
	if (ret)
		dev_err(&priv->client->dev, "TP2854 video(%d) link fail", index);

	return 0;
}

void tp2854_notify_unbind(struct v4l2_async_notifier *notifier,
			struct v4l2_subdev *subdev,
			struct v4l2_async_subdev *asd)
{
	struct tp2854_priv *priv = sd_to_tp2854(notifier->sd);
	struct tp2854_source *source = to_tp2854_asd(asd)->source;
	unsigned int index = to_index(priv, source);

	source->sd = NULL;
	priv->bound_sources &= ~BIT(index);
}

const struct v4l2_async_notifier_operations tp2854_notify_ops = {
	.bound = tp2854_notify_bound,
	.unbind = tp2854_notify_unbind,
};

static int tp2854_v4l2_notifier_register(struct tp2854_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct tp2854_source *source = NULL;
	int ret;

	if (!priv->nsources) {
		dev_warn(dev, "nsources is null");
		return 0;
	}

	v4l2_async_notifier_init(&priv->notifier);

	for_each_source(priv, source) {
		unsigned int i = to_index(priv, source);
		struct v4l2_async_subdev *asd;

		dev_info(dev, "%s %d", __func__, __LINE__);
		asd = v4l2_async_notifier_add_fwnode_subdev(&priv->notifier,
							    source->fwnode,
							    sizeof(struct tp2854_asd));
		if (IS_ERR(asd)) {
			dev_err(dev, "Failed to add subdev for source %u: %ld",
				i, PTR_ERR(asd));
			v4l2_async_notifier_cleanup(&priv->notifier);
			return PTR_ERR(asd);
		}

		to_tp2854_asd(asd)->source = source;
	}

	priv->notifier.ops = &tp2854_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void tp2854_v4l2_notifier_unregister(struct tp2854_priv *priv)
{
	if (!priv->nsources)
		return;

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

static int tp2854_s_power(struct v4l2_subdev *sd, int on)
{
	struct tp2854_priv *priv = sd_to_tp2854(sd);

	//dev_info(&priv->client->dev, "%s %d", __func__, __LINE__);
	tp2854_enable_power(priv, !on);

	return 0;

}

static struct v4l2_subdev_core_ops tp2854_core_ops = {
	.s_power = tp2854_s_power,
};

static int tp2854_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tp2854_priv *priv = sd_to_tp2854(sd);
	struct tp2854_source *source;
	int ret;
	//dev_info(&priv->client->dev, "%s %d", __func__, __LINE__);
	tp2854_enable_power(priv, enable);

	if (enable) {
		/* Start all cameras. */
		for_each_source(priv, source) {
			ret = v4l2_subdev_call(source->sd, video, s_stream, 1);
			if (ret)
				return ret;
		}
	} else {
		/* Stop all cameras. */
		for_each_source(priv, source)
			v4l2_subdev_call(source->sd, video, s_stream, 0);
	}

	return 0;
}

static int tp2854_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{

	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;
	return 0;
}

static struct v4l2_mbus_framefmt *
tp2854_get_pad_format(struct tp2854_priv *priv,
		struct v4l2_subdev_pad_config *cfg, unsigned int pad, u32 which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&priv->sd, cfg, pad);

	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &priv->fmt[pad];

	default:
		return NULL;
	}
}

static int tp2854_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format)
{
	struct tp2854_priv *priv = sd_to_tp2854(sd);
	struct v4l2_mbus_framefmt *cfg_fmt;

	if (format->pad == TP2854_SRC_PAD)
		return -EINVAL;

	/* Refuse non YUV422 formats as we hardcode DT to 8 bit YUV422 */
	switch (format->format.code) {
	case MEDIA_BUS_FMT_UYVY8_2X8:
		break;

	default:
		format->format.code = MEDIA_BUS_FMT_UYVY8_2X8;
		break;
	}

	cfg_fmt = tp2854_get_pad_format(priv, cfg, format->pad, format->which);
	if (!cfg_fmt)
		return -EINVAL;

	mutex_lock(&priv->mutex);
	*cfg_fmt = format->format;
	mutex_unlock(&priv->mutex);

	return 0;
}

static int tp2854_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format)
{
	struct tp2854_priv *priv = sd_to_tp2854(sd);
	struct v4l2_mbus_framefmt *cfg_fmt;
	unsigned int pad = format->pad;

	/*
	 * Multiplexed Stream Support: Support link validation by returning the
	 * format of the first bound link. All links must have the same format,
	 * as we do not support mixing and matching of cameras connected to the
	 * tp2854.
	 */
	if (pad == TP2854_SRC_PAD)
		pad = __ffs(priv->bound_sources);

	cfg_fmt = tp2854_get_pad_format(priv, cfg, pad, format->which);
	if (!cfg_fmt)
		return -EINVAL;

	mutex_lock(&priv->mutex);
	format->format = *cfg_fmt;
	mutex_unlock(&priv->mutex);

	return 0;
}

static int tp2854_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_config *config)
{
	config->flags = V4L2_MBUS_CSI2_4_LANE |
		V4L2_MBUS_CSI2_CHANNEL_0 | V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	config->type = V4L2_MBUS_CSI2_DPHY;

	return 0;
}

static const struct v4l2_subdev_video_ops tp2854_video_ops = {
	.s_stream = tp2854_s_stream,
};

static const struct v4l2_subdev_pad_ops tp2854_pad_ops = {
	.enum_mbus_code = tp2854_enum_mbus_code,
	.get_fmt = tp2854_get_fmt,
	.set_fmt = tp2854_set_fmt,
	.get_mbus_config = tp2854_get_mbus_config,
};

static const struct v4l2_subdev_ops tp2854_subdev_ops = {
	.video = &tp2854_video_ops,
	.pad = &tp2854_pad_ops,
	.core = &tp2854_core_ops,
};

static void tp2854_init_format(struct v4l2_mbus_framefmt *fmt)
{
	fmt->width		= 1280;
	fmt->height		= 720;
	fmt->code		= MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace		= V4L2_COLORSPACE_SMPTE170M;
	fmt->field		= V4L2_FIELD_NONE;
	fmt->ycbcr_enc		= V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization	= V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func		= V4L2_XFER_FUNC_DEFAULT;
}

static int tp2854_open(struct v4l2_subdev *subdev, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	for (i = 0; i < TP2854_N_SINKS; i++) {
		format = v4l2_subdev_get_try_format(subdev, fh->pad, i);
		tp2854_init_format(format);
	}

	return 0;
}

static const struct v4l2_subdev_internal_ops tp2854_subdev_internal_ops = {
	.open = tp2854_open,
};

static int tp2854_s_ctrl(struct v4l2_ctrl *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_PIXEL_RATE:
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops tp2854_ctrl_ops = {
	.s_ctrl = tp2854_s_ctrl,
};

static int tp2854_v4l2_register(struct tp2854_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct fwnode_handle *ep;
	int ret, i;

	/* Register v4l2 async notifiers for connected Camera subdevices */
	ret = tp2854_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "Unable to register V4L2 async notifiers\n");
		return ret;
	}

	/* Configure V4L2 for the TP2854 itself */
	for (i = 0; i < TP2854_N_SINKS; i++)
		tp2854_init_format(&priv->fmt[i]);

	v4l2_ctrl_handler_init(&priv->ctrls, 1);

	/* for rcar mipi-csi2 driver compute clk */
	priv->pixelrate = v4l2_ctrl_new_std(&priv->ctrls,
						&tp2854_ctrl_ops,
						V4L2_CID_PIXEL_RATE,
						1, INT_MAX, 1, priv->pixelclk * TP2854_CH_NUM);

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &tp2854_subdev_ops);
	priv->sd.internal_ops = &tp2854_subdev_internal_ops;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	priv->sd.ctrl_handler = &priv->ctrls;
	ret = priv->ctrls.error;
	if (ret)
		goto err_async;

	v4l2_set_subdevdata(&priv->sd, priv);

	priv->pads[TP2854_SRC_PAD].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 0; i < TP2854_SRC_PAD; i++)
		priv->pads[i].flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&priv->sd.entity, TP2854_N_PADS,
				     priv->pads);
	if (ret)
		goto err_async;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), TP2854_SRC_PAD,
					     0, 0);
	if (!ep) {
		dev_err(dev, "Unable to retrieve endpoint on \"port@4\"\n");
		ret = -ENOENT;
		goto err_async;
	}
	priv->sd.fwnode = ep;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret < 0) {
		dev_err(dev, "Unable to register subdevice\n");
		goto err_put_node;
	}

	return 0;

err_put_node:
	fwnode_handle_put(ep);
err_async:
	tp2854_v4l2_notifier_unregister(priv);

	return ret;
}

static void tp2854_v4l2_unregister(struct tp2854_priv *priv)
{
	fwnode_handle_put(priv->sd.fwnode);
	v4l2_async_unregister_subdev(&priv->sd);
	tp2854_v4l2_notifier_unregister(priv);
}

int tp2854_setup_nxpl_stage2_720p(struct tp2854_priv *priv)
{
	struct i2c_client *client = priv->client;
	int ret, *tp2854_type;

	tp2854_type = (int *)of_device_get_match_data(&client->dev);

	switch (*tp2854_type) {
	case TP2854A:
		/* 8bitYUV Y first, BT656 720p HD mode */
		ret = tp2854_write(priv, TP2854_DECODE_CTRL, 0xc2);
		break;

	case TP2854B:
		//ret = tp2854_write(priv, TP2854_PCLAMP_CTRL, 0x30);
		//ret = tp2854_write(priv, TP2854_ClAMP_GAIN_CTRL, 0x84);
		//
		//ret = tp2854_write(priv, TP2854_SYNC_CLAMP_CTRL, 0x02);
		///* 8bitYUV Y first, BT656 720p HD mode */
		//ret = tp2854_write(priv, TP2854_SYNC_AMPACG_CTRL, 0x36);
		//ret = tp2854_write(priv, TP2854_SYNC_CLAMP_CTRL, 0x3c);
		//ret = tp2854_write(priv, TP2854_EQ2_CTRL, 0xc0);
		//ret = tp2854_write(priv, TP2854_EQ1_CTRL, 0xc0);
		//ret = tp2854_write(priv, TP2854_COLOR_KILL_TH, 0x60);
		//ret = tp2854_write(priv, 0x22, 0x36);
		//ret = tp2854_write(priv, 0x38, 0x00);
		//ret = tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_3, 0xbb);
		//ret = tp2854_write(priv, TP2854_LPF_CTRL, 0x18);

		tp2854_write(priv, TP2854_SYNC_CLAMP_CTRL, 0x02);
		/* 8bitYUV Y first, BT656 720p HD mode */
		tp2854_write(priv, TP2854_SYNC_AMPACG_CTRL, 0x38);
		tp2854_write(priv, TP2854_SYNC_CLAMP_CTRL, 0x3c);
		tp2854_write(priv, TP2854_EQ2_CTRL, 0xc0);
		tp2854_write(priv, TP2854_EQ1_CTRL, 0xc0);
		tp2854_write(priv, TP2854_COLOR_KILL_TH, 0x60);
		tp2854_write(priv, 0x22, 0x36);
		tp2854_write(priv, 0x38, 0x00);
		break;

	default:
		dev_err(&client->dev, "%s fault", __func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}

int tp2854_lans_clk_set(struct tp2854_priv *priv)
{
	struct i2c_client *client = priv->client;
	int *tp2854_type, pll_ctrl5, pll_ctrl6, ret;;
	unsigned char read_val;
	unsigned int use_mipi_lans = priv->csi2_data_lanes;

	dev_info(&client->dev, "%s %d", __func__, __LINE__);
	if (priv->csi2_data_lanes != 4 && priv->csi2_data_lanes != 2) {
			dev_err(&client->dev, "not support lanes=%d",
				priv->csi2_data_lanes);
			return -EINVAL;
	}

	tp2854_type = (int *)of_device_get_match_data(&client->dev);

	tp2854_write(priv, TP2854_PAGE, MIPI_PAGE_ENABLE);

	switch (*tp2854_type) {
	case TP2854A:
		dev_info(&client->dev, "%s %d", __func__, __LINE__);
		pll_ctrl6 = use_mipi_lans == 4 ? 0x00 : 0x04;
		ret = tp2854_write(priv, TP2854_MIPI_PLL_CTRL6, pll_ctrl6);
		break;

	case TP2854B:
		dev_info(&client->dev, "%s %d", __func__, __LINE__);
		#define PLL_CTRL_5_LAN4 (DIV_BIT_CLK_FPLL_2 | DIV_PHY_CLK_FPLL_7)
		#define PLL_CTRL_5_LAN2 (DIV_PHY_CLK_FPLL_6)
		#define PLL_CTRL_6_LAN4 DIV_CSI_CLK_16
		#define PLL_CTRL_6_LAN2 DIV_CSI_CLK_16
		/*
		* PLL_CONTROL_5
		* 7    : RST_CLK_GEN
		* 6-4  : DIV_BIT_CLK, set to desired mipi clk x2
		* 3    : reserved
		* 2-0  : DIV_PHY_CLK, set to desired mipi clk / 4
		*/

		pll_ctrl5 = use_mipi_lans == 4 ? PLL_CTRL_5_LAN4 : PLL_CTRL_5_LAN2;

		ret = tp2854_write(priv, TP2854_MIPI_PLL_CTRL5, pll_ctrl5);

		/*
		 * PLL_CONTROL_6
		 * 7-5  : reserved
		 * 3-2  : DIV_CLK_DEC
		 * 1-0  : DIV_CSI_CLK
		 */
		pll_ctrl6 = use_mipi_lans == 4 ? PLL_CTRL_6_LAN4 : PLL_CTRL_6_LAN2;
		ret = tp2854_write(priv, TP2854_MIPI_PLL_CTRL6, pll_ctrl6);

		if (use_mipi_lans == 2)
			ret = tp2854_write(priv, 0x0a, 0x80);

		/* PLL CLK reset */
		read_val = tp2854_read(priv, TP2854_MIPI_PLL_CTRL5);
		read_val |= RST_CLK_GEN;
		tp2854_write(priv, TP2854_MIPI_PLL_CTRL5, read_val);
		msleep(1);
		read_val &= ~RST_CLK_GEN;
		tp2854_write(priv, TP2854_MIPI_PLL_CTRL5, read_val);


		/* tx parameter */
		if (use_mipi_lans == 4) {
			tp2854_write(priv, TP2854_T_LPX, TP2854B_TX_TLPX_4LAN);
			tp2854_write(priv, TP2854_T_PREP, TP2854B_TX_PREP_4LAN);
			tp2854_write(priv, TP2854_T_TRAIL, TP2854B_TX_TRAIL_4LAN);
			tp2854_write(priv, TP2854_T_EXIT, TP2854B_TX_EXIT_4LAN);
		} else {
			tp2854_write(priv, TP2854_T_LPX, TP2854B_TX_TLPX_2LAN);
			tp2854_write(priv, TP2854_T_PREP, TP2854B_TX_PREP_2LAN);
			tp2854_write(priv, TP2854_T_TRAIL, TP2854B_TX_TRAIL_2LAN);
			tp2854_write(priv, TP2854_T_EXIT, TP2854B_TX_EXIT_2LAN);
		}

		break;

	default:
		dev_info(&client->dev, "%s %d", __func__, __LINE__);
		dev_err(&client->dev, "%s fault", __func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int tp2854_setup(struct tp2854_priv *priv)
{
	struct i2c_client *client = priv->client;
	int ret;

	ret = tp2854_read_id(priv);
	if (ret)
		return -EINVAL;

	/* disable MIPI register access */
	tp2854_write(priv, TP2854_PAGE, MIPI_PAGE_DISABLE);
	switch (priv->nxpl) {
		case NPXL_720P_60:
			dev_info(&client->dev, "USE NPXL_720P_60");
			priv->hsync = 1280;
			priv->vsync = 720;
			tp2854_write(priv ,TP2854_NPXL_H, 0x06);
			tp2854_write(priv ,TP2854_NPXL_L, 0x72);
			tp2854_write(priv ,TP2854_OUT_H_DELAY_H, 0x13);
			tp2854_write(priv ,TP2854_OUT_H_DELAY_L, 0x15);
			tp2854_write(priv ,TP2854_OUT_V_DELAY_L, 0x19);
			break;

		case NPXL_720P_50:
			dev_info(&client->dev, "USE NPXL_720P_50");
			priv->hsync = 1280;
			priv->vsync = 720;
			break;

		case NPXL_720P_30:
			dev_info(&client->dev, "USE NPXL_720P_30");
			priv->hsync = 1280;
			priv->vsync = 720;
			//priv->pixelclk = 60000000;
			priv->pixelclk = 74250000;
			//priv->pixelclk = 100000000;
			//priv->pixelclk = 90000000;
			if (priv->tvi_clk == TP2854_TVP_CLK_148M) {
				tp2854_write(priv, TP2854_NPXL_H, 0x0C);
				tp2854_write(priv, TP2854_NPXL_L, 0xE4);
			} else {
				tp2854_write(priv, TP2854_NPXL_H, 0x06);
				tp2854_write(priv, TP2854_NPXL_L, 0x72);
			}

			tp2854_write(priv, TP2854_OUT_H_DELAY_H, 0x13);
			tp2854_write(priv, TP2854_OUT_H_DELAY_L, 0x15);
			tp2854_write(priv, TP2854_OUT_V_DELAY_L, 0x19);
			break;

		case NPXL_720P_25:
			dev_info(&client->dev, "USE NPXL_720P_25");
			priv->hsync = 1280;
			priv->vsync = 720;
			tp2854_write(priv, TP2854_NPXL_H, 0x07);
			tp2854_write(priv, TP2854_NPXL_L, 0xbc);
			break;

		case NPXL_1080P_30:
			dev_info(&client->dev, "USE NPXL_1080P_30");
			priv->hsync = 1920;
			priv->vsync = 1080;
			priv->pixelclk = 108000000;
			tp2854_write(priv, TP2854_NPXL_H, 0x08);
			tp2854_write(priv, TP2854_NPXL_L, 0x98);
			tp2854_write(priv, TP2854_OUT_H_DELAY_H, 0x03);
			tp2854_write(priv, TP2854_OUT_H_DELAY_L, 0xd2);
			tp2854_write(priv, TP2854_OUT_V_DELAY_L, 0x29);
			break;

		case NPXL_1080P_25:
			dev_info(&client->dev, "USE NPXL_720P_25");
			priv->hsync = 1920;
			priv->vsync = 1080;
			break;

		case NPXL_480I:
			dev_info(&client->dev, "USE NPXL_480I NTSC");
			priv->hsync = 1920;
			priv->vsync = 240;

			if (priv->tvi_clk == TP2854_TVP_CLK_148M) {
				tp2854_write(priv, TP2854_NPXL_H, 0x23);
				tp2854_write(priv, TP2854_NPXL_L, 0x60);
			} else {
				tp2854_write(priv, TP2854_NPXL_H, 0x09);
				tp2854_write(priv, TP2854_NPXL_L, 0x38);
			}
			tp2854_write(priv, TP2854_OUT_H_DELAY_H, 0x13);
			tp2854_write(priv, TP2854_OUT_H_DELAY_L, 0x60);
			tp2854_write(priv, TP2854_OUT_H_ACTIVE_L, 0x80);
			tp2854_write(priv, TP2854_OUT_V_DELAY_L, 0x12);
			break;

		case NPXL_576I:
			dev_info(&client->dev, "USE NPXL_576I PAL");
			priv->hsync = 1920;
			priv->vsync = 288;
			tp2854_write(priv,  TP2854_NPXL_H, 0x09);
			tp2854_write(priv,  TP2854_NPXL_L, 0x48);
			tp2854_write(priv,  TP2854_OUT_H_DELAY_H, 0x13);
			tp2854_write(priv,  TP2854_OUT_H_DELAY_L, 0x76);
			tp2854_write(priv,  TP2854_OUT_H_ACTIVE_L, 0x80);
			tp2854_write(priv,  TP2854_OUT_V_DELAY_L, 0x17);
			break;

		default:
		break;
	};

	if (priv->tvi_clk == TP2854_TVP_CLK_148M || priv->nxpl == NPXL_1080P_30)
		tp2854_write(priv ,TP2854_MISC_CTL, 0x05 & ~FSL_74MHZ_148MHZ_SYS_CLK);
	else
		tp2854_write(priv ,TP2854_MISC_CTL, 0x05 | FSL_74MHZ_148MHZ_SYS_CLK);

	switch (priv->nxpl) {
		case NPXL_720P_25:
		case NPXL_720P_30:
		case NPXL_720P_50:
		case NPXL_720P_60:
			/* 8bitYUV Y first, BT656 720p HD mode */
			/* tp2854_write(priv, TP2854_DECODE_CTRL, 0xc2); */
			tp2854_setup_nxpl_stage2_720p(priv);
			break;

		case NPXL_1080P_25:
		case NPXL_1080P_30:
			tp2854_write(priv, TP2854_DECODE_CTRL, 0xc0);
			break;

		case NPXL_480I:
			tp2854_write(priv, TP2854_SYNC_CLAMP_CTRL, 0x3c);
			tp2854_write(priv, TP2854_DECODE_CTRL, 0xc7);
			tp2854_write(priv, TP2854_PCLAMP_CTRL, 0x40);
			tp2854_write(priv, TP2854_ClAMP_GAIN_CTRL, 0x84);
			tp2854_write(priv, TP2854_SYNC_AMPACG_CTRL, 0x36);
			tp2854_write(priv, TP2854_COLOR_KILL_TH, 0x70);
			tp2854_write(priv, TP2854_COLOR_PLL, 0x2a);
			tp2854_write(priv, TP2854_COLOR_BURST_GAIN, 0x68);
			tp2854_write(priv, TP2854_COLOR_GAIN_REF, 0x57);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_4, 0x62);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_3, 0xbb);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_2, 0x96);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS, 0xc0);
			tp2854_write(priv, TP2854_LPF_CTRL, 0x04);
			break;

		case NPXL_576I:
			tp2854_write(priv, TP2854_DECODE_CTRL, 0xc7);
			tp2854_write(priv, TP2854_EQ1_HYSTER, 0x13);
			tp2854_write(priv, TP2854_COMBFILTER_SDFMT_CTL, 0x51);
			tp2854_write(priv, TP2854_PCLAMP_CTRL, 0x48);
			tp2854_write(priv, TP2854_ClAMP_GAIN_CTRL, 0x84);
			tp2854_write(priv, TP2854_SYNC_AMPACG_CTRL, 0x37);
			tp2854_write(priv, TP2854_SYNC_CLAMP_CTRL, 0x3f);
			tp2854_write(priv, TP2854_COLOR_KILL_TH, 0x70);
			tp2854_write(priv, TP2854_COLOR_PLL, 0x2a);
			tp2854_write(priv, TP2854_COLOR_BURST_GAIN, 0x64);
			tp2854_write(priv, TP2854_COLOR_GAIN_REF, 0x56);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_4, 0x7a);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_3, 0x4a);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS_BFSTD_3, 0x4d);
			tp2854_write(priv, TP2854_COLOR_CARR_DDS, 0xf0);
			break;

		default:
			pr_err("Don't know what format ? priv->vsync %d\r\n", priv->vsync);
			break;
	}

	tp2854_write(priv ,TP2854_OUT_H_ACTIVE_L, priv->hsync & 0xFF);
	tp2854_write(priv ,TP2854_OUT_V_ACTIVE_L, priv->vsync & 0xFF);
	tp2854_write(priv ,TP2854_OUT_V_H_ACTIVE_H,
		(((priv->vsync & 0x0F00) >> 4) | ((priv->hsync & 0x0F00) >> 8)));

	tp2854_write(priv, TP2854_CLK_DATA_OUT, 0x00);
	tp2854_write(priv, TP2854_SYS_CLK_CTRL, 0xff);
	tp2854_write(priv, TP2854_COL_H_PLL_FR_CTL, 0x30 | BLUE_SCREEN);

	if (priv->tvi_clk == TP2854_TVP_CLK_148M || priv->nxpl == NPXL_1080P_30)
		tp2854_write(priv, TP2854_EQ1_HYSTER, 0x13 & ~EQ_CLK_FSEL);
	else
		tp2854_write(priv, TP2854_EQ1_HYSTER, 0x13 | EQ_CLK_FSEL);


	/* enable MIPI register access */
	tp2854_write(priv, TP2854_PAGE, MIPI_PAGE_ENABLE);

	//tp2854_write(priv, TP2854_MIPI_PLL_CTRL4, 0x04 | OUT_DIV_EN);
	tp2854_write(priv, TP2854_MIPI_PLL_CTRL4, 0xEF | OUT_DIV_EN);

	if (priv->tvi_clk == TP2854_TVP_CLK_148M || priv->nxpl == NPXL_1080P_30)
		tp2854_write(priv, TP2854_MIPI_PLL_CTRL4, 0xEF & ~OUT_DIV_EN);

	tp2854_write(priv, TP2854_MIPI_CLK_LAEN_CTRL, 0xf8);
	tp2854_write(priv, TP2854_MIPI_CLK_EN, 0x01);

	/* set clk foro lan 4 or lan 2 */
	tp2854_lans_clk_set(priv);

	tp2854_write(priv, TP2854_MIPI_NUM_LAN,
		NUM_CHANNELS(4) | NUM_LANES(priv->csi2_data_lanes));

	tp2854_write(priv, TP2854_MIPI_VERTUAL_CHID,
		VCID_CH4VC4_CH3VC3_CH2VC2_CH1VC1);

	/* disable CSi2 */
	tp2854_write(priv, TP2854_MIPI_STOPCLK, MIPI_CLK_STOP);
	tp2854_write(priv, TP2854_MIPI_OUTPUT_EN, DISABLE_ALL_LANES);

	/* disable MIPI register access */
	/* tp2854_write(priv, TP2854_PAGE, MIPI_PAGE_DISABLE); */

	return 0;
}


static void tp2854_i2c_mux_close(struct tp2854_priv *priv)
{
	/*
	 * Ensure that both the forward and reverse channel are disabled on the
	 * mux, and that the channel ID is invalidated to ensure we reconfigure
	 * on the next tp2854_i2c_mux_select() call.
	 */

	priv->mux_open = false;
	priv->mux_channel = -1;
}

static int tp2854_i2c_mux_select(struct i2c_mux_core *muxc, u32 chan)
{
	struct tp2854_priv *priv = i2c_mux_priv(muxc);

	/* Channel select is disabled when configured in the opened state. */
	if (priv->mux_open)
		return 0;

	if (priv->mux_channel == chan)
		return 0;

	priv->mux_channel = chan;

	return 0;
}

static int tp2854_i2c_mux_init(struct tp2854_priv *priv)
{
	struct tp2854_source *source;
	int ret;

	priv->mux = i2c_mux_alloc(priv->client->adapter, &priv->client->dev,
				  priv->nsources, 0, I2C_MUX_LOCKED,
				  tp2854_i2c_mux_select, NULL);
	if (!priv->mux) {
		dev_err(&priv->client->dev, "i2c_mux_alloc fault");
		return -ENOMEM;
	}

	priv->mux->priv = priv;

	for_each_source(priv, source) {
		unsigned int index = to_index(priv, source);

		dev_info(&priv->client->dev, "%s %d", __func__, __LINE__);
		ret = i2c_mux_add_adapter(priv->mux, 0, index, 0);
		if (ret < 0) {
			dev_err(&priv->client->dev, "i2c_mux_add_adapter fault");
			goto error;
		}
	}

	return 0;

error:
	i2c_mux_del_adapters(priv->mux);
	return ret;
}

static int tp2854_init(struct device *dev)
{
	struct tp2854_priv *priv;
	struct i2c_client *client;
	int ret;

	client = to_i2c_client(dev);
	priv = i2c_get_clientdata(client);

	ret = tp2854_setup(priv);
	if (ret) {
		dev_err(dev, "Unable to setup tp2854\n");
		goto err_v4l2_register;
	}

	/*
	 * Register all V4L2 interactions for the TP2854 and notifiers for
	 * any subdevices connected.
	 */
	ret = tp2854_v4l2_register(priv);
	if (ret) {
		dev_err(dev, "Failed to register with V4L2\n");
		goto err_v4l2_register;
	}

	ret = tp2854_i2c_mux_init(priv);
	if (ret) {
		dev_err(dev, "Unable to initialize I2C multiplexer\n");
		goto err_v4l2_register;
	}

	/* Leave the mux channels disabled until they are selected. */
	tp2854_i2c_mux_close(priv);

	return 0;

err_v4l2_register:
	tp2854_v4l2_unregister(priv);

	return ret;
}

static void tp2854_cleanup_dt(struct tp2854_priv *priv)
{
	struct tp2854_source *source;

	for_each_source(priv, source) {
		fwnode_handle_put(source->fwnode);
		source->fwnode = NULL;
	}
}

static int tp2854_parse_dt(struct tp2854_priv *priv)
{
	struct device *dev = &priv->client->dev;
	struct device_node *node = NULL;
	struct device_node *np = priv->client->dev.of_node;

	if (of_property_read_u32(np, "npxl", &priv->nxpl))
		priv->nxpl = 2; /* 720P  30 frame, npxl = 2 */

	if (of_property_read_u32(np, "tvi-clk", &priv->tvi_clk))
		priv->tvi_clk = TP2854_TVP_CLK_74M;

	/* Parse the endpoints */
	for_each_endpoint_of_node(dev->of_node, node) {
		struct tp2854_source *source;
		struct of_endpoint ep;

		dev_info(dev, "%s %d", __func__, __LINE__);
		of_graph_parse_endpoint(node, &ep);
		dev_info(dev, "Endpoint %pOF on port %d", ep.local_node, ep.port);

		if (ep.port > TP2854_CH_NUM) {
			dev_err(dev, "Invalid endpoint %s on port %d",
				of_node_full_name(ep.local_node), ep.port);
			continue;
		}
		dev_info(dev, "%s %d", __func__, __LINE__);
		/* For the source endpoint just parse the bus configuration. */
		if (ep.port == TP2854_SRC_PAD) {
			struct v4l2_fwnode_endpoint vep = {
				.bus_type = V4L2_MBUS_CSI2_DPHY
			};
			int ret;

			dev_info(dev, "%s %d", __func__, __LINE__);
			ret = v4l2_fwnode_endpoint_parse(
					of_fwnode_handle(node), &vep);
			if (ret) {
				of_node_put(node);
				return ret;
			}

			priv->csi2_data_lanes =
				vep.bus.mipi_csi2.num_data_lanes;

			continue;
		}

		dev_info(dev, "%s %d", __func__, __LINE__);
		if (priv->sources[ep.port].fwnode) {
			dev_err(dev,
				"Multiple port endpoints are not supported: %d",
				ep.port);

			continue;
		}

		dev_info(dev, "%s %d", __func__, __LINE__);
		source = &priv->sources[ep.port];
		source->fwnode = fwnode_graph_get_remote_endpoint(
						of_fwnode_handle(node));
		if (!source->fwnode) {
			dev_err(dev,
				"Endpoint %pOF has no remote endpoint connection\n",
				ep.local_node);

			continue;
		}

		dev_info(dev, "%s %d", __func__, __LINE__);
		priv->source_mask |= BIT(ep.port);
		priv->nsources++;
	}
	of_node_put(node);

	priv->route_mask = priv->source_mask;

	return 0;
}

static int tp2854_probe(struct i2c_client *client)
{
	struct tp2854_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->mutex);

	priv->client = client;
	i2c_set_clientdata(client, priv);

	ret = tp2854_parse_dt(priv);
	if (ret) {
		dev_err(&client->dev, "tp2854_parse_dt fail");
		goto err_cleanup_dt;
	}

	ret = tp2854_init(&client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "tp2854_init fail");
		goto err_cleanup_dt;
	}

	return 0;

err_cleanup_dt:
	tp2854_cleanup_dt(priv);
	return ret;
}

static int tp2854_remove(struct i2c_client *client)
{
	struct tp2854_priv *priv = i2c_get_clientdata(client);

	tp2854_v4l2_unregister(priv);

	tp2854_cleanup_dt(priv);

	return 0;
}

static const struct of_device_id tp2854_dt_ids[] = {
	{ .compatible = "tvi,tp2854a", .data = (void *)&tp2854_typeset[0]},
	{ .compatible = "tvi,tp2854b", .data = (void *)&tp2854_typeset[1]},
	{},
};
MODULE_DEVICE_TABLE(of, tp2854_dt_ids);

static struct i2c_driver tp2854_i2c_driver = {
	.driver	= {
		.name		= "tp2854",
		.of_match_table	= of_match_ptr(tp2854_dt_ids),
	},
	.probe_new	= tp2854_probe,
	.remove		= tp2854_remove,
};

module_i2c_driver(tp2854_i2c_driver);

MODULE_DESCRIPTION("TP2854 Deserializer Driver");
MODULE_AUTHOR("slash.linux.c@gmail.com");
MODULE_LICENSE("GPL");
