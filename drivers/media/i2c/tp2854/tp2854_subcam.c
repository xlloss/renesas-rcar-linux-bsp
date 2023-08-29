/*
 * TECHPOINT HD-TVI TP2854 Driver
 *
 * Copyright (C) 2021 Regulus
 * Slash.Huang <slash.linux.c@gmail.com>
 * Slash.Huang <slash.huang@regulus.com.tw>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <linux/of_graph.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/of_device.h>
#include <linux/delay.h>

#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "tp2854.h"

#define TP2854_SINK_NAME "tvi_deser"

struct tp2854sub_priv {
	struct v4l2_subdev sd;
	struct mutex lock;
	struct v4l2_ctrl_handler hdl;
	struct media_pad pad;
	struct v4l2_rect rect;
	struct v4l2_rect crop_rect;
	int fps_denominator;
	int init_complete;
	int video_mode;
	int lanes;
	unsigned int slave_addr;
	struct list_head subcam_list;
	u8 power_on;
	u32 group;
};

static struct list_head subcam_devs = LIST_HEAD_INIT(subcam_devs);

enum {
	CAMERA_TVI = 0,
	CAMERA_AHD,
	CAMERA_NTSC,
	CAMERA_PAL,
	CAMERA_TYPE_TOTAL
};

int camera_typeset[CAMERA_TYPE_TOTAL] =
	{CAMERA_TVI, CAMERA_AHD, CAMERA_NTSC, CAMERA_PAL};

static inline struct tp2854sub_priv *to_tp2854(const struct i2c_client *client)
{
    return container_of(i2c_get_clientdata(client), struct tp2854sub_priv, sd);
}

static int tp2854sub_read_reg(struct i2c_client *client,
	unsigned char reg, unsigned char *val)
{
	int ret, tmpaddr;
	struct tp2854sub_priv *priv = i2c_get_clientdata(client);

	tmpaddr = client->addr;
	client->addr = priv->slave_addr;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
			"read fail: chip 0x%x register 0x%x: %d",
			client->addr, reg, ret);

		client->addr = tmpaddr;
		return ret;
	}

	*val = ret;
	client->addr = tmpaddr;

	return 0;
}

static int tp2854sub_write_reg(struct i2c_client *client,
	unsigned char reg, unsigned char val)
{
	int ret, tmpaddr;
	struct tp2854sub_priv *priv = i2c_get_clientdata(client);

	tmpaddr = client->addr;
	client->addr = priv->slave_addr;

		ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s:write reg error:reg=%2x,val=%2x", __func__,
			reg, val);

		client->addr = tmpaddr;
		return -1;
	}

	client->addr = tmpaddr;

	return 0;
}

static int tp2854_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int tp2854sub_get_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_pad_config *cfg,
			struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tp2854sub_priv *priv = i2c_get_clientdata(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = MEDIA_BUS_FMT_UYVY8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;

	if (priv->video_mode == NPXL_480I || priv->video_mode == NPXL_576I) {
		/* interlaced mode */
		mf->field = V4L2_FIELD_INTERLACED;
	} else {
		/* Progressive mode */
		mf->field = V4L2_FIELD_NONE;
	}

    return 0;
}

static int tp2854sub_set_fmt(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = MEDIA_BUS_FMT_UYVY8_2X8;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int tp2854sub_get_selection(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tp2854sub_priv *priv = to_tp2854(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = priv->crop_rect.width;
		sel->r.height = priv->crop_rect.height;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = priv->crop_rect.width;
		sel->r.height = priv->crop_rect.height;
		return 0;

	case V4L2_SEL_TGT_CROP:
		sel->r = priv->crop_rect;
		return 0;

	default:
		return -EINVAL;
	}
}

static int tp2854sub_set_selection(struct v4l2_subdev *sd,
    struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tp2854sub_priv *priv = to_tp2854(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
		sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	if ((rect->left + rect->width > priv->rect.width) ||
		(rect->top + rect->height > priv->rect.height))
		*rect = priv->rect;

	priv->crop_rect.left = rect->left;
	priv->crop_rect.top = rect->top;
	priv->crop_rect.width = rect->width;
	priv->crop_rect.height = rect->height;

	return 0;
}

static int tp2854sub_enum_mbus_code(struct v4l2_subdev *sd,
	struct v4l2_subdev_pad_config *cfg, struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;
	return 0;
}

#ifdef CONFIG_VIDEO_TVI_DEBUG
static int tp2854sub_g_register(struct v4l2_subdev *sd,
	struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 val = 0;

	ret = tp2854sub_read_reg(client, (u8)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u8);

	return 0;
}

static int tp2854sub_s_register(struct v4l2_subdev *sd,
                      const struct v4l2_dbg_register *reg)
{
	/* struct i2c_client *client = v4l2_get_subdevdata(sd); */

	return 0;
}
#endif

static struct v4l2_subdev_core_ops tp2854sub_core_ops = {
#ifdef CONFIG_VIDEO_TVP_DEBUG
    .g_register = tp2854sub_g_register,
    .s_register = tp2854sub_s_register,
#endif
};

static const struct v4l2_subdev_pad_ops tp2854sub_subdev_pad_ops = {
	.enum_mbus_code = tp2854sub_enum_mbus_code,
	.set_selection = tp2854sub_set_selection,
	.get_selection = tp2854sub_get_selection,
	.set_fmt = tp2854sub_set_fmt,
	.get_fmt = tp2854sub_get_fmt,
};

static const struct v4l2_subdev_video_ops tp2854sub_video_ops = {
	.s_stream = tp2854_s_stream,
};

static struct v4l2_subdev_ops tp2854_subdev_ops = {
	.video = &tp2854sub_video_ops,
	.pad = &tp2854sub_subdev_pad_ops,
	.core = &tp2854sub_core_ops,
};

static int tp2854sub_parse_dt(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	struct device_node *endpoint = NULL, *rendpoint = NULL;
	struct device_node *p_endpoint = NULL;
	struct device_node *main_parent = NULL;
	struct tp2854sub_priv *priv = i2c_get_clientdata(client);
	const __be32 *addr_be;
	u32 addr;
	int len;

	if (!np) {
		dev_err(&client->dev, "device node is null");
		return -EINVAL;
	}

	endpoint = of_graph_get_next_endpoint(np, endpoint);
	if (!endpoint) {
		dev_err(&client->dev, "cat't find endpoint");
		goto no_endpoint;
	}

	rendpoint = of_parse_phandle(endpoint, "remote-endpoint", 0);
	if (!rendpoint) {
		dev_err(&client->dev, "cat't find remote-endpoint");
		goto no_endpoint;
	}

	p_endpoint = rendpoint;
	while (1) {
		main_parent = of_get_next_parent(p_endpoint);
		if (!main_parent) {
			dev_err(&client->dev, "can' find sink %s", TP2854_SINK_NAME);
			return -EINVAL;
		}

		if (!strcmp(TP2854_SINK_NAME, main_parent->name))
			break;

		if (!main_parent)
			main_parent = p_endpoint;

		p_endpoint = main_parent;
	}

	if (of_property_read_u32(main_parent, "npxl", &priv->video_mode)) {
		dev_info(&client->dev, "DTS npxl read fail, use default");
		priv->video_mode = NPXL_720P_30;
	}

	if (of_property_read_u32(main_parent, "lanes", &priv->lanes)) {
		dev_info(&client->dev, "DTS lanes read fail, use default");
		priv->lanes = 4;
	}

	dev_info(&client->dev, "video_mode %d", priv->video_mode);
	dev_info(&client->dev, "lanes %d", priv->lanes);

	addr_be = of_get_property(main_parent, "reg", &len);
	if (!addr_be || (len < sizeof(*addr_be))) {
		dev_err(&client->dev, "of_i2c: invalid reg on %pOF",
			main_parent);
		return -EINVAL;
	}

	addr = be32_to_cpup(addr_be);
	priv->slave_addr = addr;
	dev_info(&client->dev, "slave_addr : 0x%x", priv->slave_addr);

	return 0;

no_endpoint:
	dev_err(&client->dev, "can't find remote node");
	return -EINVAL;
}

static void tp2854sub_video_mode(struct i2c_client *client,
    struct tp2854sub_priv *priv)
{

	switch (priv->video_mode) {
		case NPXL_720P_60:
			dev_info(&client->dev, "USE NPXL_720P_60");
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1280;
			priv->rect.height = 720;
			priv->fps_denominator = 60;
			break;

		case NPXL_720P_50:
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1280;
			priv->rect.height = 720;
			priv->fps_denominator = 50;
			break;

		case NPXL_720P_30:
			dev_info(&client->dev, "USE NPXL_720P_30");
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1280;
			priv->rect.height = 720;
			priv->fps_denominator = 30;
			break;

		case NPXL_720P_25:
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1280;
			priv->rect.height = 720;
			priv->fps_denominator = 25;
			break;

		case NPXL_1080P_30:
			dev_info(&client->dev, "USE NPXL_1080P_30");
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1920;
			priv->rect.height = 1080;
			priv->fps_denominator = 30;
			break;

		case NPXL_1080P_25:
			dev_info(&client->dev, "USE NPXL_1080P_25");
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1920;
			priv->rect.height = 1080;
			priv->fps_denominator = 25;
			break;

		case NPXL_480I:
			dev_info(&client->dev, "USE NPXL_480I");
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1920;
			priv->rect.height = 240;
			priv->fps_denominator = 50;
			break;

		case NPXL_576I:
			dev_info(&client->dev, "USE NPXL_576I");
			priv->rect.left = 0;
			priv->rect.top = 0;
			priv->rect.width = 1920;
			priv->rect.height = 288;
			priv->fps_denominator = 50;
			break;

		default:
		break;
	};

	priv->crop_rect = priv->rect;
}

static int tp2854sub_set_ahdfmt(struct i2c_client *client)
{
	int ret;

	dev_info(&client->dev, "%s %d", __func__, __LINE__);
	ret = tp2854sub_write_reg(client, TP2854_PAGE, MIPI_PAGE_DISABLE);
	ret = tp2854sub_write_reg(client, TP2854_DECODE_CTRL, 0x46);
	ret = tp2854sub_write_reg(client, TP2854_COMBFILTER_SDFMT_CTL, 0x70);
	ret = tp2854sub_write_reg(client, TP2854_PCLAMP_CTRL, 0x40);
	ret = tp2854sub_write_reg(client, TP2854_ClAMP_GAIN_CTRL, 0x46);
	ret = tp2854sub_write_reg(client, TP2854_T_LPX, 0xfe);
	ret = tp2854sub_write_reg(client, TP2854_T_PREP, 0x01);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_PLL, 0x3a);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_BURST_GAIN, 0x50);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_GAIN_REF, 0x40);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_CARR_DDS_BFSTD_4, 0x9d);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_CARR_DDS_BFSTD_3, 0xca);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_CARR_DDS_BFSTD_2, 0x01);
	ret = tp2854sub_write_reg(client, TP2854_COLOR_CARR_DDS, 0xd0);
	ret = tp2854sub_write_reg(client, TP2854_PAGE, MIPI_PAGE_ENABLE);
	ret = tp2854sub_write_reg(client, TP2854_MIPI_OUTPUT_EN, DISABLE_ALL_LANES);
	ret = tp2854sub_write_reg(client, TP2854_MIPI_STOPCLK, MIPI_CLK_STOP);

	return ret;
}

static int tp2854sub_probe(struct i2c_client *client)
{
	struct tp2854sub_priv *priv;
	int ret, *camera_type;

	dev_info(&client->dev, "%s", __func__);

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "kzalloc fault");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, priv);
	tp2854sub_parse_dt(client);

	/* initial camera format */
	camera_type = (int *)of_device_get_match_data(&client->dev);
	switch (*camera_type) {
	case CAMERA_TVI:
		dev_info(&client->dev, "CAMERA_TVI");
		break;

	case CAMERA_AHD:
		dev_info(&client->dev, "CAMERA_AHD");
		if (client->addr == TP2854_SUBCAM_0_FAKE_SLAV)
			tp2854sub_set_ahdfmt(client);
		break;

	case CAMERA_NTSC:
		dev_info(&client->dev, "CAMERA_NTSC");
		break;

	case CAMERA_PAL:
		dev_info(&client->dev, "CAMERA_PAL");
		break;

	default:
		dev_info(&client->dev, "default use CAMERA_TVI");
		break;
	}

	mutex_init(&priv->lock);
	tp2854sub_video_mode(client, priv);
	v4l2_i2c_subdev_init(&priv->sd, client, &tp2854_subdev_ops);
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0) {
		dev_err(&client->dev, "media_entity_pads_init fail");
		goto cleanup;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(&client->dev, "v4l2_async_register_subdev fault");
		goto cleanup;
	}

	list_add_tail(&priv->subcam_list, &subcam_devs);
	priv->init_complete = 1;
	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_device_unregister_subdev(&priv->sd);

	return ret;
}

static int tp2854sub_remove(struct i2c_client *client)
{
	struct tp2854sub_priv *priv = i2c_get_clientdata(client);
	struct tp2854sub_priv *cur_priv = NULL, *n;

	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	list_for_each_entry_safe(cur_priv, n, &subcam_devs, subcam_list)
		list_del(&cur_priv->subcam_list);

	kfree(priv);

	return 0;
}

static const struct i2c_device_id tp2854subi2c_id[] = {
	{ "tvi_cam", 0 },
	{ "ahd_cam", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tp2854subi2c_id);

static const struct of_device_id tp2854sub_of_ids[] = {
	{ .compatible = "tvi_cam", .data = (void *)&camera_typeset[0]},
	{ .compatible = "ahd_cam", .data = (void *)&camera_typeset[1]},
	{ }
};
MODULE_DEVICE_TABLE(of, tp2854sub_of_ids);

static struct i2c_driver tp2854_sub_driver = {
	.driver = {
		.name       = "tp2854_subcam",
		.of_match_table = tp2854sub_of_ids,
	},
	.probe_new = tp2854sub_probe,
	.remove = tp2854sub_remove,
	.id_table = tp2854subi2c_id,
};

module_i2c_driver(tp2854_sub_driver);

MODULE_AUTHOR("slash.linux.c@gmail.com");
MODULE_DESCRIPTION("TP2854 SUB_CAM Driver");
MODULE_LICENSE("GPL");
