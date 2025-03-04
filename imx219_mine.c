
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

// Register Definitions
#define IMX219_REG_CHIP_ID		0x0000
#define IMX219_REG_MODE_SELECT		0x0100
#define IMX219_REG_CSI_LANE_MODE	0x0114
#define IMX219_REG_DPHY_CTRL		0x0128
#define IMX219_REG_EXCK_FREQ		0x012a
#define IMX219_REG_ANALOG_GAIN		0x0157
#define IMX219_REG_DIGITAL_GAIN		0x0158
#define IMX219_REG_EXPOSURE		0x015a
#define IMX219_REG_VTS			0x0160
#define IMX219_REG_X_ADD_STA_A		0x0164
#define IMX219_REG_Y_ADD_STA_A		0x0168
#define IMX219_REG_X_OUTPUT_SIZE	0x016c
#define IMX219_REG_Y_OUTPUT_SIZE	0x016e
#define IMX219_REG_ORIENTATION		0x0172
#define IMX219_REG_BINNING_MODE_H	0x0174
#define IMX219_REG_BINNING_MODE_V	0x0175
#define IMX219_REG_CSI_DATA_FORMAT_A	0x018c

// Mode Configuration
struct imx219_mode {
	u32 width;
	u32 height;
	u16 vts_def;
};

static const struct imx219_mode supported_modes[] = {
	{ .width = 3280, .height = 2464, .vts_def = 3526 }, // 8MP
	{ .width = 1920, .height = 1080, .vts_def = 1763 },  // 1080p
	{ .width = 1640, .height = 1232, .vts_def = 1763 },  // 2x2 Binning
	{ .width = 640, .height = 480, .vts_def = 1763 },    // VGA
};

// IMX219 Structure
struct imx219 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[3];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	u8 lanes;
};

static inline struct imx219 *to_imx219(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx219, sd);
}

// Register Access
static int imx219_read_reg(struct imx219 *imx219, u16 reg, u16 *val)
{
	return regmap_read(imx219->regmap, reg, val);
}

static int imx219_write_reg(struct imx219 *imx219, u16 reg, u16 val)
{
	return regmap_write(imx219->regmap, reg, val);
}

// Mode Configuration
static int imx219_set_mode(struct imx219 *imx219,
			   struct v4l2_subdev_state *state)
{
	const struct imx219_mode *mode = v4l2_get_format_mode(state);
	int ret;

	ret = imx219_write_reg(imx219, IMX219_REG_X_OUTPUT_SIZE, mode->width);
	if (ret)
		return ret;

	ret = imx219_write_reg(imx219, IMX219_REG_Y_OUTPUT_SIZE, mode->height);
	if (ret)
		return ret;

	ret = imx219_write_reg(imx219, IMX219_REG_VTS, mode->vts_def);
	if (ret)
		return ret;

	// Additional mode-specific register setups...
	return 0;
}

// Power Management
static int imx219_power_on(struct device *dev)
{
	struct imx219 *imx219 = dev_get_drvdata(dev);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx219->supplies), imx219->supplies);
	if (ret)
		return ret;

	clk_prepare_enable(imx219->xclk);
	gpiod_set_value_cansleep(imx219->reset_gpio, 1);
	usleep_range(1000, 1100); // Wake-up delay for TDA4VM

	return 0;
}

static int imx219_power_off(struct device *dev)
{
	struct imx219 *imx219 = dev_get_drvdata(dev);

	gpiod_set_value_cansleep(imx219->reset_gpio, 0);
	clk_disable_unprepare(imx219->xclk);
	regulator_bulk_disable(ARRAY_SIZE(imx219->supplies), imx219->supplies);
	return 0;
}

// Streaming Control
static int imx219_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx219 *imx219 = to_imx219(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(sd);

	if (enable) {
		ret = imx219_set_mode(imx219, state);
		if (ret)
			return ret;

		ret = imx219_write_reg(imx219, IMX219_REG_MODE_SELECT,
				       IMX219_MODE_STREAMING);
	} else {
		ret = imx219_write_reg(imx219, IMX219_REG_MODE_SELECT,
				       IMX219_MODE_STANDBY);
	}

	return ret;
}

// Control Operations
static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_set_ctrl,
};

static int imx219_init_controls(struct imx219 *imx219)
{
	struct v4l2_ctrl_handler *hdl = &imx219->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 12);
	if (ret)
		return ret;

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  imx219_get_pixel_rate(imx219),
			  imx219_get_pixel_rate(imx219), 1,
			  imx219_get_pixel_rate(imx219));

	v4l2_ctrl_new_std_menu(hdl, &imx219_ctrl_ops,
			       V4L2_CID_LINK_FREQ,
			       0, 0,
			       imx219->lanes == 2 ? IMX219_LINK_FREQ : 2 * IMX219_LINK_FREQ);

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_EXPOSURE,
			  IMX219_EXPOSURE_MIN, IMX219_EXPOSURE_MAX,
			  IMX219_EXPOSURE_STEP,
			  IMX219_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX219_ANA_GAIN_MIN, IMX219_ANA_GAIN_MAX,
			  IMX219_ANA_GAIN_STEP, IMX219_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX219_DGTL_GAIN_MIN, IMX219_DGTL_GAIN_MAX,
			  IMX219_DGTL_GAIN_STEP, IMX219_DGTL_GAIN_DEFAULT);

	imx219->vflip = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx219->hflip = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx219->sd.ctrl_handler = hdl;
	return 0;
}

// Probing
static int imx219_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx219 *imx219;
	int ret;

	imx219 = devm_kzalloc(dev, sizeof(*imx219), GFP_KERNEL);
	if (!imx219)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx219->sd, client, &imx219_subdev_ops);
	imx219->sd.internal_ops = &imx219_internal_ops;

	// Regulators
	imx219->supplies[0].supply = "vdd_sensor";
	imx219->supplies[1].supply = "vdd_core";
	imx219->supplies[2].supply = "vdd_if";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx219->supplies), imx219->supplies);
	if (ret)
		return ret;

	// Clock
	imx219->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx219->xclk))
		return PTR_ERR(imx219->xclk);

	// GPIO Reset
	imx219->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx219->reset_gpio))
		return PTR_ERR(imx219->reset_gpio);

	// Device Tree Configuration
	if (imx219_check_hwcfg(dev, imx219))
		return -EINVAL;

	// Initialize Controls
	ret = imx219_init_controls(imx219);
	if (ret)
		return ret;

	// Register Subdevice
	ret = v4l2_async_register_subdev_sensor(&imx219->sd);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;
}

// Cleanup
static void imx219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx219 *imx219 = to_imx219(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx219_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

// Device Table
static const struct of_device_id imx219_dt_ids[] = {
	{ .compatible = "sony,imx219" },
	{}
};
MODULE_DEVICE_TABLE(of, imx219_dt_ids);

static const struct dev_pm_ops imx219_pm_ops = {
	SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
};

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = "imx219",
		.of_match_table = imx219_dt_ids,
		.pm = &imx219_pm_ops,
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Sony IMX219 Sensor Driver for TDA4VM");
MODULE_LICENSE("GPL v2");


// SPDX-License-Identifier: GPL-2.0+
// Copyright (C) 2024 Your Name <your.email@example.com>
// IMX219 Camera Driver for TDA4VM

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

// Register Definitions
#define IMX219_REG_CHIP_ID		0x0000
#define IMX219_REG_MODE_SELECT		0x0100
#define IMX219_REG_CSI_LANE_MODE	0x0114
#define IMX219_REG_DPHY_CTRL		0x0128
#define IMX219_REG_EXCK_FREQ		0x012a
#define IMX219_REG_ANALOG_GAIN		0x0157
#define IMX219_REG_DIGITAL_GAIN		0x0158
#define IMX219_REG_EXPOSURE		0x015a
#define IMX219_REG_VTS			0x0160
#define IMX219_REG_X_ADD_STA_A		0x0164
#define IMX219_REG_Y_ADD_STA_A		0x0168
#define IMX219_REG_X_OUTPUT_SIZE	0x016c
#define IMX219_REG_Y_OUTPUT_SIZE	0x016e
#define IMX219_REG_ORIENTATION		0x0172
#define IMX219_REG_BINNING_MODE_H	0x0174
#define IMX219_REG_BINNING_MODE_V	0x0175
#define IMX219_REG_CSI_DATA_FORMAT_A	0x018c

// Constants
#define IMX219_PIXEL_RATE		200000000
#define IMX219_LINK_FREQ		288000000
#define IMX219_EXPOSURE_MIN		4
#define IMX219_EXPOSURE_MAX		65535
#define IMX219_EXPOSURE_DEFAULT		1000
#define IMX219_ANA_GAIN_MIN		0
#define IMX219_ANA_GAIN_MAX		232
#define IMX219_ANA_GAIN_DEFAULT		0
#define IMX219_DGTL_GAIN_MIN		0x0100
#define IMX219_DGTL_GAIN_MAX		0x0fff
#define IMX219_DGTL_GAIN_DEFAULT	0x0100

// Mode Configuration
struct imx219_mode {
	u32 width;
	u32 height;
	u16 vts_def;
};

static const struct imx219_mode supported_modes[] = {
	{ .width = 3280, .height = 2464, .vts_def = 3526 }, // 8MP
	{ .width = 1920, .height = 1080, .vts_def = 1763 },  // 1080p
	{ .width = 1640, .height = 1232, .vts_def = 1763 },  // 2x2 Binning
	{ .width = 640, .height = 480, .vts_def = 1763 },    // VGA
};

// IMX219 Structure
struct imx219 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;
	struct clk *xclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[3];
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	u8 lanes;
};

static inline struct imx219 *to_imx219(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx219, sd);
}

// Register Access
static int imx219_write_reg(struct imx219 *imx219, u16 reg, u16 val)
{
	return regmap_write(imx219->regmap, reg, val);
}

// Control Operations
static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
	.s_ctrl = imx219_set_ctrl,
};

static int imx219_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx219 *imx219 =
		container_of(ctrl->handler, struct imx219, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	int ret = 0;

	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	state = v4l2_subdev_get_locked_active_state(&imx219->sd);
	format = v4l2_subdev_state_get_format(state, 0);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		imx219_write_reg(imx219, IMX219_REG_EXPOSURE,
				 ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		imx219_write_reg(imx219, IMX219_REG_ANALOG_GAIN,
				 ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		imx219_write_reg(imx219, IMX219_REG_DIGITAL_GAIN,
				 ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		imx219_write_reg(imx219, IMX219_REG_TEST_PATTERN,
				 ctrl->val);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		imx219_write_reg(imx219, IMX219_REG_ORIENTATION,
				 imx219->hflip->val | imx219->vflip->val << 1);
		break;
	case V4L2_CID_VBLANK:
		imx219_write_reg(imx219, IMX219_REG_VTS,
				 ctrl->val + format->height);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

// Initialization
static int imx219_init_controls(struct imx219 *imx219)
{
	struct v4l2_ctrl_handler *hdl = &imx219->ctrl_handler;
	int ret;

	ret = v4l2_ctrl_handler_init(hdl, 12);
	if (ret)
		return ret;

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_PIXEL_RATE,
			  IMX219_PIXEL_RATE,
			  IMX219_PIXEL_RATE, 1,
			  IMX219_PIXEL_RATE);

	v4l2_ctrl_new_std_menu(hdl, &imx219_ctrl_ops,
			       V4L2_CID_LINK_FREQ,
			       0, 0,
			       imx219->lanes == 2 ? IMX219_LINK_FREQ : 2 * IMX219_LINK_FREQ);

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_EXPOSURE,
			  IMX219_EXPOSURE_MIN, IMX219_EXPOSURE_MAX,
			  1, IMX219_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX219_ANA_GAIN_MIN, IMX219_ANA_GAIN_MAX,
			  1, IMX219_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX219_DGTL_GAIN_MIN, IMX219_DGTL_GAIN_MAX,
			  1, IMX219_DGTL_GAIN_DEFAULT);

	imx219->vflip = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	imx219->hflip = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx219->sd.ctrl_handler = hdl;
	return 0;
}

// Probing
static int imx219_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx219 *imx219;
	int ret;

	imx219 = devm_kzalloc(dev, sizeof(*imx219), GFP_KERNEL);
	if (!imx219)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx219->sd, client, &imx219_subdev_ops);
	imx219->sd.internal_ops = &imx219_internal_ops;

	// Regulators
	imx219->supplies[0].supply = "vdd_sensor";
	imx219->supplies[1].supply = "vdd_core";
	imx219->supplies[2].supply = "vdd_if";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx219->supplies), imx219->supplies);
	if (ret)
		return ret;

	// Clock
	imx219->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(imx219->xclk))
		return PTR_ERR(imx219->xclk);

	// GPIO Reset
	imx219->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(imx219->reset_gpio))
		return PTR_ERR(imx219->reset_gpio);

	// Initialize Controls
	ret = imx219_init_controls(imx219);
	if (ret)
		return ret;

	// Register Subdevice
	ret = v4l2_async_register_subdev_sensor(&imx219->sd);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;
}

// Cleanup
static void imx219_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx219 *imx219 = to_imx219(sd);

	v4l2_async_unregister_subdev(sd);
	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx219_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

// Device Table
static const struct of_device_id imx219_dt_ids[] = {
	{ .compatible = "sony,imx219" },
	{}
};
MODULE_DEVICE_TABLE(of, imx219_dt_ids);

static const struct dev_pm_ops imx219_pm_ops = {
	SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
};

static struct i2c_driver imx219_i2c_driver = {
	.driver = {
		.name = "imx219",
		.of_match_table = imx219_dt_ids,
		.pm = &imx219_pm_ops,
	},
	.probe = imx219_probe,
	.remove = imx219_remove,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Sony IMX219 Sensor Driver for TDA4VM");
MODULE_LICENSE("GPL v2");