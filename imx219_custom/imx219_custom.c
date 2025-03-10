#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

/* Chip ID */
#define IMX219_REG_CHIP_ID      0x0000
#define IMX219_CHIP_ID          0x0219

/* Mode Control */
#define IMX219_REG_MODE_SELECT  0x0100
#define IMX219_MODE_STANDBY     0x00
#define IMX219_MODE_STREAMING   0x01

/* Analog Gain */
#define IMX219_REG_ANALOG_GAIN  0x0157
#define IMX219_ANA_GAIN_MIN     0
#define IMX219_ANA_GAIN_MAX     232
#define IMX219_ANA_GAIN_DEFAULT 100

/* Digital Gain */
#define IMX219_REG_DIGITAL_GAIN 0x0158
#define IMX219_DGTL_GAIN_MIN    0x0100
#define IMX219_DGTL_GAIN_MAX    0x0FFF
#define IMX219_DGTL_GAIN_DEFAULT 0x0100

/* Exposure */
#define IMX219_REG_EXPOSURE     0x015A
#define IMX219_EXPOSURE_MIN     4
#define IMX219_EXPOSURE_MAX     65535
#define IMX219_EXPOSURE_DEFAULT 1000
#define IMX219_REG_CSI_LANE_MODE  0x0114
/* Supported Formats */
static const u32 imx219_mbus_formats[] = {
    MEDIA_BUS_FMT_SRGGB10_1X10,
    MEDIA_BUS_FMT_SGRBG10_1X10,
    MEDIA_BUS_FMT_SGBRG10_1X10,
    MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* Sensor Modes */
struct imx219_mode {
    u32 width;
    u32 height;
    u32 vts_def;
};

static const struct imx219_mode supported_modes[] = {
    {   // 3280x2464 @ 15fps
        .width = 3280,
        .height = 2464,
        .vts_def = 3526,
    },
    {   // 1920x1080 @ 30fps
        .width = 1920,
        .height = 1080,
        .vts_def = 1763,
    },
};

struct imx219 {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct regmap *regmap;
    struct clk *xclk;
    struct regulator_bulk_data supplies[3];
    struct gpio_desc *reset_gpio;
    
    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *analog_gain;
    struct v4l2_ctrl *digital_gain;
    
    const struct imx219_mode *current_mode;
    u32 lanes;
};

static inline struct imx219 *to_imx219(struct v4l2_subdev *sd)
{
    return container_of(sd, struct imx219, sd);
}

/* Chip ID Verification */
static int imx219_check_identity(struct imx219 *imx219)
{
    struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
    struct device *dev = &client->dev;
    u32 lanes;
    unsigned int val;
    int ret;

    ret = regmap_read(imx219->regmap, IMX219_REG_CHIP_ID, &val);
    if (ret) {
        dev_err(&client->dev, "Failed to read chip ID: %d\n", ret);
        return ret;
    }

    if ((u16)val != IMX219_CHIP_ID) {
        dev_err(&client->dev, 
               "Chip ID mismatch: 0x%04X != 0x%04X\n",
               (u16)val, IMX219_CHIP_ID);
        return -ENODEV;
    }
    dev_info(&client->dev, "Detected IMX219 sensor\n");
    ret = of_property_read_u32(dev->of_node, "data-lanes", &imx219->lanes);
	if (ret) {
    		dev_err(dev, "Missing 'data-lanes' in DT\n");
    		return -EINVAL;
    }
    imx219->lanes = lanes;

    
    return 0;
}

/* Power Management */
static int imx219_power_on(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct imx219 *imx219 = to_imx219(sd);
    int ret;

    ret = regulator_bulk_enable(ARRAY_SIZE(imx219->supplies),
                               imx219->supplies);
    if (ret) {
        dev_err(dev, "Failed to enable regulators\n");
        return ret;
    }

    ret = clk_prepare_enable(imx219->xclk);
    if (ret) {
        dev_err(dev, "Failed to enable clock\n");
        goto disable_regulators;
    }
    switch (imx219->lanes) {
    case 2:
        regmap_write(imx219->regmap, IMX219_REG_CSI_LANE_MODE, 0x01);
        break;
    case 4:
        regmap_write(imx219->regmap, IMX219_REG_CSI_LANE_MODE, 0x03);
        break;
    default:
        dev_err(dev, "Unsupported lane count: %d\n", imx219->lanes);
        return -EINVAL;
    }
    gpiod_set_value_cansleep(imx219->reset_gpio, 1);
    usleep_range(5000, 6000);

    return 0;

disable_regulators:
    regulator_bulk_disable(ARRAY_SIZE(imx219->supplies),
                          imx219->supplies);
    return ret;
}

static int imx219_power_off(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct imx219 *imx219 = to_imx219(sd);

    gpiod_set_value_cansleep(imx219->reset_gpio, 0);
    clk_disable_unprepare(imx219->xclk);
    regulator_bulk_disable(ARRAY_SIZE(imx219->supplies),
                          imx219->supplies);

    return 0;
}

/* Control Operations */
static int imx219_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct imx219 *imx219 = container_of(ctrl->handler,
                                       struct imx219,
                                       ctrl_handler);
    int ret = 0;

    if (pm_runtime_resume_and_get(imx219->sd.dev) < 0)
        return 0;
    
    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        ret = regmap_write(imx219->regmap, IMX219_REG_EXPOSURE,
                       ctrl->val);
        break;
    case V4L2_CID_ANALOGUE_GAIN:
        ret = regmap_write(imx219->regmap, IMX219_REG_ANALOG_GAIN,
                       ctrl->val);
        break;
    case V4L2_CID_DIGITAL_GAIN:
        ret = regmap_write(imx219->regmap, IMX219_REG_DIGITAL_GAIN,
                       ctrl->val);
        break;
    default:
        ret = -EINVAL;
        break;
    }

    pm_runtime_put(imx219->sd.dev);
    return ret;
}

static const struct v4l2_ctrl_ops imx219_ctrl_ops = {
    .s_ctrl = imx219_set_ctrl,
};

static int imx219_init_controls(struct imx219 *imx219)
{
    struct v4l2_ctrl_handler *hdl = &imx219->ctrl_handler;
    int ret;

    ret = v4l2_ctrl_handler_init(hdl, 4);
    if (ret)
        return ret;

    imx219->exposure = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
                                        V4L2_CID_EXPOSURE,
                                        IMX219_EXPOSURE_MIN,
                                        IMX219_EXPOSURE_MAX,
                                        1, IMX219_EXPOSURE_DEFAULT);

    imx219->analog_gain = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
                                          V4L2_CID_ANALOGUE_GAIN,
                                          IMX219_ANA_GAIN_MIN,
                                          IMX219_ANA_GAIN_MAX,
                                          1, IMX219_ANA_GAIN_DEFAULT);

    imx219->digital_gain = v4l2_ctrl_new_std(hdl, &imx219_ctrl_ops,
                                           V4L2_CID_DIGITAL_GAIN,
                                           IMX219_DGTL_GAIN_MIN,
                                           IMX219_DGTL_GAIN_MAX,
                                           1, IMX219_DGTL_GAIN_DEFAULT);

    if (hdl->error) {
        ret = hdl->error;
        v4l2_ctrl_handler_free(hdl);
        return ret;
    }

    imx219->sd.ctrl_handler = hdl;
    return 0;
}

/* Subdev Operations */
static int imx219_set_stream(struct v4l2_subdev *sd, int enable)
{
    struct imx219 *imx219 = to_imx219(sd);
    int ret;

    if (enable) {
        ret = regmap_write(imx219->regmap, IMX219_REG_MODE_SELECT,
                       IMX219_MODE_STREAMING);
        if (ret)
            return ret;
         usleep_range(1000, 2000);
    } else {
        ret = regmap_write(imx219->regmap, IMX219_REG_MODE_SELECT,
                       IMX219_MODE_STANDBY);
    }

    return ret;
}

static int imx219_enum_mbus_code(struct v4l2_subdev *sd,
                                struct v4l2_subdev_state *state,
                                struct v4l2_subdev_mbus_code_enum *code)
{
    if (code->index >= ARRAY_SIZE(imx219_mbus_formats))
        return -EINVAL;

    code->code = imx219_mbus_formats[code->index];
    return 0;
}

static int imx219_set_format(struct v4l2_subdev *sd,
                            struct v4l2_subdev_state *state,
                            struct v4l2_subdev_format *fmt)
{
    struct imx219 *imx219 = to_imx219(sd);
    struct v4l2_mbus_framefmt *format = &fmt->format;

    imx219->current_mode = &supported_modes[1]; // Match 1920x1080 mode
    imx219->lanes = 2;

    format->code = MEDIA_BUS_FMT_SRGGB10_1X10;
    format->width = imx219->current_mode->width;
    format->height = imx219->current_mode->height;
    format->field = V4L2_FIELD_NONE;
    format->colorspace = V4L2_COLORSPACE_RAW;

    return 0;
}

static const struct v4l2_subdev_video_ops imx219_video_ops = {
    .s_stream = imx219_set_stream,
};

static const struct v4l2_subdev_pad_ops imx219_pad_ops = {
    .enum_mbus_code = imx219_enum_mbus_code,
    .set_fmt = imx219_set_format,
    .get_fmt = v4l2_subdev_get_fmt,
};

static const struct v4l2_subdev_ops imx219_subdev_ops = {
    .video = &imx219_video_ops,
    .pad = &imx219_pad_ops,
};
static const struct regmap_config imx219_regmap_config = {
    .reg_bits = 16,
    .val_bits = 16,
    .max_register = 0xffff,
};
/* Probe & Remove */
static int imx219_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct imx219 *imx219;
    u32 lanes;
    int ret;

    imx219 = devm_kzalloc(dev, sizeof(*imx219), GFP_KERNEL);
    if (!imx219)
        return -ENOMEM;
    imx219->lanes = 2;

    imx219->regmap = devm_regmap_init_i2c(client, &imx219_regmap_config);
    if (IS_ERR(imx219->regmap)) {
        return PTR_ERR(imx219->regmap);
        }

    /* Power Management */
    imx219->xclk = devm_clk_get(dev, NULL);
    if (IS_ERR(imx219->xclk))
        return PTR_ERR(imx219->xclk);

    imx219->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(imx219->reset_gpio))
        return PTR_ERR(imx219->reset_gpio);

    imx219->supplies[0].supply = "vana";
    imx219->supplies[1].supply = "vdig";
    imx219->supplies[2].supply = "vddl";
    ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(imx219->supplies),
                                 imx219->supplies);
    if (device_property_read_u32(&client->dev, "data-lanes", &imx219->lanes)) {
        dev_err(&client->dev, "Failed to read data-lanes property\n");
        return -EINVAL;
    }
    imx219->lanes = lanes;

    if (imx219->lanes < 1 || imx219->lanes > 4) {
        dev_err(&client->dev, "Invalid number of lanes: %u\n", imx219->lanes);
        return -EINVAL;
    }
    if (ret)
        return ret;
        
    /* Verify Chip Identity */
    ret = imx219_power_on(dev);
    if (ret)
        return ret;

    ret = imx219_check_identity(imx219);
    if (ret)
        goto error_power_off;
    /* Initialize Subdev */
    v4l2_i2c_subdev_init(&imx219->sd, client, &imx219_subdev_ops);
    imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;



    /* Initialize Controls */
    ret = imx219_init_controls(imx219);
    if (ret)
        goto error_power_off;

    /* Media Entity */
    imx219->pad.flags = MEDIA_PAD_FL_SOURCE;
    ret = media_entity_pads_init(&imx219->sd.entity, 1, &imx219->pad);
    if (ret)
        goto error_handler_free;

    /* Runtime PM */
    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);

    return 0;

error_handler_free:
    v4l2_ctrl_handler_free(&imx219->ctrl_handler);
error_power_off:
    imx219_power_off(dev);
    return ret;
}

static void imx219_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx219 *imx219 = to_imx219(sd);

    pm_runtime_disable(&client->dev);
    v4l2_async_unregister_subdev(sd);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(&imx219->ctrl_handler);
}

static const struct of_device_id imx219_of_match[] = {
    { .compatible = "sony,imx219" },
    { }
};
MODULE_DEVICE_TABLE(of, imx219_of_match);

static struct i2c_driver imx219_i2c_driver = {
    .driver = {
        .name = "imx219_custom",
        .of_match_table = imx219_of_match,
        .pm = &(const struct dev_pm_ops){
            SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
    },
},
    .probe = imx219_probe,
    .remove = imx219_remove,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_AUTHOR("Sriram");
MODULE_DESCRIPTION("Custom IMX219 Driver");
MODULE_LICENSE("GPL v2");
