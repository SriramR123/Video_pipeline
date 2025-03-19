#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <asm/unaligned.h>

/* Chip ID and basic registers */
#define IMX219_REG_CHIP_ID          0x0000
#define IMX219_CHIP_ID_VALUE        0x0219
#define IMX219_REG_MODE_SELECT      0x0100
#define IMX219_MODE_STANDBY         0x00
#define IMX219_MODE_STREAMING       0x01
#define IMX219_REG_ORIENTATION      0x0172

/* Clock and frequency settings */
#define IMX219_EXT_CLK_FREQ         24000000 /* 24 MHz */
#define IMX219_PIXEL_RATE           182400000
#define IMX219_DEFAULT_LINK_FREQ    456000000

/* Exposure and gain control */
#define IMX219_REG_EXPOSURE         0x015a
#define IMX219_REG_ANALOG_GAIN      0x0157
#define IMX219_REG_DIGITAL_GAIN     0x0158
#define IMX219_REG_VTS              0x0160
#define IMX219_EXPOSURE_MIN         4
#define IMX219_EXPOSURE_MAX         65535
#define IMX219_EXPOSURE_DEFAULT     0x640
#define IMX219_GAIN_MIN             0
#define IMX219_GAIN_MAX             232
#define IMX219_GAIN_DEFAULT         0
#define IMX219_DGTL_GAIN_MIN        0x0100
#define IMX219_DGTL_GAIN_MAX        0x0fff
#define IMX219_DGTL_GAIN_DEFAULT    0x0100
#define IMX219_VBLANK_MIN           4
#define IMX219_VTS_MAX              0xffff
#define IMX219_PPL_DEFAULT          3448

/* Additional registers for cropping and output */
#define IMX219_REG_LINE_LENGTH_A    0x0162
#define IMX219_REG_X_ADD_STA_A      0x0164
#define IMX219_REG_X_ADD_END_A      0x0166
#define IMX219_REG_Y_ADD_STA_A      0x0168
#define IMX219_REG_Y_ADD_END_A      0x016a
#define IMX219_REG_X_OUTPUT_SIZE    0x016c
#define IMX219_REG_Y_OUTPUT_SIZE    0x016e
#define IMX219_REG_X_ODD_INC_A      0x0170
#define IMX219_REG_Y_ODD_INC_A      0x0171
#define IMX219_REG_CSI_DATA_FORMAT_A 0x018c

/* Supported formats */
static const u32 supported_codes[] = {
    MEDIA_BUS_FMT_SRGGB8_1X8,  MEDIA_BUS_FMT_SGRBG8_1X8,
    MEDIA_BUS_FMT_SGBRG8_1X8,  MEDIA_BUS_FMT_SBGGR8_1X8,
    MEDIA_BUS_FMT_SRGGB10_1X10, MEDIA_BUS_FMT_SGRBG10_1X10,
    MEDIA_BUS_FMT_SGBRG10_1X10, MEDIA_BUS_FMT_SBGGR10_1X10,
};

/* Link frequency menu */
static const s64 link_freq_menu[] = { IMX219_DEFAULT_LINK_FREQ };

/* Regulator supplies */
static const char * const supplies[] = {"vana", "vdig", "vddl"};
#define NUM_SUPPLIES ARRAY_SIZE(supplies)

/* Mode definitions */
struct imx219_mode {
    u32 width;
    u32 height;
    u32 vts_def;
    u32 hblank_def;
};

static const struct imx219_mode supported_modes[] = {
    { .width = 3280, .height = 2464, .vts_def = 3526, .hblank_def = IMX219_PPL_DEFAULT - 3280 },
    { .width = 1920, .height = 1080, .vts_def = 1763, .hblank_def = IMX219_PPL_DEFAULT - 1920 },
    { .width = 1640, .height = 1232, .vts_def = 1763, .hblank_def = IMX219_PPL_DEFAULT - 1640 },
    { .width = 640,  .height = 480,  .vts_def = 1763, .hblank_def = IMX219_PPL_DEFAULT - 640  },
};

/* Common register initialization */
static const struct reg_sequence {
    u16 reg;
    u32 val;
    u32 len;
} imx219_common_regs[] = {
    { IMX219_REG_MODE_SELECT, IMX219_MODE_STANDBY, 1 }, /* Standby */
    { 0x0301, 5, 1 },  /* VTPXCK_DIV */
    { 0x0303, 1, 1 },  /* VTSYCK_DIV */
    { 0x0304, 3, 1 },  /* PREPLLCK_VT_DIV */
    { 0x0305, 3, 1 },  /* PREPLLCK_OP_DIV */
    { 0x0306, 57, 2 }, /* PLL_VT_MPY */
    { 0x030b, 1, 1 },  /* OPSYCK_DIV */
    { 0x030c, 114, 2 },/* PLL_OP_MPY */
    { IMX219_REG_LINE_LENGTH_A, IMX219_PPL_DEFAULT, 2 },
    { IMX219_REG_X_ODD_INC_A, 1, 1 },
    { IMX219_REG_Y_ODD_INC_A, 1, 1 },
    { 0x455e, 0x00, 1 },
    { 0x471e, 0x4b, 1 },
    { 0x4767, 0x0f, 1 },
    { 0x4750, 0x14, 1 },
    { 0x4540, 0x00, 1 },
    { 0x47b4, 0x14, 1 },
    { 0x4713, 0x30, 1 },
    { 0x478b, 0x10, 1 },
    { 0x478f, 0x10, 1 },
    { 0x4793, 0x10, 1 },
    { 0x4797, 0x0e, 1 },
    { 0x479b, 0x0e, 1 },
};

/* Private driver structure */
struct imx219_priv {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_mbus_framefmt fmt;
    const struct imx219_mode *mode;
    struct clk *xclk;
    struct gpio_desc *reset_gpio;
    struct regulator_bulk_data regs[NUM_SUPPLIES];
    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *exposure;
    struct v4l2_ctrl *analogue_gain;
    struct v4l2_ctrl *digital_gain;
    struct v4l2_ctrl *vblank;
    struct v4l2_ctrl *hblank;
    struct v4l2_ctrl *hflip;
    struct v4l2_ctrl *vflip;
    struct mutex lock;
    bool streaming;
    struct v4l2_async_notifier notifier;
    struct v4l2_async_subdev *asd;
};

/* Register access functions */
static int imx219_write_reg(struct i2c_client *client, u16 reg, u32 len, u32 val)
{
    u8 buf[6];
    int ret;

    if (len > 4)
        return -EINVAL;

    put_unaligned_be16(reg, buf);
    put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
    ret = i2c_master_send(client, buf, len + 2);
    if (ret != len + 2) {
        dev_err(&client->dev, "Failed to write reg 0x%04x: %d\n", reg, ret);
        return -EIO;
    }
    return 0;
}

static int imx219_read_reg(struct i2c_client *client, u16 reg, u32 len, u32 *val)
{
    u8 addr[2] = { reg >> 8, reg & 0xFF };
    u8 data[4] = {0};
    struct i2c_msg msgs[] = {
        { .addr = client->addr, .flags = 0, .len = 2, .buf = addr },
        { .addr = client->addr, .flags = I2C_M_RD, .len = len, .buf = data },
    };
    int ret;

    if (len > 4)
        return -EINVAL;

    ret = i2c_transfer(client->adapter, msgs, 2);
    if (ret != 2) {
        dev_err(&client->dev, "Failed to read reg 0x%04x: %d\n", reg, ret < 0 ? ret : -EIO);
        return ret < 0 ? ret : -EIO;
    }

    switch (len) {
    case 1: *val = data[0]; break;
    case 2: *val = get_unaligned_be16(data); break;
    case 4: *val = get_unaligned_be32(data); break;
    default: return -EINVAL;
    }
    return 0;
}

static int imx219_write_regs(struct i2c_client *client, const struct reg_sequence *regs, size_t count)
{
    int ret = 0;
    for (size_t i = 0; i < count; i++) {
        ret = imx219_write_reg(client, regs[i].reg, regs[i].len, regs[i].val);
        if (ret) {
            dev_err(&client->dev, "Failed to write reg 0x%04x: %d\n", regs[i].reg, ret);
            return ret;
        }
    }
    return 0;
}

/* Power management */
static int imx219_power_on(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);
    int ret;

    dev_info(dev, "Powering on IMX219\n");

    ret = regulator_bulk_enable(NUM_SUPPLIES, priv->regs);
    if (ret)
        dev_info(dev, "No real regulators found or failed to enable: %d, continuing\n", ret);

    ret = clk_prepare_enable(priv->xclk);
    if (ret) {
        dev_err(dev, "Failed to enable clock: %d\n", ret);
        regulator_bulk_disable(NUM_SUPPLIES, priv->regs);
        return ret;
    }

    if (priv->reset_gpio) {
        gpiod_set_value_cansleep(priv->reset_gpio, 1);
        usleep_range(10000, 12000);
    } else {
        dev_info(dev, "No reset GPIO, adding delay for stability\n");
        usleep_range(10000, 12000);
    }

    return 0;
}

static int imx219_power_off(struct device *dev)
{
    struct v4l2_subdev *sd = dev_get_drvdata(dev);
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);

    if (priv->reset_gpio)
        gpiod_set_value_cansleep(priv->reset_gpio, 0);
    clk_disable_unprepare(priv->xclk);
    regulator_bulk_disable(NUM_SUPPLIES, priv->regs);
    return 0;
}

/* Format handling */
static u32 imx219_get_format_code(struct imx219_priv *priv, u32 code)
{
    unsigned int i;

    lockdep_assert_held(&priv->lock);

    for (i = 0; i < ARRAY_SIZE(supported_codes); i++)
        if (supported_codes[i] == code)
            break;

    if (i >= ARRAY_SIZE(supported_codes))
        i = 0; /* Default to UYVY8_1X16 */

    return supported_codes[i];
}

static void imx219_set_default_format(struct imx219_priv *priv)
{
    priv->fmt.width = 1920;
    priv->fmt.height = 1080;
    priv->fmt.code = MEDIA_BUS_FMT_SRGGB8_1X8; 
    priv->fmt.field = V4L2_FIELD_NONE;
    priv->fmt.colorspace = V4L2_COLORSPACE_SRGB;
    priv->fmt.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SRGB);
    priv->fmt.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    priv->fmt.xfer_func = V4L2_XFER_FUNC_DEFAULT;
    priv->mode = &supported_modes[1];
}

/* V4L2 controls */
static int imx219_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct imx219_priv *priv = container_of(ctrl->handler, struct imx219_priv, ctrl_handler);
    struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
    int ret;

    if (ctrl->id == V4L2_CID_VBLANK) {
        int exposure_max = priv->mode->height + ctrl->val - 4;
        int exposure_def = min(exposure_max, IMX219_EXPOSURE_DEFAULT);
        __v4l2_ctrl_modify_range(priv->exposure, IMX219_EXPOSURE_MIN,
                                 exposure_max, 1, exposure_def);
    }

    if (pm_runtime_get_if_in_use(&client->dev) <= 0)
        return 0;

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        ret = imx219_write_reg(client, IMX219_REG_EXPOSURE, 2, ctrl->val); break;
    case V4L2_CID_ANALOGUE_GAIN:
        ret = imx219_write_reg(client, IMX219_REG_ANALOG_GAIN, 1, ctrl->val); break;
    case V4L2_CID_DIGITAL_GAIN:
        ret = imx219_write_reg(client, IMX219_REG_DIGITAL_GAIN, 2, ctrl->val); break;
    case V4L2_CID_VBLANK:
        ret = imx219_write_reg(client, IMX219_REG_VTS, 2, priv->mode->height + ctrl->val); break;
    case V4L2_CID_HFLIP:
    case V4L2_CID_VFLIP:
        ret = imx219_write_reg(client, IMX219_REG_ORIENTATION, 1,
                               (priv->hflip->val | (priv->vflip->val << 1))); break;
    default:
        dev_info(&client->dev, "Control 0x%x not handled\n", ctrl->id);
        ret = -EINVAL;
        break;
    }

    pm_runtime_put(&client->dev);
    return ret;
}

static const struct v4l2_ctrl_ops imx219_ctrl_ops = { .s_ctrl = imx219_set_ctrl };

static int imx219_init_controls(struct imx219_priv *priv)
{
    struct v4l2_ctrl_handler *ctrl_hdl = &priv->ctrl_handler;
    int ret, hblank;

    ret = v4l2_ctrl_handler_init(ctrl_hdl, 8);
    if (ret)
        return ret;

    priv->pixel_rate = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_PIXEL_RATE,
                                         IMX219_PIXEL_RATE, IMX219_PIXEL_RATE, 1, IMX219_PIXEL_RATE);
    if (priv->pixel_rate)
        priv->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    priv->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_LINK_FREQ,
                                             ARRAY_SIZE(link_freq_menu) - 1, 0, link_freq_menu);
    if (priv->link_freq)
        priv->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    priv->exposure = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_EXPOSURE,
                                       IMX219_EXPOSURE_MIN, IMX219_EXPOSURE_MAX, 1, IMX219_EXPOSURE_DEFAULT);
    priv->analogue_gain = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                                            IMX219_GAIN_MIN, IMX219_GAIN_MAX, 1, IMX219_GAIN_DEFAULT);
    priv->digital_gain = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
                                           IMX219_DGTL_GAIN_MIN, IMX219_DGTL_GAIN_MAX, 1, IMX219_DGTL_GAIN_DEFAULT);
    priv->vblank = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_VBLANK,
                                     IMX219_VBLANK_MIN, IMX219_VTS_MAX - priv->mode->height,
                                     1, priv->mode->vts_def - priv->mode->height);
    hblank = IMX219_PPL_DEFAULT - priv->mode->width;
    priv->hblank = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_HBLANK,
                                     hblank, hblank, 1, hblank);
    if (priv->hblank)
        priv->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    priv->hflip = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
    priv->vflip = v4l2_ctrl_new_std(ctrl_hdl, &imx219_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

    if (ctrl_hdl->error) {
        ret = ctrl_hdl->error;
        v4l2_ctrl_handler_free(ctrl_hdl);
        return ret;
    }

    priv->sd.ctrl_handler = ctrl_hdl;
    return 0;
}

/* V4L2 subdev operations */
static int imx219_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)
{
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);

    mutex_lock(&priv->lock);
    fmt->format = priv->fmt;
    fmt->format.code = imx219_get_format_code(priv, priv->fmt.code);
    mutex_unlock(&priv->lock);
    return 0;
}

static int imx219_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)
{
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);
    const struct imx219_mode *mode;
    u32 code_index;

    mutex_lock(&priv->lock);

    for (code_index = 0; code_index < ARRAY_SIZE(supported_codes); code_index++)
        if (supported_codes[code_index] == fmt->format.code)
            break;
    if (code_index >= ARRAY_SIZE(supported_codes))
        code_index = 0; /* Default to UYVY8_1X16 */

    fmt->format.code = imx219_get_format_code(priv, supported_codes[code_index]);

    mode = v4l2_find_nearest_size(supported_modes, ARRAY_SIZE(supported_modes),
                                  width, height, fmt->format.width, fmt->format.height);
    fmt->format.width = mode->width;
    fmt->format.height = mode->height;
    fmt->format.field = V4L2_FIELD_NONE;
    fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
    fmt->format.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SRGB);
    fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
    fmt->format.xfer_func = V4L2_XFER_FUNC_DEFAULT;

    if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
        priv->fmt = fmt->format;
        priv->mode = mode;
        __v4l2_ctrl_modify_range(priv->vblank, IMX219_VBLANK_MIN,
                                 IMX219_VTS_MAX - mode->height, 1,
                                 mode->vts_def - mode->height);
        __v4l2_ctrl_s_ctrl(priv->vblank, mode->vts_def - mode->height);
        __v4l2_ctrl_modify_range(priv->exposure, IMX219_EXPOSURE_MIN,
                                 mode->vts_def - 4, 1, IMX219_EXPOSURE_DEFAULT);
        __v4l2_ctrl_modify_range(priv->hblank, mode->hblank_def, mode->hblank_def,
                                 1, mode->hblank_def);
    }

    mutex_unlock(&priv->lock);
    return 0;
}

static int imx219_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);

    if (code->index >= ARRAY_SIZE(supported_codes))
        return -EINVAL;

    mutex_lock(&priv->lock);
    code->code = imx219_get_format_code(priv, supported_codes[code->index]);
    mutex_unlock(&priv->lock);
    return 0;
}

static int imx219_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                                  struct v4l2_subdev_frame_size_enum *fse)
{
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);
    u32 code;

    if (fse->index >= ARRAY_SIZE(supported_modes))
        return -EINVAL;

    mutex_lock(&priv->lock);
    code = imx219_get_format_code(priv, fse->code);
    mutex_unlock(&priv->lock);
    if (fse->code != code)
        return -EINVAL;

    fse->min_width = supported_modes[fse->index].width;
    fse->max_width = fse->min_width;
    fse->min_height = supported_modes[fse->index].height;
    fse->max_height = fse->min_height;
    return 0;
}

static int imx219_start_streaming(struct imx219_priv *priv)
{
    struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
    int ret;
    u32 bpp = (priv->fmt.code == MEDIA_BUS_FMT_SRGGB8_1X8 || 
        priv->fmt.code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
        priv->fmt.code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
        priv->fmt.code == MEDIA_BUS_FMT_SBGGR8_1X8) ? 8 : 10;

    ret = pm_runtime_resume_and_get(&client->dev);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to resume power: %d\n", ret);
        return ret;
    }

    ret = imx219_write_regs(client, imx219_common_regs, ARRAY_SIZE(imx219_common_regs));
    if (ret) {
        dev_err(&client->dev, "Failed to write common regs: %d\n", ret);
        goto err;
    }

    ret = imx219_write_reg(client, 0x0114, 1, 0x01); // 2-lane mode
    if (ret) {
        dev_err(&client->dev, "Failed to set 2-lane mode: %d\n", ret);
        goto err;
    }

    /* Cropping and output size */
    ret = imx219_write_reg(client, IMX219_REG_X_ADD_STA_A, 2, 8);
    ret |= imx219_write_reg(client, IMX219_REG_X_ADD_END_A, 2, 8 + priv->fmt.width - 1);
    ret |= imx219_write_reg(client, IMX219_REG_Y_ADD_STA_A, 2, 8);
    ret |= imx219_write_reg(client, IMX219_REG_Y_ADD_END_A, 2, 8 + priv->fmt.height - 1);
    ret |= imx219_write_reg(client, IMX219_REG_X_OUTPUT_SIZE, 2, priv->fmt.width);
    ret |= imx219_write_reg(client, IMX219_REG_Y_OUTPUT_SIZE, 2, priv->fmt.height);
    ret |= imx219_write_reg(client, IMX219_REG_CSI_DATA_FORMAT_A, 2, (bpp << 8) | bpp);
    ret |= imx219_write_reg(client, 0x0309, 1, bpp);
    if (ret) {
        dev_err(&client->dev, "Failed to configure cropping/output: %d\n", ret);
        goto err;
    }

    ret = __v4l2_ctrl_handler_setup(&priv->ctrl_handler);
    if (ret) {
        dev_err(&client->dev, "Failed to apply controls: %d\n", ret);
        goto err;
    }

    ret = imx219_write_reg(client, IMX219_REG_MODE_SELECT, 1, IMX219_MODE_STREAMING);
    if (ret) {
        dev_err(&client->dev, "Failed to start streaming: %d\n", ret);
        goto err;
    }

    __v4l2_ctrl_grab(priv->hflip, true);
    __v4l2_ctrl_grab(priv->vflip, true);
    priv->streaming = true;
    dev_info(&client->dev, "Streaming started at %dx%d, format code 0x%x\n",
             priv->fmt.width, priv->fmt.height, priv->fmt.code);
    return 0;

err:
    pm_runtime_put(&client->dev);
    return ret;
}

static int imx219_stop_streaming(struct imx219_priv *priv)
{
    struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
    int ret;

    ret = imx219_write_reg(client, IMX219_REG_MODE_SELECT, 1, IMX219_MODE_STANDBY);
    if (ret)
        dev_err(&client->dev, "Failed to stop streaming: %d\n", ret);

    __v4l2_ctrl_grab(priv->hflip, false);
    __v4l2_ctrl_grab(priv->vflip, false);
    priv->streaming = false;
    pm_runtime_put(&client->dev);
    return ret;
}

static int imx219_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);
    int ret = 0;

    mutex_lock(&priv->lock);
    if (priv->streaming == !!enable)
        goto out;

    if (enable) {
        ret = imx219_start_streaming(priv);
        if (!ret)
            priv->streaming = true;
    } else {
        ret = imx219_stop_streaming(priv);
        if (!ret)
            priv->streaming = false;
    }

out:
    mutex_unlock(&priv->lock);
    return ret;
}

/* V4L2 subdev ops */
static const struct v4l2_subdev_pad_ops imx219_pad_ops = {
    .get_fmt = imx219_get_fmt,
    .set_fmt = imx219_set_fmt,
    .enum_mbus_code = imx219_enum_mbus_code,
    .enum_frame_size = imx219_enum_frame_size,
};

static const struct v4l2_subdev_video_ops imx219_video_ops = {
    .s_stream = imx219_s_stream,
};

static const struct v4l2_subdev_ops imx219_ops = {
    .pad = &imx219_pad_ops,
    .video = &imx219_video_ops,
};

/* Async notifier callbacks */
static int imx219_async_bound(struct v4l2_async_notifier *notifier,
                              struct v4l2_subdev *subdev,
                              struct v4l2_async_subdev *asd)
{
    struct imx219_priv *priv = container_of(notifier, struct imx219_priv, notifier);
    struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);

    dev_info(&client->dev, "Async subdev bound successfully to %s\n", subdev->name);
    return 0;
}

static void imx219_async_unbind(struct v4l2_async_notifier *notifier,
                                struct v4l2_subdev *subdev,
                                struct v4l2_async_subdev *asd)
{
    struct imx219_priv *priv = container_of(notifier, struct imx219_priv, notifier);
    struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);

    dev_info(&client->dev, "Async subdev unbound from %s\n", subdev->name);
}

static const struct v4l2_async_notifier_operations imx219_async_ops = {
    .bound = imx219_async_bound,
    .unbind = imx219_async_unbind,
};

/* Probe function */
static int imx219_probe(struct i2c_client *client)
{
    struct device *dev = &client->dev;
    struct imx219_priv *priv;
    struct v4l2_subdev *sd;
    struct fwnode_handle *fwnode;
    u32 val = 0;
    int ret;

    dev_info(dev, "Starting IMX219 custom probe on bus %d, addr 0x%02x\n",
             client->adapter->nr, client->addr);

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv) {
        dev_err(dev, "Failed to allocate memory\n");
        return -ENOMEM;
    }

    sd = &priv->sd;
    v4l2_i2c_subdev_init(sd, client, &imx219_ops);
    dev_set_drvdata(dev, sd);

    priv->xclk = devm_clk_get(dev, "xclk");
    if (IS_ERR(priv->xclk)) {
        ret = PTR_ERR(priv->xclk);
        dev_err(dev, "Failed to get clock 'xclk': %d\n", ret);
        return ret;
    }

    ret = clk_set_rate(priv->xclk, IMX219_EXT_CLK_FREQ);
    if (ret) {
        dev_err(dev, "Failed to set clock rate to 24MHz: %d\n", ret);
        return ret;
    }

    for (int i = 0; i < NUM_SUPPLIES; i++)
        priv->regs[i].supply = supplies[i];
    ret = devm_regulator_bulk_get(dev, NUM_SUPPLIES, priv->regs);
    if (ret)
        dev_info(dev, "Failed to get regulators, assuming dummies: %d\n", ret);

    priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(priv->reset_gpio)) {
        ret = PTR_ERR(priv->reset_gpio);
        dev_err(dev, "Failed to get reset GPIO: %d\n", ret);
        return ret;
    }

    mutex_init(&priv->lock);
    imx219_set_default_format(priv);

    ret = imx219_power_on(dev);
    if (ret) {
        dev_err(dev, "Failed to power on: %d\n", ret);
        goto err_mutex;
    }

    /* Try stabilization with retries, skip if it fails */
    for (int i = 0; i < 3; i++) {
        ret = imx219_write_reg(client, IMX219_REG_MODE_SELECT, 1, IMX219_MODE_STREAMING);
        if (!ret) {
            usleep_range(100, 110);
            ret = imx219_write_reg(client, IMX219_REG_MODE_SELECT, 1, IMX219_MODE_STANDBY);
            if (!ret)
                break;
            dev_err(dev, "Failed to stop streaming for init: %d\n", ret);
        } else {
            dev_err(dev, "Failed to start streaming for init: %d\n", ret);
        }
        usleep_range(5000, 6000);
    }
    if (ret) {
        dev_warn(dev, "Stabilization failed, proceeding anyway: %d\n", ret);
    }

    for (int i = 0; i < 3; i++) {
        ret = imx219_read_reg(client, IMX219_REG_CHIP_ID, 2, &val);
        if (!ret && val == IMX219_CHIP_ID_VALUE)
            break;
        dev_warn(dev, "Retry %d: Failed to read chip ID, got 0x%04x (%d)\n", i + 1, val, ret);
        usleep_range(5000, 6000);
    }
    if (ret || val != IMX219_CHIP_ID_VALUE) {
        dev_err(dev, "Chip ID mismatch after retries: expected 0x%04x, got 0x%04x (%d)\n",
                IMX219_CHIP_ID_VALUE, val, ret);
        ret = ret ? ret : -ENODEV;
        goto err_power;
    }

    ret = imx219_init_controls(priv);
    if (ret) {
        dev_err(dev, "Failed to init controls: %d\n", ret);
        goto err_power;
    }

    priv->pad.flags = MEDIA_PAD_FL_SOURCE;
    sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
    ret = media_entity_pads_init(&sd->entity, 1, &priv->pad);
    if (ret) {
        dev_err(dev, "Failed to init media entity: %d\n", ret);
        goto err_ctrl;
    }

    fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
    if (!fwnode) {
        dev_err(dev, "Failed to get endpoint fwnode\n");
        ret = -EINVAL;
        goto err_media;
    }

    v4l2_async_nf_init(&priv->notifier);
    priv->notifier.ops = &imx219_async_ops;

    priv->asd = v4l2_async_nf_add_fwnode(&priv->notifier, fwnode,
                                         struct v4l2_async_subdev);
    if (IS_ERR(priv->asd)) {
        ret = PTR_ERR(priv->asd);
        dev_err(dev, "Failed to add async fwnode: %d\n", ret);
        fwnode_handle_put(fwnode);
        goto err_notifier;
    }

    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    sd->dev = dev;
    snprintf(sd->name, sizeof(sd->name), "imx219 %d-00%02x", client->adapter->nr, client->addr);
    dev_info(dev, "Registering async notifier for subdev '%s'\n", sd->name);

    ret = v4l2_async_register_subdev_sensor(&priv->sd);
    if (ret) {
        dev_err(dev, "Failed to register async notifier: %d\n", ret);
        fwnode_handle_put(fwnode);
        goto err_notifier;
    }

    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);

    dev_info(dev, "IMX219 custom driver probed successfully: %dx%d\n",
             priv->fmt.width, priv->fmt.height);
    return 0;

err_notifier:
    v4l2_async_nf_cleanup(&priv->notifier);
err_media:
    media_entity_cleanup(&sd->entity);
    fwnode_handle_put(fwnode);
err_ctrl:
    v4l2_ctrl_handler_free(&priv->ctrl_handler);
err_power:
    imx219_power_off(dev);
err_mutex:
    mutex_destroy(&priv->lock);
    return ret;
}

static void imx219_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx219_priv *priv = container_of(sd, struct imx219_priv, sd);

    v4l2_async_unregister_subdev(sd);
    v4l2_async_nf_cleanup(&priv->notifier);
    media_entity_cleanup(&sd->entity);
    v4l2_ctrl_handler_free(&priv->ctrl_handler);
    pm_runtime_disable(&client->dev);
    if (!pm_runtime_status_suspended(&client->dev))
        imx219_power_off(&client->dev);
    pm_runtime_set_suspended(&client->dev);
    mutex_destroy(&priv->lock);
}

/* Device tree and module info */
static const struct of_device_id imx219_dt_ids[] = {
    { .compatible = "sony,imx219" },
    { }
};
MODULE_DEVICE_TABLE(of, imx219_dt_ids);

static const struct dev_pm_ops imx219_pm_ops = {
    SET_RUNTIME_PM_OPS(imx219_power_off, imx219_power_on, NULL)
};

static struct i2c_driver imx219_i2c_driver = {
    .driver = {
        .name = "imx219_custom",
        .of_match_table = imx219_dt_ids,
        .pm = &imx219_pm_ops,
    },
    .probe_new = imx219_probe,
    .remove = imx219_remove,
};

module_i2c_driver(imx219_i2c_driver);

MODULE_DESCRIPTION("Custom IMX219 Camera Driver");
MODULE_AUTHOR("Sriram");
MODULE_LICENSE("GPL v2");