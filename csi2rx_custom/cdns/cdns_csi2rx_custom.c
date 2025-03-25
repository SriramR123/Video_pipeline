// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Cadence MIPI-CSI2 RX Controller v1.3
 * Copyright (C) 2017 Cadence Design Systems Inc.
 * Compatible with custom IMX219 driver supporting Bayer formats and 2-lane mode.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define CSI2RX_DEVICE_CFG_REG           0x000
#define CSI2RX_SOFT_RESET_REG           0x004
#define CSI2RX_SOFT_RESET_PROTOCOL      BIT(1)
#define CSI2RX_SOFT_RESET_FRONT         BIT(0)
#define CSI2RX_STATIC_CFG_REG           0x008
#define CSI2RX_STATIC_CFG_DLANE_MAP(llane, plane)   ((plane) << (16 + (llane) * 4))
#define CSI2RX_STATIC_CFG_LANES_MASK    GENMASK(11, 8)
#define CSI2RX_DPHY_LANE_CTRL_REG       0x40
#define CSI2RX_DPHY_CL_RST              BIT(16)
#define CSI2RX_DPHY_DL_RST(i)           BIT((i) + 12)
#define CSI2RX_DPHY_CL_EN               BIT(4)
#define CSI2RX_DPHY_DL_EN(i)            BIT(i)
#define CSI2RX_STREAM_BASE(n)           (((n) + 1) * 0x100)
#define CSI2RX_STREAM_CTRL_REG(n)       (CSI2RX_STREAM_BASE(n) + 0x000)
#define CSI2RX_STREAM_CTRL_SOFT_RST     BIT(4)
#define CSI2RX_STREAM_CTRL_STOP         BIT(1)
#define CSI2RX_STREAM_CTRL_START        BIT(0)
#define CSI2RX_STREAM_STATUS_REG(n)     (CSI2RX_STREAM_BASE(n) + 0x004)
#define CSI2RX_STREAM_STATUS_RDY        BIT(31)
#define CSI2RX_STREAM_DATA_CFG_REG(n)   (CSI2RX_STREAM_BASE(n) + 0x008)
#define CSI2RX_STREAM_DATA_CFG_VC_SELECT(n) BIT((n) + 16)
#define CSI2RX_STREAM_CFG_REG(n)        (CSI2RX_STREAM_BASE(n) + 0x00c)
#define CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF   (1 << 8)

#define CSI2RX_LANES_MAX    4
#define CSI2RX_STREAMS_MAX  4

enum csi2rx_pads {
    CSI2RX_PAD_SINK,
    CSI2RX_PAD_SOURCE_STREAM0,
    CSI2RX_PAD_SOURCE_STREAM1,
    CSI2RX_PAD_SOURCE_STREAM2,
    CSI2RX_PAD_SOURCE_STREAM3,
    CSI2RX_PAD_MAX,
};

struct csi2rx_fmt {
    u32 code;
    u8  bpp;
};

/* Format array compatible with IMX219 supported_codes */
static const struct csi2rx_fmt formats[] = {
    { .code = MEDIA_BUS_FMT_YUYV8_1X16, .bpp = 16, },
    { .code = MEDIA_BUS_FMT_UYVY8_1X16, .bpp = 16, },
    { .code = MEDIA_BUS_FMT_YVYU8_1X16, .bpp = 16, },
    { .code = MEDIA_BUS_FMT_VYUY8_1X16, .bpp = 16, },
    { .code = MEDIA_BUS_FMT_SBGGR8_1X8, .bpp = 8, },  // Matches IMX219
    { .code = MEDIA_BUS_FMT_SGBRG8_1X8, .bpp = 8, },  // Matches IMX219
    { .code = MEDIA_BUS_FMT_SGRBG8_1X8, .bpp = 8, },  // Matches IMX219
    { .code = MEDIA_BUS_FMT_SRGGB8_1X8, .bpp = 8, },  // Matches IMX219
    { .code = MEDIA_BUS_FMT_Y8_1X8,     .bpp = 8, },
    { .code = MEDIA_BUS_FMT_SBGGR10_1X10, .bpp = 10, }, // Matches IMX219
    { .code = MEDIA_BUS_FMT_SGBRG10_1X10, .bpp = 10, }, // Matches IMX219
    { .code = MEDIA_BUS_FMT_SGRBG10_1X10, .bpp = 10, }, // Matches IMX219
    { .code = MEDIA_BUS_FMT_SRGGB10_1X10, .bpp = 10, }, // Matches IMX219
    { .code = MEDIA_BUS_FMT_RGB565_1X16,  .bpp = 16, },
    { .code = MEDIA_BUS_FMT_RGB888_1X24,  .bpp = 24, },
    { .code = MEDIA_BUS_FMT_BGR888_1X24,  .bpp = 24, },
};

struct csi2rx_priv {
    struct device           *dev;
    unsigned int            count;
    struct mutex            lock;
    void __iomem            *base;
    struct clk              *sys_clk;
    struct clk              *p_clk;
    struct clk              *pixel_clk[CSI2RX_STREAMS_MAX];
    struct reset_control    *sys_rst;
    struct reset_control    *p_rst;
    struct reset_control    *pixel_rst[CSI2RX_STREAMS_MAX];
    struct phy              *dphy;
    u8                      lanes[CSI2RX_LANES_MAX];
    u8                      num_lanes; // Configurable via DT, set to 2 for IMX219
    u8                      max_lanes;
    u8                      max_streams;
    bool                    has_internal_dphy;
    struct v4l2_subdev      subdev;
    struct v4l2_async_notifier notifier;
    struct media_pad        pads[CSI2RX_PAD_MAX];
    struct v4l2_subdev      *source_subdev; // Links to IMX219 subdev
    int                     source_pad;
};

static const struct csi2rx_fmt *csi2rx_get_fmt_by_code(u32 code)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(formats); i++)
        if (formats[i].code == code)
            return &formats[i];
    return NULL;
}

static inline struct csi2rx_priv *v4l2_subdev_to_csi2rx(struct v4l2_subdev *subdev)
{
    return container_of(subdev, struct csi2rx_priv, subdev);
}

static void csi2rx_reset(struct csi2rx_priv *csi2rx)
{
    unsigned int i;
    writel(CSI2RX_SOFT_RESET_PROTOCOL | CSI2RX_SOFT_RESET_FRONT,
           csi2rx->base + CSI2RX_SOFT_RESET_REG);
    for (i = 0; i < csi2rx->max_streams; i++) {
        writel(CSI2RX_STREAM_CTRL_SOFT_RST,
               csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
    }
    usleep_range(10, 20);
    writel(0, csi2rx->base + CSI2RX_SOFT_RESET_REG);
    for (i = 0; i < csi2rx->max_streams; i++)
        writel(0, csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
}

static int csi2rx_configure_ext_dphy(struct csi2rx_priv *csi2rx)
{
    union phy_configure_opts opts = { };
    struct phy_configure_opts_mipi_dphy *cfg = &opts.mipi_dphy;
    struct v4l2_subdev_format sd_fmt = {
        .which = V4L2_SUBDEV_FORMAT_ACTIVE,
        .pad = CSI2RX_PAD_SINK,
    };
    const struct csi2rx_fmt *csi_fmt;
    s64 link_freq;
    int ret;

    ret = v4l2_subdev_call(csi2rx->source_subdev, pad, get_fmt, NULL, &sd_fmt);
    if (ret < 0)
        return ret;

    csi_fmt = csi2rx_get_fmt_by_code(sd_fmt.format.code);
    if (!csi_fmt)
        return -EINVAL;

    link_freq = v4l2_get_link_freq(csi2rx->source_subdev->ctrl_handler,
                                   csi_fmt->bpp, 2 * csi2rx->num_lanes); // Matches IMX219 456MHz
    if (link_freq < 0)
        return link_freq;

    ret = phy_mipi_dphy_get_default_config_for_hsclk(link_freq, csi2rx->num_lanes, cfg);
    if (ret)
        return ret;

    ret = phy_power_on(csi2rx->dphy);
    if (ret)
        return ret;

    ret = phy_configure(csi2rx->dphy, &opts);
    if (ret) {
        phy_power_off(csi2rx->dphy);
        return ret;
    }
    return 0;
}

static int csi2rx_start(struct csi2rx_priv *csi2rx)
{
    unsigned int i;
    unsigned long lanes_used = 0;
    u32 reg;
    int ret;

    ret = clk_prepare_enable(csi2rx->p_clk);
    if (ret)
        return ret;

    reset_control_deassert(csi2rx->p_rst);
    csi2rx_reset(csi2rx);

    reg = csi2rx->num_lanes << 8; // Set to 2 for IMX219
    for (i = 0; i < csi2rx->num_lanes; i++) {
        reg |= CSI2RX_STATIC_CFG_DLANE_MAP(i, csi2rx->lanes[i]);
        set_bit(csi2rx->lanes[i], &lanes_used);
    }
    for (i = csi2rx->num_lanes; i < csi2rx->max_lanes; i++) {
        unsigned int idx = find_first_zero_bit(&lanes_used, csi2rx->max_lanes);
        set_bit(idx, &lanes_used);
        reg |= CSI2RX_STATIC_CFG_DLANE_MAP(i, i + 1);
    }
    writel(reg, csi2rx->base + CSI2RX_STATIC_CFG_REG);

    if (csi2rx->dphy) {
        reg = CSI2RX_DPHY_CL_EN | CSI2RX_DPHY_CL_RST;
        for (i = 0; i < csi2rx->num_lanes; i++) {
            reg |= CSI2RX_DPHY_DL_EN(csi2rx->lanes[i] - 1);
            reg |= CSI2RX_DPHY_DL_RST(csi2rx->lanes[i] - 1);
        }
        writel(reg, csi2rx->base + CSI2RX_DPHY_LANE_CTRL_REG);

        ret = csi2rx_configure_ext_dphy(csi2rx);
        if (ret) {
            dev_err(csi2rx->dev, "Failed to configure external DPHY: %d\n", ret);
            goto err_disable_pclk;
        }
    }

    for (i = 0; i < csi2rx->max_streams; i++) {
        ret = clk_prepare_enable(csi2rx->pixel_clk[i]);
        if (ret)
            goto err_disable_pixclk;

        reset_control_deassert(csi2rx->pixel_rst[i]);
        writel(CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF,
               csi2rx->base + CSI2RX_STREAM_CFG_REG(i));
        writel(CSI2RX_STREAM_DATA_CFG_VC_SELECT(0),
               csi2rx->base + CSI2RX_STREAM_DATA_CFG_REG(i));
        writel(CSI2RX_STREAM_CTRL_START,
               csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
    }

    ret = clk_prepare_enable(csi2rx->sys_clk);
    if (ret)
        goto err_disable_pixclk;

    reset_control_deassert(csi2rx->sys_rst);
    ret = v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, true); // Calls IMX219 s_stream
    if (ret)
        goto err_disable_sysclk;

    clk_disable_unprepare(csi2rx->p_clk);
    return 0;

err_disable_sysclk:
    clk_disable_unprepare(csi2rx->sys_clk);
err_disable_pixclk:
    for (; i > 0; i--) {
        reset_control_assert(csi2rx->pixel_rst[i - 1]);
        clk_disable_unprepare(csi2rx->pixel_clk[i - 1]);
    }
    if (csi2rx->dphy) {
        writel(0, csi2rx->base + CSI2RX_DPHY_LANE_CTRL_REG);
        phy_power_off(csi2rx->dphy);
    }
err_disable_pclk:
    clk_disable_unprepare(csi2rx->p_clk);
    return ret;
}

static void csi2rx_stop(struct csi2rx_priv *csi2rx)
{
    unsigned int i;
    u32 val;
    int ret;

    clk_prepare_enable(csi2rx->p_clk);
    reset_control_assert(csi2rx->sys_rst);
    clk_disable_unprepare(csi2rx->sys_clk);

    for (i = 0; i < csi2rx->max_streams; i++) {
        writel(CSI2RX_STREAM_CTRL_STOP,
               csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
        ret = readl_relaxed_poll_timeout(csi2rx->base + CSI2RX_STREAM_STATUS_REG(i),
                                        val, !(val & CSI2RX_STREAM_STATUS_RDY),
                                        10, 10000);
        if (ret)
            dev_warn(csi2rx->dev, "Failed to stop streaming on pad%u\n", i);
        reset_control_assert(csi2rx->pixel_rst[i]);
        clk_disable_unprepare(csi2rx->pixel_clk[i]);
    }

    reset_control_assert(csi2rx->p_rst);
    clk_disable_unprepare(csi2rx->p_clk);

    if (v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, false))
        dev_warn(csi2rx->dev, "Couldn't disable our subdev\n");

    if (csi2rx->dphy) {
        writel(0, csi2rx->base + CSI2RX_DPHY_LANE_CTRL_REG);
        if (phy_power_off(csi2rx->dphy))
            dev_warn(csi2rx->dev, "Couldn't power off DPHY\n");
    }
}

static int csi2rx_s_stream(struct v4l2_subdev *subdev, int enable)
{
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(subdev);
    int ret = 0;

    mutex_lock(&csi2rx->lock);
    if (enable) {
        if (!csi2rx->count) {
            ret = csi2rx_start(csi2rx);
            if (ret)
                goto out;
        }
        csi2rx->count++;
    } else {
        csi2rx->count--;
        if (!csi2rx->count)
            csi2rx_stop(csi2rx);
    }
out:
    mutex_unlock(&csi2rx->lock);
    return ret;
}

static int csi2rx_enum_mbus_code(struct v4l2_subdev *subdev,
                                 struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code_enum)
{
    if (code_enum->index >= ARRAY_SIZE(formats))
        return -EINVAL;
    code_enum->code = formats[code_enum->index].code;
    return 0;
}

static int csi2rx_set_fmt(struct v4l2_subdev *subdev,
                          struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format)
{
    struct v4l2_mbus_framefmt fmt_default = {
        .width = 1920,
        .height = 1080,
        .code = MEDIA_BUS_FMT_SRGGB10_1X10, // Default for IMX219
        .field = V4L2_FIELD_NONE,
    };
    struct v4l2_mbus_framefmt *fmt;
    unsigned int i;

    if (format->pad != CSI2RX_PAD_SINK)
        return v4l2_subdev_get_fmt(subdev, state, format);

    if (!csi2rx_get_fmt_by_code(format->format.code))
        format->format.code = formats[0].code;

    format->format.field = V4L2_FIELD_NONE;

    // Use the format directly from the state or set a default
    fmt = v4l2_subdev_get_try_format(subdev, state, format->pad);
    if (!fmt)
        *fmt = fmt_default; // Fallback to default if no try format
    *fmt = format->format;

    for (i = CSI2RX_PAD_SOURCE_STREAM0; i < CSI2RX_PAD_MAX; i++) {
        fmt = v4l2_subdev_get_try_format(subdev, state, i);
        if (!fmt)
            *fmt = fmt_default;
        *fmt = format->format;
    }
    return 0;
}

static const struct v4l2_subdev_pad_ops csi2rx_pad_ops = {
    .enum_mbus_code = csi2rx_enum_mbus_code,
    .get_fmt        = v4l2_subdev_get_fmt,
    .set_fmt        = csi2rx_set_fmt,
};

static const struct v4l2_subdev_video_ops csi2rx_video_ops = {
    .s_stream   = csi2rx_s_stream,
};

static const struct v4l2_subdev_ops csi2rx_subdev_ops = {
    .video  = &csi2rx_video_ops,
    .pad    = &csi2rx_pad_ops,
};

static const struct v4l2_subdev_internal_ops csi2rx_internal_ops = {
    // No init_state needed for kernel 6.1
};

static const struct media_entity_operations csi2rx_media_ops = {
    .link_validate = v4l2_subdev_link_validate,
};

static int csi2rx_async_bound(struct v4l2_async_notifier *notifier,
                              struct v4l2_subdev *s_subdev,
                              struct v4l2_async_subdev *asd)
{
    struct v4l2_subdev *subdev = notifier->sd;
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(subdev);

    csi2rx->source_pad = media_entity_get_fwnode_pad(&s_subdev->entity,
                                                    asd->match.fwnode,
                                                    MEDIA_PAD_FL_SOURCE);
    if (csi2rx->source_pad < 0) {
        dev_err(csi2rx->dev, "Couldn't find output pad for subdev %s\n",
                s_subdev->name);
        return csi2rx->source_pad;
    }
    csi2rx->source_subdev = s_subdev;
    dev_dbg(csi2rx->dev, "Bound %s pad: %d\n", s_subdev->name, csi2rx->source_pad);
    return media_create_pad_link(&csi2rx->source_subdev->entity,
                                 csi2rx->source_pad,
                                 &csi2rx->subdev.entity, 0,
                                 MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
}

static const struct v4l2_async_notifier_operations csi2rx_notifier_ops = {
    .bound = csi2rx_async_bound,
};

static int csi2rx_get_resources(struct csi2rx_priv *csi2rx,
                                struct platform_device *pdev)
{
    unsigned char i;
    u32 dev_cfg;
    int ret;

    csi2rx->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(csi2rx->base))
        return PTR_ERR(csi2rx->base);

    csi2rx->sys_clk = devm_clk_get(&pdev->dev, "sys_clk");
    if (IS_ERR(csi2rx->sys_clk))
        return PTR_ERR(csi2rx->sys_clk);

    csi2rx->p_clk = devm_clk_get(&pdev->dev, "p_clk");
    if (IS_ERR(csi2rx->p_clk))
        return PTR_ERR(csi2rx->p_clk);

    csi2rx->sys_rst = devm_reset_control_get_optional_exclusive(&pdev->dev, "sys");
    if (IS_ERR(csi2rx->sys_rst))
        return PTR_ERR(csi2rx->sys_rst);

    csi2rx->p_rst = devm_reset_control_get_optional_exclusive(&pdev->dev, "reg_bank");
    if (IS_ERR(csi2rx->p_rst))
        return PTR_ERR(csi2rx->p_rst);

    csi2rx->dphy = devm_phy_optional_get(&pdev->dev, "dphy");
    if (IS_ERR(csi2rx->dphy))
        return PTR_ERR(csi2rx->dphy);

    ret = clk_prepare_enable(csi2rx->p_clk);
    if (ret)
        return ret;

    dev_cfg = readl(csi2rx->base + CSI2RX_DEVICE_CFG_REG);
    clk_disable_unprepare(csi2rx->p_clk);

    csi2rx->max_lanes = dev_cfg & 7;
    if (csi2rx->max_lanes > CSI2RX_LANES_MAX)
        return -EINVAL;

    csi2rx->max_streams = (dev_cfg >> 4) & 7;
    if (csi2rx->max_streams > CSI2RX_STREAMS_MAX)
        return -EINVAL;

    csi2rx->has_internal_dphy = dev_cfg & BIT(3) ? true : false;
    if (!csi2rx->dphy && csi2rx->has_internal_dphy)
        return -EINVAL; // Internal DPHY not supported yet

    for (i = 0; i < csi2rx->max_streams; i++) {
        char name[16];
        snprintf(name, sizeof(name), "pixel_if%u_clk", i);
        csi2rx->pixel_clk[i] = devm_clk_get(&pdev->dev, name);
        if (IS_ERR(csi2rx->pixel_clk[i]))
            return PTR_ERR(csi2rx->pixel_clk[i]);
        snprintf(name, sizeof(name), "pixel_if%u", i);
        csi2rx->pixel_rst[i] = devm_reset_control_get_optional_exclusive(&pdev->dev, name);
        if (IS_ERR(csi2rx->pixel_rst[i]))
            return PTR_ERR(csi2rx->pixel_rst[i]);
    }
    return 0;
}

static int csi2rx_parse_dt(struct csi2rx_priv *csi2rx)
{
    struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = 0 };
    struct v4l2_async_subdev *asd;
    struct fwnode_handle *fwh;
    struct device_node *ep;
    int ret;

    ep = of_graph_get_endpoint_by_regs(csi2rx->dev->of_node, 0, 0);
    if (!ep)
        return -EINVAL;

    fwh = of_fwnode_handle(ep);
    ret = v4l2_fwnode_endpoint_parse(fwh, &v4l2_ep);
    if (ret) {
        of_node_put(ep);
        return ret;
    }

    if (v4l2_ep.bus_type != V4L2_MBUS_CSI2_DPHY)
        return -EINVAL;

    memcpy(csi2rx->lanes, v4l2_ep.bus.mipi_csi2.data_lanes, sizeof(csi2rx->lanes));
    csi2rx->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes; // Set to 2 in DT for IMX219
    if (csi2rx->num_lanes > csi2rx->max_lanes)
        return -EINVAL;

    v4l2_async_nf_init(&csi2rx->notifier);
    asd = v4l2_async_nf_add_fwnode_remote(&csi2rx->notifier, fwh,
                                          struct v4l2_async_subdev);
    of_node_put(ep);
    if (IS_ERR(asd))
        return PTR_ERR(asd);

    csi2rx->notifier.ops = &csi2rx_notifier_ops;
    ret = v4l2_async_nf_register(csi2rx->subdev.v4l2_dev, &csi2rx->notifier);
    if (ret)
        v4l2_async_nf_cleanup(&csi2rx->notifier);
    return ret;
}

static int csi2rx_probe(struct platform_device *pdev)
{
    struct csi2rx_priv *csi2rx;
    unsigned int i;
    int ret;

    csi2rx = kzalloc(sizeof(*csi2rx), GFP_KERNEL);
    if (!csi2rx)
        return -ENOMEM;
    platform_set_drvdata(pdev, csi2rx);
    csi2rx->dev = &pdev->dev;
    mutex_init(&csi2rx->lock);

    ret = csi2rx_get_resources(csi2rx, pdev);
    if (ret)
        goto err_free_priv;

    csi2rx->subdev.owner = THIS_MODULE;
    csi2rx->subdev.dev = &pdev->dev;
    v4l2_subdev_init(&csi2rx->subdev, &csi2rx_subdev_ops);
    csi2rx->subdev.internal_ops = &csi2rx_internal_ops;
    v4l2_set_subdevdata(&csi2rx->subdev, &pdev->dev);
    snprintf(csi2rx->subdev.name, sizeof(csi2rx->subdev.name),
             "%s.%s", KBUILD_MODNAME, dev_name(&pdev->dev));
    csi2rx->subdev.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
    csi2rx->pads[CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
    for (i = CSI2RX_PAD_SOURCE_STREAM0; i < CSI2RX_PAD_MAX; i++)
        csi2rx->pads[i].flags = MEDIA_PAD_FL_SOURCE;
    csi2rx->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
    csi2rx->subdev.entity.ops = &csi2rx_media_ops;

    ret = media_entity_pads_init(&csi2rx->subdev.entity, CSI2RX_PAD_MAX, csi2rx->pads);
    if (ret)
        goto err_cleanup;

    ret = v4l2_async_register_subdev(&csi2rx->subdev);
    if (ret < 0)
        goto err_cleanup;

    ret = csi2rx_parse_dt(csi2rx);
    if (ret)
        goto err_unregister;

    dev_info(&pdev->dev, "Probed CSI2RX with %u/%u lanes, %u streams, %s D-PHY\n",
             csi2rx->num_lanes, csi2rx->max_lanes, csi2rx->max_streams,
             csi2rx->dphy ? "external" : csi2rx->has_internal_dphy ? "internal" : "no");
    return 0;

err_unregister:
    v4l2_async_unregister_subdev(&csi2rx->subdev);
err_cleanup:
    media_entity_cleanup(&csi2rx->subdev.entity);
err_free_priv:
    kfree(csi2rx);
    return ret;
}

static int csi2rx_remove(struct platform_device *pdev)
{
    struct csi2rx_priv *csi2rx = platform_get_drvdata(pdev);
    v4l2_async_nf_unregister(&csi2rx->notifier);
    v4l2_async_nf_cleanup(&csi2rx->notifier);
    v4l2_async_unregister_subdev(&csi2rx->subdev);
    media_entity_cleanup(&csi2rx->subdev.entity);
    kfree(csi2rx);
    return 0;
}

static const struct of_device_id csi2rx_of_table[] = {
    { .compatible = "starfive,jh7110-csi2rx" },
    { .compatible = "cdns,csi2rx" },
    { },
};
MODULE_DEVICE_TABLE(of, csi2rx_of_table);

static struct platform_driver csi2rx_driver = {
    .probe  = csi2rx_probe,
    .remove = csi2rx_remove,
    .driver = {
        .name           = "cdns-csi2rx",
        .of_match_table = csi2rx_of_table,
    },
};
module_platform_driver(csi2rx_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@bootlin.com>");
MODULE_DESCRIPTION("Cadence CSI2-RX controller");
MODULE_LICENSE("GPL");
