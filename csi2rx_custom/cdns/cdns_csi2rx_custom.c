#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>

#define CSI2RX_DEVICE_CFG_REG 0x000
#define CSI2RX_SOFT_RESET_REG 0x004
#define CSI2RX_SOFT_RESET_PROTOCOL BIT(1)
#define CSI2RX_SOFT_RESET_FRONT BIT(0)
#define CSI2RX_STATIC_CFG_REG 0x008
#define CSI2RX_STATIC_CFG_DLANE_MAP(llane, plane) ((plane) << (16 + (llane) * 4))
#define CSI2RX_STREAM_BASE(n) (((n) + 1) * 0x100)
#define CSI2RX_STREAM_CTRL_REG(n) (CSI2RX_STREAM_BASE(n) + 0x000)
#define CSI2RX_STREAM_CTRL_START BIT(0)
#define CSI2RX_STREAM_DATA_CFG_REG(n) (CSI2RX_STREAM_BASE(n) + 0x008)
#define CSI2RX_STREAM_DATA_CFG_VC_SELECT(n) BIT((n) + 16)
#define CSI2RX_STREAM_CFG_REG(n) (CSI2RX_STREAM_BASE(n) + 0x00c)
#define CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF (1 << 8)
#define CSI2RX_LANES_MAX 4
#define CSI2RX_STREAMS_MAX 1

enum csi2rx_pads {
    CSI2RX_PAD_SINK,
    CSI2RX_PAD_SOURCE_STREAM0,
    CSI2RX_PAD_MAX,
};

struct csi2rx_fmt {
    u32 code;
    u8 bpp;
};

static const struct csi2rx_fmt formats[] = {
    { MEDIA_BUS_FMT_SRGGB8_1X8,  8  },
    { MEDIA_BUS_FMT_SGRBG8_1X8,  8  },
    { MEDIA_BUS_FMT_SGBRG8_1X8,  8  },
    { MEDIA_BUS_FMT_SBGGR8_1X8,  8  },
    { MEDIA_BUS_FMT_SRGGB10_1X10, 10 },
    { MEDIA_BUS_FMT_SGRBG10_1X10, 10 },
    { MEDIA_BUS_FMT_SGBRG10_1X10, 10 },
    { MEDIA_BUS_FMT_SBGGR10_1X10, 10 },
};

struct csi2rx_priv {
    struct device *dev;
    struct mutex lock;
    void __iomem *base;
    struct clk *sys_clk;
    struct clk *p_clk;
    struct clk *pixel_clk[CSI2RX_STREAMS_MAX];
    struct v4l2_subdev subdev;
    struct v4l2_async_notifier notifier;
    struct media_pad pads[CSI2RX_PAD_MAX];
    struct v4l2_subdev *source_subdev;
    int source_pad;
    u8 lanes[CSI2RX_LANES_MAX];
    u8 num_lanes;
    struct v4l2_mbus_framefmt format;
};

/* Format handling */
static const struct csi2rx_fmt *csi2rx_get_fmt_by_code(u32 code) {
    int i;
    for (i = 0; i < ARRAY_SIZE(formats); i++)
        if (formats[i].code == code)
            return &formats[i];
    return NULL;
}

static inline struct csi2rx_priv *v4l2_subdev_to_csi2rx(struct v4l2_subdev *subdev) {
    return container_of(subdev, struct csi2rx_priv, subdev);
}

/* Hardware control */
static void csi2rx_reset(struct csi2rx_priv *csi2rx) {
    writel(CSI2RX_SOFT_RESET_PROTOCOL | CSI2RX_SOFT_RESET_FRONT,
           csi2rx->base + CSI2RX_SOFT_RESET_REG);
    udelay(10);
    writel(0, csi2rx->base + CSI2RX_SOFT_RESET_REG);
}

static int csi2rx_start(struct csi2rx_priv *csi2rx) {
    u32 reg = 2 << 8; /* 2 lanes */
    int ret;

    ret = clk_prepare_enable(csi2rx->p_clk);
    if (ret)
        return ret;

    csi2rx_reset(csi2rx);
    reg |= CSI2RX_STATIC_CFG_DLANE_MAP(0, csi2rx->lanes[0]) |
           CSI2RX_STATIC_CFG_DLANE_MAP(1, csi2rx->lanes[1]);
    writel(reg, csi2rx->base + CSI2RX_STATIC_CFG_REG);

    ret = clk_prepare_enable(csi2rx->pixel_clk[0]);
    if (ret)
        goto err_pclk;
    writel(CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF, csi2rx->base + CSI2RX_STREAM_CFG_REG(0));
    writel(CSI2RX_STREAM_DATA_CFG_VC_SELECT(0), csi2rx->base + CSI2RX_STREAM_DATA_CFG_REG(0));
    writel(CSI2RX_STREAM_CTRL_START, csi2rx->base + CSI2RX_STREAM_CTRL_REG(0));

    ret = clk_prepare_enable(csi2rx->sys_clk);
    if (ret)
        goto err_pixel;
    ret = v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, true);
    if (ret)
        goto err_sysclk;

    clk_disable_unprepare(csi2rx->p_clk);
    return 0;

err_sysclk:
    clk_disable_unprepare(csi2rx->sys_clk);
err_pixel:
    clk_disable_unprepare(csi2rx->pixel_clk[0]);
err_pclk:
    clk_disable_unprepare(csi2rx->p_clk);
    return ret;
}

static void csi2rx_stop(struct csi2rx_priv *csi2rx) {
    clk_prepare_enable(csi2rx->p_clk);
    writel(0, csi2rx->base + CSI2RX_STREAM_CTRL_REG(0));
    clk_disable_unprepare(csi2rx->sys_clk);
    clk_disable_unprepare(csi2rx->pixel_clk[0]);
    clk_disable_unprepare(csi2rx->p_clk);
    v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, false);
}

/* V4L2 subdev operations */
static int csi2rx_s_stream(struct v4l2_subdev *subdev, int enable) {
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(subdev);
    int ret = 0;

    mutex_lock(&csi2rx->lock);
    if (enable) {
        if (!csi2rx->source_subdev) {
            ret = -ENODEV;
            goto out;
        }
        ret = csi2rx_start(csi2rx);
    } else {
        csi2rx_stop(csi2rx);
    }
out:
    mutex_unlock(&csi2rx->lock);
    return ret;
}

static int csi2rx_enum_mbus_code(struct v4l2_subdev *subdev, struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_mbus_code_enum *code) {
    if (code->index >= ARRAY_SIZE(formats))
        return -EINVAL;
    code->code = formats[code->index].code;
    return 0;
}

static int csi2rx_get_fmt(struct v4l2_subdev *subdev, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format) {
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(subdev);
    format->format = csi2rx->format;
    return 0;
}

static int csi2rx_set_fmt(struct v4l2_subdev *subdev, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *format) {
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(subdev);
    if (format->pad != CSI2RX_PAD_SINK)
        return csi2rx_get_fmt(subdev, state, format);
    if (!csi2rx_get_fmt_by_code(format->format.code))
        format->format.code = formats[0].code; /* Default SRGGB8 */
    format->format.field = V4L2_FIELD_NONE;
    csi2rx->format = format->format;
    return 0;
}

static const struct v4l2_subdev_pad_ops csi2rx_pad_ops = {
    .enum_mbus_code = csi2rx_enum_mbus_code,
    .get_fmt = csi2rx_get_fmt,
    .set_fmt = csi2rx_set_fmt,
};

static const struct v4l2_subdev_video_ops csi2rx_video_ops = {
    .s_stream = csi2rx_s_stream,
};

static const struct v4l2_subdev_ops csi2rx_subdev_ops = {
    .video = &csi2rx_video_ops,
    .pad = &csi2rx_pad_ops,
};

/* Async notifier operations */
static int csi2rx_notify_bound(struct v4l2_async_notifier *notifier,
                               struct v4l2_subdev *subdev,
                               struct v4l2_async_subdev *asd) {
    struct csi2rx_priv *csi2rx = container_of(notifier, struct csi2rx_priv, notifier);
    int source_pad;

    source_pad = media_entity_get_fwnode_pad(&subdev->entity, asd->match.fwnode,
                                             MEDIA_PAD_FL_SOURCE);
    if (source_pad < 0)
        return source_pad;

    csi2rx->source_subdev = subdev;
    csi2rx->source_pad = source_pad;
    return media_create_pad_link(&subdev->entity, source_pad,
                                 &csi2rx->subdev.entity, CSI2RX_PAD_SINK,
                                 MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
}

static const struct v4l2_async_notifier_operations csi2rx_notifier_ops = {
    .bound = csi2rx_notify_bound,
};

/* Device tree parsing and probe */
static int csi2rx_parse_dt(struct csi2rx_priv *csi2rx) {
    struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
    struct v4l2_async_subdev *asd;
    struct device_node *ep;
    struct fwnode_handle *fwh;
    int ret;

    ep = of_graph_get_endpoint_by_regs(csi2rx->dev->of_node, 0, 0);
    if (!ep)
        return -EINVAL;
    fwh = of_fwnode_handle(ep);
    ret = v4l2_fwnode_endpoint_parse(fwh, &v4l2_ep);
    if (ret || v4l2_ep.bus.mipi_csi2.num_data_lanes != 2) {
        dev_err(csi2rx->dev, "Requires exactly 2 lanes, got %d\n",
                v4l2_ep.bus.mipi_csi2.num_data_lanes);
        of_node_put(ep);
        return -EINVAL;
    }
    memcpy(csi2rx->lanes, v4l2_ep.bus.mipi_csi2.data_lanes, sizeof(csi2rx->lanes));
    csi2rx->num_lanes = 2;

    v4l2_async_nf_init(&csi2rx->notifier);
    asd = v4l2_async_nf_add_fwnode(&csi2rx->notifier, fwh, struct v4l2_async_subdev);
    of_node_put(ep);
    if (IS_ERR(asd)) {
        v4l2_async_nf_cleanup(&csi2rx->notifier);
        return PTR_ERR(asd);
    }
    csi2rx->notifier.ops = &csi2rx_notifier_ops;
    ret = v4l2_async_subdev_nf_register(&csi2rx->subdev, &csi2rx->notifier);
    if (ret)
        v4l2_async_nf_cleanup(&csi2rx->notifier);
    return ret;
}

static int csi2rx_probe(struct platform_device *pdev) {
    struct csi2rx_priv *csi2rx;
    int ret;

    csi2rx = kzalloc(sizeof(*csi2rx), GFP_KERNEL);
    if (!csi2rx)
        return -ENOMEM;
    platform_set_drvdata(pdev, csi2rx);
    csi2rx->dev = &pdev->dev;
    mutex_init(&csi2rx->lock);

    csi2rx->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(csi2rx->base)) {
        ret = PTR_ERR(csi2rx->base);
        goto err_free;
    }

    csi2rx->sys_clk = devm_clk_get(&pdev->dev, "sys_clk");
    if (IS_ERR(csi2rx->sys_clk)) {
        ret = PTR_ERR(csi2rx->sys_clk);
        goto err_free;
    }
    csi2rx->p_clk = devm_clk_get(&pdev->dev, "p_clk");
    if (IS_ERR(csi2rx->p_clk)) {
        ret = PTR_ERR(csi2rx->p_clk);
        goto err_free;
    }
    csi2rx->pixel_clk[0] = devm_clk_get(&pdev->dev, "pixel_if0_clk");
    if (IS_ERR(csi2rx->pixel_clk[0])) {
        ret = PTR_ERR(csi2rx->pixel_clk[0]);
        goto err_free;
    }

    /* Initialize default format */
    csi2rx->format.width = 1920;
    csi2rx->format.height = 1080;
    csi2rx->format.code = MEDIA_BUS_FMT_SRGGB8_1X8;
    csi2rx->format.field = V4L2_FIELD_NONE;
    csi2rx->format.colorspace = V4L2_COLORSPACE_SRGB;

    ret = csi2rx_parse_dt(csi2rx);
    if (ret)
        goto err_free;

    csi2rx->subdev.owner = THIS_MODULE;
    csi2rx->subdev.dev = &pdev->dev;
    v4l2_subdev_init(&csi2rx->subdev, &csi2rx_subdev_ops);
    snprintf(csi2rx->subdev.name, sizeof(csi2rx->subdev.name), "cdns-csi2rx-imx219");
    csi2rx->subdev.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
    csi2rx->pads[CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
    csi2rx->pads[CSI2RX_PAD_SOURCE_STREAM0].flags = MEDIA_PAD_FL_SOURCE;
    csi2rx->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    ret = media_entity_pads_init(&csi2rx->subdev.entity, CSI2RX_PAD_MAX, csi2rx->pads);
    if (ret)
        goto err_cleanup;
    ret = v4l2_async_register_subdev(&csi2rx->subdev);
    if (ret < 0)
        goto err_cleanup;

    return 0;

err_cleanup:
    v4l2_async_nf_unregister(&csi2rx->notifier);
    v4l2_async_nf_cleanup(&csi2rx->notifier);
    media_entity_cleanup(&csi2rx->subdev.entity);
err_free:
    kfree(csi2rx);
    return ret;
}

static int csi2rx_remove(struct platform_device *pdev) {
    struct csi2rx_priv *csi2rx = platform_get_drvdata(pdev);

    v4l2_async_nf_unregister(&csi2rx->notifier);
    v4l2_async_nf_cleanup(&csi2rx->notifier);
    v4l2_async_unregister_subdev(&csi2rx->subdev);
    media_entity_cleanup(&csi2rx->subdev.entity);
    mutex_destroy(&csi2rx->lock);
    kfree(csi2rx);
    return 0;
}

static const struct of_device_id csi2rx_of_table[] = {
    { .compatible = "cdns,csi2rx-imx219" },
    { },
};
MODULE_DEVICE_TABLE(of, csi2rx_of_table);

static struct platform_driver csi2rx_driver = {
    .probe = csi2rx_probe,
    .remove = csi2rx_remove,
    .driver = {
        .name = "cdns-csi2rx_custom",
        .of_match_table = csi2rx_of_table,
    },
};
module_platform_driver(csi2rx_driver);
MODULE_DESCRIPTION("Cadence CSI2RX Driver for Custom IMX219");
MODULE_LICENSE("GPL v2");