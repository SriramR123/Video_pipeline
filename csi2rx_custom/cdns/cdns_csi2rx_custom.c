#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <linux/iopoll.h>

/* Register Definitions */
#define CSI2RX_DEVICE_CFG_REG           0x000
#define CSI2RX_SOFT_RESET_REG           0x004
#define CSI2RX_SOFT_RESET_PROTOCOL      BIT(1)
#define CSI2RX_SOFT_RESET_FRONT         BIT(0)
#define CSI2RX_STATIC_CFG_REG           0x008
#define CSI2RX_STATIC_CFG_DLANE_MAP(llane, plane) ((plane) << (16 + (llane) * 4))
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
#define CSI2RX_STREAM_DATA_CFG_VC_ALL   0
#define CSI2RX_STREAM_CFG_REG(n)        (CSI2RX_STREAM_BASE(n) + 0x00c)
#define CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF (1 << 8)

#define CSI2RX_LANES_MAX    4
#define CSI2RX_STREAMS_MAX  1 /* Single stream for IMX219 */

/* Pad Definitions */
enum csi2rx_pads {
    CSI2RX_PAD_SINK,
    CSI2RX_PAD_SOURCE_STREAM0,
    CSI2RX_PAD_MAX,
};

/* Supported Formats */
struct csi2rx_fmt {
    u32 code;
    u8 bpp;
};

static const struct csi2rx_fmt formats[] = {
    { .code = MEDIA_BUS_FMT_SRGGB8_1X8,   .bpp = 8  },
    { .code = MEDIA_BUS_FMT_SGRBG8_1X8,   .bpp = 8  },
    { .code = MEDIA_BUS_FMT_SGBRG8_1X8,   .bpp = 8  },
    { .code = MEDIA_BUS_FMT_SBGGR8_1X8,   .bpp = 8  },
    { .code = MEDIA_BUS_FMT_SRGGB10_1X10, .bpp = 10 },
    { .code = MEDIA_BUS_FMT_SGRBG10_1X10, .bpp = 10 },
    { .code = MEDIA_BUS_FMT_SGBRG10_1X10, .bpp = 10 },
    { .code = MEDIA_BUS_FMT_SBGGR10_1X10, .bpp = 10 },
};

/* Private Driver Structure */
struct csi2rx_priv {
    struct device           *dev;
    struct v4l2_subdev      sd;
    struct media_pad        pads[CSI2RX_PAD_MAX];
    struct v4l2_mbus_framefmt fmt;
    struct mutex            lock;
    unsigned int            count;

    void __iomem            *base;
    struct clk              *sys_clk;
    struct clk              *p_clk;
    struct clk              *pixel_clk[CSI2RX_STREAMS_MAX];
    struct phy              *dphy;

    u8                      lanes[CSI2RX_LANES_MAX];
    u8                      num_lanes;
    u8                      max_lanes;
    u8                      max_streams;
    bool                    has_internal_dphy;

    struct v4l2_async_notifier notifier;
    struct v4l2_subdev      *source_subdev; /* IMX219 */
    int                     source_pad;
};

/* Helper Functions */
static inline struct csi2rx_priv *v4l2_subdev_to_csi2rx(struct v4l2_subdev *subdev)
{
    return container_of(subdev, struct csi2rx_priv, sd);
}

static const struct csi2rx_fmt *csi2rx_get_fmt_by_code(u32 code)
{
    for (unsigned int i = 0; i < ARRAY_SIZE(formats); i++)
        if (formats[i].code == code)
            return &formats[i];
    return &formats[0]; /* Default to SRGGB8_1X8 */
}

static void csi2rx_reset(struct csi2rx_priv *csi2rx)
{
    writel(CSI2RX_SOFT_RESET_PROTOCOL | CSI2RX_SOFT_RESET_FRONT,
           csi2rx->base + CSI2RX_SOFT_RESET_REG);
    udelay(10);
    writel(0, csi2rx->base + CSI2RX_SOFT_RESET_REG);

    for (unsigned int i = 0; i < csi2rx->max_streams; i++) {
        writel(CSI2RX_STREAM_CTRL_SOFT_RST, csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
        usleep_range(10, 20);
        writel(0, csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
    }
}

/* Power Management */
static int csi2rx_power_on(struct device *dev)
{
    struct csi2rx_priv *csi2rx = dev_get_drvdata(dev);
    int ret;

    ret = clk_prepare_enable(csi2rx->sys_clk);
    if (ret) {
        dev_err(dev, "Failed to enable sys_clk: %d\n", ret);
        return ret;
    }

    ret = clk_prepare_enable(csi2rx->p_clk);
    if (ret) {
        dev_err(dev, "Failed to enable p_clk: %d\n", ret);
        clk_disable_unprepare(csi2rx->sys_clk);
        return ret;
    }

    if (csi2rx->dphy) {
        ret = phy_power_on(csi2rx->dphy);
        if (ret) {
            dev_err(dev, "Failed to power on DPHY: %d\n", ret);
            clk_disable_unprepare(csi2rx->p_clk);
            clk_disable_unprepare(csi2rx->sys_clk);
            return ret;
        }
    }

    usleep_range(10000, 12000); /* Stabilization delay */
    return 0;
}

static int csi2rx_power_off(struct device *dev)
{
    struct csi2rx_priv *csi2rx = dev_get_drvdata(dev);

    if (csi2rx->dphy)
        phy_power_off(csi2rx->dphy);
    clk_disable_unprepare(csi2rx->p_clk);
    clk_disable_unprepare(csi2rx->sys_clk);
    return 0;
}

/* Streaming Functions */
static int csi2rx_start(struct csi2rx_priv *csi2rx)
{
    unsigned int i;
    unsigned long lanes_used = 0;
    u32 reg;
    int ret;

    ret = clk_prepare_enable(csi2rx->p_clk);
    if (ret)
        return ret;

    csi2rx_reset(csi2rx);

    reg = csi2rx->num_lanes << 8;
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
    }

    for (i = 0; i < csi2rx->max_streams; i++) {
        ret = clk_prepare_enable(csi2rx->pixel_clk[i]);
        if (ret)
            goto err_disable_pixclk;

        writel(CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF, csi2rx->base + CSI2RX_STREAM_CFG_REG(i));
        writel(CSI2RX_STREAM_DATA_CFG_VC_ALL, csi2rx->base + CSI2RX_STREAM_DATA_CFG_REG(i));
        writel(CSI2RX_STREAM_CTRL_START, csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
    }

    clk_disable_unprepare(csi2rx->p_clk);
    return 0;

err_disable_pixclk:
    for (; i > 0; i--)
        clk_disable_unprepare(csi2rx->pixel_clk[i - 1]);
    if (csi2rx->dphy)
        writel(0, csi2rx->base + CSI2RX_DPHY_LANE_CTRL_REG);
    clk_disable_unprepare(csi2rx->p_clk);
    return ret;
}

static void csi2rx_stop(struct csi2rx_priv *csi2rx)
{
    unsigned int i;
    u32 val;
    int ret;

    clk_prepare_enable(csi2rx->p_clk);
    for (i = 0; i < csi2rx->max_streams; i++) {
        writel(CSI2RX_STREAM_CTRL_STOP, csi2rx->base + CSI2RX_STREAM_CTRL_REG(i));
        ret = readl_poll_timeout(csi2rx->base + CSI2RX_STREAM_STATUS_REG(i), val,
                                 !(val & CSI2RX_STREAM_STATUS_RDY), 10, 10000);
        if (ret)
            dev_warn(csi2rx->dev, "Failed to stop stream %u\n", i);
        clk_disable_unprepare(csi2rx->pixel_clk[i]);
    }
    clk_disable_unprepare(csi2rx->p_clk);

    if (csi2rx->dphy)
        writel(0, csi2rx->base + CSI2RX_DPHY_LANE_CTRL_REG);
}

static int csi2rx_s_stream(struct v4l2_subdev *sd, int enable)
{
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(sd);
    int ret = 0;

    mutex_lock(&csi2rx->lock);
    if (enable) {
        if (!csi2rx->count) {
            ret = pm_runtime_resume_and_get(csi2rx->dev);
            if (ret < 0)
                goto out;

            /* Start streaming on the source (IMX219) */
            if (csi2rx->source_subdev) {
                ret = v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, 1);
                if (ret) {
                    dev_err(csi2rx->dev, "Failed to start IMX219: %d\n", ret);
                    pm_runtime_put(csi2rx->dev);
                    goto out;
                }
            }

            ret = csi2rx_start(csi2rx);
            if (ret) {
                v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, 0);
                pm_runtime_put(csi2rx->dev);
                goto out;
            }
        }
        csi2rx->count++;
    } else {
        csi2rx->count--;
        if (!csi2rx->count) {
            csi2rx_stop(csi2rx);
            if (csi2rx->source_subdev)
                v4l2_subdev_call(csi2rx->source_subdev, video, s_stream, 0);
            pm_runtime_put(csi2rx->dev);
        }
    }
out:
    mutex_unlock(&csi2rx->lock);
    return ret;
}

/* Format Handling */
static int csi2rx_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)
{
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(sd);

    mutex_lock(&csi2rx->lock);
    fmt->format = csi2rx->fmt;
    mutex_unlock(&csi2rx->lock);
    return 0;
}

static int csi2rx_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                          struct v4l2_subdev_format *fmt)
{
    struct csi2rx_priv *csi2rx = v4l2_subdev_to_csi2rx(sd);

    mutex_lock(&csi2rx->lock);
    if (fmt->pad == CSI2RX_PAD_SOURCE_STREAM0)
        fmt->format = csi2rx->fmt; /* Source reflects sink */
    else if (fmt->pad == CSI2RX_PAD_SINK) {
        if (!csi2rx_get_fmt_by_code(fmt->format.code))
            fmt->format.code = formats[0].code;
        fmt->format.field = V4L2_FIELD_NONE;
        csi2rx->fmt = fmt->format;
    } else {
        mutex_unlock(&csi2rx->lock);
        return -EINVAL;
    }
    mutex_unlock(&csi2rx->lock);
    return 0;
}

/* V4L2 Subdev Operations */
static const struct v4l2_subdev_pad_ops csi2rx_pad_ops = {
    .get_fmt = csi2rx_get_fmt,
    .set_fmt = csi2rx_set_fmt,
};

static const struct v4l2_subdev_video_ops csi2rx_video_ops = {
    .s_stream = csi2rx_s_stream,
};

static const struct v4l2_subdev_ops csi2rx_subdev_ops = {
    .pad = &csi2rx_pad_ops,
    .video = &csi2rx_video_ops,
};

/* Media Entity Operations */
static const struct media_entity_operations csi2rx_media_ops = {
    .link_validate = v4l2_subdev_link_validate,
};

/* Async Notifier Callbacks */
static int csi2rx_async_bound(struct v4l2_async_notifier *notifier,
                              struct v4l2_subdev *s_subdev,
                              struct v4l2_async_subdev *asd)
{
    struct csi2rx_priv *csi2rx = container_of(notifier, struct csi2rx_priv, notifier);
    int ret;

    csi2rx->source_pad = media_entity_get_fwnode_pad(&s_subdev->entity,
                                                     asd->match.fwnode,
                                                     MEDIA_PAD_FL_SOURCE);
    if (csi2rx->source_pad < 0) {
        dev_err(csi2rx->dev, "Couldn't find output pad for subdev %s\n", s_subdev->name);
        return csi2rx->source_pad;
    }

    csi2rx->source_subdev = s_subdev;
    dev_info(csi2rx->dev, "Bound to source subdev %s\n", s_subdev->name);

    /* Link IMX219 source to CDNS sink */
    ret = media_create_pad_link(&s_subdev->entity, csi2rx->source_pad,
                                &csi2rx->sd.entity, CSI2RX_PAD_SINK,
                                MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
    if (ret) {
        dev_err(csi2rx->dev, "Failed to create link from IMX219 to CDNS\n");
        return ret;
    }

    /* Link CDNS source to J721E sink (assumed as parent in device tree) */
    struct v4l2_subdev *parent_sd = v4l2_get_subdev_hostdata(&csi2rx->sd);
    if (parent_sd) {
        ret = media_create_pad_link(&csi2rx->sd.entity, CSI2RX_PAD_SOURCE_STREAM0,
                                    &parent_sd->entity, 0,
                                    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
        if (ret)
            dev_err(csi2rx->dev, "Failed to link CDNS to J721E\n");
    } else {
        dev_warn(csi2rx->dev, "Parent J721E subdev not found\n");
    }

    return ret;
}

static const struct v4l2_async_notifier_operations csi2rx_async_ops = {
    .bound = csi2rx_async_bound,
};

/* Resource Acquisition */
static int csi2rx_get_resources(struct csi2rx_priv *csi2rx, struct platform_device *pdev)
{
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

    csi2rx->dphy = devm_phy_optional_get(&pdev->dev, "dphy");
    if (IS_ERR(csi2rx->dphy))
        return PTR_ERR(csi2rx->dphy);

    ret = clk_prepare_enable(csi2rx->p_clk);
    if (ret)
        return ret;

    dev_cfg = readl(csi2rx->base + CSI2RX_DEVICE_CFG_REG);
    clk_disable_unprepare(csi2rx->p_clk);

    csi2rx->max_lanes = dev_cfg & 7;
    csi2rx->max_streams = (dev_cfg >> 4) & 7;
    csi2rx->has_internal_dphy = dev_cfg & BIT(3);

    if (csi2rx->max_lanes > CSI2RX_LANES_MAX || csi2rx->max_streams > CSI2RX_STREAMS_MAX)
        return -EINVAL;

    csi2rx->max_streams = 1; /* Force single stream for IMX219 */

    for (unsigned int i = 0; i < csi2rx->max_streams; i++) {
        char clk_name[16];
        snprintf(clk_name, sizeof(clk_name), "pixel_if%u_clk", i);
        csi2rx->pixel_clk[i] = devm_clk_get(&pdev->dev, clk_name);
        if (IS_ERR(csi2rx->pixel_clk[i]))
            return PTR_ERR(csi2rx->pixel_clk[i]);
    }

    return 0;
}

/* Device Tree Parsing */
static int csi2rx_parse_dt(struct csi2rx_priv *csi2rx)
{
    struct v4l2_fwnode_endpoint v4l2_ep = { .bus_type = 0 };
    struct device_node *ep;
    struct v4l2_async_subdev *asd;
    int ret;

    ep = of_graph_get_endpoint_by_regs(csi2rx->dev->of_node, 0, 0);
    if (!ep)
        return -EINVAL;

    ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep), &v4l2_ep);
    if (ret)
        goto out;

    if (v4l2_ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
        ret = -EINVAL;
        goto out;
    }

    memcpy(csi2rx->lanes, v4l2_ep.bus.mipi_csi2.data_lanes, sizeof(csi2rx->lanes));
    csi2rx->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;

    v4l2_async_nf_init(&csi2rx->notifier);
    asd = v4l2_async_nf_add_fwnode(&csi2rx->notifier, of_fwnode_handle(ep),
                                   struct v4l2_async_subdev);
    if (IS_ERR(asd)) {
        ret = PTR_ERR(asd);
        v4l2_async_nf_cleanup(&csi2rx->notifier);
        goto out;
    }

    csi2rx->notifier.ops = &csi2rx_async_ops;
    ret = v4l2_async_subdev_nf_register(&csi2rx->sd, &csi2rx->notifier);
    if (ret)
        v4l2_async_nf_cleanup(&csi2rx->notifier);

out:
    of_node_put(ep);
    return ret;
}

/* Probe and Remove */
static int csi2rx_probe(struct platform_device *pdev)
{
    struct csi2rx_priv *csi2rx;
    int ret;

    csi2rx = devm_kzalloc(&pdev->dev, sizeof(*csi2rx), GFP_KERNEL);
    if (!csi2rx)
        return -ENOMEM;

    csi2rx->dev = &pdev->dev;
    platform_set_drvdata(pdev, csi2rx);
    mutex_init(&csi2rx->lock);

    ret = csi2rx_get_resources(csi2rx, pdev);
    if (ret)
        goto err_free;

    csi2rx->fmt.width = 1920;
    csi2rx->fmt.height = 1080;
    csi2rx->fmt.code = MEDIA_BUS_FMT_SRGGB8_1X8;
    csi2rx->fmt.field = V4L2_FIELD_NONE;
    csi2rx->fmt.colorspace = V4L2_COLORSPACE_SRGB;

    v4l2_subdev_init(&csi2rx->sd, &csi2rx_subdev_ops);
    csi2rx->sd.owner = THIS_MODULE;
    csi2rx->sd.dev = &pdev->dev;
    snprintf(csi2rx->sd.name, sizeof(csi2rx->sd.name), "cdns-csi2rx.%s", dev_name(&pdev->dev));

    csi2rx->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
    csi2rx->pads[CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
    csi2rx->pads[CSI2RX_PAD_SOURCE_STREAM0].flags = MEDIA_PAD_FL_SOURCE;
    csi2rx->sd.entity.ops = &csi2rx_media_ops;
    csi2rx->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

    ret = media_entity_pads_init(&csi2rx->sd.entity, CSI2RX_PAD_MAX, csi2rx->pads);
    if (ret)
        goto err_free;

    ret = csi2rx_parse_dt(csi2rx);
    if (ret)
        goto err_cleanup;

    ret = csi2rx_power_on(&pdev->dev);
    if (ret)
        goto err_cleanup;

    pm_runtime_set_active(&pdev->dev);
    pm_runtime_enable(&pdev->dev);
    pm_runtime_idle(&pdev->dev);

    ret = v4l2_async_register_subdev(&csi2rx->sd);
    if (ret)
        goto err_pm;

    dev_info(&pdev->dev, "CSI2RX probed with %u lanes\n", csi2rx->num_lanes);
    return 0;

err_pm:
    pm_runtime_disable(&pdev->dev);
err_cleanup:
    media_entity_cleanup(&csi2rx->sd.entity);
err_free:
    mutex_destroy(&csi2rx->lock);
    return ret;
}

static int csi2rx_remove(struct platform_device *pdev)
{
    struct csi2rx_priv *csi2rx = platform_get_drvdata(pdev);

    v4l2_async_unregister_subdev(&csi2rx->sd);
    v4l2_async_nf_cleanup(&csi2rx->notifier);
    media_entity_cleanup(&csi2rx->sd.entity);
    pm_runtime_disable(&pdev->dev);
    if (!pm_runtime_status_suspended(&pdev->dev))
        csi2rx_power_off(&pdev->dev);
    pm_runtime_set_suspended(&pdev->dev);
    mutex_destroy(&csi2rx->lock);
    return 0;
}

/* PM Operations */
static const struct dev_pm_ops csi2rx_pm_ops = {
    SET_RUNTIME_PM_OPS(csi2rx_power_off, csi2rx_power_on, NULL)
};

/* Device Tree Match Table */
static const struct of_device_id csi2rx_of_table[] = {
    { .compatible = "cdns,csi2rx" },
    { }
};
MODULE_DEVICE_TABLE(of, csi2rx_of_table);

/* Platform Driver */
static struct platform_driver csi2rx_driver = {
    .probe  = csi2rx_probe,
    .remove = csi2rx_remove,
    .driver = {
        .name           = "cdns-csi2rx-custom",
        .of_match_table = csi2rx_of_table,
        .pm             = &csi2rx_pm_ops,
    },
};

module_platform_driver(csi2rx_driver);

MODULE_DESCRIPTION("Custom Cadence CSI2-RX Controller Driver for IMX219");
MODULE_AUTHOR("Your Name");
MODULE_LICENSE("GPL v2");