#include <linux/bitfield.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

/*
 * TI J721E CSI-2 RX Driver
 * Compatible with custom IMX219 driver with Bayer formats and 2-lane mode
 */

#define TI_CSI2RX_MODULE_NAME       "j721e-csi2rx"
#define SHIM_CNTL                   0x10
#define SHIM_CNTL_PIX_RST           BIT(0)
#define SHIM_DMACNTX                0x20
#define SHIM_DMACNTX_EN             BIT(31)
#define SHIM_DMACNTX_YUV422         GENMASK(27, 26)
#define SHIM_DMACNTX_SIZE           GENMASK(21, 20)
#define SHIM_DMACNTX_FMT            GENMASK(5, 0)
#define SHIM_DMACNTX_YUV422_MODE_11 3
#define SHIM_DMACNTX_SIZE_8         0
#define SHIM_DMACNTX_SIZE_16        1
#define SHIM_DMACNTX_SIZE_32        2
#define SHIM_PSI_CFG0               0x24
#define SHIM_PSI_CFG0_SRC_TAG       GENMASK(15, 0)
#define SHIM_PSI_CFG0_DST_TAG       GENMASK(31, 16)

#define PSIL_WORD_SIZE_BYTES        16
#define MAX_WIDTH_BYTES             SZ_16K
#define MAX_HEIGHT_LINES            SZ_16K
#define DRAIN_TIMEOUT_MS            50
#define DRAIN_BUFFER_SIZE           SZ_32K

struct ti_csi2rx_fmt {
    u32 fourcc;
    u32 code;    // Media bus code from IMX219
    u32 csi_dt;  // CSI-2 data type
    u8  bpp;
    u8  size;
};

/* Format array compatible with IMX219 supported_codes */
static const struct ti_csi2rx_fmt ti_csi2rx_formats[] = {
    { .fourcc = V4L2_PIX_FMT_YUYV, .code = MEDIA_BUS_FMT_YUYV8_1X16, .csi_dt = MIPI_CSI2_DT_YUV422_8B, .bpp = 16, .size = SHIM_DMACNTX_SIZE_8 },
    { .fourcc = V4L2_PIX_FMT_UYVY, .code = MEDIA_BUS_FMT_UYVY8_1X16, .csi_dt = MIPI_CSI2_DT_YUV422_8B, .bpp = 16, .size = SHIM_DMACNTX_SIZE_8 },
    { .fourcc = V4L2_PIX_FMT_YVYU, .code = MEDIA_BUS_FMT_YVYU8_1X16, .csi_dt = MIPI_CSI2_DT_YUV422_8B, .bpp = 16, .size = SHIM_DMACNTX_SIZE_8 },
    { .fourcc = V4L2_PIX_FMT_VYUY, .code = MEDIA_BUS_FMT_VYUY8_1X16, .csi_dt = MIPI_CSI2_DT_YUV422_8B, .bpp = 16, .size = SHIM_DMACNTX_SIZE_8 },
    { .fourcc = V4L2_PIX_FMT_SBGGR8, .code = MEDIA_BUS_FMT_SBGGR8_1X8, .csi_dt = MIPI_CSI2_DT_RAW8, .bpp = 8, .size = SHIM_DMACNTX_SIZE_8 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_SGBRG8, .code = MEDIA_BUS_FMT_SGBRG8_1X8, .csi_dt = MIPI_CSI2_DT_RAW8, .bpp = 8, .size = SHIM_DMACNTX_SIZE_8 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_SGRBG8, .code = MEDIA_BUS_FMT_SGRBG8_1X8, .csi_dt = MIPI_CSI2_DT_RAW8, .bpp = 8, .size = SHIM_DMACNTX_SIZE_8 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_SRGGB8, .code = MEDIA_BUS_FMT_SRGGB8_1X8, .csi_dt = MIPI_CSI2_DT_RAW8, .bpp = 8, .size = SHIM_DMACNTX_SIZE_8 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_GREY,  .code = MEDIA_BUS_FMT_Y8_1X8,     .csi_dt = MIPI_CSI2_DT_RAW8, .bpp = 8, .size = SHIM_DMACNTX_SIZE_8 },
    { .fourcc = V4L2_PIX_FMT_SBGGR10, .code = MEDIA_BUS_FMT_SBGGR10_1X10, .csi_dt = MIPI_CSI2_DT_RAW10, .bpp = 16, .size = SHIM_DMACNTX_SIZE_16 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_SGBRG10, .code = MEDIA_BUS_FMT_SGBRG10_1X10, .csi_dt = MIPI_CSI2_DT_RAW10, .bpp = 16, .size = SHIM_DMACNTX_SIZE_16 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_SGRBG10, .code = MEDIA_BUS_FMT_SGRBG10_1X10, .csi_dt = MIPI_CSI2_DT_RAW10, .bpp = 16, .size = SHIM_DMACNTX_SIZE_16 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_SRGGB10, .code = MEDIA_BUS_FMT_SRGGB10_1X10, .csi_dt = MIPI_CSI2_DT_RAW10, .bpp = 16, .size = SHIM_DMACNTX_SIZE_16 }, // Matches IMX219
    { .fourcc = V4L2_PIX_FMT_RGB565X, .code = MEDIA_BUS_FMT_RGB565_1X16, .csi_dt = MIPI_CSI2_DT_RGB565, .bpp = 16, .size = SHIM_DMACNTX_SIZE_16 },
    { .fourcc = V4L2_PIX_FMT_XBGR32,  .code = MEDIA_BUS_FMT_RGB888_1X24, .csi_dt = MIPI_CSI2_DT_RGB888, .bpp = 32, .size = SHIM_DMACNTX_SIZE_32 },
    { .fourcc = V4L2_PIX_FMT_RGBX32,  .code = MEDIA_BUS_FMT_BGR888_1X24, .csi_dt = MIPI_CSI2_DT_RGB888, .bpp = 32, .size = SHIM_DMACNTX_SIZE_32 },
};

struct ti_csi2rx_buffer {
    struct vb2_v4l2_buffer  vb;
    struct list_head        list;
    struct ti_csi2rx_dev    *csi;
};

enum ti_csi2rx_dma_state {
    TI_CSI2RX_DMA_STOPPED,
    TI_CSI2RX_DMA_IDLE,
    TI_CSI2RX_DMA_ACTIVE,
};

struct ti_csi2rx_dma {
    spinlock_t              lock;
    struct dma_chan         *chan;
    struct list_head        queue;
    enum ti_csi2rx_dma_state state;
    struct list_head        submitted;
    struct {
        void                *vaddr;
        dma_addr_t          paddr;
        size_t              len;
    } drain;
};

struct ti_csi2rx_dev {
    struct device           *dev;
    void __iomem            *shim;
    struct v4l2_device      v4l2_dev;
    struct video_device     vdev;
    struct media_device     mdev;
    struct media_pipeline   pipe;
    struct media_pad        pad;
    struct v4l2_async_notifier notifier;
    struct v4l2_subdev      *source; // Links to IMX219 subdev
    struct vb2_queue        vidq;
    struct mutex            mutex;
    struct v4l2_format      v_fmt;
    struct ti_csi2rx_dma    dma;
    u32                     sequence;
};

static const struct ti_csi2rx_fmt *find_format_by_fourcc(u32 pixelformat)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(ti_csi2rx_formats); i++)
        if (ti_csi2rx_formats[i].fourcc == pixelformat)
            return &ti_csi2rx_formats[i];
    return NULL;
}

static const struct ti_csi2rx_fmt *find_format_by_code(u32 code)
{
    unsigned int i;
    for (i = 0; i < ARRAY_SIZE(ti_csi2rx_formats); i++)
        if (ti_csi2rx_formats[i].code == code)
            return &ti_csi2rx_formats[i];
    return NULL;
}

static void ti_csi2rx_fill_fmt(const struct ti_csi2rx_fmt *csi_fmt,
                               struct v4l2_format *v4l2_fmt)
{
    struct v4l2_pix_format *pix = &v4l2_fmt->fmt.pix;
    unsigned int pixels_in_word;

    pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / csi_fmt->bpp;
    pix->width = clamp_t(unsigned int, pix->width, pixels_in_word,
                         MAX_WIDTH_BYTES * 8 / csi_fmt->bpp);
    pix->height = clamp_t(unsigned int, pix->height, 1, MAX_HEIGHT_LINES);
    pix->width = rounddown(pix->width, pixels_in_word);

    v4l2_fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    pix->pixelformat = csi_fmt->fourcc;
    pix->bytesperline = pix->width * (csi_fmt->bpp / 8);
    pix->sizeimage = pix->bytesperline * pix->height;
}

static int ti_csi2rx_querycap(struct file *file, void *priv,
                              struct v4l2_capability *cap)
{
    strscpy(cap->driver, TI_CSI2RX_MODULE_NAME, sizeof(cap->driver));
    strscpy(cap->card, TI_CSI2RX_MODULE_NAME, sizeof(cap->card));
    return 0;
}

static int ti_csi2rx_enum_fmt_vid_cap(struct file *file, void *priv,
                                      struct v4l2_fmtdesc *f)
{
    const struct ti_csi2rx_fmt *fmt = NULL;

    if (f->mbus_code) {
        if (f->index > 0)
            return -EINVAL;
        fmt = find_format_by_code(f->mbus_code);
    } else {
        if (f->index >= ARRAY_SIZE(ti_csi2rx_formats))
            return -EINVAL;
        fmt = &ti_csi2rx_formats[f->index];
    }
    if (!fmt)
        return -EINVAL;

    f->pixelformat = fmt->fourcc;
    memset(f->reserved, 0, sizeof(f->reserved));
    f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return 0;
}

static int ti_csi2rx_g_fmt_vid_cap(struct file *file, void *prov,
                                   struct v4l2_format *f)
{
    struct ti_csi2rx_dev *csi = video_drvdata(file);
    *f = csi->v_fmt;
    return 0;
}

static int ti_csi2rx_try_fmt_vid_cap(struct file *file, void *priv,
                                     struct v4l2_format *f)
{
    const struct ti_csi2rx_fmt *fmt;

    fmt = find_format_by_fourcc(f->fmt.pix.pixelformat);
    if (!fmt)
        fmt = &ti_csi2rx_formats[0];
    f->fmt.pix.field = V4L2_FIELD_NONE;
    ti_csi2rx_fill_fmt(fmt, f);
    return 0;
}

static int ti_csi2rx_s_fmt_vid_cap(struct file *file, void *priv,
                                   struct v4l2_format *f)
{
    struct ti_csi2rx_dev *csi = video_drvdata(file);
    struct vb2_queue *q = &csi->vidq;
    int ret;

    if (vb2_is_busy(q))
        return -EBUSY;

    ret = ti_csi2rx_try_fmt_vid_cap(file, priv, f);
    if (ret < 0)
        return ret;

    csi->v_fmt = *f;
    return 0;
}

static int ti_csi2rx_enum_framesizes(struct file *file, void *fh,
                                     struct v4l2_frmsizeenum *fsize)
{
    const struct ti_csi2rx_fmt *fmt;
    unsigned int pixels_in_word;

    fmt = find_format_by_fourcc(fsize->pixel_format);
    if (!fmt || fsize->index != 0)
        return -EINVAL;

    pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / fmt->bpp;
    fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
    fsize->stepwise.min_width = pixels_in_word;
    fsize->stepwise.max_width = rounddown(MAX_WIDTH_BYTES * 8 / fmt->bpp, pixels_in_word);
    fsize->stepwise.step_width = pixels_in_word;
    fsize->stepwise.min_height = 1;
    fsize->stepwise.max_height = MAX_HEIGHT_LINES;
    fsize->stepwise.step_height = 1;
    return 0;
}

static const struct v4l2_ioctl_ops csi_ioctl_ops = {
    .vidioc_querycap      = ti_csi2rx_querycap,
    .vidioc_enum_fmt_vid_cap = ti_csi2rx_enum_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap = ti_csi2rx_try_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap = ti_csi2rx_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = ti_csi2rx_s_fmt_vid_cap,
    .vidioc_enum_framesizes = ti_csi2rx_enum_framesizes,
    .vidioc_reqbufs       = vb2_ioctl_reqbufs,
    .vidioc_create_bufs   = vb2_ioctl_create_bufs,
    .vidioc_prepare_buf   = vb2_ioctl_prepare_buf,
    .vidioc_querybuf      = vb2_ioctl_querybuf,
    .vidioc_qbuf          = vb2_ioctl_qbuf,
    .vidioc_dqbuf         = vb2_ioctl_dqbuf,
    .vidioc_expbuf        = vb2_ioctl_expbuf,
    .vidioc_streamon      = vb2_ioctl_streamon,
    .vidioc_streamoff     = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations csi_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
    .read = vb2_fop_read,
    .poll = vb2_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = vb2_fop_mmap,
};

static int csi_async_notifier_bound(struct v4l2_async_notifier *notifier,
                                    struct v4l2_subdev *subdev,
                                    struct v4l2_async_connection *asc)
{
    struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
    csi->source = subdev; // Binds to IMX219
    return 0;
}

static int csi_async_notifier_complete(struct v4l2_async_notifier *notifier)
{
    struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
    struct video_device *vdev = &csi->vdev;
    int ret;

    ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
    if (ret)
        return ret;

    ret = v4l2_create_fwnode_links_to_pad(csi->source, &csi->pad,
                                          MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
    if (ret) {
        video_unregister_device(vdev);
        return ret;
    }

    ret = v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
    if (ret)
        video_unregister_device(vdev);
    return ret;
}

static const struct v4l2_async_notifier_operations csi_async_notifier_ops = {
    .bound = csi_async_notifier_bound,
    .complete = csi_async_notifier_complete,
};

static int ti_csi2rx_notifier_register(struct ti_csi2rx_dev *csi)
{
    struct fwnode_handle *fwnode;
    struct v4l2_async_connection *asc;
    struct device_node *node;
    int ret;

    node = of_get_child_by_name(csi->dev->of_node, "csi-bridge");
    if (!node)
        return -EINVAL;

    fwnode = of_fwnode_handle(node);
    if (!fwnode) {
        of_node_put(node);
        return -EINVAL;
    }

    v4l2_async_nf_init(&csi->notifier, &csi->v4l2_dev);
    csi->notifier.ops = &csi_async_notifier_ops;
    asc = v4l2_async_nf_add_fwnode(&csi->notifier, fwnode,
                                   struct v4l2_async_connection);
    of_node_put(node);
    if (IS_ERR(asc))
        return PTR_ERR(asc);

    ret = v4l2_async_nf_register(&csi->notifier);
    if (ret)
        v4l2_async_nf_cleanup(&csi->notifier);
    return ret;
}

static void ti_csi2rx_setup_shim(struct ti_csi2rx_dev *csi)
{
    const struct ti_csi2rx_fmt *fmt;
    unsigned int reg;

    fmt = find_format_by_fourcc(csi->v_fmt.fmt.pix.pixelformat);
    reg = SHIM_CNTL_PIX_RST;
    writel(reg, csi->shim + SHIM_CNTL);

    reg = SHIM_DMACNTX_EN;
    reg |= FIELD_PREP(SHIM_DMACNTX_FMT, fmt->csi_dt);
    switch (fmt->fourcc) {
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_VYUY:
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_YVYU:
        reg |= FIELD_PREP(SHIM_DMACNTX_YUV422, SHIM_DMACNTX_YUV422_MODE_11);
        break;
    default:
        break;
    }
    reg |= FIELD_PREP(SHIM_DMACNTX_SIZE, fmt->size);
    writel(reg, csi->shim + SHIM_DMACNTX);

    reg = FIELD_PREP(SHIM_PSI_CFG0_SRC_TAG, 0) |
          FIELD_PREP(SHIM_PSI_CFG0_DST_TAG, 0);
    writel(reg, csi->shim + SHIM_PSI_CFG0);
}

static void ti_csi2rx_drain_callback(void *param)
{
    struct completion *drain_complete = param;
    complete(drain_complete);
}

static int ti_csi2rx_drain_dma(struct ti_csi2rx_dev *csi)
{
    struct dma_async_tx_descriptor *desc;
    struct completion drain_complete;
    dma_cookie_t cookie;
    int ret;

    init_completion(&drain_complete);
    desc = dmaengine_prep_slave_single(csi->dma.chan, csi->dma.drain.paddr,
                                       csi->dma.drain.len, DMA_DEV_TO_MEM,
                                       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
    if (!desc)
        return -EIO;

    desc->callback = ti_csi2rx_drain_callback;
    desc->callback_param = &drain_complete;
    cookie = dmaengine_submit(desc);
    ret = dma_submit_error(cookie);
    if (ret)
        return ret;

    dma_async_issue_pending(csi->dma.chan);
    if (!wait_for_completion_timeout(&drain_complete,
                                     msecs_to_jiffies(DRAIN_TIMEOUT_MS))) {
        dmaengine_terminate_sync(csi->dma.chan);
        return -ETIMEDOUT;
    }
    return 0;
}

static void ti_csi2rx_dma_callback(void *param)
{
    struct ti_csi2rx_buffer *buf = param;
    struct ti_csi2rx_dev *csi = buf->csi;
    struct ti_csi2rx_dma *dma = &csi->dma;
    unsigned long flags;

    buf->vb.vb2_buf.timestamp = ktime_get_ns();
    buf->vb.sequence = csi->sequence++;

    spin_lock_irqsave(&dma->lock, flags);
    WARN_ON(!list_is_first(&buf->list, &dma->submitted));
    vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
    list_del(&buf->list);

    while (!list_empty(&dma->queue)) {
        buf = list_entry(dma->queue.next, struct ti_csi2rx_buffer, list);
        if (ti_csi2rx_start_dma(csi, buf)) {
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
        } else {
            list_move_tail(&buf->list, &dma->submitted);
        }
    }
    if (list_empty(&dma->submitted))
        dma->state = TI_CSI2RX_DMA_IDLE;
    spin_unlock_irqrestore(&dma->lock, flags);
}

static int ti_csi2rx_start_dma(struct ti_csi2rx_dev *csi,
                               struct ti_csi2rx_buffer *buf)
{
    unsigned long addr;
    struct dma_async_tx_descriptor *desc;
    size_t len = csi->v_fmt.fmt.pix.sizeimage;
    dma_cookie_t cookie;
    int ret = 0;

    addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
    desc = dmaengine_prep_slave_single(csi->dma.chan, addr, len,
                                       DMA_DEV_TO_MEM,
                                       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
    if (!desc)
        return -EIO;

    desc->callback = ti_csi2rx_dma_callback;
    desc->callback_param = buf;
    cookie = dmaengine_submit(desc);
    ret = dma_submit_error(cookie);
    if (ret)
        return ret;

    dma_async_issue_pending(csi->dma.chan);
    return 0;
}

static void ti_csi2rx_stop_dma(struct ti_csi2rx_dev *csi)
{
    enum ti_csi2rx_dma_state state;
    unsigned long flags;
    int ret;

    spin_lock_irqsave(&csi->dma.lock, flags);
    state = csi->dma.state;
    csi->dma.state = TI_CSI2RX_DMA_STOPPED;
    spin_unlock_irqrestore(&csi->dma.lock, flags);

    if (state != TI_CSI2RX_DMA_STOPPED) {
        ret = ti_csi2rx_drain_dma(csi);
        if (ret && ret != -ETIMEDOUT)
            dev_warn(csi->dev, "Failed to drain DMA\n");
    }

    ret = dmaengine_terminate_sync(csi->dma.chan);
    if (ret)
        dev_err(csi->dev, "Failed to stop DMA: %d\n", ret);
}

static void ti_csi2rx_cleanup_buffers(struct ti_csi2rx_dev *csi,
                                      enum vb2_buffer_state state)
{
    struct ti_csi2rx_buffer *buf, *tmp;
    unsigned long flags;

    spin_lock_irqsave(&csi->dma.lock, flags);
    list_for_each_entry_safe(buf, tmp, &csi->dma.queue, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, state);
    }
    list_for_each_entry_safe(buf, tmp, &csi->dma.submitted, list) {
        list_del(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, state);
    }
    spin_unlock_irqrestore(&csi->dma.lock, flags);
}

static int ti_csi2rx_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
                                 unsigned int *nplanes, unsigned int sizes[],
                                 struct device *alloc_devs[])
{
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(q);
    unsigned int size = csi->v_fmt.fmt.pix.sizeimage;

    if (*nplanes) {
        if (sizes[0] < size)
            return -EINVAL;
        size = sizes[0];
    }
    *nplanes = 1;
    sizes[0] = size;
    return 0;
}

static int ti_csi2rx_buffer_prepare(struct vb2_buffer *vb)
{
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vb->vb2_queue);
    unsigned long size = csi->v_fmt.fmt.pix.sizeimage;

    if (vb2_plane_size(vb, 0) < size)
        return -EINVAL;

    vb2_set_plane_payload(vb, 0, size);
    return 0;
}

static void ti_csi2rx_buffer_queue(struct vb2_buffer *vb)
{
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vb->vb2_queue);
    struct ti_csi2rx_buffer *buf = container_of(vb, struct ti_csi2rx_buffer, vb.vb2_buf);
    struct ti_csi2rx_dma *dma = &csi->dma;
    bool restart_dma = false;
    unsigned long flags;
    int ret;

    buf->csi = csi;
    spin_lock_irqsave(&dma->lock, flags);
    if (dma->state == TI_CSI2RX_DMA_IDLE) {
        restart_dma = true;
        dma->state = TI_CSI2RX_DMA_ACTIVE;
    } else {
        list_add_tail(&buf->list, &dma->queue);
    }
    spin_unlock_irqrestore(&dma->lock, flags);

    if (restart_dma) {
        ret = ti_csi2rx_drain_dma(csi);
        if (ret && ret != -ETIMEDOUT)
            dev_warn(csi->dev, "Failed to drain DMA\n");

        spin_lock_irqsave(&dma->lock, flags);
        ret = ti_csi2rx_start_dma(csi, buf);
        if (ret) {
            vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
            dma->state = TI_CSI2RX_DMA_IDLE;
        } else {
            list_add_tail(&buf->list, &dma->submitted);
        }
        spin_unlock_irqrestore(&dma->lock, flags);
    }
}

static int ti_csi2rx_start_streaming(struct vb2_queue *vq, unsigned int count)
{
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vq);
    struct ti_csi2rx_buffer *buf;
    unsigned long flags;
    int ret = 0;

    spin_lock_irqsave(&csi->dma.lock, flags);
    if (list_empty(&csi->dma.queue))
        ret = -EIO;
    spin_unlock_irqrestore(&csi->dma.lock, flags);
    if (ret)
        return ret;

    ret = video_device_pipeline_start(&csi->vdev, &csi->pipe);
    if (ret)
        goto err;

    ti_csi2rx_setup_shim(csi);
    csi->sequence = 0;

    spin_lock_irqsave(&csi->dma.lock, flags);
    buf = list_entry(csi->dma.queue.next, struct ti_csi2rx_buffer, list);
    ret = ti_csi2rx_start_dma(csi, buf);
    if (ret) {
        spin_unlock_irqrestore(&csi->dma.lock, flags);
        goto err_pipeline;
    }
    list_move_tail(&buf->list, &csi->dma.submitted);
    csi->dma.state = TI_CSI2RX_DMA_ACTIVE;
    spin_unlock_irqrestore(&csi->dma.lock, flags);

    ret = v4l2_subdev_call(csi->source, video, s_stream, 1); // Calls IMX219 s_stream
    if (ret)
        goto err_dma;

    return 0;

err_dma:
    ti_csi2rx_stop_dma(csi);
err_pipeline:
    video_device_pipeline_stop(&csi->vdev);
    writel(0, csi->shim + SHIM_CNTL);
    writel(0, csi->shim + SHIM_DMACNTX);
err:
    ti_csi2rx_cleanup_buffers(csi, VB2_BUF_STATE_QUEUED);
    return ret;
}

static void ti_csi2rx_stop_streaming(struct vb2_queue *vq)
{
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vq);
    int ret;

    video_device_pipeline_stop(&csi->vdev);
    writel(0, csi->shim + SHIM_CNTL);
    writel(0, csi->shim + SHIM_DMACNTX);

    ret = v4l2_subdev_call(csi->source, video, s_stream, 0);
    if (ret)
        dev_err(csi->dev, "Failed to stop subdev stream\n");

    ti_csi2rx_stop_dma(csi);
    ti_csi2rx_cleanup_buffers(csi, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops csi_vb2_qops = {
    .queue_setup = ti_csi2rx_queue_setup,
    .buf_prepare = ti_csi2rx_buffer_prepare,
    .buf_queue = ti_csi2rx_buffer_queue,
    .start_streaming = ti_csi2rx_start_streaming,
    .stop_streaming = ti_csi2rx_stop_streaming,
};

static int ti_csi2rx_init_vb2q(struct ti_csi2rx_dev *csi)
{
    struct vb2_queue *q = &csi->vidq;
    int ret;

    q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    q->io_modes = VB2_MMAP | VB2_DMABUF;
    q->drv_priv = csi;
    q->buf_struct_size = sizeof(struct ti_csi2rx_buffer);
    q->ops = &csi_vb2_qops;
    q->mem_ops = &vb2_dma_contig_memops;
    q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    q->dev = dmaengine_get_dma_device(csi->dma.chan);
    q->lock = &csi->mutex;
    q->min_queued_buffers = 1;

    ret = vb2_queue_init(q);
    if (ret)
        return ret;

    csi->vdev.queue = q;
    return 0;
}

static int ti_csi2rx_link_validate(struct media_link *link)
{
    struct media_entity *entity = link->sink->entity;
    struct video_device *vdev = media_entity_to_video_device(entity);
    struct ti_csi2rx_dev *csi = container_of(vdev, struct ti_csi2rx_dev, vdev);
    struct v4l2_pix_format *csi_fmt = &csi->v_fmt.fmt.pix;
    struct v4l2_subdev_format source_fmt = {
        .which  = V4L2_SUBDEV_FORMAT_ACTIVE,
        .pad    = link->source->index,
    };
    const struct ti_csi2rx_fmt *ti_fmt;
    int ret;

    ret = v4l2_subdev_call_state_active(csi->source, pad, get_fmt, &source_fmt);
    if (ret)
        return ret;

    if (source_fmt.format.width != csi_fmt->width ||
        source_fmt.format.height != csi_fmt->height ||
        (source_fmt.format.field != csi_fmt->field && csi_fmt->field != V4L2_FIELD_NONE))
        return -EPIPE;

    ti_fmt = find_format_by_code(source_fmt.format.code);
    if (!ti_fmt || ti_fmt->fourcc != csi_fmt->pixelformat)
        return -EPIPE;

    // Optional strict check for IMX219 format, can be removed if flexible formats needed
    if (source_fmt.format.code != MEDIA_BUS_FMT_SRGGB10_1X10)
        dev_warn(csi->dev, "Expected SRGGB10_1X10, got 0x%x\n", source_fmt.format.code);

    return 0;
}

static const struct media_entity_operations ti_csi2rx_video_entity_ops = {
    .link_validate = ti_csi2rx_link_validate,
};

static int ti_csi2rx_init_dma(struct ti_csi2rx_dev *csi)
{
    struct dma_slave_config cfg = { .src_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES };
    int ret;

    INIT_LIST_HEAD(&csi->dma.queue);
    INIT_LIST_HEAD(&csi->dma.submitted);
    spin_lock_init(&csi->dma.lock);
    csi->dma.state = TI_CSI2RX_DMA_STOPPED;

    csi->dma.chan = dma_request_chan(csi->dev, "rx0");
    if (IS_ERR(csi->dma.chan))
        return PTR_ERR(csi->dma.chan);

    ret = dmaengine_slave_config(csi->dma.chan, &cfg);
    if (ret) {
        dma_release_channel(csi->dma.chan);
        return ret;
    }

    csi->dma.drain.len = DRAIN_BUFFER_SIZE;
    csi->dma.drain.vaddr = dma_alloc_coherent(csi->dev, csi->dma.drain.len,
                                              &csi->dma.drain.paddr, GFP_KERNEL);
    if (!csi->dma.drain.vaddr)
        return -ENOMEM;

    return 0;
}

static int ti_csi2rx_v4l2_init(struct ti_csi2rx_dev *csi)
{
    struct media_device *mdev = &csi->mdev;
    struct video_device *vdev = &csi->vdev;
    const struct ti_csi2rx_fmt *fmt;
    struct v4l2_pix_format *pix_fmt = &csi->v_fmt.fmt.pix;
    int ret;

    fmt = find_format_by_fourcc(V4L2_PIX_FMT_UYVY);
    if (!fmt)
        return -EINVAL;

    pix_fmt->width = 640;
    pix_fmt->height = 480;
    pix_fmt->field = V4L2_FIELD_NONE;
    pix_fmt->colorspace = V4L2_COLORSPACE_SRGB;
    pix_fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
    pix_fmt->quantization = V4L2_QUANTIZATION_LIM_RANGE;
    pix_fmt->xfer_func = V4L2_XFER_FUNC_SRGB;
    ti_csi2rx_fill_fmt(fmt, &csi->v_fmt);

    mdev->dev = csi->dev;
    mdev->hw_revision = 1;
    strscpy(mdev->model, "TI-CSI2RX", sizeof(mdev->model));
    media_device_init(mdev);

    strscpy(vdev->name, TI_CSI2RX_MODULE_NAME, sizeof(vdev->name));
    vdev->v4l2_dev = &csi->v4l2_dev;
    vdev->vfl_dir = VFL_DIR_RX;
    vdev->fops = &csi_fops;
    vdev->ioctl_ops = &csi_ioctl_ops;
    vdev->release = video_device_release_empty;
    vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;
    vdev->lock = &csi->mutex;
    video_set_drvdata(vdev, csi);

    csi->pad.flags = MEDIA_PAD_FL_SINK;
    vdev->entity.ops = &ti_csi2rx_video_entity_ops;
    ret = media_entity_pads_init(&csi->vdev.entity, 1, &csi->pad);
    if (ret)
        return ret;

    csi->v4l2_dev.mdev = mdev;
    ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
    if (ret)
        return ret;

    ret = media_device_register(mdev);
    if (ret) {
        v4l2_device_unregister(&csi->v4l2_dev);
        media_device_cleanup(mdev);
        return ret;
    }
    return 0;
}

static void ti_csi2rx_cleanup_dma(struct ti_csi2rx_dev *csi)
{
    dma_free_coherent(csi->dev, csi->dma.drain.len,
                      csi->dma.drain.vaddr, csi->dma.drain.paddr);
    csi->dma.drain.vaddr = NULL;
    dma_release_channel(csi->dma.chan);
}

static void ti_csi2rx_cleanup_v4l2(struct ti_csi2rx_dev *csi)
{
    media_device_unregister(&csi->mdev);
    v4l2_device_unregister(&csi->v4l2_dev);
    media_device_cleanup(&csi->mdev);
}

static void ti_csi2rx_cleanup_subdev(struct ti_csi2rx_dev *csi)
{
    v4l2_async_nf_unregister(&csi->notifier);
    v4l2_async_nf_cleanup(&csi->notifier);
}

static void ti_csi2rx_cleanup_vb2q(struct ti_csi2rx_dev *csi)
{
    vb2_queue_release(&csi->vidq);
}

static int ti_csi2rx_probe(struct platform_device *pdev)
{
    struct ti_csi2rx_dev *csi;
    int ret;

    csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
    if (!csi)
        return -ENOMEM;

    csi->dev = &pdev->dev;
    platform_set_drvdata(pdev, csi);
    mutex_init(&csi->mutex);

    csi->shim = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(csi->shim))
        return PTR_ERR(csi->shim);

    ret = ti_csi2rx_init_dma(csi);
    if (ret)
        goto err_mutex;

    ret = ti_csi2rx_v4l2_init(csi);
    if (ret)
        goto err_dma;

    ret = ti_csi2rx_init_vb2q(csi);
    if (ret)
        goto err_v4l2;

    ret = ti_csi2rx_notifier_register(csi);
    if (ret)
        goto err_vb2q;

    ret = of_platform_populate(csi->dev->of_node, NULL, NULL, csi->dev);
    if (ret)
        goto err_subdev;

    return 0;

err_subdev:
    ti_csi2rx_cleanup_subdev(csi);
err_vb2q:
    ti_csi2rx_cleanup_vb2q(csi);
err_v4l2:
    ti_csi2rx_cleanup_v4l2(csi);
err_dma:
    ti_csi2rx_cleanup_dma(csi);
err_mutex:
    mutex_destroy(&csi->mutex);
    return ret;
}

static void ti_csi2rx_remove(struct platform_device *pdev)
{
    struct ti_csi2rx_dev *csi = platform_get_drvdata(pdev);

    video_unregister_device(&csi->vdev);
    ti_csi2rx_cleanup_vb2q(csi);
    ti_csi2rx_cleanup_subdev(csi);
    ti_csi2rx_cleanup_v4l2(csi);
    ti_csi2rx_cleanup_dma(csi);
    mutex_destroy(&csi->mutex);
}

static const struct of_device_id ti_csi2rx_of_match[] = {
    { .compatible = "ti,j721e-csi2rx-shim" },
    { },
};
MODULE_DEVICE_TABLE(of, ti_csi2rx_of_match);

static struct platform_driver ti_csi2rx_pdrv = {
    .probe = ti_csi2rx_probe,
    .remove = ti_csi2rx_remove,
    .driver = {
        .name = TI_CSI2RX_MODULE_NAME,
        .of_match_table = ti_csi2rx_of_match,
    },
};

module_platform_driver(ti_csi2rx_pdrv);

MODULE_DESCRIPTION("TI J721E CSI2 RX Driver");
MODULE_AUTHOR("Jai Luthra <j-luthra@ti.com>");
MODULE_LICENSE("GPL");