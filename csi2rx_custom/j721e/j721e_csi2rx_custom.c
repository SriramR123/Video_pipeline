#include <linux/bitfield.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-fwnode.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>


#define TI_CSI2RX_MODULE_NAME "ti-csi2rx-custom"

#define SHIM_CNTL 0x10
#define SHIM_CNTL_PIX_RST BIT(0)
#define SHIM_DMACNTX 0x20
#define SHIM_DMACNTX_EN BIT(31)
#define SHIM_DMACNTX_YUV422 GENMASK(27, 26)
#define SHIM_DMACNTX_SIZE GENMASK(21, 20)
#define SHIM_DMACNTX_FMT GENMASK(5, 0)
#define SHIM_DMACNTX_SIZE_8 0
#define SHIM_DMACNTX_SIZE_16 1
#define SHIM_PSI_CFG0 0x24
#define SHIM_PSI_CFG0_SRC_TAG GENMASK(15, 0)
#define SHIM_PSI_CFG0_DST_TAG GENMASK(31, 16)
#define PSIL_WORD_SIZE_BYTES 16
#define MAX_WIDTH_BYTES SZ_16K
#define MAX_HEIGHT_LINES SZ_16K
#define DRAIN_TIMEOUT_MS 50
#define DRAIN_BUFFER_SIZE SZ_32K

struct ti_csi2rx_fmt {
    u32 fourcc;
    u32 code;
    u32 csi_dt;
    u8 bpp;
    u8 size;
};

/* Formats supported by custom IMX219 */
static const struct ti_csi2rx_fmt ti_csi2rx_formats[] = {
    { V4L2_PIX_FMT_SRGGB8,  MEDIA_BUS_FMT_SRGGB8_1X8,  MIPI_CSI2_DT_RAW8,  8,  SHIM_DMACNTX_SIZE_8  },
    { V4L2_PIX_FMT_SGRBG8,  MEDIA_BUS_FMT_SGRBG8_1X8,  MIPI_CSI2_DT_RAW8,  8,  SHIM_DMACNTX_SIZE_8  },
    { V4L2_PIX_FMT_SGBRG8,  MEDIA_BUS_FMT_SGBRG8_1X8,  MIPI_CSI2_DT_RAW8,  8,  SHIM_DMACNTX_SIZE_8  },
    { V4L2_PIX_FMT_SBGGR8,  MEDIA_BUS_FMT_SBGGR8_1X8,  MIPI_CSI2_DT_RAW8,  8,  SHIM_DMACNTX_SIZE_8  },
    { V4L2_PIX_FMT_SRGGB10, MEDIA_BUS_FMT_SRGGB10_1X10, MIPI_CSI2_DT_RAW10, 16, SHIM_DMACNTX_SIZE_16 },
    { V4L2_PIX_FMT_SGRBG10, MEDIA_BUS_FMT_SGRBG10_1X10, MIPI_CSI2_DT_RAW10, 16, SHIM_DMACNTX_SIZE_16 },
    { V4L2_PIX_FMT_SGBRG10, MEDIA_BUS_FMT_SGBRG10_1X10, MIPI_CSI2_DT_RAW10, 16, SHIM_DMACNTX_SIZE_16 },
    { V4L2_PIX_FMT_SBGGR10, MEDIA_BUS_FMT_SBGGR10_1X10, MIPI_CSI2_DT_RAW10, 16, SHIM_DMACNTX_SIZE_16 },
};

struct ti_csi2rx_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
    struct ti_csi2rx_dev *csi;
};

enum ti_csi2rx_dma_state {
    TI_CSI2RX_DMA_STOPPED,
    TI_CSI2RX_DMA_IDLE,
    TI_CSI2RX_DMA_ACTIVE,
};

struct ti_csi2rx_dma {
    spinlock_t lock;
    struct dma_chan *chan;
    struct list_head queue;
    enum ti_csi2rx_dma_state state;
    struct list_head submitted;
    struct { void *vaddr; dma_addr_t paddr; size_t len; } drain;
};

struct ti_csi2rx_dev {
    struct device *dev;
    void __iomem *shim;
    struct v4l2_device v4l2_dev;
    struct video_device vdev;
    struct media_device mdev;
    struct media_pipeline pipe;
    struct media_pad pad;
    struct v4l2_async_notifier notifier;
    struct v4l2_subdev *source;
    struct vb2_queue vidq;
    struct mutex mutex;
    struct v4l2_format v_fmt;
    struct ti_csi2rx_dma dma;
    u32 sequence;
    u32 num_lanes;
};

/* Format handling */
static const struct ti_csi2rx_fmt *find_format_by_fourcc(u32 pixelformat) {
    int i;
    for (i = 0; i < ARRAY_SIZE(ti_csi2rx_formats); i++)
        if (ti_csi2rx_formats[i].fourcc == pixelformat)
            return &ti_csi2rx_formats[i];
    return NULL;
}

static const struct ti_csi2rx_fmt *find_format_by_code(u32 code) {
    int i;
    for (i = 0; i < ARRAY_SIZE(ti_csi2rx_formats); i++)
        if (ti_csi2rx_formats[i].code == code)
            return &ti_csi2rx_formats[i];
    return NULL;
}

static void ti_csi2rx_fill_fmt(const struct ti_csi2rx_fmt *csi_fmt, struct v4l2_format *v4l2_fmt) {
    struct v4l2_pix_format *pix = &v4l2_fmt->fmt.pix;
    unsigned int pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / csi_fmt->bpp;

    pix->width = clamp_t(unsigned int, pix->width, pixels_in_word, MAX_WIDTH_BYTES * 8 / csi_fmt->bpp);
    pix->height = clamp_t(unsigned int, pix->height, 1, MAX_HEIGHT_LINES);
    pix->width = rounddown(pix->width, pixels_in_word);
    v4l2_fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    pix->pixelformat = csi_fmt->fourcc;
    pix->bytesperline = pix->width * (csi_fmt->bpp / 8);
    pix->sizeimage = pix->bytesperline * pix->height;
    pix->field = V4L2_FIELD_NONE;
}

/* V4L2 IOCTL operations */
static int ti_csi2rx_querycap(struct file *file, void *priv, struct v4l2_capability *cap) {
    strscpy(cap->driver, TI_CSI2RX_MODULE_NAME, sizeof(cap->driver));
    strscpy(cap->card, TI_CSI2RX_MODULE_NAME, sizeof(cap->card));
    return 0;
}

static int ti_csi2rx_enum_fmt_vid_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f) {
    if (f->index >= ARRAY_SIZE(ti_csi2rx_formats))
        return -EINVAL;
    f->pixelformat = ti_csi2rx_formats[f->index].fourcc;
    f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    return 0;
}

static int ti_csi2rx_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f) {
    struct ti_csi2rx_dev *csi = video_drvdata(file);
    *f = csi->v_fmt;
    return 0;
}

static int ti_csi2rx_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f) {
    const struct ti_csi2rx_fmt *fmt = find_format_by_fourcc(f->fmt.pix.pixelformat);
    if (!fmt)
        fmt = &ti_csi2rx_formats[0]; /* Default to SRGGB8 */
    f->fmt.pix.field = V4L2_FIELD_NONE;
    ti_csi2rx_fill_fmt(fmt, f);
    return 0;
}

static int ti_csi2rx_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f) {
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

static int ti_csi2rx_enum_framesizes(struct file *file, void *fh, struct v4l2_frmsizeenum *fsize) {
    const struct ti_csi2rx_fmt *fmt = find_format_by_fourcc(fsize->pixel_format);
    if (!fmt || fsize->index != 0)
        return -EINVAL;
    unsigned int pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / fmt->bpp;
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
    .vidioc_querycap = ti_csi2rx_querycap,
    .vidioc_enum_fmt_vid_cap = ti_csi2rx_enum_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap = ti_csi2rx_try_fmt_vid_cap,
    .vidioc_g_fmt_vid_cap = ti_csi2rx_g_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = ti_csi2rx_s_fmt_vid_cap,
    .vidioc_enum_framesizes = ti_csi2rx_enum_framesizes,
    .vidioc_reqbufs = vb2_ioctl_reqbufs,
    .vidioc_create_bufs = vb2_ioctl_create_bufs,
    .vidioc_prepare_buf = vb2_ioctl_prepare_buf,
    .vidioc_querybuf = vb2_ioctl_querybuf,
    .vidioc_qbuf = vb2_ioctl_qbuf,
    .vidioc_dqbuf = vb2_ioctl_dqbuf,
    .vidioc_expbuf = vb2_ioctl_expbuf,
    .vidioc_streamon = vb2_ioctl_streamon,
    .vidioc_streamoff = vb2_ioctl_streamoff,
};

/* DMA handling */
static void ti_csi2rx_dma_callback(void *param) {
    struct ti_csi2rx_buffer *buf = param;
    struct ti_csi2rx_dev *csi = buf->csi;
    unsigned long flags;

    spin_lock_irqsave(&csi->dma.lock, flags);
    if (csi->dma.state != TI_CSI2RX_DMA_ACTIVE) {
        spin_unlock_irqrestore(&csi->dma.lock, flags);
        return;
    }
    list_del_init(&buf->list);
    buf->vb.vb2_buf.timestamp = ktime_get_ns();
    buf->vb.sequence = csi->sequence++;
    vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
    if (list_empty(&csi->dma.submitted))
        csi->dma.state = TI_CSI2RX_DMA_IDLE;
    spin_unlock_irqrestore(&csi->dma.lock, flags);
}

static int ti_csi2rx_start_dma(struct ti_csi2rx_dev *csi, struct ti_csi2rx_buffer *buf) {
    unsigned long addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
    struct dma_async_tx_descriptor *desc;
    size_t len = csi->v_fmt.fmt.pix.sizeimage;
    dma_cookie_t cookie;

    desc = dmaengine_prep_slave_single(csi->dma.chan, addr, len, DMA_DEV_TO_MEM,
                                       DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
    if (!desc)
        return -EIO;
    desc->callback = ti_csi2rx_dma_callback;
    desc->callback_param = buf;
    cookie = dmaengine_submit(desc);
    if (dma_submit_error(cookie))
        return -EIO;
    dma_async_issue_pending(csi->dma.chan);
    return 0;
}

static void ti_csi2rx_setup_shim(struct ti_csi2rx_dev *csi) {
    const struct ti_csi2rx_fmt *fmt = find_format_by_fourcc(csi->v_fmt.fmt.pix.pixelformat);
    unsigned int reg;

    reg = SHIM_CNTL_PIX_RST;
    writel(reg, csi->shim + SHIM_CNTL);
    reg = SHIM_DMACNTX_EN | FIELD_PREP(SHIM_DMACNTX_FMT, fmt->csi_dt) |
          FIELD_PREP(SHIM_DMACNTX_SIZE, fmt->size);
    writel(reg, csi->shim + SHIM_DMACNTX);
    reg = FIELD_PREP(SHIM_PSI_CFG0_SRC_TAG, 0) | FIELD_PREP(SHIM_PSI_CFG0_DST_TAG, 0);
    writel(reg, csi->shim + SHIM_PSI_CFG0);
}

static int ti_csi2rx_start_streaming(struct vb2_queue *vq, unsigned int count) {
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vq);
    struct ti_csi2rx_buffer *buf, *tmp;
    unsigned long flags;
    int ret;

    mutex_lock(&csi->mutex);
    spin_lock_irqsave(&csi->dma.lock, flags);
    csi->dma.state = TI_CSI2RX_DMA_ACTIVE;
    list_for_each_entry_safe(buf, tmp, &csi->dma.queue, list) {
        list_del_init(&buf->list);
        list_add_tail(&buf->list, &csi->dma.submitted);
        ret = ti_csi2rx_start_dma(csi, buf);
        if (ret) {
            dev_err(csi->dev, "Failed to start DMA: %d\n", ret);
            goto err_dma;
        }
    }
    spin_unlock_irqrestore(&csi->dma.lock, flags);

    ret = media_pipeline_start(&csi->pad, &csi->pipe);
    if (ret)
        goto err_pipe;

    ti_csi2rx_setup_shim(csi);
    ret = v4l2_subdev_call(csi->source, video, s_stream, 1);
    if (ret)
        goto err_stream;
    mutex_unlock(&csi->mutex);
    return 0;

err_stream:
    media_pipeline_stop(&csi->pad);
err_pipe:
    spin_lock_irqsave(&csi->dma.lock, flags);
err_dma:
    csi->dma.state = TI_CSI2RX_DMA_STOPPED;
    list_for_each_entry_safe(buf, tmp, &csi->dma.submitted, list) {
        list_del_init(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
    }
    list_for_each_entry_safe(buf, tmp, &csi->dma.queue, list) {
        list_del_init(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
    }
    spin_unlock_irqrestore(&csi->dma.lock, flags);
    mutex_unlock(&csi->mutex);
    return ret;
}

static void ti_csi2rx_stop_streaming(struct vb2_queue *vq) {
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vq);
    struct ti_csi2rx_buffer *buf, *tmp;
    unsigned long flags;

    mutex_lock(&csi->mutex);
    v4l2_subdev_call(csi->source, video, s_stream, 0);
    media_pipeline_stop(&csi->pad);

    spin_lock_irqsave(&csi->dma.lock, flags);
    csi->dma.state = TI_CSI2RX_DMA_STOPPED;
    dmaengine_terminate_sync(csi->dma.chan);
    list_for_each_entry_safe(buf, tmp, &csi->dma.submitted, list) {
        list_del_init(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    list_for_each_entry_safe(buf, tmp, &csi->dma.queue, list) {
        list_del_init(&buf->list);
        vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
    spin_unlock_irqrestore(&csi->dma.lock, flags);
    mutex_unlock(&csi->mutex);
}

/* VB2 operations */
static int ti_csi2rx_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
                                 unsigned int *nplanes, unsigned int sizes[],
                                 struct device *alloc_devs[]) {
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(q);

    if (*nplanes)
        return sizes[0] < csi->v_fmt.fmt.pix.sizeimage ? -EINVAL : 0;
    *nplanes = 1;
    sizes[0] = csi->v_fmt.fmt.pix.sizeimage;
    return 0;
}

static int ti_csi2rx_buffer_prepare(struct vb2_buffer *vb) {
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vb->vb2_queue);
    unsigned long size = csi->v_fmt.fmt.pix.sizeimage;

    if (vb2_plane_size(vb, 0) < size) {
        dev_err(csi->dev, "Buffer too small: %lu < %lu\n", vb2_plane_size(vb, 0), size);
        return -EINVAL;
    }
    vb2_set_plane_payload(vb, 0, size);
    return 0;
}

static void ti_csi2rx_buffer_queue(struct vb2_buffer *vb) {
    struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vb->vb2_queue);
    struct ti_csi2rx_buffer *buf = container_of(vb, struct ti_csi2rx_buffer, vb.vb2_buf);
    unsigned long flags;

    buf->csi = csi;
    spin_lock_irqsave(&csi->dma.lock, flags);
    if (csi->dma.state == TI_CSI2RX_DMA_ACTIVE) {
        list_add_tail(&buf->list, &csi->dma.submitted);
        ti_csi2rx_start_dma(csi, buf);
    } else {
        list_add_tail(&buf->list, &csi->dma.queue);
    }
    spin_unlock_irqrestore(&csi->dma.lock, flags);
}

static const struct vb2_ops ti_csi2rx_vb2_ops = {
    .queue_setup = ti_csi2rx_queue_setup,
    .buf_prepare = ti_csi2rx_buffer_prepare,
    .buf_queue = ti_csi2rx_buffer_queue,
    .start_streaming = ti_csi2rx_start_streaming,
    .stop_streaming = ti_csi2rx_stop_streaming,
};

/* Media entity operations */
static int ti_csi2rx_link_validate(struct media_link *link) {
    struct video_device *vdev = media_entity_to_video_device(link->sink->entity);
    struct ti_csi2rx_dev *csi = container_of(vdev, struct ti_csi2rx_dev, vdev);
    struct v4l2_pix_format *csi_fmt = &csi->v_fmt.fmt.pix;
    struct v4l2_subdev_format source_fmt = { .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = link->source->index };
    const struct ti_csi2rx_fmt *ti_fmt;
    int ret;

    ret = v4l2_subdev_call(csi->source, pad, get_fmt, NULL, &source_fmt);
    if (ret)
        return ret;

    if (source_fmt.format.width != csi_fmt->width || source_fmt.format.height != csi_fmt->height) {
        dev_err(csi->dev, "Resolution mismatch: source %ux%u, sink %ux%u\n",
                source_fmt.format.width, source_fmt.format.height, csi_fmt->width, csi_fmt->height);
        return -EPIPE;
    }

    ti_fmt = find_format_by_code(source_fmt.format.code);
    if (!ti_fmt || ti_fmt->fourcc != csi_fmt->pixelformat) {
        dev_err(csi->dev, "Format mismatch: source 0x%x, sink 0x%x\n",
                source_fmt.format.code, csi_fmt->pixelformat);
        return -EPIPE;
    }

    if (csi->num_lanes != 2) {
        dev_err(csi->dev, "Lane mismatch: Expected 2 lanes, got %u\n", csi->num_lanes);
        return -EPIPE;
    }

    return 0;
}

static const struct media_entity_operations ti_csi2rx_video_entity_ops = {
    .link_validate = ti_csi2rx_link_validate,
};

/* File operations */
static const struct v4l2_file_operations csi_fops = {
    .owner = THIS_MODULE,
    .open = v4l2_fh_open,
    .release = vb2_fop_release,
    .unlocked_ioctl = video_ioctl2,
    .mmap = vb2_fop_mmap,
    .poll = vb2_fop_poll,
};

/* V4L2 and media initialization */
static int ti_csi2rx_init_v4l2(struct ti_csi2rx_dev *csi) {
    struct media_device *mdev = &csi->mdev;
    struct video_device *vdev = &csi->vdev;
    const struct ti_csi2rx_fmt *fmt;
    struct v4l2_pix_format *pix_fmt = &csi->v_fmt.fmt.pix;
    int ret;

    fmt = find_format_by_fourcc(V4L2_PIX_FMT_SRGGB8); /* Default format */
    pix_fmt->width = 1920;
    pix_fmt->height = 1080;
    pix_fmt->field = V4L2_FIELD_NONE;
    pix_fmt->colorspace = V4L2_COLORSPACE_SRGB;
    ti_csi2rx_fill_fmt(fmt, &csi->v_fmt);

    mdev->dev = csi->dev;
    strscpy(mdev->model, "TI-CSI2RX-IMX219", sizeof(mdev->model));
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
    ret = media_entity_pads_init(&vdev->entity, 1, &csi->pad);
    if (ret)
        return ret;

    csi->v4l2_dev.mdev = mdev;
    ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
    if (ret)
        goto err_entity;

    ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
    if (ret)
        goto err_v4l2;

    ret = media_device_register(mdev);
    if (ret)
        goto err_video;

    return 0;

err_video:
    video_unregister_device(vdev);
err_v4l2:
    v4l2_device_unregister(&csi->v4l2_dev);
err_entity:
    media_entity_cleanup(&vdev->entity);
    media_device_cleanup(mdev);
    return ret;
}

/* Async notifier operations */
static int ti_csi2rx_notify_bound(struct v4l2_async_notifier *notifier,
    struct v4l2_subdev *subdev,
    struct v4l2_async_subdev *asd) {
struct ti_csi2rx_dev *csi = container_of(notifier, struct ti_csi2rx_dev, notifier);
struct media_entity *source = &subdev->entity;
struct media_entity *sink = &csi->vdev.entity;

csi->source = subdev;
return media_create_pad_link(source, 0, sink, 0,
   MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
}

static const struct v4l2_async_notifier_operations ti_csi2rx_notifier_ops = {
    .bound = ti_csi2rx_notify_bound,
};

/* Probe and remove */
static int ti_csi2rx_probe(struct platform_device *pdev) {
    struct ti_csi2rx_dev *csi;
    struct fwnode_handle *fwnode;
    struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
    struct v4l2_async_subdev *asd;  /* Changed from v4l2_async_connection */
    int ret;

    csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
    if (!csi)
        return -ENOMEM;

    csi->dev = &pdev->dev;
    platform_set_drvdata(pdev, csi);
    mutex_init(&csi->mutex);
    spin_lock_init(&csi->dma.lock);
    INIT_LIST_HEAD(&csi->dma.queue);
    INIT_LIST_HEAD(&csi->dma.submitted);

    csi->shim = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(csi->shim))
        return PTR_ERR(csi->shim);

    csi->dma.chan = dma_request_chan(&pdev->dev, "rx0");
    if (IS_ERR(csi->dma.chan)) {
        return PTR_ERR(csi->dma.chan);
    }

    fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(&pdev->dev), NULL);
    if (!fwnode)
        return -EINVAL;

    ret = v4l2_fwnode_endpoint_parse(fwnode, &ep);
    if (ret)
        goto err_fwnode;
    csi->num_lanes = ep.bus.mipi_csi2.num_data_lanes;
    if (csi->num_lanes != 2) {
        dev_err(&pdev->dev, "Only 2 lanes supported for custom IMX219\n");
        ret = -EINVAL;
        goto err_fwnode;
    }

    csi->vidq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    csi->vidq.io_modes = VB2_MMAP | VB2_DMABUF;
    csi->vidq.drv_priv = csi;
    csi->vidq.ops = &ti_csi2rx_vb2_ops;
    csi->vidq.mem_ops = &vb2_dma_contig_memops;
    csi->vidq.buf_struct_size = sizeof(struct ti_csi2rx_buffer);
    csi->vidq.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    csi->vidq.lock = &csi->mutex;
    ret = vb2_queue_init(&csi->vidq);
    if (ret) {
        goto err_fwnode;
    }

    ret = ti_csi2rx_init_v4l2(csi);
    if (ret) {
        goto err_fwnode;
    }

    v4l2_async_nf_init(&csi->notifier);
    asd = v4l2_async_nf_add_fwnode(&csi->notifier, fwnode, struct v4l2_async_subdev);
    if (IS_ERR(asd)) {
        ret = PTR_ERR(asd);
        goto err_media;
    }
    csi->notifier.ops = &ti_csi2rx_notifier_ops;
    ret = v4l2_async_nf_register(&csi->v4l2_dev, &csi->notifier);
    if (ret)
        goto err_notifier;

    fwnode_handle_put(fwnode);
    pm_runtime_enable(&pdev->dev);
    return 0;

err_notifier:
    v4l2_async_nf_cleanup(&csi->notifier);
err_media:
    media_device_unregister(&csi->mdev);
    video_unregister_device(&csi->vdev);
    v4l2_device_unregister(&csi->v4l2_dev);
    media_entity_cleanup(&csi->vdev.entity);
    media_device_cleanup(&csi->mdev);
err_fwnode:
    fwnode_handle_put(fwnode);
    dma_release_channel(csi->dma.chan);
    return ret;
}

static int ti_csi2rx_remove(struct platform_device *pdev) {
    struct ti_csi2rx_dev *csi = platform_get_drvdata(pdev);

    pm_runtime_disable(&pdev->dev);
    v4l2_async_nf_unregister(&csi->notifier);
    v4l2_async_nf_cleanup(&csi->notifier);
    media_device_unregister(&csi->mdev);
    video_unregister_device(&csi->vdev);
    v4l2_device_unregister(&csi->v4l2_dev);
    media_entity_cleanup(&csi->vdev.entity);
    media_device_cleanup(&csi->mdev);
    dma_release_channel(csi->dma.chan);
    mutex_destroy(&csi->mutex);
    return 0;
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
MODULE_DESCRIPTION("TI J721E CSI2RX Driver for Custom IMX219");
MODULE_AUTHOR("Adapted for custom IMX219");
MODULE_LICENSE("GPL v2");