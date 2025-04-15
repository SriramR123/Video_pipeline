// SPDX-License-Identifier: GPL-2.0-only
/*
 * Custom TI J721E CSI2-RX Driver for Cadence CSI2-RX and IMX219
 *
 * Copyright (C) 2025 [Your Name]
 * Based on original work by Pratyush Yadav <p.yadav@ti.com>
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 */

 
#include <linux/bitfield.h>
#include <linux/dmaengine.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

#define TI_CSI2RX_MODULE_NAME		"j721e-csi2rx"

#define PSIL_WORD_SIZE_BYTES		16

/*
 * Arbitrary max width and height limits for the video device.
 * Set to 16K x 16K to accommodate high-resolution sensors like IMX219.
 */
#define MAX_WIDTH_BYTES			SZ_16K
#define MAX_HEIGHT_LINES		SZ_16K

#define TI_CSI2RX_PAD_SINK		0
#define TI_CSI2RX_PAD_SOURCE		1
#define TI_CSI2RX_NUM_PADS		2  /* One sink, one source */

#define DRAIN_TIMEOUT_MS		50
#define DRAIN_BUFFER_SIZE		SZ_32K

struct ti_csi2rx_fmt {
	u32	fourcc;		/* Four character code. */
	u32	code;		/* Mbus code. */
	u32	csi_dt;		/* CSI Data type. */
	u8	bpp;	      /* Bits per pixel. */
    u8	size;	
};

/* Buffer structure for video buffer queueing */
struct ti_csi2rx_buffer {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
	struct ti_csi2rx_ctx	*ctx;
};

/* DMA state enumeration */
enum ti_csi2rx_dma_state {
	TI_CSI2RX_DMA_STOPPED,
	TI_CSI2RX_DMA_IDLE,
	TI_CSI2RX_DMA_ACTIVE,
};

/* DMA context structure */
struct ti_csi2rx_dma {
	spinlock_t		lock;
	struct dma_chan		*chan;
	struct list_head	queue;
	struct list_head	submitted;
	enum ti_csi2rx_dma_state state;
};

/* Forward declaration */
struct ti_csi2rx_dev;

struct ti_csi2rx_ctx {
	struct ti_csi2rx_dev	*csi;
	struct video_device	vdev;
	struct vb2_queue	vidq;
	struct mutex		mutex; /* Serialize ioctls */
	struct v4l2_format	v_fmt;
	struct ti_csi2rx_dma	dma;
	struct media_pad	pad;
	u32			sequence;
	u32			idx;
	u32			stream;
};

struct ti_csi2rx_dev {
	struct device		*dev;
	struct mutex		mutex; /* Serialize subdev ioctls */
	unsigned int		enable_count;
	struct v4l2_async_notifier notifier;
	struct media_device	mdev;
	struct media_pipeline	pipe;
	struct media_pad	pads[TI_CSI2RX_NUM_PADS];
	struct v4l2_device	v4l2_dev;
	struct v4l2_subdev	*source; /* Cadence CSI2-RX subdev */
	struct v4l2_subdev	subdev;
	struct ti_csi2rx_ctx	ctx[1]; /* Single context */
	u64			enabled_streams_mask;
	struct {
		void		*vaddr;
		dma_addr_t	paddr;
		size_t		len;
	} drain;
};

/* Supported formats (subset compatible with IMX219) */
static const struct ti_csi2rx_fmt formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_SBGGR8,
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.csi_dt = MIPI_CSI2_DT_RAW8,
		.bpp = 8,
	},
	{
		.fourcc = V4L2_PIX_FMT_SRGGB10,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.csi_dt = MIPI_CSI2_DT_RAW10,
		.bpp = 16,
	},
};

static const unsigned int num_formats = ARRAY_SIZE(formats);

static const struct ti_csi2rx_fmt *find_format_by_pix(u32 pixelformat)
{
	unsigned int i;
	for (i = 0; i < num_formats; i++)
		if (formats[i].fourcc == pixelformat)
			return &formats[i];
	return &formats[0]; /* Default */
}

static const struct ti_csi2rx_fmt *find_format_by_code(u32 code)
{
	unsigned int i;
	for (i = 0; i < num_formats; i++)
		if (formats[i].code == code)
			return &formats[i];
	return &formats[0]; /* Default */
}

static void ti_csi2rx_fill_fmt(const struct ti_csi2rx_fmt *csi_fmt,
			       struct v4l2_format *v4l2_fmt)
{
	struct v4l2_pix_format *pix = &v4l2_fmt->fmt.pix;
    u8 bpp = csi_fmt->bpp;
	unsigned int pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;
	u32 bpl;

	pix->width = clamp_t(unsigned int, pix->width, pixels_in_word,
			     MAX_WIDTH_BYTES * 8 / bpp);
	pix->width = rounddown(pix->width, pixels_in_word);
	pix->height = clamp_t(unsigned int, pix->height, 1, MAX_HEIGHT_LINES);
	v4l2_fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pix->pixelformat = csi_fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->sizeimage = pix->height * pix->width * (bpp / 8);
	bpl = (pix->width * ALIGN(bpp, 8)) >> 3;
	pix->bytesperline = ALIGN(bpl, 16);
}

/* V4L2 IOCTL Operations */
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

	if (f->index >= num_formats)
		return -EINVAL;

	fmt = &formats[f->index];
	f->pixelformat = fmt->fourcc;
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	return 0;
}

static int ti_csi2rx_g_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct ti_csi2rx_ctx *ctx = video_drvdata(file);
	*f = ctx->v_fmt;
	return 0;
}

static int ti_csi2rx_try_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	const struct ti_csi2rx_fmt *fmt;

	fmt = find_format_by_pix(f->fmt.pix.pixelformat);
	if (!fmt)
		fmt = &formats[0];

	if (f->fmt.pix.field == V4L2_FIELD_ANY)
		f->fmt.pix.field = V4L2_FIELD_NONE;
	if (f->fmt.pix.field != V4L2_FIELD_NONE)
		f->fmt.pix.field = V4L2_FIELD_NONE;

	ti_csi2rx_fill_fmt(fmt, f);
	return 0;
}

static int ti_csi2rx_s_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct ti_csi2rx_ctx *ctx = video_drvdata(file);
	struct vb2_queue *q = &ctx->vidq;
	int ret;

	if (vb2_is_busy(q))
		return -EBUSY;

	ret = ti_csi2rx_try_fmt_vid_cap(file, priv, f);
	if (ret < 0)
		return ret;

	ctx->v_fmt = *f;
	return 0;
}

static int ti_csi2rx_enum_framesizes(struct file *file, void *fh,
				     struct v4l2_frmsizeenum *fsize)
{
	const struct ti_csi2rx_fmt *fmt;
	unsigned int pixels_in_word;
	u8 bpp;

	fmt = find_format_by_pix(fsize->pixel_format);
	if (!fmt || fsize->index != 0)
		return -EINVAL;

	bpp = ALIGN(fmt->bpp, 8);
	pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = pixels_in_word;
	fsize->stepwise.max_width = rounddown(MAX_WIDTH_BYTES * 8 / bpp, pixels_in_word);
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

/* Video Device Registration */
static inline int ti_csi2rx_video_register(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct video_device *vdev = &ctx->vdev;
	int ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	ret = media_create_pad_link(&csi->subdev.entity, TI_CSI2RX_PAD_SOURCE,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret) {
		video_unregister_device(vdev);
		return ret;
	}
	return 0;
}

/* Async Notifier Callbacks */
static int csi_async_notifier_bound(struct v4l2_async_notifier *notifier,
				    struct v4l2_subdev *subdev,
				    struct v4l2_async_subdev *asd)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
	csi->source = subdev; /* Bind to Cadence CSI2-RX */
	return 0;
}

static int csi_async_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
	int ret;

	/* Link Cadence CSI2-RX source to J721E sink */
	ret = v4l2_create_fwnode_links_to_pad(csi->source, &csi->pads[TI_CSI2RX_PAD_SINK],
					      MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	if (ret)
		return ret;

	ret = ti_csi2rx_video_register(&csi->ctx[0]);
	if (ret)
		return ret;

	return v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
}

static const struct v4l2_async_notifier_operations csi_async_notifier_ops = {
	.bound = csi_async_notifier_bound,
	.complete = csi_async_notifier_complete,
};

/* Subdevice Initialization */
static int ti_csi2rx_init_subdev(struct ti_csi2rx_dev *csi)
{
	struct fwnode_handle *fwnode;
	struct v4l2_async_subdev *asd;
	int ret;

	fwnode = fwnode_get_named_child_node(csi->dev->fwnode, "csi-bridge");
	if (!fwnode)
		return -EINVAL;

	v4l2_async_nf_init(&csi->notifier);
	csi->notifier.ops = &csi_async_notifier_ops;

	asd = v4l2_async_nf_add_fwnode(&csi->notifier, fwnode, struct v4l2_async_subdev);
	fwnode_handle_put(fwnode);
	if (IS_ERR(asd)) {
		v4l2_async_nf_cleanup(&csi->notifier);
		return PTR_ERR(asd);
	}

	ret = v4l2_async_nf_register(&csi->v4l2_dev, &csi->notifier);
	if (ret) {
		v4l2_async_nf_cleanup(&csi->notifier);
		return ret;
	}
	return 0;
}

/* DMA Operations */
static void ti_csi2rx_drain_callback(void *param)
{
	struct completion *drain_complete = param;
	complete(drain_complete);
}

static int ti_csi2rx_drain_dma(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct dma_async_tx_descriptor *desc;
	struct completion drain_complete;
	dma_cookie_t cookie;
	int ret;

	init_completion(&drain_complete);
	desc = dmaengine_prep_slave_single(ctx->dma.chan, csi->drain.paddr, csi->drain.len,
					   DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;

	desc->callback = ti_csi2rx_drain_callback;
	desc->callback_param = &drain_complete;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		return ret;

	dma_async_issue_pending(ctx->dma.chan);

	if (!wait_for_completion_timeout(&drain_complete, msecs_to_jiffies(DRAIN_TIMEOUT_MS))) {
		dmaengine_terminate_sync(ctx->dma.chan);
		dev_dbg(csi->dev, "DMA drain timed out\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int ti_csi2rx_dma_submit_pending(struct ti_csi2rx_ctx *ctx);

static int ti_csi2rx_dma_submit_pending(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dma *dma = &ctx->dma;
	struct ti_csi2rx_buffer *buf;
	int ret = 0;

	while (!list_empty(&dma->queue)) {
		buf = list_entry(dma->queue.next, struct ti_csi2rx_buffer, list);
		ret = ti_csi2rx_start_dma(ctx, buf);
		if (ret) {
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			break;
		}
		list_move_tail(&buf->list, &dma->submitted);
	}
	return ret;
}

static void ti_csi2rx_dma_callback(void *param)
{
	struct ti_csi2rx_buffer *buf = param;
	struct ti_csi2rx_ctx *ctx = buf->ctx;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	unsigned long flags;

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = ctx->sequence++;

	spin_lock_irqsave(&dma->lock, flags);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	list_del(&buf->list);
	ti_csi2rx_dma_submit_pending(ctx);
	if (list_empty(&dma->submitted))
		dma->state = TI_CSI2RX_DMA_IDLE;
	spin_unlock_irqrestore(&dma->lock, flags);
}

static int ti_csi2rx_start_dma(struct ti_csi2rx_ctx *ctx, struct ti_csi2rx_buffer *buf)
{
	unsigned long addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	struct dma_async_tx_descriptor *desc;
	size_t len = ctx->v_fmt.fmt.pix.sizeimage;
	dma_cookie_t cookie;
	int ret;

	desc = dmaengine_prep_slave_single(ctx->dma.chan, addr, len, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;

	desc->callback = ti_csi2rx_dma_callback;
	desc->callback_param = buf;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		return ret;

	dma_async_issue_pending(ctx->dma.chan);
	return 0;
}



static void ti_csi2rx_cleanup_buffers(struct ti_csi2rx_ctx *ctx, enum vb2_buffer_state buf_state)
{
	struct ti_csi2rx_buffer *buf, *tmp;
	enum ti_csi2rx_dma_state state;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ctx->dma.lock, flags);
	state = ctx->dma.state;
	ctx->dma.state = TI_CSI2RX_DMA_STOPPED;
	spin_unlock_irqrestore(&ctx->dma.lock, flags);

	if (state != TI_CSI2RX_DMA_STOPPED) {
		ret = ti_csi2rx_drain_dma(ctx);
		if (ret)
			dev_dbg(ctx->csi->dev, "Failed to drain DMA\n");
	}
	dmaengine_terminate_sync(ctx->dma.chan);

	spin_lock_irqsave(&ctx->dma.lock, flags);
	list_for_each_entry_safe(buf, tmp, &ctx->dma.queue, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, buf_state);
	}
	list_for_each_entry_safe(buf, tmp, &ctx->dma.submitted, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, buf_state);
	}
	spin_unlock_irqrestore(&ctx->dma.lock, flags);
}

/* VB2 Queue Operations */
static int ti_csi2rx_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
				 unsigned int *nplanes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(q);
	unsigned int size = ctx->v_fmt.fmt.pix.sizeimage;

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
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = ctx->v_fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(ctx->csi->dev, "Buffer too small\n");
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void ti_csi2rx_buffer_queue(struct vb2_buffer *vb)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ti_csi2rx_buffer *buf = container_of(vb, struct ti_csi2rx_buffer, vb.vb2_buf);
	struct ti_csi2rx_dma *dma = &ctx->dma;
	bool restart_dma = false;
	unsigned long flags;

	buf->ctx = ctx;

	spin_lock_irqsave(&dma->lock, flags);
	if (dma->state == TI_CSI2RX_DMA_IDLE) {
		restart_dma = true;
		dma->state = TI_CSI2RX_DMA_ACTIVE;
	} else {
		list_add_tail(&buf->list, &dma->queue);
	}
	spin_unlock_irqrestore(&dma->lock, flags);

	if (restart_dma) {
		int ret = ti_csi2rx_drain_dma(ctx);
		if (ret)
			dev_warn(ctx->csi->dev, "Failed to drain DMA\n");

		spin_lock_irqsave(&dma->lock, flags);
		ret = ti_csi2rx_start_dma(ctx, buf);
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
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	struct v4l2_subdev_krouting *routing;
	struct v4l2_subdev_route *route = NULL;
	struct media_pad *remote_pad;
	unsigned long flags;
	int ret, i;
	struct v4l2_subdev_state *state;

	ret = pm_runtime_resume_and_get(csi->dev);
	if (ret)
		return ret;

	spin_lock_irqsave(&dma->lock, flags);
	if (list_empty(&dma->queue))
		ret = -EIO;
	spin_unlock_irqrestore(&dma->lock, flags);
	if (ret)
		return ret;

	ret = video_device_pipeline_start(&ctx->vdev, &csi->pipe);
	if (ret)
		goto err;

	remote_pad = media_entity_remote_source_pad_unique(&csi->subdev.entity);
	if (!remote_pad) {
		ret = -ENODEV;
		goto err_pipeline;
	}

	state = v4l2_subdev_lock_and_get_active_state(&csi->subdev);
	routing = &state->routing;

	for (i = 0; i < routing->num_routes; i++) {
		struct v4l2_subdev_route *r = &routing->routes[i];
		if (!(r->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;
		if (r->source_pad != TI_CSI2RX_PAD_SOURCE)
			continue;
		route = r;
		break;
	}

	if (!route) {
		ret = -ENODEV;
		v4l2_subdev_unlock_state(state);
		goto err_pipeline;
	}

	ctx->stream = route->sink_stream;
	v4l2_subdev_unlock_state(state);

	/* Cadence CSI2-RX handles its own configuration */
	ctx->sequence = 0;

	spin_lock_irqsave(&dma->lock, flags);
	ret = ti_csi2rx_dma_submit_pending(ctx);
	if (ret) {
		spin_unlock_irqrestore(&dma->lock, flags);
		goto err_pipeline;
	}
	dma->state = TI_CSI2RX_DMA_ACTIVE;
	spin_unlock_irqrestore(&dma->lock, flags);

	ret = v4l2_subdev_enable_streams(&csi->subdev, TI_CSI2RX_PAD_SOURCE, BIT(0));
	if (ret)
		goto err_pipeline;

	return 0;

err_pipeline:
	video_device_pipeline_stop(&ctx->vdev);
err:
	ti_csi2rx_cleanup_buffers(ctx, VB2_BUF_STATE_QUEUED);
	pm_runtime_put(csi->dev);
	return ret;
}

static void ti_csi2rx_stop_streaming(struct vb2_queue *vq)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dev *csi = ctx->csi;
	int ret;

	video_device_pipeline_stop(&ctx->vdev);

	ret = v4l2_subdev_disable_streams(&csi->subdev, TI_CSI2RX_PAD_SOURCE, BIT(0));
	if (ret)
		dev_err(csi->dev, "Failed to stop subdev stream\n");

	ti_csi2rx_cleanup_buffers(ctx, VB2_BUF_STATE_ERROR);
	pm_runtime_put(csi->dev);
}

static const struct vb2_ops csi_vb2_qops = {
	.queue_setup = ti_csi2rx_queue_setup,
	.buf_prepare = ti_csi2rx_buffer_prepare,
	.buf_queue = ti_csi2rx_buffer_queue,
	.start_streaming = ti_csi2rx_start_streaming,
	.stop_streaming = ti_csi2rx_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* Subdevice Operations */
static inline struct ti_csi2rx_dev *to_csi2rx_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ti_csi2rx_dev, subdev);
}

static int ti_csi2rx_sd_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad == TI_CSI2RX_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, format);

	if (!find_format_by_code(format->format.code))
		format->format.code = formats[0].code;

	fmt = v4l2_subdev_state_get_stream_format(state, format->pad, format->stream);
	if (!fmt)
		return -EINVAL;
	*fmt = format->format;

	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad, format->stream);
	if (!fmt)
		return -EINVAL;
	*fmt = format->format;

	return 0;
}

static int _ti_csi2rx_sd_set_routing(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_krouting *routing)
{
	const struct v4l2_mbus_framefmt format = {
		.width = 1920,
		.height = 1080,
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
	};
	int ret;

	ret = v4l2_subdev_routing_validate(sd, routing, V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
}

static int ti_csi2rx_sd_set_routing(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    enum v4l2_subdev_format_whence which,
				    struct v4l2_subdev_krouting *routing)
{
	return _ti_csi2rx_sd_set_routing(sd, state, routing);
}

static int ti_csi2rx_sd_init_cfg(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = TI_CSI2RX_PAD_SINK,
			.sink_stream = 0,
			.source_pad = TI_CSI2RX_PAD_SOURCE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		}
	};
	struct v4l2_subdev_krouting routing = {
		.num_routes = 1,
		.routes = routes,
	};
	return _ti_csi2rx_sd_set_routing(sd, state, &routing);
}

static int ti_csi2rx_sd_all_sink_streams(struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_krouting *routing = &state->routing;
	u64 sink_streams = 0;
	int i;

	for (i = 0; i < routing->num_routes; i++) {
		struct v4l2_subdev_route *r = &routing->routes[i];
		if (r->sink_pad == TI_CSI2RX_PAD_SINK)
			sink_streams |= BIT(r->sink_stream);
	}
	return sink_streams;
}

static int ti_csi2rx_sd_enable_streams(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *state,
				       u32 pad, u64 streams_mask)
{
	struct ti_csi2rx_dev *csi = to_csi2rx_dev(sd);
	struct media_pad *remote_pad;
	int ret = 0;

	remote_pad = media_entity_remote_source_pad_unique(&csi->subdev.entity);
	if (!remote_pad)
		return -ENODEV;

	mutex_lock(&csi->mutex);
	if (!csi->enable_count) {
		u64 sink_streams = ti_csi2rx_sd_all_sink_streams(state);
		ret = v4l2_subdev_enable_streams(csi->source, remote_pad->index, sink_streams);
		if (ret)
			goto out;
		csi->enabled_streams_mask = sink_streams;
	}
	csi->enable_count++;
out:
	mutex_unlock(&csi->mutex);
	return ret;
}

static int ti_csi2rx_sd_disable_streams(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					u32 pad, u64 streams_mask)
{
	struct ti_csi2rx_dev *csi = to_csi2rx_dev(sd);
	struct media_pad *remote_pad;
	int ret = 0;

	remote_pad = media_entity_remote_source_pad_unique(&csi->subdev.entity);
	if (!remote_pad)
		return -ENODEV;

	mutex_lock(&csi->mutex);
	if (csi->enable_count == 0) {
		ret = -EINVAL;
		goto out;
	}

	if (csi->enable_count == 1) {
		u64 sink_streams = ti_csi2rx_sd_all_sink_streams(state);
		ret = v4l2_subdev_disable_streams(csi->source, remote_pad->index, sink_streams);
		if (ret)
			goto out;
		csi->enabled_streams_mask = 0;
	}
	--csi->enable_count;
out:
	mutex_unlock(&csi->mutex);
	return ret;
}

static const struct v4l2_subdev_pad_ops ti_csi2rx_subdev_pad_ops = {
	.init_cfg = ti_csi2rx_sd_init_cfg,
	.set_routing = ti_csi2rx_sd_set_routing,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ti_csi2rx_sd_set_fmt,
	.enable_streams = ti_csi2rx_sd_enable_streams,
	.disable_streams = ti_csi2rx_sd_disable_streams,
};

static const struct v4l2_subdev_ops ti_csi2rx_subdev_ops = {
	.pad = &ti_csi2rx_subdev_pad_ops,
};

/* Cleanup Functions */
static void ti_csi2rx_cleanup_dma(struct ti_csi2rx_ctx *ctx)
{
	dma_release_channel(ctx->dma.chan);
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

static void ti_csi2rx_cleanup_vb2q(struct ti_csi2rx_ctx *ctx)
{
	vb2_queue_release(&ctx->vidq);
}

static void ti_csi2rx_cleanup_ctx(struct ti_csi2rx_ctx *ctx)
{
	if (!pm_runtime_status_suspended(ctx->csi->dev))
		ti_csi2rx_cleanup_dma(ctx);
	ti_csi2rx_cleanup_vb2q(ctx);
	video_unregister_device(&ctx->vdev);
	mutex_destroy(&ctx->mutex);
}

/* Initialization Functions */
static int ti_csi2rx_init_vb2q(struct ti_csi2rx_ctx *ctx)
{
	struct vb2_queue *q = &ctx->vidq;
	int ret;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = ctx;
	q->buf_struct_size = sizeof(struct ti_csi2rx_buffer);
	q->ops = &csi_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = dmaengine_get_dma_device(ctx->dma.chan);
	q->lock = &ctx->mutex;
	q->min_buffers_needed = 1;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	ctx->vdev.queue = q;
	return 0;
}

static int ti_csi2rx_init_dma(struct ti_csi2rx_ctx *ctx)
{
	struct dma_slave_config cfg = { .src_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES };
	int ret;

	ctx->dma.chan = dma_request_chan(ctx->csi->dev, "rx0");
	if (IS_ERR(ctx->dma.chan))
		return PTR_ERR(ctx->dma.chan);

	ret = dmaengine_slave_config(ctx->dma.chan, &cfg);
	if (ret) {
		dma_release_channel(ctx->dma.chan);
		return ret;
	}
	return 0;
}

static int ti_csi2rx_v4l2_init(struct ti_csi2rx_dev *csi)
{
	struct media_device *mdev = &csi->mdev;
	struct v4l2_subdev *sd = &csi->subdev;
	int ret;

	mdev->dev = csi->dev;
	mdev->hw_revision = 1;
	strscpy(mdev->model, "TI-CSI2RX-Custom", sizeof(mdev->model));
	media_device_init(mdev);

	csi->v4l2_dev.mdev = mdev;
	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret)
		goto cleanup_media;

	ret = media_device_register(mdev);
	if (ret)
		goto unregister_v4l2;

	v4l2_subdev_init(sd, &ti_csi2rx_subdev_ops);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	strscpy(sd->name, dev_name(csi->dev), sizeof(sd->name));
	sd->dev = csi->dev;

	csi->pads[TI_CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	csi->pads[TI_CSI2RX_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, TI_CSI2RX_NUM_PADS, csi->pads);
	if (ret)
		goto unregister_media;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto unregister_media;

	ret = v4l2_device_register_subdev(&csi->v4l2_dev, sd);
	if (ret)
		goto cleanup_subdev;

	return 0;

cleanup_subdev:
	v4l2_subdev_cleanup(sd);
unregister_media:
	media_device_unregister(mdev);
unregister_v4l2:
	v4l2_device_unregister(&csi->v4l2_dev);
cleanup_media:
	media_device_cleanup(mdev);
	return ret;
}

static int ti_csi2rx_init_ctx(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct video_device *vdev = &ctx->vdev;
	const struct ti_csi2rx_fmt *fmt;
	struct v4l2_pix_format *pix_fmt = &ctx->v_fmt.fmt.pix;
	int ret;

	mutex_init(&ctx->mutex);

	fmt = find_format_by_pix(V4L2_PIX_FMT_SRGGB10);
	if (!fmt)
		return -EINVAL;

	pix_fmt->width = 1920;  /* Default to 1080p */
	pix_fmt->height = 1080;
	ti_csi2rx_fill_fmt(fmt, &ctx->v_fmt);

	ctx->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ctx->vdev.entity, 1, &ctx->pad);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), "%s ctx 0", dev_name(csi->dev));
	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->fops = &csi_fops;
	vdev->ioctl_ops = &csi_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;
	vdev->lock = &ctx->mutex;
	video_set_drvdata(vdev, ctx);

	INIT_LIST_HEAD(&ctx->dma.queue);
	INIT_LIST_HEAD(&ctx->dma.submitted);
	spin_lock_init(&ctx->dma.lock);
	ctx->dma.state = TI_CSI2RX_DMA_STOPPED;

	ret = ti_csi2rx_init_dma(ctx);
	if (ret)
		return ret;

	ret = ti_csi2rx_init_vb2q(ctx);
	if (ret)
		goto cleanup_dma;

	return 0;

cleanup_dma:
	ti_csi2rx_cleanup_dma(ctx);
	return ret;
}

/* Probe and Remove */
static int ti_csi2rx_probe(struct platform_device *pdev)
{
	struct ti_csi2rx_dev *csi;
	int ret;

	csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = &pdev->dev;
	platform_set_drvdata(pdev, csi);

	/* Single context for Cadence CSI2-RX compatibility */
	csi->ctx[0].idx = 0;
	csi->ctx[0].csi = csi;

	csi->drain.len = DRAIN_BUFFER_SIZE;
	csi->drain.vaddr = dma_alloc_coherent(csi->dev, csi->drain.len,
					      &csi->drain.paddr, GFP_KERNEL);
	if (!csi->drain.vaddr)
		return -ENOMEM;

	mutex_init(&csi->mutex);

	ret = ti_csi2rx_v4l2_init(csi);
	if (ret)
		goto cleanup_drain;

	ret = ti_csi2rx_init_ctx(&csi->ctx[0]);
	if (ret)
		goto cleanup_v4l2;

	ret = ti_csi2rx_init_subdev(csi);
	if (ret)
		goto cleanup_ctx;

	ret = of_platform_populate(csi->dev->of_node, NULL, NULL, csi->dev);
	if (ret) {
		dev_err(csi->dev, "Failed to populate children: %d\n", ret);
		goto cleanup_subdev;
	}

	pm_runtime_set_active(csi->dev);
	pm_runtime_enable(csi->dev);
	pm_runtime_idle(csi->dev);

	return 0;

cleanup_subdev:
	ti_csi2rx_cleanup_subdev(csi);
cleanup_ctx:
	ti_csi2rx_cleanup_ctx(&csi->ctx[0]);
cleanup_v4l2:
	ti_csi2rx_cleanup_v4l2(csi);
cleanup_drain:
	mutex_destroy(&csi->mutex);
	dma_free_coherent(csi->dev, csi->drain.len, csi->drain.vaddr, csi->drain.paddr);
	return ret;
}

static int ti_csi2rx_remove(struct platform_device *pdev)
{
	struct ti_csi2rx_dev *csi = platform_get_drvdata(pdev);

	if (vb2_is_busy(&csi->ctx[0].vidq))
		return -EBUSY;

	ti_csi2rx_cleanup_ctx(&csi->ctx[0]);
	ti_csi2rx_cleanup_subdev(csi);
	ti_csi2rx_cleanup_v4l2(csi);

	mutex_destroy(&csi->mutex);
	dma_free_coherent(csi->dev, csi->drain.len, csi->drain.vaddr, csi->drain.paddr);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
	return 0;
}

/* Device Tree Matching */
static const struct of_device_id ti_csi2rx_of_match[] = {
	{ .compatible = "ti,j721e-csi2rx", },
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

MODULE_DESCRIPTION("Custom TI J721E CSI2-RX Driver for Cadence CSI2-RX and IMX219");
MODULE_AUTHOR("[Your Name]");
MODULE_LICENSE("GPL");