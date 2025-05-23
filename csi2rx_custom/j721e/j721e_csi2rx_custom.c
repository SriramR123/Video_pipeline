/**
 * @file ti_csi2rx_alt.c
 * @brief TI J721E CSI-2 RX Alternate Driver
 * @author RAM
 * @date April 20, 2025
 * @copyright Copyright (C) 2025 [Your Organization or Name]
 *
 * This driver manages the Texas Instruments J721E SoC's MIPI CSI-2 Receiver (RX)
 * controller, interfacing with camera sensors to capture video data. It operates
 * as a V4L2 capture device, supporting up to 32 contexts (virtual channels or
 * streams), DMA-based data transfer, and integration with the Linux media
 * controller framework. The driver supports YUV422, RAW8, RAW10, and RAW12 formats
 * and includes runtime power management for efficient operation.
 *
 * SPDX-License-Identifier: GPL-2.0+
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
 
 /** @brief Module name for the TI CSI-2 RX driver */
 #define TI_CSI2RX_ALT_MODULE_NAME	"j721e-csi2rx-alt"
 
 /** @brief SHIM control register offset */
 #define SHIM_CONTROL			0x10
 /** @brief Pixel reset bit in SHIM control register */
 #define SHIM_CONTROL_PIX_RESET		BIT(0)
 
 /** @brief SHIM DMA control register offset for context i */
 #define SHIM_DMA_CONTROL(i)		(0x20 + ((i) * 0x20))
 /** @brief DMA enable bit */
 #define SHIM_DMA_CONTROL_ENABLE		BIT(31)
 /** @brief YUV422 mode field */
 #define SHIM_DMA_CONTROL_YUV422		GENMASK(27, 26)
 /** @brief Data size field */
 #define SHIM_DMA_CONTROL_SIZE		GENMASK(21, 20)
 /** @brief Virtual channel field */
 #define SHIM_DMA_CONTROL_VC		GENMASK(9, 6)
 /** @brief Format field (CSI-2 data type) */
 #define SHIM_DMA_CONTROL_FORMAT		GENMASK(5, 0)
 /** @brief YUV422 mode 1:1 */
 #define SHIM_DMA_CONTROL_YUV422_MODE_11	3
 /** @brief 8-bit data size */
 #define SHIM_DMA_CONTROL_SIZE_8		0
 /** @brief 16-bit data size */
 #define SHIM_DMA_CONTROL_SIZE_16	1
 /** @brief 32-bit data size */
 #define SHIM_DMA_CONTROL_SIZE_32	2
 
 /** @brief SHIM PSI-L configuration register offset for context i */
 #define SHIM_PSI_CONFIG(i)		(0x24 + ((i) * 0x20))
 /** @brief Source tag field */
 #define SHIM_PSI_CONFIG_SRC_TAG		GENMASK(15, 0)
 /** @brief Destination tag field */
 #define SHIM_PSI_CONFIG_DST_TAG		GENMASK(31, 16)
 
 /** @brief PSI-L word size in bytes */
 #define PSIL_WORD_SIZE_BYTES		16
 /** @brief Maximum number of contexts */
 #define TI_CSI2RX_ALT_MAX_CTX		32
 
 /** @brief Maximum frame width in bytes */
 #define MAX_WIDTH_BYTES			SZ_16K
 /** @brief Maximum frame height in lines */
 #define MAX_HEIGHT_LINES		SZ_16K
 
 /** @brief Sink pad index */
 #define TI_CSI2RX_ALT_PAD_SINK		0
 /** @brief First source pad index */
 #define TI_CSI2RX_ALT_PAD_FIRST_SOURCE	1
 /** @brief Maximum number of source pads */
 #define TI_CSI2RX_ALT_MAX_SOURCE_PADS	TI_CSI2RX_ALT_MAX_CTX
 /** @brief Total number of pads (1 sink + max source) */
 #define TI_CSI2RX_ALT_MAX_PADS		(1 + TI_CSI2RX_ALT_MAX_SOURCE_PADS)
 
 /** @brief DMA drain timeout in milliseconds */
 #define DRAIN_TIMEOUT_MS		50
 /** @brief DMA drain buffer size */
 #define DRAIN_BUFFER_SIZE		SZ_32K
 
 /**
  * @brief Structure defining supported pixel formats
  *
  * This structure maps V4L2 pixel formats to media bus codes, CSI-2 data types,
  * bits per pixel, and DMA data sizes.
  */
 struct ti_csi2rx_alt_fmt {
	 u32				fourcc;		/**< V4L2 pixel format (fourcc) */
	 u32				code;		/**< Media bus code */
	 u32				csi_dt;		/**< MIPI CSI-2 data type */
	 u8				bpp;		/**< Bits per pixel */
	 u8				size;		/**< DMA data size (8/16/32 bits) */
 };
 
 /**
  * @brief Structure representing a video buffer
  *
  * This structure encapsulates a V4L2 buffer, its context, and list linkage for
  * DMA queue management.
  */
 struct ti_csi2rx_alt_buffer {
	 struct vb2_v4l2_buffer		vb;		/**< V4L2 buffer */
	 struct list_head		list;		/**< Linked list for queue */
	 struct ti_csi2rx_alt_ctx	*ctx;		/**< Owning context */
 };
 
 /**
  * @brief Enum defining DMA states
  *
  * Represents the operational state of the DMA engine for a context.
  */
 enum ti_csi2rx_alt_dma_state {
	 TI_CSI2RX_ALT_DMA_STOPPED,			/**< DMA is inactive */
	 TI_CSI2RX_ALT_DMA_IDLE,				/**< DMA is ready but idle */
	 TI_CSI2RX_ALT_DMA_ACTIVE,			/**< DMA is actively transferring */
 };
 
 /**
  * @brief Structure managing DMA operations for a context
  *
  * This structure handles DMA channel configuration, buffer queues, and state.
  */
 struct ti_csi2rx_alt_dma {
	 spinlock_t			lock;		/**< Spinlock for thread safety */
	 struct dma_chan			*chan;		/**< DMA channel */
	 struct list_head		queue;		/**< Pending buffer queue */
	 enum ti_csi2rx_alt_dma_state	state;		/**< Current DMA state */
	 struct list_head		submitted;	/**< Submitted buffer queue */
 };
 
 /** Forward declaration of the device structure */
 struct ti_csi2rx_alt_dev;
 
 /**
  * @brief Structure representing a capture context
  *
  * Each context corresponds to a virtual channel or stream, managing its own
  * video device, VB2 queue, DMA, and format.
  */
 struct ti_csi2rx_alt_ctx {
	 struct ti_csi2rx_alt_dev	*csi;		/**< Parent device */
	 struct video_device		vdev;		/**< V4L2 video device */
	 struct vb2_queue		vidq;		/**< VB2 buffer queue */
	 struct mutex			mutex;		/**< Context mutex */
	 struct v4l2_format		v_fmt;		/**< Current format */
	 struct ti_csi2rx_alt_dma	dma;		/**< DMA configuration */
	 struct media_pad		pad;		/**< Media pad */
	 u32				sequence;	/**< Frame sequence number */
	 u32				idx;		/**< Context index (0-31) */
	 u32				vc;		/**< Virtual channel */
	 u32				stream;		/**< Stream ID */
 };
 
 /**
  * @brief Structure representing the CSI-2 RX device
  *
  * This structure encapsulates the entire CSI-2 RX controller, including hardware
  * registers, V4L2 device, subdevice, contexts, and media pipeline.
  */
 struct ti_csi2rx_alt_dev {
	 struct device			*dev;		/**< Platform device */
	 void __iomem			*shim;		/**< SHIM register base */
	 struct mutex			mutex;		/**< Device mutex */
	 unsigned int			enable_count;	/**< Active context count */
	 unsigned int			num_ctx;	/**< Number of contexts */
	 struct v4l2_async_notifier	notifier;	/**< Async notifier */
	 struct media_device		mdev;		/**< Media device */
	 struct media_pipeline		pipe;		/**< Media pipeline */
	 struct media_pad		pads[TI_CSI2RX_ALT_MAX_PADS]; /**< Media pads */
	 struct v4l2_device		v4l2_dev;	/**< V4L2 device */
	 struct v4l2_subdev		*source;	/**< Remote sensor subdevice */
	 struct v4l2_subdev		subdev;		/**< CSI-2 RX subdevice */
	 struct ti_csi2rx_alt_ctx	ctx[TI_CSI2RX_ALT_MAX_CTX]; /**< Contexts */
	 u64				enabled_streams_mask; /**< Enabled streams */
	 struct {
		 void			*vaddr;		/**< Drain buffer virtual address */
		 dma_addr_t		paddr;		/**< Drain buffer physical address */
		 size_t			len;		/**< Drain buffer size */
	 } drain;					/**< DMA drain buffer */
 };
 
 /**
  * @brief Start DMA transfer for a buffer
  * @param ctx Context owning the buffer
  * @param buf Buffer to transfer
  * @return 0 on success, negative error code on failure
  *
  * Prepares and submits a DMA transfer for the specified buffer.
  */
 static int ti_csi2rx_alt_start_dma(struct ti_csi2rx_alt_ctx *ctx,
					struct ti_csi2rx_alt_buffer *buf);
 
 /**
  * @brief Array of supported formats
  *
  * Defines the mapping of V4L2 pixel formats to media bus codes, CSI-2 data types,
  * bits per pixel, and DMA sizes.
  */
 static const struct ti_csi2rx_alt_fmt formats[] = {
	 {
		 .fourcc		= V4L2_PIX_FMT_YUYV,
		 .code		= MEDIA_BUS_FMT_YUYV8_1X16,
		 .csi_dt		= MIPI_CSI2_DT_YUV422_8B,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_UYVY,
		 .code		= MEDIA_BUS_FMT_UYVY8_1X16,
		 .csi_dt		= MIPI_CSI2_DT_YUV422_8B,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_YVYU,
		 .code		= MEDIA_BUS_FMT_YVYU8_1X16,
		 .csi_dt		= MIPI_CSI2_DT_YUV422_8B,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_VYUY,
		 .code		= MEDIA_BUS_FMT_VYUY8_1X16,
		 .csi_dt		= MIPI_CSI2_DT_YUV422_8B,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SBGGR8,
		 .code		= MEDIA_BUS_FMT_SBGGR8_1X8,
		 .csi_dt		= MIPI_CSI2_DT_RAW8,
		 .bpp		= 8,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGBRG8,
		 .code		= MEDIA_BUS_FMT_SGBRG8_1X8,
		 .csi_dt		= MIPI_CSI2_DT_RAW8,
		 .bpp		= 8,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGRBG8,
		 .code		= MEDIA_BUS_FMT_SGRBG8_1X8,
		 .csi_dt		= MIPI_CSI2_DT_RAW8,
		 .bpp		= 8,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SRGGB8,
		 .code		= MEDIA_BUS_FMT_SRGGB8_1X8,
		 .csi_dt		= MIPI_CSI2_DT_RAW8,
		 .bpp		= 8,
		 .size		= SHIM_DMA_CONTROL_SIZE_8,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SBGGR10,
		 .code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGBRG10,
		 .code		= MEDIA_BUS_FMT_SGBRG10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGRBG10,
		 .code		= MEDIA_BUS_FMT_SGRBG10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SRGGB10,
		 .code		= MEDIA_BUS_FMT_SRGGB10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SBGGR12,
		 .code		= MEDIA_BUS_FMT_SBGGR12_1X12,
		 .csi_dt		= MIPI_CSI2_DT_RAW12,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGBRG12,
		 .code		= MEDIA_BUS_FMT_SGBRG12_1X12,
		 .csi_dt		= MIPI_CSI2_DT_RAW12,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGRBG12,
		 .code		= MEDIA_BUS_FMT_SGRBG12_1X12,
		 .csi_dt		= MIPI_CSI2_DT_RAW12,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SRGGB12,
		 .code		= MEDIA_BUS_FMT_SRGGB12_1X12,
		 .csi_dt		= MIPI_CSI2_DT_RAW12,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SRGGI10,
		 .code		= MEDIA_BUS_FMT_SRGGI10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGRIG10,
		 .code		= MEDIA_BUS_FMT_SGRIG10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SBGGI10,
		 .code		= MEDIA_BUS_FMT_SBGGI10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGBIG10,
		 .code		= MEDIA_BUS_FMT_SGBIG10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGIRG10,
		 .code		= MEDIA_BUS_FMT_SGIRG10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SIGGR10,
		 .code		= MEDIA_BUS_FMT_SIGGR10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SGIBG10,
		 .code		= MEDIA_BUS_FMT_SGIBG10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 }, {
		 .fourcc		= V4L2_PIX_FMT_SIGGB10,
		 .code		= MEDIA_BUS_FMT_SIGGB10_1X10,
		 .csi_dt		= MIPI_CSI2_DT_RAW10,
		 .bpp		= 16,
		 .size		= SHIM_DMA_CONTROL_SIZE_16,
	 },
 };
 
 /** @brief Number of supported formats */
 static const unsigned int num_formats = ARRAY_SIZE(formats);
 
 /**
  * @brief Find format by V4L2 pixel format
  * @param pixelformat V4L2 pixel format (fourcc)
  * @return Pointer to the format structure, or NULL if not found
  */
 static const struct ti_csi2rx_alt_fmt *find_format_by_pix(u32 pixelformat)
 {
	 unsigned int i;
 
	 for (i = 0; i < num_formats; i++) {
		 if (formats[i].fourcc == pixelformat)
			 return &formats[i];
	 }
 
	 return NULL;
 }
 
 /**
  * @brief Find format by media bus code
  * @param code Media bus code
  * @return Pointer to the format structure, or NULL if not found
  */
 static const struct ti_csi2rx_alt_fmt *find_format_by_code(u32 code)
 {
	 unsigned int i;
 
	 for (i = 0; i < num_formats; i++) {
		 if (formats[i].code == code)
			 return &formats[i];
	 }
 
	 return NULL;
 }
 
 /**
  * @brief Fill V4L2 format structure based on CSI format
  * @param csi_fmt CSI format structure
  * @param v4l2_fmt V4L2 format to fill
  *
  * Adjusts width, height, and other format parameters to comply with hardware
  * constraints and DMA alignment.
  */
 static void ti_csi2rx_alt_fill_fmt(const struct ti_csi2rx_alt_fmt *csi_fmt,
					struct v4l2_format *v4l2_fmt)
 {
	 struct v4l2_pix_format *pix = &v4l2_fmt->fmt.pix;
	 unsigned int pixels_in_word;
	 u8 bpp = csi_fmt->bpp;
	 u32 bpl;
 
	 pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;
 
	 pix->width = clamp_t(unsigned int, pix->width,
				  pixels_in_word,
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
 
 /**
  * @brief Query driver capabilities
  * @param file File handle
  * @param priv Private data
  * @param cap V4L2 capability structure
  * @return 0 on success
  */
 static int ti_csi2rx_alt_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
 {
	 strscpy(cap->driver, TI_CSI2RX_ALT_MODULE_NAME, sizeof(cap->driver));
	 strscpy(cap->card, TI_CSI2RX_ALT_MODULE_NAME, sizeof(cap->card));
 
	 return 0;
 }
 
 /**
  * @brief Enumerate supported video capture formats
  * @param file File handle
  * @param priv Private data
  * @param f Format description structure
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_enum_fmt_vid_cap(struct file *file, void *priv,
					  struct v4l2_fmtdesc *f)
 {
	 const struct ti_csi2rx_alt_fmt *fmt = NULL;
 
	 if (f->mbus_code) {
		 if (f->index > 0)
			 return -EINVAL;
 
		 fmt = find_format_by_code(f->mbus_code);
	 } else {
		 if (f->index >= num_formats)
			 return -EINVAL;
 
		 fmt = &formats[f->index];
	 }
 
	 if (!fmt)
		 return -EINVAL;
 
	 f->pixelformat = fmt->fourcc;
	 memset(f->reserved, 0, sizeof(f->reserved));
	 f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
 
	 return 0;
 }
 
 /**
  * @brief Get current video capture format
  * @param file File handle
  * @param prov Private data
  * @param f V4L2 format structure
  * @return 0 on success
  */
 static int ti_csi2rx_alt_g_fmt_vid_cap(struct file *file, void *prov,
						struct v4l2_format *f)
 {
	 struct ti_csi2rx_alt_ctx *ctx = video_drvdata(file);
 
	 *f = ctx->v_fmt;
 
	 return 0;
 }
 
 /**
  * @brief Try a video capture format
  * @param file File handle
  * @param priv Private data
  * @param f V4L2 format structure
  * @return 0 on success, negative error code on failure
  *
  * Validates and adjusts the format to ensure compatibility with hardware.
  */
 static int ti_csi2rx_alt_try_fmt_vid_cap(struct file *file, void *priv,
					  struct v4l2_format *f)
 {
	 const struct ti_csi2rx_alt_fmt *fmt;
 
	 fmt = find_format_by_pix(f->fmt.pix.pixelformat);
	 if (!fmt)
		 fmt = &formats[0];
 
	 if (f->fmt.pix.field == V4L2_FIELD_ANY)
		 f->fmt.pix.field = V4L2_FIELD_NONE;
 
	 if (f->fmt.pix.field != V4L2_FIELD_NONE)
		 f->fmt.pix.field = V4L2_FIELD_NONE;
 
	 ti_csi2rx_alt_fill_fmt(fmt, f);
 
	 return 0;
 }
 
 /**
  * @brief Set video capture format
  * @param file File handle
  * @param priv Private data
  * @param f V4L2 format structure
  * @return 0 on success, negative error code on failure
  *
  * Applies the format if the VB2 queue is not busy.
  */
 static int ti_csi2rx_alt_s_fmt_vid_cap(struct file *file, void *priv,
						struct v4l2_format *f)
 {
	 struct ti_csi2rx_alt_ctx *ctx = video_drvdata(file);
	 struct vb2_queue *q = &ctx->vidq;
	 int ret;
 
	 if (vb2_is_busy(q))
		 return -EBUSY;
 
	 ret = ti_csi2rx_alt_try_fmt_vid_cap(file, priv, f);
	 if (ret < 0)
		 return ret;
 
	 ctx->v_fmt = *f;
 
	 return 0;
 }
 
 /**
  * @brief Enumerate frame sizes
  * @param file File handle
  * @param fh File handle data
  * @param fsize Frame size enumeration structure
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_enum_framesizes(struct file *file, void *fh,
					  struct v4l2_frmsizeenum *fsize)
 {
	 const struct ti_csi2rx_alt_fmt *fmt;
	 unsigned int pixels_in_word;
	 u8 bpp;
 
	 fmt = find_format_by_pix(fsize->pixel_format);
	 if (!fmt || fsize->index != 0)
		 return -EINVAL;
 
	 bpp = ALIGN(fmt->bpp, 8);
 
	 pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;
 
	 fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	 fsize->stepwise.min_width = pixels_in_word;
	 fsize->stepwise.max_width = rounddown(MAX_WIDTH_BYTES * 8 / bpp,
						  pixels_in_word);
	 fsize->stepwise.step_width = pixels_in_word;
	 fsize->stepwise.min_height = 1;
	 fsize->stepwise.max_height = MAX_HEIGHT_LINES;
	 fsize->stepwise.step_height = 1;
 
	 return 0;
 }
 
 /**
  * @brief V4L2 IOCTL operations
  *
  * Defines the IOCTL handlers for video capture operations.
  */
 static const struct v4l2_ioctl_ops csi_alt_ioctl_ops = {
	 .vidioc_querycap      = ti_csi2rx_alt_querycap,
	 .vidioc_enum_fmt_vid_cap = ti_csi2rx_alt_enum_fmt_vid_cap,
	 .vidioc_try_fmt_vid_cap = ti_csi2rx_alt_try_fmt_vid_cap,
	 .vidioc_g_fmt_vid_cap = ti_csi2rx_alt_g_fmt_vid_cap,
	 .vidioc_s_fmt_vid_cap = ti_csi2rx_alt_s_fmt_vid_cap,
	 .vidioc_enum_framesizes = ti_csi2rx_alt_enum_framesizes,
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
 
 /**
  * @brief V4L2 file operations
  *
  * Defines the file operations for the video device.
  */
 static const struct v4l2_file_operations csi_alt_fops = {
	 .owner = THIS_MODULE,
	 .open = v4l2_fh_open,
	 .release = vb2_fop_release,
	 .read = vb2_fop_read,
	 .poll = vb2_fop_poll,
	 .unlocked_ioctl = video_ioctl2,
	 .mmap = vb2_fop_mmap,
 };
 
 /**
  * @brief Register a video device for a context
  * @param ctx Context to register
  * @return 0 on success, negative error code on failure
  *
  * Registers the video device and creates a media link to the subdevice.
  */
 static inline int ti_csi2rx_alt_video_register(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 struct video_device *vdev = &ctx->vdev;
	 int ret;
 
	 ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	 if (ret)
		 return ret;
 
	 ret = media_create_pad_link(&csi->subdev.entity,
					 TI_CSI2RX_ALT_PAD_FIRST_SOURCE + ctx->idx,
					 &vdev->entity, 0,
					 MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
 
	 if (ret) {
		 video_unregister_device(vdev);
		 return ret;
	 }
 
	 return 0;
 }
 
 /**
  * @brief Async notifier callback for subdevice binding
  * @param notifier V4L2 async notifier
  * @param subdev Subdevice being bound
  * @param asd Async subdevice descriptor
  * @return 0 on success, negative error code on failure
  */
 static int csi_alt_async_notifier_bound(struct v4l2_async_notifier *notifier,
					 struct v4l2_subdev *subdev,
					 struct v4l2_async_subdev *asd)
 {
	 struct ti_csi2rx_alt_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
 
	 WARN_ON(csi->source);
 
	 csi->source = subdev;
 
	 return 0;
 }
 
 /**
  * @brief Async notifier callback for completion
  * @param notifier V4L2 async notifier
  * @return 0 on success, negative error code on failure
  *
  * Creates media links and registers video devices for all contexts.
  */
 static int csi_alt_async_notifier_complete(struct v4l2_async_notifier *notifier)
 {
	 struct ti_csi2rx_alt_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
	 int ret, i;
 
	 ret = v4l2_create_fwnode_links_to_pad(csi->source, &csi->pads[TI_CSI2RX_ALT_PAD_SINK],
						   MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
	 if (ret)
		 return ret;
 
	 for (i = 0; i < csi->num_ctx; i++) {
		 ret = ti_csi2rx_alt_video_register(&csi->ctx[i]);
		 if (ret)
			 return ret;
	 }
 
	 return v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
 }
 
 /**
  * @brief Async notifier operations
  */
 static const struct v4l2_async_notifier_operations csi_alt_async_notifier_ops = {
	 .bound = csi_alt_async_notifier_bound,
	 .complete = csi_alt_async_notifier_complete,
 };
 
 /**
  * @brief Initialize the CSI-2 RX subdevice
  * @param csi Device structure
  * @return 0 on success, negative error code on failure
  *
  * Sets up the async notifier for binding with the remote sensor subdevice.
  */
 static int ti_csi2rx_alt_init_subdev(struct ti_csi2rx_alt_dev *csi)
 {
	 struct fwnode_handle *fwnode;
	 struct v4l2_async_subdev *asd;
	 int ret;
 
	 fwnode = fwnode_get_named_child_node(csi->dev->fwnode, "csi-bridge");
	 if (!fwnode)
		 return -EINVAL;
 
	 v4l2_async_nf_init(&csi->notifier);
	 csi->notifier.ops = &csi_alt_async_notifier_ops;
 
	 asd = v4l2_async_nf_add_fwnode(&csi->notifier, fwnode,
						struct v4l2_async_subdev);
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
 
 /**
  * @brief Configure SHIM registers for a context
  * @param ctx Context to configure
  *
  * Sets up DMA control and PSI-L configuration based on format and virtual channel.
  */
 static void ti_csi2rx_alt_setup_shim(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 const struct ti_csi2rx_alt_fmt *fmt;
	 unsigned int reg;
 
	 fmt = find_format_by_pix(ctx->v_fmt.fmt.pix.pixelformat);
	 if (!fmt) {
		 dev_err(csi->dev, "Unknown format\n");
		 return;
	 }
 
	 if (!csi->enable_count) {
		 reg = SHIM_CONTROL_PIX_RESET;
		 writel(reg, csi->shim + SHIM_CONTROL);
	 }
 
	 reg = SHIM_DMA_CONTROL_ENABLE;
	 reg |= FIELD_PREP(SHIM_DMA_CONTROL_FORMAT, fmt->csi_dt);
 
	 switch (fmt->fourcc) {
	 case V4L2_PIX_FMT_UYVY:
	 case V4L2_PIX_FMT_VYUY:
	 case V4L2_PIX_FMT_YUYV:
	 case V4L2_PIX_FMT_YVYU:
		 reg |= FIELD_PREP(SHIM_DMA_CONTROL_YUV422,
				   SHIM_DMA_CONTROL_YUV422_MODE_11);
		 break;
	 default:
		 break;
	 }
 
	 reg |= FIELD_PREP(SHIM_DMA_CONTROL_SIZE, fmt->size);
	 reg |= FIELD_PREP(SHIM_DMA_CONTROL_VC, ctx->vc);
 
	 writel(reg, csi->shim + SHIM_DMA_CONTROL(ctx->idx));
 
	 reg = FIELD_PREP(SHIM_PSI_CONFIG_SRC_TAG, 0) |
		   FIELD_PREP(SHIM_PSI_CONFIG_DST_TAG, 0);
	 writel(reg, csi->shim + SHIM_PSI_CONFIG(ctx->idx));
 }
 
 /**
  * @brief DMA drain callback
  * @param param Completion structure
  *
  * Signals completion of the DMA drain operation.
  */
 static void ti_csi2rx_alt_drain_callback(void *param)
 {
	 struct completion *drain_complete = param;
 
	 complete(drain_complete);
 }
 
 /**
  * @brief Drain DMA to flush pending data
  * @param ctx Context to drain
  * @return 0 on success, negative error code on failure
  *
  * Transfers data to a drain buffer to ensure clean DMA termination.
  */
 static int ti_csi2rx_alt_drain_dma(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 struct dma_async_tx_descriptor *desc;
	 struct completion drain_complete;
	 dma_cookie_t cookie;
	 int ret;
 
	 init_completion(&drain_complete);
 
	 desc = dmaengine_prep_slave_single(ctx->dma.chan, csi->drain.paddr,
						csi->drain.len, DMA_DEV_TO_MEM,
						DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	 if (!desc) {
		 ret = -EIO;
		 goto out;
	 }
 
	 desc->callback = ti_csi2rx_alt_drain_callback;
	 desc->callback_param = &drain_complete;
 
	 cookie = dmaengine_submit(desc);
	 ret = dma_submit_error(cookie);
	 if (ret)
		 goto out;
 
	 dma_async_issue_pending(ctx->dma.chan);
 
	 if (!wait_for_completion_timeout(&drain_complete,
					  msecs_to_jiffies(DRAIN_TIMEOUT_MS))) {
		 dmaengine_terminate_sync(ctx->dma.chan);
		 dev_dbg(csi->dev, "DMA transfer timed out for drain buffer\n");
		 ret = -ETIMEDOUT;
		 goto out;
	 }
 out:
	 return ret;
 }
 
 /**
  * @brief Submit pending DMA buffers
  * @param ctx Context with pending buffers
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_dma_submit_pending(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct ti_csi2rx_alt_dma *dma = &ctx->dma;
	 struct ti_csi2rx_alt_buffer *buf;
	 int ret = 0;
 
	 while (!list_empty(&dma->queue)) {
		 buf = list_entry(dma->queue.next, struct ti_csi2rx_alt_buffer, list);
		 ret = ti_csi2rx_alt_start_dma(ctx, buf);
		 if (ret) {
			 dev_err(ctx->csi->dev,
				 "Failed to queue the next buffer for DMA\n");
			 vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
			 break;
		 }
		 list_move_tail(&buf->list, &dma->submitted);
	 }
	 return ret;
 }
 
 /**
  * @brief DMA completion callback
  * @param param Buffer that completed transfer
  *
  * Marks the buffer as done, updates sequence, and queues the next buffer.
  */
 static void ti_csi2rx_alt_dma_callback(void *param)
 {
	 struct ti_csi2rx_alt_buffer *buf = param;
	 struct ti_csi2rx_alt_ctx *ctx = buf->ctx;
	 struct ti_csi2rx_alt_dma *dma = &ctx->dma;
	 unsigned long flags = 0;
 
	 buf->vb.vb2_buf.timestamp = ktime_get_ns();
	 buf->vb.sequence = ctx->sequence++;
 
	 spin_lock_irqsave(&dma->lock, flags);
 
	 WARN_ON(!list_is_first(&buf->list, &dma->submitted));
	 vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	 list_del(&buf->list);
 
	 ti_csi2rx_alt_dma_submit_pending(ctx);
 
	 if (list_empty(&dma->submitted))
		 dma->state = TI_CSI2RX_ALT_DMA_IDLE;
 
	 spin_unlock_irqrestore(&dma->lock, flags);
 }
 
 /**
  * @brief Start DMA transfer for a buffer
  * @param ctx Context owning the buffer
  * @param buf Buffer to transfer
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_start_dma(struct ti_csi2rx_alt_ctx *ctx,
					struct ti_csi2rx_alt_buffer *buf)
 {
	 unsigned long addr;
	 struct dma_async_tx_descriptor *desc;
	 size_t len = ctx->v_fmt.fmt.pix.sizeimage;
	 dma_cookie_t cookie;
	 int ret = 0;
 
	 addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	 desc = dmaengine_prep_slave_single(ctx->dma.chan, addr, len,
						DMA_DEV_TO_MEM,
						DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	 if (!desc)
		 return -EIO;
 
	 desc->callback = ti_csi2rx_alt_dma_callback;
	 desc->callback_param = buf;
 
	 cookie = dmaengine_submit(desc);
	 ret = dma_submit_error(cookie);
	 if (ret)
		 return ret;
 
	 dma_async_issue_pending(ctx->dma.chan);
 
	 return 0;
 }
 
 /**
  * @brief Clean up DMA buffers
  * @param ctx Context to clean up
  * @param buf_state State to set for buffers (e.g., VB2_BUF_STATE_ERROR)
  *
  * Terminates DMA and returns all queued or submitted buffers.
  */
 static void ti_csi2rx_alt_cleanup_buffers(struct ti_csi2rx_alt_ctx *ctx,
					  enum vb2_buffer_state buf_state)
 {
	 struct ti_csi2rx_alt_dma *dma = &ctx->dma;
	 struct ti_csi2rx_alt_buffer *buf = NULL, *tmp;
	 enum ti_csi2rx_alt_dma_state state;
	 unsigned long flags;
	 int ret;
 
	 spin_lock_irqsave(&dma->lock, flags);
	 state = ctx->dma.state;
	 dma->state = TI_CSI2RX_ALT_DMA_STOPPED;
	 spin_unlock_irqrestore(&dma->lock, flags);
 
	 if (state != TI_CSI2RX_ALT_DMA_STOPPED) {
		 ret = ti_csi2rx_alt_drain_dma(ctx);
		 if (ret)
			 dev_dbg(ctx->csi->dev,
				 "Failed to drain DMA. Next frame might be bogus\n");
	 }
	 dmaengine_terminate_sync(ctx->dma.chan);
 
	 spin_lock_irqsave(&dma->lock, flags);
	 list_for_each_entry_safe(buf, tmp, &ctx->dma.queue, list) {
		 list_del(&buf->list);
		 vb2_buffer_done(&buf->vb.vb2_buf, buf_state);
	 }
	 list_for_each_entry_safe(buf, tmp, &ctx->dma.submitted, list) {
		 list_del(&buf->list);
		 vb2_buffer_done(&buf->vb.vb2_buf, buf_state);
	 }
	 spin_unlock_irqrestore(&dma->lock, flags);
 }
 
 /**
  * @brief Restart DMA after draining
  * @param ctx Context to restart
  * @param buf Buffer to start with
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_restart_dma(struct ti_csi2rx_alt_ctx *ctx,
					  struct ti_csi2rx_alt_buffer *buf)
 {
	 struct ti_csi2rx_alt_dma *dma = &ctx->dma;
	 unsigned long flags = 0;
	 int ret = 0;
 
	 ret = ti_csi2rx_alt_drain_dma(ctx);
	 if (ret)
		 dev_warn(ctx->csi->dev,
			  "Failed to drain DMA. Next frame might be bogus\n");
 
	 spin_lock_irqsave(&dma->lock, flags);
	 ret = ti_csi2rx_alt_start_dma(ctx, buf);
	 if (ret) {
		 vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		 dma->state = TI_CSI2RX_ALT_DMA_IDLE;
		 spin_unlock_irqrestore(&dma->lock, flags);
		 dev_err(ctx->csi->dev, "Failed to start DMA: %d\n", ret);
	 } else {
		 list_add_tail(&buf->list, &dma->submitted);
		 spin_unlock_irqrestore(&dma->lock, flags);
	 }
 
	 return ret;
 }
 
 /**
  * @brief Set up VB2 queue
  * @param q VB2 queue
  * @param nbuffers Number of buffers
  * @param nplanes Number of planes
  * @param sizes Plane sizes
  * @param alloc_devs Allocation devices
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
					  unsigned int *nplanes, unsigned int sizes[],
					  struct device *alloc_devs[])
 {
	 struct ti_csi2rx_alt_ctx *ctx = vb2_get_drv_priv(q);
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
 
 /**
  * @brief Prepare a VB2 buffer
  * @param vb VB2 buffer
  * @return 0 on success, negative error code on failure
  *
  * Validates buffer size against frame size.
  */
 static int ti_csi2rx_alt_buffer_prepare(struct vb2_buffer *vb)
 {
	 struct ti_csi2rx_alt_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	 unsigned long size = ctx->v_fmt.fmt.pix.sizeimage;
 
	 if (vb2_plane_size(vb, 0) < size) {
		 dev_err(ctx->csi->dev, "Data will not fit into plane\n");
		 return -EINVAL;
	 }
 
	 vb2_set_plane_payload(vb, 0, size);
	 return 0;
 }
 
 /**
  * @brief Queue a VB2 buffer
  * @param vb VB2 buffer
  *
  * Adds the buffer to the DMA queue or starts DMA if idle.
  */
 static void ti_csi2rx_alt_buffer_queue(struct vb2_buffer *vb)
 {
	 struct ti_csi2rx_alt_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	 struct ti_csi2rx_alt_buffer *buf;
	 struct ti_csi2rx_alt_dma *dma = &ctx->dma;
	 bool restart_dma = false;
	 unsigned long flags = 0;
 
	 buf = container_of(vb, struct ti_csi2rx_alt_buffer, vb.vb2_buf);
	 buf->ctx = ctx;
 
	 spin_lock_irqsave(&dma->lock, flags);
	 if (dma->state == TI_CSI2RX_ALT_DMA_IDLE) {
		 restart_dma = true;
		 dma->state = TI_CSI2RX_ALT_DMA_ACTIVE;
	 } else {
		 list_add_tail(&buf->list, &dma->queue);
	 }
	 spin_unlock_irqrestore(&dma->lock, flags);
 
	 if (restart_dma) {
		 ti_csi2rx_alt_restart_dma(ctx, buf);
	 }
 }
 
 /**
  * @brief Get virtual channel for a context
  * @param ctx Context to query
  * @return Virtual channel number, or negative error code on failure
  */
 static int ti_csi2rx_alt_get_vc(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 struct v4l2_mbus_frame_desc fd;
	 struct media_pad *pad;
	 int ret, i;
 
	 pad = media_entity_remote_pad_unique(&csi->subdev.entity, MEDIA_PAD_FL_SOURCE);
	 if (!pad)
		 return -ENODEV;
 
	 ret = v4l2_subdev_call(csi->source, pad, get_frame_desc, pad->index,
					&fd);
	 if (ret)
		 return ret;
 
	 if (fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2)
		 return -EINVAL;
 
	 for (i = 0; i < fd.num_entries; i++) {
		 if (ctx->stream == fd.entry[i].stream)
			 return fd.entry[i].bus.csi2.vc;
	 }
 
	 return -ENODEV;
 }
 
 /**
  * @brief Start streaming for a VB2 queue
  * @param vq VB2 queue
  * @param count Number of buffers
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_start_streaming(struct vb2_queue *vq, unsigned int count)
 {
	 struct ti_csi2rx_alt_ctx *ctx = vb2_get_drv_priv(vq);
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 struct ti_csi2rx_alt_dma *dma = &ctx->dma;
	 struct v4l2_subdev_krouting *routing;
	 struct v4l2_subdev_route *route = NULL;
	 struct media_pad *remote_pad;
	 unsigned long flags = 0;
	 int ret = 0, i;
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
 
	 remote_pad = media_entity_remote_source_pad_unique(ctx->pad.entity);
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
 
		 if (r->source_pad != remote_pad->index)
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
 
	 ret = ti_csi2rx_alt_get_vc(ctx);
	 if (ret == -ENOIOCTLCMD)
		 ctx->vc = 0;
	 else if (ret < 0)
		 goto err_pipeline;
	 else
		 ctx->vc = ret;
 
	 ti_csi2rx_alt_setup_shim(ctx);
 
	 ctx->sequence = 0;
 
	 spin_lock_irqsave(&dma->lock, flags);
	 ret = ti_csi2rx_alt_dma_submit_pending(ctx);
	 if (ret) {
		 spin_unlock_irqrestore(&dma->lock, flags);
		 goto err_dma;
	 }
 
	 dma->state = TI_CSI2RX_ALT_DMA_ACTIVE;
	 spin_unlock_irqrestore(&dma->lock, flags);
 
	 ret = v4l2_subdev_enable_streams(&csi->subdev,
					  TI_CSI2RX_ALT_PAD_FIRST_SOURCE + ctx->idx,
					  BIT(0));
	 if (ret)
		 goto err_dma;
 
	 return 0;
 
 err_dma:
	 writel(0, csi->shim + SHIM_DMA_CONTROL(ctx->idx));
 err_pipeline:
	 video_device_pipeline_stop(&ctx->vdev);
 err:
	 ti_csi2rx_alt_cleanup_buffers(ctx, VB2_BUF_STATE_QUEUED);
	 pm_runtime_put(csi->dev);
	 return ret;
 }
 
 /**
  * @brief Stop streaming for a VB2 queue
  * @param vq VB2 queue
  *
  * Stops the media pipeline, subdevice stream, and DMA operations.
  */
 static void ti_csi2rx_alt_stop_streaming(struct vb2_queue *vq)
 {
	 struct ti_csi2rx_alt_ctx *ctx = vb2_get_drv_priv(vq);
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 int ret;
 
	 if (csi->enable_count == 1)
		 writel(0, csi->shim + SHIM_CONTROL);
 
	 video_device_pipeline_stop(&ctx->vdev);
 
	 ret = v4l2_subdev_disable_streams(&csi->subdev,
					   TI_CSI2RX_ALT_PAD_FIRST_SOURCE + ctx->idx,
					   BIT(0));
	 if (ret)
		 dev_err(csi->dev, "Failed to stop subdev stream\n");
 
	 writel(0, csi->shim + SHIM_DMA_CONTROL(ctx->idx));
 
	 ti_csi2rx_alt_cleanup_buffers(ctx, VB2_BUF_STATE_ERROR);
	 pm_runtime_put(csi->dev);
 }
 
 /**
  * @brief VB2 queue operations
  */
 static const struct vb2_ops csi_alt_vb2_qops = {
	 .queue_setup = ti_csi2rx_alt_queue_setup,
	 .buf_prepare = ti_csi2rx_alt_buffer_prepare,
	 .buf_queue = ti_csi2rx_alt_buffer_queue,
	 .start_streaming = ti_csi2rx_alt_start_streaming,
	 .stop_streaming = ti_csi2rx_alt_stop_streaming,
	 .wait_prepare = vb2_ops_wait_prepare,
	 .wait_finish = vb2_ops_wait_finish,
 };
 
 /**
  * @brief Convert subdevice to device structure
  * @param sd V4L2 subdevice
  * @return Pointer to the device structure
  */
 static inline struct ti_csi2rx_alt_dev *to_csi2rx_alt_dev(struct v4l2_subdev *sd)
 {
	 return container_of(sd, struct ti_csi2rx_alt_dev, subdev);
 }
 
 /**
  * @brief Set subdevice format
  * @param sd V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param format Format to set
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_sd_set_fmt(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *state,
					 struct v4l2_subdev_format *format)
 {
	 struct v4l2_mbus_framefmt *fmt;
	 int ret = 0;
 
	 if (format->pad >= TI_CSI2RX_ALT_PAD_FIRST_SOURCE)
		 return v4l2_subdev_get_fmt(sd, state, format);
 
	 if (!find_format_by_code(format->format.code))
		 format->format.code = formats[0].code;
 
	 fmt = v4l2_subdev_state_get_stream_format(state, format->pad,
						   format->stream);
	 if (!fmt) {
		 ret = -EINVAL;
		 goto out;
	 }
	 *fmt = format->format;
 
	 fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
								format->stream);
	 if (!fmt) {
		 ret = -EINVAL;
		 goto out;
	 }
	 *fmt = format->format;
 
 out:
	 return ret;
 }
 
 /**
  * @brief Internal function to set subdevice routing
  * @param sd V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param routing Routing configuration
  * @return 0 on success, negative error code on failure
  */
 static int _ti_csi2rx_alt_sd_set_routing(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  struct v4l2_subdev_krouting *routing)
 {
	 int ret;
 
	 const struct v4l2_mbus_framefmt format = {
		 .width = 640,
		 .height = 480,
		 .code = MEDIA_BUS_FMT_UYVY8_1X16,
		 .field = V4L2_FIELD_NONE,
		 .colorspace = V4L2_COLORSPACE_SRGB,
		 .ycbcr_enc = V4L2_YCBCR_ENC_601,
		 .quantization = V4L2_QUANTIZATION_LIM_RANGE,
		 .xfer_func = V4L2_XFER_FUNC_SRGB,
	 };
 
	 ret = v4l2_subdev_routing_validate(sd, routing,
						V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
 
	 if (ret)
		 return ret;
 
	 ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);
 
	 return ret;
 }
 
 /**
  * @brief Set subdevice routing
  * @param sd V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param which Format type (active or try)
  * @param routing Routing configuration
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_sd_set_routing(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *state,
					 enum v4l2_subdev_format_whence which,
					 struct v4l2_subdev_krouting *routing)
 {
	 return _ti_csi2rx_alt_sd_set_routing(sd, state, routing);
 }
 
 /**
  * @brief Initialize subdevice configuration
  * @param sd V4L2 subdevice
  * @param state V4L2 subdevice state
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_sd_init_cfg(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state)
 {
	 struct v4l2_subdev_route routes[] = {
		 {
			 .sink_pad = 0,
			 .sink_stream = 0,
			 .source_pad = TI_CSI2RX_ALT_PAD_FIRST_SOURCE,
			 .source_stream = 0,
			 .flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		 },
	 };
 
	 struct v4l2_subdev_krouting routing = {
		 .num_routes = 1,
		 .routes = routes,
	 };
 
	 return _ti_csi2rx_alt_sd_set_routing(sd, state, &routing);
 }
 
 /**
  * @brief Get all sink streams
  * @param state V4L2 subdevice state
  * @return Bitmask of active sink streams
  */
 static int ti_csi2rx_alt_sd_all_sink_streams(struct v4l2_subdev_state *state)
 {
	 struct v4l2_subdev_krouting *routing = &state->routing;
	 u64 sink_streams = 0;
	 int i;
 
	 for (i = 0; i < routing->num_routes; i++) {
		 struct v4l2_subdev_route *r = &routing->routes[i];
 
		 if (r->sink_pad == TI_CSI2RX_ALT_PAD_SINK)
			 sink_streams |= BIT(r->sink_stream);
	 }
 
	 return sink_streams;
 }
 
 /**
  * @brief Enable subdevice streams
  * @param sd V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param pad Pad index
  * @param streams_mask Streams to enable
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_sd_enable_streams(struct v4l2_subdev *sd,
						struct v4l2_subdev_state *state,
						u32 pad, u64 streams_mask)
 {
	 struct ti_csi2rx_alt_dev *csi = to_csi2rx_alt_dev(sd);
	 struct media_pad *remote_pad;
	 int ret = 0;
 
	 remote_pad = media_entity_remote_source_pad_unique(&csi->subdev.entity);
	 if (!remote_pad)
		 return -ENODEV;
 
	 mutex_lock(&csi->mutex);
	 if (!csi->enable_count) {
		 u64 sink_streams;
 
		 sink_streams = ti_csi2rx_alt_sd_all_sink_streams(state);
		 dev_dbg(csi->dev, "Enabling all streams (%llx) on sink.\n",
			 sink_streams);
		 ret = v4l2_subdev_enable_streams(csi->source, remote_pad->index,
						  sink_streams);
		 if (ret)
			 goto out;
		 csi->enabled_streams_mask = sink_streams;
	 }
 
	 csi->enable_count++;
 out:
	 mutex_unlock(&csi->mutex);
	 return ret;
 }
 
 /**
  * @brief Disable subdevice streams
  * @param sd V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param pad Pad index
  * @param streams_mask Streams to disable
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_sd_disable_streams(struct v4l2_subdev *sd,
						 struct v4l2_subdev_state *state,
						 u32 pad, u64 streams_mask)
 {
	 struct ti_csi2rx_alt_dev *csi = to_csi2rx_alt_dev(sd);
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
		 u64 sink_streams;
 
		 sink_streams = ti_csi2rx_alt_sd_all_sink_streams(state);
		 dev_dbg(csi->dev, "Disabling all streams (%llx) on sink.\n",
			 sink_streams);
		 ret = v4l2_subdev_disable_streams(csi->source, remote_pad->index,
						   sink_streams);
		 if (ret)
			 goto out;
		 csi->enabled_streams_mask = 0;
	 }
 
	 --csi->enable_count;
 out:
	 mutex_unlock(&csi->mutex);
	 return ret;
 }
 
 /**
  * @brief Subdevice pad operations
  */
 static const struct v4l2_subdev_pad_ops ti_csi2rx_alt_subdev_pad_ops = {
	 .init_cfg = ti_csi2rx_alt_sd_init_cfg,
	 .set_routing = ti_csi2rx_alt_sd_set_routing,
	 .get_fmt = v4l2_subdev_get_fmt,
	 .set_fmt = ti_csi2rx_alt_sd_set_fmt,
	 .enable_streams = ti_csi2rx_alt_sd_enable_streams,
	 .disable_streams = ti_csi2rx_alt_sd_disable_streams,
 };
 
 /**
  * @brief Subdevice operations
  */
 static const struct v4l2_subdev_ops ti_csi2rx_alt_subdev_ops = {
	 .pad = &ti_csi2rx_alt_subdev_pad_ops,
 };
 
 /**
  * @brief Clean up DMA resources for a context
  * @param ctx Context to clean up
  */
 static void ti_csi2rx_alt_cleanup_dma(struct ti_csi2rx_alt_ctx *ctx)
 {
	 dma_release_channel(ctx->dma.chan);
 }
 
 /**
  * @brief Clean up V4L2 and media resources
  * @param csi Device to clean up
  */
 static void ti_csi2rx_alt_cleanup_v4l2(struct ti_csi2rx_alt_dev *csi)
 {
	 media_device_unregister(&csi->mdev);
	 v4l2_device_unregister(&csi->v4l2_dev);
	 media_device_cleanup(&csi->mdev);
 }
 
 /**
  * @brief Clean up subdevice resources
  * @param csi Device to clean up
  */
 static void ti_csi2rx_alt_cleanup_subdev(struct ti_csi2rx_alt_dev *csi)
 {
	 v4l2_async_nf_unregister(&csi->notifier);
	 v4l2_async_nf_cleanup(&csi->notifier);
 }
 
 /**
  * @brief Clean up VB2 queue for a context
  * @param ctx Context to clean up
  */
 static void ti_csi2rx_alt_cleanup_vb2q(struct ti_csi2rx_alt_ctx *ctx)
 {
	 vb2_queue_release(&ctx->vidq);
 }
 
 /**
  * @brief Clean up a context
  * @param ctx Context to clean up
  *
  * Releases all resources associated with the context.
  */
 static void ti_csi2rx_alt_cleanup_ctx(struct ti_csi2rx_alt_ctx *ctx)
 {
	 if (!pm_runtime_status_suspended(ctx->csi->dev))
		 ti_csi2rx_alt_cleanup_dma(ctx);
 
	 ti_csi2rx_alt_cleanup_vb2q(ctx);
 
	 video_unregister_device(&ctx->vdev);
 
	 mutex_destroy(&ctx->mutex);
 }
 
 /**
  * @brief Initialize VB2 queue for a context
  * @param ctx Context to initialize
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_init_vb2q(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct vb2_queue *q = &ctx->vidq;
	 int ret;
 
	 q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	 q->io_modes = VB2_MMAP | VB2_DMABUF;
	 q->drv_priv = ctx;
	 q->buf_struct_size = sizeof(struct ti_csi2rx_alt_buffer);
	 q->ops = &csi_alt_vb2_qops;
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
 
 /**
  * @brief Initialize DMA for a context
  * @param ctx Context to initialize
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_init_dma(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct dma_slave_config cfg = {
		 .src_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES };
	 char name[32];
	 int ret;
 
	 snprintf(name, sizeof(name), "rx%u", ctx->idx);
	 ctx->dma.chan = dma_request_chan(ctx->csi->dev, name);
	 if (IS_ERR(ctx->dma.chan))
		 return PTR_ERR(ctx->dma.chan);
 
	 ret = dmaengine_slave_config(ctx->dma.chan, &cfg);
	 if (ret) {
		 dma_release_channel(ctx->dma.chan);
		 return ret;
	 }
 
	 return 0;
 }
 
 /**
  * @brief Initialize V4L2 and media devices
  * @param csi Device to initialize
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_v4l2_init(struct ti_csi2rx_alt_dev *csi)
 {
	 struct media_device *mdev = &csi->mdev;
	 struct v4l2_subdev *sd = &csi->subdev;
	 int ret, i;
 
	 mdev->dev = csi->dev;
	 mdev->hw_revision = 1;
	 strscpy(mdev->model, "TI-CSI2RX-Alt", sizeof(mdev->model));
 
	 media_device_init(mdev);
 
	 csi->v4l2_dev.mdev = mdev;
 
	 ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	 if (ret)
		 goto cleanup_media;
 
	 ret = media_device_register(mdev);
	 if (ret)
		 goto unregister_v4l2;
 
	 v4l2_subdev_init(sd, &ti_csi2rx_alt_subdev_ops);
	 sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	 sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	 strscpy(sd->name, dev_name(csi->dev), sizeof(sd->name));
	 sd->dev = csi->dev;
 
	 csi->pads[TI_CSI2RX_ALT_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
 
	 for (i = TI_CSI2RX_ALT_PAD_FIRST_SOURCE;
		  i < TI_CSI2RX_ALT_PAD_FIRST_SOURCE + csi->num_ctx; i++)
		 csi->pads[i].flags = MEDIA_PAD_FL_SOURCE;
 
	 ret = media_entity_pads_init(&sd->entity,
					  TI_CSI2RX_ALT_PAD_FIRST_SOURCE + csi->num_ctx,
					  csi->pads);
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
 
 /**
  * @brief Initialize a context
  * @param ctx Context to initialize
  * @return 0 on success, negative error code on failure
  *
  * Sets up the video device, format, DMA, and VB2 queue for the context.
  */
 static int ti_csi2rx_alt_init_ctx(struct ti_csi2rx_alt_ctx *ctx)
 {
	 struct ti_csi2rx_alt_dev *csi = ctx->csi;
	 struct video_device *vdev = &ctx->vdev;
	 const struct ti_csi2rx_alt_fmt *fmt;
	 struct v4l2_pix_format *pix_fmt = &ctx->v_fmt.fmt.pix;
	 int ret;
 
	 mutex_init(&ctx->mutex);
 
	 fmt = find_format_by_pix(V4L2_PIX_FMT_SRGGB8);
	 if (!fmt)
		 return -EINVAL;
 
	 pix_fmt->width = 1920;
	 pix_fmt->height = 1080;
 
	 ti_csi2rx_alt_fill_fmt(fmt, &ctx->v_fmt);
 
	 ctx->pad.flags = MEDIA_PAD_FL_SINK;
	 ret = media_entity_pads_init(&ctx->vdev.entity, 1, &ctx->pad);
	 if (ret)
		 return ret;
 
	 snprintf(vdev->name, sizeof(vdev->name), "%s context %u",
		  dev_name(csi->dev), ctx->idx);
	 vdev->v4l2_dev = &csi->v4l2_dev;
	 vdev->vfl_dir = VFL_DIR_RX;
	 vdev->fops = &csi_alt_fops;
	 vdev->ioctl_ops = &csi_alt_ioctl_ops;
	 vdev->release = video_device_release_empty;
	 vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
				 V4L2_CAP_IO_MC;
	 vdev->lock = &ctx->mutex;
	 video_set_drvdata(vdev, ctx);
 
	 INIT_LIST_HEAD(&ctx->dma.queue);
	 INIT_LIST_HEAD(&ctx->dma.submitted);
	 spin_lock_init(&ctx->dma.lock);
	 ctx->dma.state = TI_CSI2RX_ALT_DMA_STOPPED;
 
	 ret = ti_csi2rx_alt_init_dma(ctx);
	 if (ret)
		 return ret;
 
	 ret = ti_csi2rx_alt_init_vb2q(ctx);
	 if (ret)
		 goto cleanup_dma;
 
	 return 0;
 
 cleanup_dma:
	 ti_csi2rx_alt_cleanup_dma(ctx);
	 return ret;
 }
 
 #ifdef CONFIG_PM
 /**
  * @brief Suspend the device
  * @param dev Platform device
  * @return 0 on success, negative error code on failure
  *
  * Stops DMA and subdevice streams for all contexts.
  */
 static int ti_csi2rx_alt_suspend(struct device *dev)
 {
	 struct ti_csi2rx_alt_dev *csi = dev_get_drvdata(dev);
	 enum ti_csi2rx_alt_dma_state state;
	 struct ti_csi2rx_alt_ctx *ctx;
	 struct ti_csi2rx_alt_dma *dma;
	 unsigned long flags = 0;
	 int i, ret = 0;
 
	 if (pm_runtime_status_suspended(dev))
		 return 0;
 
	 writel(0, csi->shim + SHIM_CONTROL);
 
	 for (i = 0; i < csi->num_ctx; i++) {
		 ctx = &csi->ctx[i];
		 dma = &ctx->dma;
 
		 spin_lock_irqsave(&dma->lock, flags);
		 state = dma->state;
		 spin_unlock_irqrestore(&dma->lock, flags);
 
		 if (state != TI_CSI2RX_ALT_DMA_STOPPED) {
			 ret = v4l2_subdev_disable_streams(&csi->subdev,
							   TI_CSI2RX_ALT_PAD_FIRST_SOURCE + ctx->idx,
							   BIT(0));
			 if (ret)
				 dev_err(csi->dev, "Failed to stop subdev stream\n");
		 }
 
		 writel(0, csi->shim + SHIM_DMA_CONTROL(ctx->idx));
 
		 ti_csi2rx_alt_drain_dma(ctx);
 
		 ret = dmaengine_terminate_sync(ctx->dma.chan);
		 if (ret)
			 dev_err(csi->dev, "Failed to stop DMA\n");
	 }
 
	 return ret;
 }
 
 /**
  * @brief Resume the device
  * @param dev Platform device
  * @return 0 on success, negative error code on failure
  *
  * Restarts DMA and subdevice streams for active contexts.
  */
 static int ti_csi2rx_alt_resume(struct device *dev)
 {
	 struct ti_csi2rx_alt_dev *csi = dev_get_drvdata(dev);
	 struct ti_csi2rx_alt_ctx *ctx;
	 struct ti_csi2rx_alt_dma *dma;
	 struct ti_csi2rx_alt_buffer *buf;
	 unsigned long flags = 0;
	 unsigned int reg;
	 int i, ret = 0;
 
	 if (pm_runtime_status_suspended(dev))
		 return 0;
 
	 reg = SHIM_CONTROL_PIX_RESET;
	 writel(reg, csi->shim + SHIM_CONTROL);
 
	 for (i = 0; i < csi->num_ctx; i++) {
		 ctx = &csi->ctx[i];
		 dma = &ctx->dma;
		 spin_lock_irqsave(&dma->lock, flags);
		 if (dma->state != TI_CSI2RX_ALT_DMA_STOPPED) {
			 list_for_each_entry(buf, &ctx->dma.submitted, list) {
				 ti_csi2rx_alt_start_dma(ctx, buf);
			 }
			 spin_unlock_irqrestore(&dma->lock, flags);
 
			 ti_csi2rx_alt_setup_shim(ctx);
 
			 ret = v4l2_subdev_enable_streams(&csi->subdev,
							  TI_CSI2RX_ALT_PAD_FIRST_SOURCE + ctx->idx,
							  BIT(0));
			 if (ret)
				 dev_err(ctx->csi->dev, "Failed to start subdev\n");
		 } else {
			 spin_unlock_irqrestore(&dma->lock, flags);
		 }
	 }
 
	 return ret;
 }
 
 /**
  * @brief Runtime suspend the device
  * @param dev Platform device
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_runtime_suspend(struct device *dev)
 {
	 struct ti_csi2rx_alt_dev *csi = dev_get_drvdata(dev);
	 int i;
 
	 if (csi->enable_count != 0)
		 return -EBUSY;
 
	 for (i = 0; i < csi->num_ctx; i++)
		 ti_csi2rx_alt_cleanup_dma(&csi->ctx[i]);
 
	 return 0;
 }
 
 /**
  * @brief Runtime resume the device
  * @param dev Platform device
  * @return 0 on success, negative error code on failure
  */
 static int ti_csi2rx_alt_runtime_resume(struct device *dev)
 {
	 struct ti_csi2rx_alt_dev *csi = dev_get_drvdata(dev);
	 int ret, i;
 
	 for (i = 0; i < csi->num_ctx; i++) {
		 ret = ti_csi2rx_alt_init_dma(&csi->ctx[i]);
		 if (ret)
			 return ret;
	 }
 
	 return 0;
 }
 
 /**
  * @brief Power management operations
  */
 static const struct dev_pm_ops ti_csi2rx_alt_pm_ops = {
	 SET_SYSTEM_SLEEP_PM_OPS(ti_csi2rx_alt_suspend, ti_csi2rx_alt_resume)
	 SET_RUNTIME_PM_OPS(ti_csi2rx_alt_runtime_suspend, ti_csi2rx_alt_runtime_resume,
				NULL)
 };
 #endif /* CONFIG_PM */
 
 /**
  * @brief Probe the platform device
  * @param pdev Platform device
  * @return 0 on success, negative error code on failure
  *
  * Initializes the CSI-2 RX controller, contexts, V4L2, and media devices.
  */
 static int ti_csi2rx_alt_probe(struct platform_device *pdev)
 {
	 struct device_node *np = pdev->dev.of_node;
	 struct ti_csi2rx_alt_dev *csi;
	 struct resource *res;
	 int ret, i, count;
 
	 csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	 if (!csi)
		 return -ENOMEM;
 
	 csi->dev = &pdev->dev;
	 platform_set_drvdata(pdev, csi);
 
	 res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	 csi->shim = devm_ioremap_resource(&pdev->dev, res);
	 if (IS_ERR(csi->shim)) {
		 ret = PTR_ERR(csi->shim);
		 return ret;
	 }
 
	 count = of_property_count_strings(np, "dma-names");
	 if (count < 0) {
		 dev_err(csi->dev, "Failed to get DMA channel count: %d\n",
			 count);
		 return count;
	 }
 
	 csi->num_ctx = count;
	 if (csi->num_ctx > TI_CSI2RX_ALT_MAX_CTX) {
		 dev_warn(csi->dev,
			  "%u DMA channels passed. Maximum is %u. Ignoring the rest.\n",
			  csi->num_ctx, TI_CSI2RX_ALT_MAX_CTX);
		 csi->num_ctx = TI_CSI2RX_ALT_MAX_CTX;
	 }
 
	csi->drain.len = DRAIN_BUFFER_SIZE;
	csi->drain.vaddr = dma_alloc_coherent(csi->dev, csi->drain.len,
						 &csi->drain.paddr,
						 GFP_KERNEL);
	if (!csi->drain.vaddr)
		return -ENOMEM;

	mutex_init(&csi->mutex);

	ret = ti_csi2rx_alt_v4l2_init(csi);
	if (ret)
		goto cleanup_drain;

	for (i = 0; i < csi->num_ctx; i++) {
		csi->ctx[i].idx = i;
		csi->ctx[i].csi = csi;
		ret = ti_csi2rx_alt_init_ctx(&csi->ctx[i]);
		if (ret)
			goto cleanup_ctx;
	}

	ret = ti_csi2rx_alt_init_subdev(csi);
	if (ret)
		goto cleanup_ctx;

	ret = of_platform_populate(csi->dev->of_node, NULL, NULL, csi->dev);
	if (ret) {
		dev_err(csi->dev, "Failed to create children: %d\n", ret);
		goto cleanup_subdev;
	}

	pm_runtime_set_active(csi->dev);
	pm_runtime_enable(csi->dev);
	pm_runtime_idle(csi->dev);

	return 0;

cleanup_subdev:
	ti_csi2rx_alt_cleanup_subdev(csi);
cleanup_ctx:
	i--;
	for (; i >= 0; i--)
		ti_csi2rx_alt_cleanup_ctx(&csi->ctx[i]);

	ti_csi2rx_alt_cleanup_v4l2(csi);
cleanup_drain:
	mutex_destroy(&csi->mutex);
	dma_free_coherent(csi->dev, csi->drain.len, csi->drain.vaddr,
			 csi->drain.paddr);
	return ret;
}
/**
 * @brief Remove the platform device
 * @param pdev Platform device to remove
 * @return 0 on success, negative error code on failure
 *
 * Cleans up all resources associated with the CSI-2 RX controller, including
 * contexts, subdevice, V4L2 and media devices, mutexes, and DMA buffers. Ensures
 * no VB2 queues are busy before proceeding with cleanup. Disables runtime power
 * management and sets the device to suspended state.
 */
static int ti_csi2rx_alt_remove(struct platform_device *pdev)
{
	struct ti_csi2rx_alt_dev *csi = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < csi->num_ctx; i++) {
		if (vb2_is_busy(&csi->ctx[i].vidq))
			return -EBUSY;
	}

	for (i = 0; i < csi->num_ctx; i++)
		ti_csi2rx_alt_cleanup_ctx(&csi->ctx[i]);

	ti_csi2rx_alt_cleanup_subdev(csi);
	ti_csi2rx_alt_cleanup_v4l2(csi);

	mutex_destroy(&csi->mutex);
	dma_free_coherent(csi->dev, csi->drain.len, csi->drain.vaddr,
			 csi->drain.paddr);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	return 0;
}

/**
 * @brief Device tree matching table
 *
 * Defines the compatible strings for the TI J721E CSI-2 RX controller,
 * allowing the driver to bind to devices with the specified compatible
 * property in the device tree.
 */
static const struct of_device_id ti_csi2rx_alt_of_match[] = {
	{ .compatible = "ti,j721e-csi2rx", },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_csi2rx_alt_of_match);

/**
 * @brief Platform driver structure
 *
 * Defines the platform driver for the TI J721E CSI-2 RX controller,
 * including probe and remove functions, driver name, device tree
 * matching table, and power management operations (if enabled).
 */
static struct platform_driver ti_csi2rx_alt_pdrv = {
	.probe = ti_csi2rx_alt_probe,
	.remove = ti_csi2rx_alt_remove,
	.driver = {
		.name		= TI_CSI2RX_ALT_MODULE_NAME,
		.of_match_table	= ti_csi2rx_alt_of_match,
#ifdef CONFIG_PM
		.pm		= &ti_csi2rx_alt_pm_ops,
#endif
	},
};

/**
 * @brief Module registration
 *
 * Registers the platform driver for the TI J721E CSI-2 RX controller.
 * This macro handles the initialization and cleanup of the driver module.
 */
module_platform_driver(ti_csi2rx_alt_pdrv);

/**
 * @brief Module description
 */
MODULE_DESCRIPTION("TI J721E CSI2 RX Alternate Driver");
/**
 * @brief Module author
 */
MODULE_AUTHOR("RAM");
/**
 * @brief Module license
 */
MODULE_LICENSE("GPL");
