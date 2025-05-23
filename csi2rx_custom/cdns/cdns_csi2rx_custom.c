/**
 * @file cdns_csi2rx_custom.c
 * @brief Custom Cadence CSI-2 RX Controller Driver
 * @author RAM
 * @date April 20, 2025
 * @license GPL
 *
 * This driver manages the Cadence CSI-2 Receiver (RX) controller, interfacing
 * with camera sensors (e.g., IMX219) to receive MIPI CSI-2 data. It operates as
 * a V4L2 subdevice, supporting up to 4 data lanes and 4 streams, with runtime
 * power management, external D-PHY configuration, and asynchronous subdevice
 * registration.
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
 #include <linux/slab.h>
 
 #include <media/v4l2-ctrls.h>
 #include <media/v4l2-device.h>
 #include <media/v4l2-fwnode.h>
 #include <media/v4l2-subdev.h>
 
 /** @brief Device configuration register */
 #define CUSTOM_CSI2RX_DEVICE_CFG_REG            0x000
 
 /** @brief Soft reset register */
 #define CUSTOM_CSI2RX_SOFT_RESET_REG            0x004
 /** @brief Protocol logic reset bit */
 #define CUSTOM_CSI2RX_SOFT_RESET_PROTOCOL       BIT(1)
 /** @brief Front-end logic reset bit */
 #define CUSTOM_CSI2RX_SOFT_RESET_FRONT          BIT(0)
 
 /** @brief Static configuration register */
 #define CUSTOM_CSI2RX_STATIC_CFG_REG            0x008
 /** @brief Macro to map logical lane to physical lane */
 #define CUSTOM_CSI2RX_STATIC_CFG_DLANE_MAP(llane, plane) ((plane) << (16 + (llane) * 4))
 /** @brief Mask for number of lanes in static config */
 #define CUSTOM_CSI2RX_STATIC_CFG_LANES_MASK     GENMASK(11, 8)
 
 /** @brief D-PHY lane control register */
 #define CUSTOM_CSI2RX_DPHY_LANE_CTRL_REG        0x40
 /** @brief Clock lane reset bit */
 #define CUSTOM_CSI2RX_DPHY_CL_RST               BIT(16)
 /** @brief Data lane reset bit for lane i */
 #define CUSTOM_CSI2RX_DPHY_DL_RST(i)            BIT((i) + 12)
 /** @brief Clock lane enable bit */
 #define CUSTOM_CSI2RX_DPHY_CL_EN                BIT(4)
 /** @brief Data lane enable bit for lane i */
 #define CUSTOM_CSI2RX_DPHY_DL_EN(i)             BIT(i)
 
 /** @brief Base address for stream n registers */
 #define CUSTOM_CSI2RX_STREAM_BASE(n)            (((n) + 1) * 0x100)
 
 /** @brief Stream control register for stream n */
 #define CUSTOM_CSI2RX_STREAM_CTRL_REG(n)        (CUSTOM_CSI2RX_STREAM_BASE(n) + 0x000)
 /** @brief Stream soft reset bit */
 #define CUSTOM_CSI2RX_STREAM_CTRL_SOFT_RST      BIT(4)
 /** @brief Stream stop bit */
 #define CUSTOM_CSI2RX_STREAM_CTRL_STOP          BIT(1)
 /** @brief Stream start bit */
 #define CUSTOM_CSI2RX_STREAM_CTRL_START         BIT(0)
 
 /** @brief Stream status register for stream n */
 #define CUSTOM_CSI2RX_STREAM_STATUS_REG(n)      (CUSTOM_CSI2RX_STREAM_BASE(n) + 0x004)
 /** @brief Stream ready status bit */
 #define CUSTOM_CSI2RX_STREAM_STATUS_RDY         BIT(31)
 
 /** @brief Stream data configuration register for stream n */
 #define CUSTOM_CSI2RX_STREAM_DATA_CFG_REG(n)    (CUSTOM_CSI2RX_STREAM_BASE(n) + 0x008)
 /** @brief Virtual channel select bit for VC n */
 #define CUSTOM_CSI2RX_STREAM_DATA_CFG_VC_SELECT(n) BIT((n) + 16)
 /** @brief Accept all virtual channels */
 #define CUSTOM_CSI2RX_STREAM_DATA_CFG_VC_ALL    0
 
 /** @brief Stream configuration register for stream n */
 #define CUSTOM_CSI2RX_STREAM_CFG_REG(n)         (CUSTOM_CSI2RX_STREAM_BASE(n) + 0x00c)
 /** @brief Enable large FIFO buffer mode */
 #define CUSTOM_CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF (1 << 8)
 
 /** @brief Maximum number of data lanes supported */
 #define CUSTOM_CSI2RX_LANES_MAX                 4
 /** @brief Maximum number of streams supported */
 #define CUSTOM_CSI2RX_STREAMS_MAX               4
 
 /**
  * @brief Media pad identifiers for the CSI-2 RX subdevice
  */
 enum custom_csi2rx_pads {
     CUSTOM_CSI2RX_PAD_SINK,           /**< Sink pad (input from sensor) */
     CUSTOM_CSI2RX_PAD_SOURCE_STREAM0, /**< Source pad for stream 0 */
     CUSTOM_CSI2RX_PAD_SOURCE_STREAM1, /**< Source pad for stream 1 */
     CUSTOM_CSI2RX_PAD_SOURCE_STREAM2, /**< Source pad for stream 2 */
     CUSTOM_CSI2RX_PAD_SOURCE_STREAM3, /**< Source pad for stream 3 */
     CUSTOM_CSI2RX_PAD_MAX,            /**< Total number of pads */
 };
 
 /**
  * @brief Structure representing a supported media bus format
  */
 struct custom_csi2rx_fmt {
     u32 code; /**< Media bus format code (e.g., MEDIA_BUS_FMT_YUYV8_1X16) */
     u8 bpp;   /**< Bits per pixel */
 };
 
 /**
  * @brief Private data structure for the CSI-2 RX driver
  */
 struct custom_csi2rx_priv {
     struct device *dev;                           /**< Platform device */
     unsigned int count;                           /**< Number of active streams */
     struct mutex lock;                            /**< Mutex for synchronization */
     void __iomem *base;                           /**< Base address of registers */
     struct clk *sys_clk;                          /**< System clock */
     struct clk *p_clk;                            /**< Protocol clock */
     struct clk *pixel_clk[CUSTOM_CSI2RX_STREAMS_MAX]; /**< Pixel clocks for streams */
     struct phy *dphy;                             /**< D-PHY handle */
     u8 lanes[CUSTOM_CSI2RX_LANES_MAX];            /**< Data lane mapping */
     u8 num_lanes;                                 /**< Number of active lanes */
     u8 max_lanes;                                 /**< Maximum supported lanes */
     u8 max_streams;                               /**< Maximum supported streams */
     bool has_internal_dphy;                       /**< Internal D-PHY presence */
     struct v4l2_subdev subdev;                    /**< V4L2 subdevice */
     struct v4l2_async_notifier notifier;          /**< Async notifier for subdevice */
     struct media_pad pads[CUSTOM_CSI2RX_PAD_MAX]; /**< Media pads */
     struct v4l2_subdev *source_subdev;            /**< Source subdevice (e.g., sensor) */
     int source_pad;                               /**< Source subdevice output pad */
 };
 
 /**
  * @brief Array of supported media bus formats
  */
 static const struct custom_csi2rx_fmt custom_formats[] = {
     { .code = MEDIA_BUS_FMT_YUYV8_1X16, .bpp = 16 },
     { .code = MEDIA_BUS_FMT_UYVY8_1X16, .bpp = 16 },
     { .code = MEDIA_BUS_FMT_YVYU8_1X16, .bpp = 16 },
     { .code = MEDIA_BUS_FMT_VYUY8_1X16, .bpp = 16 },
     { .code = MEDIA_BUS_FMT_SBGGR8_1X8, .bpp = 8 },
     { .code = MEDIA_BUS_FMT_SGBRG8_1X8, .bpp = 8 },
     { .code = MEDIA_BUS_FMT_SGRBG8_1X8, .bpp = 8 },
     { .code = MEDIA_BUS_FMT_SRGGB8_1X8, .bpp = 8 },
     { .code = MEDIA_BUS_FMT_SBGGR10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SGBRG10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SGRBG10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SRGGB10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SBGGR12_1X12, .bpp = 12 },
     { .code = MEDIA_BUS_FMT_SGBRG12_1X12, .bpp = 12 },
     { .code = MEDIA_BUS_FMT_SGRBG12_1X12, .bpp = 12 },
     { .code = MEDIA_BUS_FMT_SRGGB12_1X12, .bpp = 12 },
     { .code = MEDIA_BUS_FMT_SRGGI10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SGRIG10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SBGGI10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SGBIG10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SGIRG10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SIGGR10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SGIBG10_1X10, .bpp = 10 },
     { .code = MEDIA_BUS_FMT_SIGGB10_1X10, .bpp = 10 },
 };
 
 /**
  * @brief Retrieve format structure by media bus code
  * @param code Media bus format code
  * @return Pointer to format structure or NULL if not found
  */
 static const struct custom_csi2rx_fmt *custom_csi2rx_get_fmt_by_code(u32 code)
 {
     unsigned int i;
 
     for (i = 0; i < ARRAY_SIZE(custom_formats); i++)
         if (custom_formats[i].code == code)
             return &custom_formats[i];
 
     return NULL;
 }
 
 /**
  * @brief Get frame descriptor from source subdevice
  * @param csi2rx Driver private data
  * @param fd Pointer to frame descriptor
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_get_frame_desc_from_source(struct custom_csi2rx_priv *csi2rx,
                                                    struct v4l2_mbus_frame_desc *fd)
 {
     struct media_pad *remote_pad;
 
     remote_pad = media_entity_remote_source_pad_unique(&csi2rx->subdev.entity);
     if (!remote_pad) {
         dev_err(csi2rx->dev, "No remote pad found for sink\n");
         return -ENODEV;
     }
 
     return v4l2_subdev_call(csi2rx->source_subdev, pad, get_frame_desc,
                             remote_pad->index, fd);
 }
 
 /**
  * @brief Convert V4L2 subdevice to driver private data
  * @param subdev V4L2 subdevice pointer
  * @return Pointer to driver private data
  */
 static inline struct custom_csi2rx_priv *v4l2_subdev_to_custom_csi2rx(struct v4l2_subdev *subdev)
 {
     return container_of(subdev, struct custom_csi2rx_priv, subdev);
 }
 
 /**
  * @brief Reset the CSI-2 RX controller and streams
  * @param csi2rx Driver private data
  */
 static void custom_csi2rx_reset(struct custom_csi2rx_priv *csi2rx)
 {
     unsigned int i;
 
     writel(CUSTOM_CSI2RX_SOFT_RESET_PROTOCOL | CUSTOM_CSI2RX_SOFT_RESET_FRONT,
            csi2rx->base + CUSTOM_CSI2RX_SOFT_RESET_REG);
 
     udelay(10);
 
     writel(0, csi2rx->base + CUSTOM_CSI2RX_SOFT_RESET_REG);
 
     for (i = 0; i < csi2rx->max_streams; i++) {
         writel(CUSTOM_CSI2RX_STREAM_CTRL_SOFT_RST,
                csi2rx->base + CUSTOM_CSI2RX_STREAM_CTRL_REG(i));
         usleep_range(10, 20);
         writel(0, csi2rx->base + CUSTOM_CSI2RX_STREAM_CTRL_REG(i));
     }
 }
 
 /**
  * @brief Configure external D-PHY for MIPI CSI-2
  * @param csi2rx Driver private data
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_configure_external_dphy(struct custom_csi2rx_priv *csi2rx)
 {
     union phy_configure_opts opts = { };
     struct phy_configure_opts_mipi_dphy *cfg = &opts.mipi_dphy;
     struct v4l2_mbus_framefmt *framefmt;
     struct v4l2_subdev_state *state;
     const struct custom_csi2rx_fmt *fmt;
     s64 link_freq;
     int ret;
 
     state = v4l2_subdev_get_locked_active_state(&csi2rx->subdev);
     framefmt = v4l2_subdev_state_get_stream_format(state, CUSTOM_CSI2RX_PAD_SINK, 0);
 
     if (framefmt) {
         fmt = custom_csi2rx_get_fmt_by_code(framefmt->code);
     } else {
         dev_err(csi2rx->dev, "Did not find active sink format\n");
         fmt = &custom_formats[0];
     }
 
     link_freq = v4l2_get_link_freq(csi2rx->source_subdev->ctrl_handler,
                                    fmt->bpp, 2 * csi2rx->num_lanes);
     if (link_freq < 0)
         return link_freq;
 
     ret = phy_mipi_dphy_get_default_config_for_hsclk(link_freq,
                                                     csi2rx->num_lanes, cfg);
     if (ret)
         return ret;
 
     ret = phy_power_on(csi2rx->dphy);
     if (ret)
         return ret;
 
     ret = phy_configure(csi2rx->dphy, &opts);
     if (ret)
         phy_power_off(csi2rx->dphy);
 
     return ret;
 }
 
 /**
  * @brief Start the CSI-2 RX controller and streams
  * @param csi2rx Driver private data
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_start(struct custom_csi2rx_priv *csi2rx)
 {
     unsigned int i;
     unsigned long lanes_used = 0;
     u32 reg;
     int ret;
 
     ret = clk_prepare_enable(csi2rx->p_clk);
     if (ret)
         return ret;
 
     custom_csi2rx_reset(csi2rx);
 
     reg = csi2rx->num_lanes << 8;
     for (i = 0; i < csi2rx->num_lanes; i++) {
         reg |= CUSTOM_CSI2RX_STATIC_CFG_DLANE_MAP(i, csi2rx->lanes[i]);
         set_bit(csi2rx->lanes[i], &lanes_used);
     }
 
     for (i = csi2rx->num_lanes; i < csi2rx->max_lanes; i++) {
         unsigned int idx = find_first_zero_bit(&lanes_used, csi2rx->max_lanes);
         set_bit(idx, &lanes_used);
         reg |= CUSTOM_CSI2RX_STATIC_CFG_DLANE_MAP(i, i + 1);
     }
 
     writel(reg, csi2rx->base + CUSTOM_CSI2RX_STATIC_CFG_REG);
 
     if (csi2rx->dphy) {
         reg = CUSTOM_CSI2RX_DPHY_CL_EN | CUSTOM_CSI2RX_DPHY_CL_RST;
         for (i = 0; i < csi2rx->num_lanes; i++) {
             reg |= CUSTOM_CSI2RX_DPHY_DL_EN(csi2rx->lanes[i] - 1);
             reg |= CUSTOM_CSI2RX_DPHY_DL_RST(csi2rx->lanes[i] - 1);
         }
 
         writel(reg, csi2rx->base + CUSTOM_CSI2RX_DPHY_LANE_CTRL_REG);
 
         ret = custom_csi2rx_configure_external_dphy(csi2rx);
         if (ret) {
             dev_err(csi2rx->dev, "Failed to configure external DPHY: %d\n", ret);
             goto err_disable_pclk;
         }
     }
 
     for (i = 0; i < csi2rx->max_streams; i++) {
         ret = clk_prepare_enable(csi2rx->pixel_clk[i]);
         if (ret)
             goto err_disable_pixclk;
 
         writel(CUSTOM_CSI2RX_STREAM_CFG_FIFO_MODE_LARGE_BUF,
                csi2rx->base + CUSTOM_CSI2RX_STREAM_CFG_REG(i));
 
         writel(CUSTOM_CSI2RX_STREAM_DATA_CFG_VC_ALL,
                csi2rx->base + CUSTOM_CSI2RX_STREAM_DATA_CFG_REG(i));
 
         writel(CUSTOM_CSI2RX_STREAM_CTRL_START,
                csi2rx->base + CUSTOM_CSI2RX_STREAM_CTRL_REG(i));
     }
 
     ret = clk_prepare_enable(csi2rx->sys_clk);
     if (ret)
         goto err_disable_pixclk;
 
     clk_disable_unprepare(csi2rx->p_clk);
 
     return 0;
 
 err_disable_pixclk:
     for (; i > 0; i--)
         clk_disable_unprepare(csi2rx->pixel_clk[i - 1]);
 
     if (csi2rx->dphy) {
         writel(0, csi2rx->base + CUSTOM_CSI2RX_DPHY_LANE_CTRL_REG);
         phy_power_off(csi2rx->dphy);
     }
 err_disable_pclk:
     clk_disable_unprepare(csi2rx->p_clk);
 
     return ret;
 }
 
 /**
  * @brief Stop the CSI-2 RX controller and streams
  * @param csi2rx Driver private data
  */
 static void custom_csi2rx_stop(struct custom_csi2rx_priv *csi2rx)
 {
     unsigned int i;
     u32 val;
     int ret;
 
     clk_prepare_enable(csi2rx->p_clk);
     clk_disable_unprepare(csi2rx->sys_clk);
 
     for (i = 0; i < csi2rx->max_streams; i++) {
         writel(CUSTOM_CSI2RX_STREAM_CTRL_STOP,
                csi2rx->base + CUSTOM_CSI2RX_STREAM_CTRL_REG(i));
 
         ret = readl_relaxed_poll_timeout(csi2rx->base +
                                          CUSTOM_CSI2RX_STREAM_STATUS_REG(i),
                                          val,
                                          !(val & CUSTOM_CSI2RX_STREAM_STATUS_RDY),
                                          10, 10000);
         if (ret)
             dev_warn(csi2rx->dev, "Failed to stop stream%u\n", i);
 
         clk_disable_unprepare(csi2rx->pixel_clk[i]);
     }
 
     clk_disable_unprepare(csi2rx->p_clk);
 
     if (csi2rx->dphy) {
         writel(0, csi2rx->base + CUSTOM_CSI2RX_DPHY_LANE_CTRL_REG);
         if (phy_power_off(csi2rx->dphy))
             dev_warn(csi2rx->dev, "Couldn't power off DPHY\n");
     }
 }
 
 /**
  * @brief Enable streams for the CSI-2 RX subdevice
  * @param subdev V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param pad Pad index
  * @param streams_mask Mask of streams to enable
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_enable_streams(struct v4l2_subdev *subdev,
                                        struct v4l2_subdev_state *state, u32 pad,
                                        u64 streams_mask)
 {
     struct custom_csi2rx_priv *csi2rx = v4l2_subdev_to_custom_csi2rx(subdev);
     struct media_pad *remote_pad;
     u64 sink_streams;
     int ret;
 
     remote_pad = media_pad_remote_pad_first(&csi2rx->pads[CUSTOM_CSI2RX_PAD_SINK]);
     if (!remote_pad) {
         dev_err(csi2rx->dev, "Failed to find connected source\n");
         return -ENODEV;
     }
 
     ret = pm_runtime_resume_and_get(csi2rx->dev);
     if (ret < 0)
         return ret;
 
     sink_streams = v4l2_subdev_state_xlate_streams(state,
                                                   CUSTOM_CSI2RX_PAD_SOURCE_STREAM0,
                                                   CUSTOM_CSI2RX_PAD_SINK,
                                                   &streams_mask);
 
     mutex_lock(&csi2rx->lock);
     if (!csi2rx->count) {
         ret = custom_csi2rx_start(csi2rx);
         if (ret)
             goto err_stream_start;
     }
 
     ret = v4l2_subdev_enable_streams(csi2rx->source_subdev, remote_pad->index,
                                      sink_streams);
     if (ret) {
         dev_err(csi2rx->dev, "Failed to start streams %#llx on subdev\n",
                 sink_streams);
         goto err_subdev_enable;
     }
 
     csi2rx->count++;
     mutex_unlock(&csi2rx->lock);
 
     return 0;
 
 err_subdev_enable:
     if (!csi2rx->count)
         custom_csi2rx_stop(csi2rx);
 err_stream_start:
     mutex_unlock(&csi2rx->lock);
     pm_runtime_put(csi2rx->dev);
     return ret;
 }
 
 /**
  * @brief Disable streams for the CSI-2 RX subdevice
  * @param subdev V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param pad Pad index
  * @param streams_mask Mask of streams to disable
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_disable_streams(struct v4l2_subdev *subdev,
                                         struct v4l2_subdev_state *state, u32 pad,
                                         u64 streams_mask)
 {
     struct custom_csi2rx_priv *csi2rx = v4l2_subdev_to_custom_csi2rx(subdev);
     struct media_pad *remote_pad;
     u64 sink_streams;
 
     sink_streams = v4l2_subdev_state_xlate_streams(state,
                                                   CUSTOM_CSI2RX_PAD_SOURCE_STREAM0,
                                                   CUSTOM_CSI2RX_PAD_SINK,
                                                   &streams_mask);
 
     remote_pad = media_pad_remote_pad_first(&csi2rx->pads[CUSTOM_CSI2RX_PAD_SINK]);
     if (!remote_pad ||
         v4l2_subdev_disable_streams(csi2rx->source_subdev,
                                     remote_pad->index, sink_streams)) {
         dev_err(csi2rx->dev, "Couldn't disable our subdev\n");
     }
 
     mutex_lock(&csi2rx->lock);
     csi2rx->count--;
     if (!csi2rx->count)
         custom_csi2rx_stop(csi2rx);
     mutex_unlock(&csi2rx->lock);
 
     pm_runtime_put(csi2rx->dev);
 
     return 0;
 }
 
 /**
  * @brief Set stream routing for the CSI-2 RX subdevice
  * @param subdev V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param which Format type (active or try)
  * @param routing Routing configuration
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_set_routing(struct v4l2_subdev *subdev,
                                     struct v4l2_subdev_state *state,
                                     enum v4l2_subdev_format_whence which,
                                     struct v4l2_subdev_krouting *routing)
 {
     struct custom_csi2rx_priv *csi2rx = v4l2_subdev_to_custom_csi2rx(subdev);
     static const struct v4l2_mbus_framefmt format = {
         .width = 1920,
         .height = 1080,
         .code = MEDIA_BUS_FMT_SRGGB8_1X8,
         .field = V4L2_FIELD_NONE,
         .colorspace = V4L2_COLORSPACE_SRGB,
         .ycbcr_enc = V4L2_YCBCR_ENC_601,
         .quantization = V4L2_QUANTIZATION_LIM_RANGE,
         .xfer_func = V4L2_XFER_FUNC_SRGB,
     };
     int ret;
 
     if (which == V4L2_SUBDEV_FORMAT_ACTIVE && csi2rx->count)
         return -EBUSY;
 
     if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
         return -EINVAL;
 
     ret = v4l2_subdev_routing_validate(subdev, routing,
                                       V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
     if (ret)
         return ret;
 
     ret = v4l2_subdev_set_routing_with_fmt(subdev, state, routing, &format);
     if (ret)
         return ret;
 
     return 0;
 }
 
 /**
  * @brief Set format for the CSI-2 RX subdevice
  * @param subdev V4L2 subdevice
  * @param state V4L2 subdevice state
  * @param format Format configuration
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_set_fmt(struct v4l2_subdev *subdev,
                                 struct v4l2_subdev_state *state,
                                 struct v4l2_subdev_format *format)
 {
     struct custom_csi2rx_priv *csi2rx = v4l2_subdev_to_custom_csi2rx(subdev);
     struct v4l2_mbus_framefmt *fmt;
 
     if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && csi2rx->count)
         return -EBUSY;
 
     if (format->pad >= CUSTOM_CSI2RX_PAD_SOURCE_STREAM0)
         return v4l2_subdev_get_fmt(subdev, state, format);
 
     if (!custom_csi2rx_get_fmt_by_code(format->format.code))
         format->format.code = custom_formats[0].code;
 
     fmt = v4l2_subdev_state_get_stream_format(state, format->pad, format->stream);
     if (!fmt)
         return -EINVAL;
 
     *fmt = format->format;
 
     fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad,
                                                       format->stream);
     if (!fmt)
         return -EINVAL;
 
     *fmt = format->format;
 
     return 0;
 }
 
 /**
  * @brief Initialize configuration for the CSI-2 RX subdevice
  * @param subdev V4L2 subdevice
  * @param state V4L2 subdevice state
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_init_cfg(struct v4l2_subdev *subdev,
                                  struct v4l2_subdev_state *state)
 {
     struct v4l2_subdev_route routes[] = {
         {
             .sink_pad = CUSTOM_CSI2RX_PAD_SINK,
             .sink_stream = 0,
             .source_pad = CUSTOM_CSI2RX_PAD_SOURCE_STREAM0,
             .source_stream = 0,
             .flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
         },
     };
 
     struct v4l2_subdev_krouting routing = {
         .num_routes = ARRAY_SIZE(routes),
         .routes = routes,
     };
 
     return custom_csi2rx_set_routing(subdev, state, V4L2_SUBDEV_FORMAT_TRY, &routing);
 }
 
 /**
  * @brief Get frame descriptor for the CSI-2 RX subdevice
  * @param subdev V4L2 subdevice
  * @param pad Pad index
  * @param fd Pointer to frame descriptor
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_get_frame_desc(struct v4l2_subdev *subdev, unsigned int pad,
                                        struct v4l2_mbus_frame_desc *fd)
 {
     struct custom_csi2rx_priv *csi2rx = v4l2_subdev_to_custom_csi2rx(subdev);
 
     return custom_csi2rx_get_frame_desc_from_source(csi2rx, fd);
 }
 
 /**
  * @brief V4L2 subdevice pad operations
  */
 static const struct v4l2_subdev_pad_ops custom_csi2rx_pad_ops = {
     .get_fmt        = v4l2_subdev_get_fmt,
     .set_fmt        = custom_csi2rx_set_fmt,
     .init_cfg       = custom_csi2rx_init_cfg,
     .get_frame_desc = custom_csi2rx_get_frame_desc,
     .set_routing    = custom_csi2rx_set_routing,
     .enable_streams = custom_csi2rx_enable_streams,
     .disable_streams = custom_csi2rx_disable_streams,
 };
 
 /**
  * @brief V4L2 subdevice operations
  */
 static const struct v4l2_subdev_ops custom_csi2rx_subdev_ops = {
     .pad = &custom_csi2rx_pad_ops,
 };
 
 /**
  * @brief Media entity operations
  */
 static const struct media_entity_operations custom_csi2rx_media_ops = {
     .link_validate = v4l2_subdev_link_validate,
 };
 
 /**
  * @brief Async notifier callback for subdevice binding
  * @param notifier Async notifier
  * @param s_subdev Source subdevice
  * @param asd Async subdevice
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_async_bound(struct v4l2_async_notifier *notifier,
                                     struct v4l2_subdev *s_subdev,
                                     struct v4l2_async_subdev *asd)
 {
     struct v4l2_subdev *subdev = notifier->sd;
     struct custom_csi2rx_priv *csi2rx = v4l2_subdev_to_custom_csi2rx(subdev);
 
     csi2rx->source_pad = media_entity_get_fwnode_pad(&s_subdev->entity,
                                                     s_subdev->fwnode,
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
 
 /**
  * @brief Async notifier operations
  */
 static const struct v4l2_async_notifier_operations custom_csi2rx_notifier_ops = {
     .bound = custom_csi2rx_async_bound,
 };
 
 /**
  * @brief Acquire hardware resources for the CSI-2 RX
  * @param csi2rx Driver private data
  * @param pdev Platform device
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_get_resources(struct custom_csi2rx_priv *csi2rx,
                                       struct platform_device *pdev)
 {
     unsigned char i;
     u32 dev_cfg;
     int ret;
 
     csi2rx->base = devm_platform_ioremap_resource(pdev, 0);
     if (IS_ERR(csi2rx->base))
         return PTR_ERR(csi2rx->base);
 
     csi2rx->sys_clk = devm_clk_get(&pdev->dev, "sys_clk");
     if (IS_ERR(csi2rx->sys_clk)) {
         dev_err(&pdev->dev, "Couldn't get sys clock\n");
         return PTR_ERR(csi2rx->sys_clk);
     }
 
     csi2rx->p_clk = devm_clk_get(&pdev->dev, "p_clk");
     if (IS_ERR(csi2rx->p_clk)) {
         dev_err(&pdev->dev, "Couldn't get P clock\n");
         return PTR_ERR(csi2rx->p_clk);
     }
 
     csi2rx->dphy = devm_phy_optional_get(&pdev->dev, "dphy");
     if (IS_ERR(csi2rx->dphy)) {
         dev_err(&pdev->dev, "Couldn't get external D-PHY\n");
         return PTR_ERR(csi2rx->dphy);
     }
 
     ret = clk_prepare_enable(csi2rx->p_clk);
     if (ret) {
         dev_err(&pdev->dev, "Couldn't prepare and enable P clock\n");
         return ret;
     }
 
     dev_cfg = readl(csi2rx->base + CUSTOM_CSI2RX_DEVICE_CFG_REG);
     clk_disable_unprepare(csi2rx->p_clk);
 
     csi2rx->max_lanes = dev_cfg & 7;
     if (csi2rx->max_lanes > CUSTOM_CSI2RX_LANES_MAX) {
         dev_err(&pdev->dev, "Invalid number of lanes: %u\n", csi2rx->max_lanes);
         return -EINVAL;
     }
 
     csi2rx->max_streams = (dev_cfg >> 4) & 7;
     if (csi2rx->max_streams > CUSTOM_CSI2RX_STREAMS_MAX) {
         dev_err(&pdev->dev, "Invalid number of streams: %u\n", csi2rx->max_streams);
         return -EINVAL;
     }
 
     csi2rx->has_internal_dphy = dev_cfg & BIT(3) ? true : false;
 
     if (!csi2rx->dphy && csi2rx->has_internal_dphy) {
         dev_err(&pdev->dev, "Internal D-PHY not supported yet\n");
         return -EINVAL;
     }
 
     for (i = 0; i < csi2rx->max_streams; i++) {
         char clk_name[16];
 
         snprintf(clk_name, sizeof(clk_name), "pixel_if%u_clk", i);
         csi2rx->pixel_clk[i] = devm_clk_get(&pdev->dev, clk_name);
         if (IS_ERR(csi2rx->pixel_clk[i])) {
             dev_err(&pdev->dev, "Couldn't get clock %s\n", clk_name);
             return PTR_ERR(csi2rx->pixel_clk[i]);
         }
     }
 
     return 0;
 }
 
 /**
  * @brief Parse device tree for CSI-2 RX configuration
  * @param csi2rx Driver private data
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_parse_dt(struct custom_csi2rx_priv *csi2rx)
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
         dev_err(csi2rx->dev, "Could not parse v4l2 endpoint\n");
         of_node_put(ep);
         return ret;
     }
 
     if (v4l2_ep.bus_type != V4L2_MBUS_CSI2_DPHY) {
         dev_err(csi2rx->dev, "Unsupported media bus type: 0x%x\n", v4l2_ep.bus_type);
         of_node_put(ep);
         return -EINVAL;
     }
 
     memcpy(csi2rx->lanes, v4l2_ep.bus.mipi_csi2.data_lanes, sizeof(csi2rx->lanes));
     csi2rx->num_lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;
     if (csi2rx->num_lanes > csi2rx->max_lanes) {
         dev_err(csi2rx->dev, "Unsupported number of data-lanes: %d\n", csi2rx->num_lanes);
         of_node_put(ep);
         return -EINVAL;
     }
 
     v4l2_async_nf_init(&csi2rx->notifier);
 
     asd = v4l2_async_nf_add_fwnode_remote(&csi2rx->notifier, fwh,
                                           struct v4l2_async_subdev);
     of_node_put(ep);
     if (IS_ERR(asd)) {
         v4l2_async_nf_cleanup(&csi2rx->notifier);
         return PTR_ERR(asd);
     }
 
     csi2rx->notifier.ops = &custom_csi2rx_notifier_ops;
 
     ret = v4l2_async_subdev_nf_register(&csi2rx->subdev, &csi2rx->notifier);
     if (ret)
         v4l2_async_nf_cleanup(&csi2rx->notifier);
 
     return ret;
 }
 
 /**
  * @brief Suspend the CSI-2 RX device
  * @param dev Platform device
  * @return 0 on success
  */
 static int custom_csi2rx_suspend(struct device *dev)
 {
     struct custom_csi2rx_priv *csi2rx = dev_get_drvdata(dev);
 
     mutex_lock(&csi2rx->lock);
     if (csi2rx->count)
         custom_csi2rx_stop(csi2rx);
     mutex_unlock(&csi2rx->lock);
 
     return 0;
 }
 
 /**
  * @brief Resume the CSI-2 RX device
  * @param dev Platform device
  * @return 0 on success
  */
 static int custom_csi2rx_resume(struct device *dev)
 {
     struct custom_csi2rx_priv *csi2rx = dev_get_drvdata(dev);
 
     mutex_lock(&csi2rx->lock);
     if (csi2rx->count)
         custom_csi2rx_start(csi2rx);
     mutex_unlock(&csi2rx->lock);
     return 0;
 }
 
 /**
  * @brief Probe function for the CSI-2 RX platform driver
  * @param pdev Platform device
  * @return 0 on success, negative error code on failure
  */
 static int custom_csi2rx_probe(struct platform_device *pdev)
 {
     struct custom_csi2rx_priv *csi2rx;
     unsigned int i;
     int ret;
 
     csi2rx = kzalloc(sizeof(*csi2rx), GFP_KERNEL);
     if (!csi2rx)
         return -ENOMEM;
     platform_set_drvdata(pdev, csi2rx);
     csi2rx->dev = &pdev->dev;
     mutex_init(&csi2rx->lock);
 
     ret = custom_csi2rx_get_resources(csi2rx, pdev);
     if (ret)
         goto err_free_priv;
 
     ret = custom_csi2rx_parse_dt(csi2rx);
     if (ret)
         goto err_free_priv;
 
     csi2rx->subdev.owner = THIS_MODULE;
     csi2rx->subdev.dev = &pdev->dev;
     v4l2_subdev_init(&csi2rx->subdev, &custom_csi2rx_subdev_ops);
     v4l2_set_subdevdata(&csi2rx->subdev, &pdev->dev);
     snprintf(csi2rx->subdev.name, V4L2_SUBDEV_NAME_SIZE, "%s.%s",
              KBUILD_MODNAME, dev_name(&pdev->dev));
 
     csi2rx->subdev.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
     csi2rx->pads[CUSTOM_CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
     for (i = CUSTOM_CSI2RX_PAD_SOURCE_STREAM0; i < CUSTOM_CSI2RX_PAD_MAX; i++)
         csi2rx->pads[i].flags = MEDIA_PAD_FL_SOURCE;
     csi2rx->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
     csi2rx->subdev.entity.ops = &custom_csi2rx_media_ops;
 
     ret = media_entity_pads_init(&csi2rx->subdev.entity, CUSTOM_CSI2RX_PAD_MAX,
                                  csi2rx->pads);
     if (ret)
         goto err_cleanup;
 
     ret = v4l2_subdev_init_finalize(&csi2rx->subdev);
     if (ret)
         goto err_cleanup;
 
     pm_runtime_enable(csi2rx->dev);
     ret = v4l2_async_register_subdev(&csi2rx->subdev);
     if (ret < 0)
         goto err_free_subdev;
 
     dev_info(&pdev->dev,
              "Probed CUSTOM_CSI2RX with %u/%u lanes, %u streams, %s D-PHY\n",
              csi2rx->num_lanes, csi2rx->max_lanes, csi2rx->max_streams,
              csi2rx->dphy ? "external" :
              csi2rx->has_internal_dphy ? "internal" : "no");
 
     return 0;
 
 err_free_subdev:
     pm_runtime_disable(csi2rx->dev);
     v4l2_subdev_cleanup(&csi2rx->subdev);
 err_cleanup:
     v4l2_async_nf_unregister(&csi2rx->notifier);
     v4l2_async_nf_cleanup(&csi2rx->notifier);
     media_entity_cleanup(&csi2rx->subdev.entity);
 err_free_priv:
     kfree(csi2rx);
     return ret;
 }
 
 /**
  * @brief Remove function for the CSI-2 RX platform driver
  * @param pdev Platform device
  * @return 0 on success
  */
 static int custom_csi2rx_remove(struct platform_device *pdev)
 {
     struct custom_csi2rx_priv *csi2rx = platform_get_drvdata(pdev);
 
     v4l2_async_nf_unregister(&csi2rx->notifier);
     v4l2_async_nf_cleanup(&csi2rx->notifier);
     v4l2_async_unregister_subdev(&csi2rx->subdev);
     v4l2_subdev_cleanup(&csi2rx->subdev);
     media_entity_cleanup(&csi2rx->subdev.entity);
     pm_runtime_disable(csi2rx->dev);
     kfree(csi2rx);
 
     return 0;
 }
 
 /**
  * @brief Power management operations
  */
 static const struct dev_pm_ops custom_csi2rx_pm_ops = {
     SET_RUNTIME_PM_OPS(custom_csi2rx_suspend, custom_csi2rx_resume, NULL)
 };
 
 /**
  * @brief Device tree compatible strings
  */
 static const struct of_device_id csi2rx_of_table[] = {
     { .compatible = "cdns,csi2rx" },
     { },
 };
 MODULE_DEVICE_TABLE(of, csi2rx_of_table);
 
 /**
  * @brief Platform driver structure
  */
 static struct platform_driver custom_csi2rx_driver = {
     .probe  = custom_csi2rx_probe,
     .remove = custom_csi2rx_remove,
     .driver = {
         .name           = "custom-csi2rx",
         .of_match_table = csi2rx_of_table,
         .pm             = &custom_csi2rx_pm_ops,
     },
 };
 
 module_platform_driver(custom_csi2rx_driver);
 
 MODULE_AUTHOR("RAM");
 MODULE_DESCRIPTION("Custom Cadence CSI2-RX controller");
 MODULE_LICENSE("GPL");
