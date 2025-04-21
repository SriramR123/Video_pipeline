/**
 * @file imx219_custom.c
 * @brief Custom Linux kernel driver for the Sony IMX219 camera sensor.
 * @author Ram
 * @date April 2025
 *
 * This driver provides support for the Sony IMX219 camera sensor, operating as a
 * V4L2 subdevice and I2C device. It manages sensor initialization, configuration,
 * streaming, and controls such as resolution, exposure, and gain. The driver
 * interfaces with the sensor via I2C and outputs image data over the MIPI CSI-2
 * interface, integrating with other drivers (e.g., cdns_custom, j721e_custom) in
 * the media controller pipeline.
 *
 * @license GPL v2
 */

 #include <linux/clk.h>
 #include <linux/delay.h>
 #include <linux/gpio/consumer.h>
 #include <linux/i2c.h>
 #include <linux/module.h>
 #include <linux/pm_runtime.h>
 #include <linux/regulator/consumer.h>
 #include <media/v4l2-ctrls.h>
 #include <media/v4l2-device.h>
 #include <media/v4l2-event.h>
 #include <media/v4l2-fwnode.h>
 #include <media/v4l2-mediabus.h>
 #include <media/v4l2-async.h>
 #include <asm/unaligned.h>
 
 /** @brief Register value size for 8-bit access. */
 #define CUSTOM_IMX219_REG_VALUE_08BIT 1
 /** @brief Register value size for 16-bit access. */
 #define CUSTOM_IMX219_REG_VALUE_16BIT 2
 
 /** @brief Mode select register address. */
 #define CUSTOM_IMX219_REG_MODE_SELECT 0x0100
 /** @brief Standby mode (sensor idle). */
 #define CUSTOM_IMX219_MODE_STANDBY 0x00
 /** @brief Streaming mode (sensor capturing data). */
 #define CUSTOM_IMX219_MODE_STREAMING 0x01
 
 /** @brief Chip ID register address. */
 #define CUSTOM_IMX219_REG_CHIP_ID 0x0000
 /** @brief Expected chip ID for IMX219. */
 #define CUSTOM_IMX219_CHIP_ID 0x0219
 
 /** @brief External clock frequency (24 MHz). */
 #define CUSTOM_IMX219_XCLK_FREQ 24000000
 /** @brief Fixed pixel rate for all modes (182.4 MHz). */
 #define CUSTOM_IMX219_PIXEL_RATE 182400000
 /** @brief Default CSI-2 link frequency (456 MHz). */
 #define CUSTOM_IMX219_DEFAULT_LINK_FREQ 456000000
 
 /** @brief Vertical Timing Size (VTS) register address. */
 #define CUSTOM_IMX219_REG_VTS 0x0160
 /** @brief VTS for 15 FPS at full resolution. */
 #define CUSTOM_IMX219_VTS_15FPS 0x0dc6
 /** @brief VTS for 30 FPS at 1080p. */
 #define CUSTOM_IMX219_VTS_30FPS_1080P 0x06e3
 /** @brief VTS for 30 FPS with binning. */
 #define CUSTOM_IMX219_VTS_30FPS_BINNED 0x06e3
 /** @brief VTS for 30 FPS at 640x480. */
 #define CUSTOM_IMX219_VTS_30FPS_640x480 0x06e3
 /** @brief Maximum VTS value. */
 #define CUSTOM_IMX219_VTS_MAX 0xffff
 
 /** @brief Minimum vertical blanking lines. */
 #define CUSTOM_IMX219_VBLANK_MIN 4
 
 /** @brief Minimum Frame Length Line (FLL). */
 #define CUSTOM_IMX219_FLL_MIN 0x08a6
 /** @brief Maximum FLL value. */
 #define CUSTOM_IMX219_FLL_MAX 0xffff
 /** @brief FLL step size. */
 #define CUSTOM_IMX219_FLL_STEP 1
 /** @brief Default FLL value. */
 #define CUSTOM_IMX219_FLL_DEFAULT 0x0c98
 
 /** @brief Default pixels per line (read-only). */
 #define CUSTOM_IMX219_PPL_DEFAULT 3448
 
 /** @brief Exposure control register address. */
 #define CUSTOM_IMX219_REG_EXPOSURE 0x015a
 /** @brief Minimum exposure value. */
 #define CUSTOM_IMX219_EXPOSURE_MIN 4
 /** @brief Exposure step size. */
 #define CUSTOM_IMX219_EXPOSURE_STEP 1
 /** @brief Default exposure value. */
 #define CUSTOM_IMX219_EXPOSURE_DEFAULT 0x640
 /** @brief Maximum exposure value. */
 #define CUSTOM_IMX219_EXPOSURE_MAX 65535
 
 /** @brief Analog gain control register address. */
 #define CUSTOM_IMX219_REG_ANALOG_GAIN 0x0157
 /** @brief Minimum analog gain. */
 #define CUSTOM_IMX219_ANA_GAIN_MIN 0
 /** @brief Maximum analog gain. */
 #define CUSTOM_IMX219_ANA_GAIN_MAX 232
 /** @brief Analog gain step size. */
 #define CUSTOM_IMX219_ANA_GAIN_STEP 1
 /** @brief Default analog gain. */
 #define CUSTOM_IMX219_ANA_GAIN_DEFAULT 0x0
 
 /** @brief Digital gain control register address. */
 #define CUSTOM_IMX219_REG_DIGITAL_GAIN 0x0158
 /** @brief Minimum digital gain. */
 #define CUSTOM_IMX219_DGTL_GAIN_MIN 0x0100
 /** @brief Maximum digital gain. */
 #define CUSTOM_IMX219_DGTL_GAIN_MAX 0x0fff
 /** @brief Default digital gain. */
 #define CUSTOM_IMX219_DGTL_GAIN_DEFAULT 0x0100
 /** @brief Digital gain step size. */
 #define CUSTOM_IMX219_DGTL_GAIN_STEP 1
 
 /** @brief Orientation control register address (flip settings). */
 #define CUSTOM_IMX219_REG_ORIENTATION 0x0172
 
 /** @brief Binning mode register address. */
 #define CUSTOM_IMX219_REG_BINNING_MODE 0x0174
 /** @brief No binning mode. */
 #define CUSTOM_IMX219_BINNING_NONE 0x0000
 /** @brief 2x2 digital binning mode. */
 #define CUSTOM_IMX219_BINNING_2X2 0x0101
 /** @brief 2x2 analog binning mode. */
 #define CUSTOM_IMX219_BINNING_2X2_ANALOG 0x0303
 
 /** @brief Test pattern control register address. */
 #define CUSTOM_IMX219_REG_TEST_PATTERN 0x0600
 /** @brief Disable test pattern. */
 #define CUSTOM_IMX219_TEST_PATTERN_DISABLE 0
 /** @brief Solid color test pattern. */
 #define CUSTOM_IMX219_TEST_PATTERN_SOLID_COLOR 1
 /** @brief Color bars test pattern. */
 #define CUSTOM_IMX219_TEST_PATTERN_COLOR_BARS 2
 /** @brief Grey color bars test pattern. */
 #define CUSTOM_IMX219_TEST_PATTERN_GREY_COLOR 3
 /** @brief PN9 test pattern. */
 #define CUSTOM_IMX219_TEST_PATTERN_PN9 4
 
 /** @brief Test pattern red component register address. */
 #define CUSTOM_IMX219_REG_TESTP_RED 0x0602
 /** @brief Test pattern green (R) component register address. */
 #define CUSTOM_IMX219_REG_TESTP_GREENR 0x0604
 /** @brief Test pattern blue component register address. */
 #define CUSTOM_IMX219_REG_TESTP_BLUE 0x0606
 /** @brief Test pattern green (B) component register address. */
 #define CUSTOM_IMX219_REG_TESTP_GREENB 0x0608
 /** @brief Minimum test pattern color value. */
 #define CUSTOM_IMX219_TESTP_COLOUR_MIN 0
 /** @brief Maximum test pattern color value. */
 #define CUSTOM_IMX219_TESTP_COLOUR_MAX 0x03ff
 /** @brief Test pattern color step size. */
 #define CUSTOM_IMX219_TESTP_COLOUR_STEP 1
 /** @brief Default red test pattern value. */
 #define CUSTOM_IMX219_TESTP_RED_DEFAULT CUSTOM_IMX219_TESTP_COLOUR_MAX
 /** @brief Default green (R) test pattern value. */
 #define CUSTOM_IMX219_TESTP_GREENR_DEFAULT 0
 /** @brief Default blue test pattern value. */
 #define CUSTOM_IMX219_TESTP_BLUE_DEFAULT 0
 /** @brief Default green (B) test pattern value. */
 #define CUSTOM_IMX219_TESTP_GREENB_DEFAULT 0
 
 /** @brief Native sensor width (pixels). */
 #define CUSTOM_IMX219_NATIVE_WIDTH 3296U
 /** @brief Native sensor height (pixels). */
 #define CUSTOM_IMX219_NATIVE_HEIGHT 2480U
 /** @brief Pixel array left offset (pixels). */
 #define CUSTOM_IMX219_PIXEL_ARRAY_LEFT 8U
 /** @brief Pixel array top offset (pixels). */
 #define CUSTOM_IMX219_PIXEL_ARRAY_TOP 8U
 /** @brief Pixel array width (pixels). */
 #define CUSTOM_IMX219_PIXEL_ARRAY_WIDTH 3280U
 /** @brief Pixel array height (pixels). */
 #define CUSTOM_IMX219_PIXEL_ARRAY_HEIGHT 2464U
 
 /** @brief Minimum delay for XCLR signal (us). */
 #define CUSTOM_IMX219_XCLR_MIN_DELAY_US 6200
 /** @brief Delay range for XCLR signal (us). */
 #define CUSTOM_IMX219_XCLR_DELAY_RANGE_US 1000
 
 /**
  * @brief Structure to hold a single register address and value pair.
  */
 struct custom_imx219_reg {
     /** @brief Register address (16-bit). */
     u16 address;
     /** @brief Register value (8-bit). */
     u8 val;
 };
 
 /**
  * @brief Structure to hold a list of register configurations.
  */
 struct custom_imx219_reg_list {
     /** @brief Number of registers in the list. */
     unsigned int num_of_regs;
     /** @brief Pointer to array of register configurations. */
     const struct custom_imx219_reg *regs;
 };
 
 /**
  * @brief Structure to define a sensor operating mode.
  */
 struct custom_imx219_mode {
     /** @brief Width of the mode (pixels). */
     unsigned int width;
     /** @brief Height of the mode (pixels). */
     unsigned int height;
     /** @brief Crop rectangle for the mode. */
     struct v4l2_rect crop;
     /** @brief Default vertical timing size (VTS). */
     unsigned int vts_def;
     /** @brief Register list for configuring the mode. */
     struct custom_imx219_reg_list reg_list;
     /** @brief Whether binning is enabled for the mode. */
     bool binning;
 };
 
 /**
  * @brief Common register settings applied to all modes.
  */
 static const struct custom_imx219_reg custom_imx219_common_regs[] = {
     {0x0100, 0x00}, /* Mode Select: Standby */
     {0x0301, 0x05}, /* VTPXCK_DIV: Pixel clock divider */
     {0x0303, 0x01}, /* VTSYSCK_DIV: System clock divider */
     {0x0304, 0x03}, /* PREPLLCK_VT_DIV: PLL divider */
     {0x0305, 0x03}, /* PREPLLCK_OP_DIV: PLL divider */
     {0x0306, 0x00}, /* PLL_VT_MPY: PLL multiplier */
     {0x0307, 0x39},
     {0x030b, 0x01}, /* OP_SYS_CLK_DIV: Output clock divider */
     {0x030c, 0x00}, /* PLL_OP_MPY: PLL multiplier */
     {0x030d, 0x72},
     {0x0114, 0x01}, /* CSI 2-Lane Mode */
     {0x0128, 0x00}, /* DPHY Auto Mode */
     {0x012a, 0x18}, /* EXCK_Freq: External clock frequency */
     {0x012b, 0x00},
 };
 
 /**
  * @brief Register settings for 3280x2464 mode.
  */
 static const struct custom_imx219_reg custom_mode_3280x2464_regs[] = {
     {0x0164, 0x00}, {0x0165, 0x00}, {0x0166, 0x0c}, {0x0167, 0xcf},
     {0x0168, 0x00}, {0x0169, 0x00}, {0x016a, 0x09}, {0x016b, 0x9f},
     {0x016c, 0x0c}, {0x016d, 0xd0}, {0x016e, 0x09}, {0x016f, 0xa0},
     {0x0624, 0x0c}, {0x0625, 0xd0}, {0x0626, 0x09}, {0x0627, 0xa0},
 };
 
 /**
  * @brief Register settings for 1920x1080 mode.
  */
 static const struct custom_imx219_reg custom_mode_1920_1080_regs[] = {
     {0x0164, 0x02}, {0x0165, 0xa8}, {0x0166, 0x0a}, {0x0167, 0x27},
     {0x0168, 0x02}, {0x0169, 0xb4}, {0x016a, 0x06}, {0x016b, 0xeb},
     {0x016c, 0x07}, {0x016d, 0x80}, {0x016e, 0x04}, {0x016f, 0x38},
     {0x0624, 0x07}, {0x0625, 0x80}, {0x0626, 0x04}, {0x0627, 0x38},
 };
 
 /**
  * @brief Register settings for 1640x1232 mode (binned).
  */
 static const struct custom_imx219_reg custom_mode_1640_1232_regs[] = {
     {0x0164, 0x00}, {0x0165, 0x00}, {0x0166, 0x0c}, {0x0167, 0xcf},
     {0x0168, 0x00}, {0x0169, 0x00}, {0x016a, 0x09}, {0x016b, 0x9f},
     {0x016c, 0x06}, {0x016d, 0x68}, {0x016e, 0x04}, {0x016f, 0xd0},
     {0x0624, 0x06}, {0x0625, 0x68}, {0x0626, 0x04}, {0x0627, 0xd0},
 };
 
 /**
  * @brief Register settings for 640x480 mode (binned).
  */
 static const struct custom_imx219_reg custom_mode_640_480_regs[] = {
     {0x0164, 0x03}, {0x0165, 0xe8}, {0x0166, 0x08}, {0x0167, 0xe7},
     {0x0168, 0x02}, {0x0169, 0xf0}, {0x016a, 0x06}, {0x016b, 0xaf},
     {0x016c, 0x02}, {0x016d, 0x80}, {0x016e, 0x01}, {0x016f, 0xe0},
     {0x0624, 0x06}, {0x0625, 0x68}, {0x0626, 0x04}, {0x0627, 0xd0},
 };
 
 /**
  * @brief Register settings for RAW8 frame format.
  */
 static const struct custom_imx219_reg custom_raw8_framefmt_regs[] = {
     {0x018c, 0x08}, {0x018d, 0x08}, {0x0309, 0x08},
 };
 
 /**
  * @brief Register settings for RAW10 frame format.
  */
 static const struct custom_imx219_reg custom_raw10_framefmt_regs[] = {
     {0x018c, 0x0a}, {0x018d, 0x0a}, {0x0309, 0x0a},
 };
 
 /**
  * @brief Supported CSI-2 link frequencies.
  */
 static const s64 custom_imx219_link_freq_menu[] = {
     CUSTOM_IMX219_DEFAULT_LINK_FREQ,
 };
 
 /**
  * @brief Test pattern menu for V4L2 controls.
  */
 static const char * const custom_imx219_test_pattern_menu[] = {
     "Disabled", "Color Bars", "Solid Color", "Grey Color Bars", "PN9"
 };
 
 /**
  * @brief Test pattern values corresponding to menu options.
  */
 static const int custom_imx219_test_pattern_val[] = {
     CUSTOM_IMX219_TEST_PATTERN_DISABLE,
     CUSTOM_IMX219_TEST_PATTERN_COLOR_BARS,
     CUSTOM_IMX219_TEST_PATTERN_SOLID_COLOR,
     CUSTOM_IMX219_TEST_PATTERN_GREY_COLOR,
     CUSTOM_IMX219_TEST_PATTERN_PN9,
 };
 
 /**
  * @brief Regulator supply names for the sensor.
  */
 static const char * const custom_imx219_supply_name[] = {
     "VANA", /* Analog supply */
     "VDIG", /* Digital supply */
     "VDDL", /* Digital low supply */
 };
 
 /** @brief Number of regulator supplies. */
 #define CUSTOM_IMX219_NUM_SUPPLIES ARRAY_SIZE(custom_imx219_supply_name)
 
 /**
  * @brief Supported media bus formats (RAW8 and RAW10 Bayer).
  */
 static const u32 custom_codes[] = {
     MEDIA_BUS_FMT_SRGGB10_1X10,
     MEDIA_BUS_FMT_SGRBG10_1X10,
     MEDIA_BUS_FMT_SGBRG10_1X10,
     MEDIA_BUS_FMT_SBGGR10_1X10,
     MEDIA_BUS_FMT_SRGGB8_1X8,
     MEDIA_BUS_FMT_SGRBG8_1X8,
     MEDIA_BUS_FMT_SGBRG8_1X8,
     MEDIA_BUS_FMT_SBGGR8_1X8,
 };
 
 /**
  * @brief Supported sensor modes.
  */
 static const struct custom_imx219_mode custom_supported_modes[] = {
     {
         .width = 3280,
         .height = 2464,
         .crop = {
             .left = CUSTOM_IMX219_PIXEL_ARRAY_LEFT,
             .top = CUSTOM_IMX219_PIXEL_ARRAY_TOP,
             .width = 3280,
             .height = 2464
         },
         .vts_def = CUSTOM_IMX219_VTS_15FPS,
         .reg_list = {
             .num_of_regs = ARRAY_SIZE(custom_mode_3280x2464_regs),
             .regs = custom_mode_3280x2464_regs,
         },
         .binning = false,
     },
     {
         .width = 1920,
         .height = 1080,
         .crop = {
             .left = 688,
             .top = 700,
             .width = 1920,
             .height = 1080
         },
         .vts_def = CUSTOM_IMX219_VTS_30FPS_1080P,
         .reg_list = {
             .num_of_regs = ARRAY_SIZE(custom_mode_1920_1080_regs),
             .regs = custom_mode_1920_1080_regs,
         },
         .binning = false,
     },
     {
         .width = 1640,
         .height = 1232,
         .crop = {
             .left = CUSTOM_IMX219_PIXEL_ARRAY_LEFT,
             .top = CUSTOM_IMX219_PIXEL_ARRAY_TOP,
             .width = 3280,
             .height = 2464
         },
         .vts_def = CUSTOM_IMX219_VTS_30FPS_BINNED,
         .reg_list = {
             .num_of_regs = ARRAY_SIZE(custom_mode_1640_1232_regs),
             .regs = custom_mode_1640_1232_regs,
         },
         .binning = true,
     },
     {
         .width = 640,
         .height = 480,
         .crop = {
             .left = 1008,
             .top = 760,
             .width = 1280,
             .height = 960
         },
         .vts_def = CUSTOM_IMX219_VTS_30FPS_640x480,
         .reg_list = {
             .num_of_regs = ARRAY_SIZE(custom_mode_640_480_regs),
             .regs = custom_mode_640_480_regs,
         },
         .binning = true,
     },
 };
 
 /**
  * @brief Main driver structure for the IMX219 sensor.
  */
 struct custom_imx219 {
     /** @brief V4L2 subdevice instance. */
     struct v4l2_subdev sd;
     /** @brief Media pad for pipeline connections. */
     struct media_pad pad;
     /** @brief Current media bus format. */
     struct v4l2_mbus_framefmt fmt;
     /** @brief External clock. */
     struct clk *xclk;
     /** @brief External clock frequency (Hz). */
     u32 xclk_freq;
     /** @brief Reset GPIO descriptor. */
     struct gpio_desc *reset_gpio;
     /** @brief Power supply regulators. */
     struct regulator_bulk_data supplies[CUSTOM_IMX219_NUM_SUPPLIES];
     /** @brief V4L2 control handler. */
     struct v4l2_ctrl_handler ctrl_handler;
     /** @brief Pixel rate control. */
     struct v4l2_ctrl *pixel_rate;
     /** @brief Link frequency control. */
     struct v4l2_ctrl *link_freq;
     /** @brief Exposure control. */
     struct v4l2_ctrl *exposure;
     /** @brief Vertical flip control. */
     struct v4l2_ctrl *vflip;
     /** @brief Horizontal flip control. */
     struct v4l2_ctrl *hflip;
     /** @brief Vertical blanking control. */
     struct v4l2_ctrl *vblank;
     /** @brief Horizontal blanking control. */
     struct v4l2_ctrl *hblank;
     /** @brief Current sensor mode. */
     const struct custom_imx219_mode *mode;
     /** @brief Mutex for protecting shared data. */
     struct mutex mutex;
     /** @brief Streaming state (true if streaming). */
     bool streaming;
     /** @brief Async notifier for media controller. */
     struct v4l2_async_notifier notifier;
     /** @brief Async subdevice for binding. */
     struct v4l2_async_subdev *asd;
 };
 
 /**
  * @brief Convert V4L2 subdevice to custom_imx219 structure.
  * @param[in] _sd Pointer to V4L2 subdevice.
  * @return Pointer to custom_imx219 structure.
  */
 static inline struct custom_imx219 *to_custom_imx219(struct v4l2_subdev *_sd)
 {
     return container_of(_sd, struct custom_imx219, sd);
 }
 
 /**
  * @brief Read a register from the IMX219 sensor via I2C.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @param[in] reg Register address (16-bit).
  * @param[in] len Number of bytes to read (1 or 2).
  * @param[out] val Pointer to store the read value.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_read_reg(struct custom_imx219 *imx219, u16 reg, u32 len, u32 *val)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     struct i2c_msg msgs[2];
     u8 addr_buf[2] = { reg >> 8, reg & 0xff };
     u8 data_buf[4] = { 0 };
     int ret;
 
     if (len > 4) {
         return -EINVAL;
     }
 
     msgs[0].addr = client->addr;
     msgs[0].flags = 0;
     msgs[0].len = ARRAY_SIZE(addr_buf);
     msgs[0].buf = addr_buf;
 
     msgs[1].addr = client->addr;
     msgs[1].flags = I2C_M_RD;
     msgs[1].len = len;
     msgs[1].buf = &data_buf[4 - len];
 
     ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
     if (ret != ARRAY_SIZE(msgs)) {
         return -EIO;
     }
 
     *val = get_unaligned_be32(data_buf);
     return 0;
 }
 
 /**
  * @brief Write a value to an IMX219 sensor register via I2C.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @param[in] reg Register address (16-bit).
  * @param[in] len Number of bytes to write (1 or 2).
  * @param[in] val Value to write.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_write_reg(struct custom_imx219 *imx219, u16 reg, u32 len, u32 val)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     u8 buf[6];
 
     if (len > 4) {
         return -EINVAL;
     }
 
     put_unaligned_be16(reg, buf);
     put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
     if (i2c_master_send(client, buf, len + 2) != len + 2) {
         return -EIO;
     }
 
     return 0;
 }
 
 /**
  * @brief Write a list of registers to the IMX219 sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @param[in] regs Array of register configurations.
  * @param[in] len Number of registers to write.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_write_regs(struct custom_imx219 *imx219,
                                     const struct custom_imx219_reg *regs, u32 len)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     unsigned int i;
     int ret;
 
     for (i = 0; i < len; i++) {
         ret = custom_imx219_write_reg(imx219, regs[i].address, 1, regs[i].val);
         if (ret) {
             dev_err_ratelimited(&client->dev,
                                 "Failed to write reg 0x%4.4x. error = %d\n",
                                 regs[i].address, ret);
             return ret;
         }
     }
     return 0;
 }
 
 /**
  * @brief Get the media bus format code, adjusting for flip settings.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @param[in] code Requested media bus format code.
  * @return Adjusted media bus format code.
  */
 static u32 custom_imx219_get_format_code(struct custom_imx219 *imx219, u32 code)
 {
     unsigned int i;
 
     lockdep_assert_held(&imx219->mutex);
 
     for (i = 0; i < ARRAY_SIZE(custom_codes); i++) {
         if (custom_codes[i] == code) {
             break;
         }
     }
 
     if (i >= ARRAY_SIZE(custom_codes)) {
         i = 0;
     }
 
     i = (i & ~3) | (imx219->vflip->val ? 2 : 0) | (imx219->hflip->val ? 1 : 0);
     return custom_codes[i];
 }
 
 /**
  * @brief Set the default media bus format for the sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  */
 static void custom_imx219_set_default_format(struct custom_imx219 *imx219)
 {
     struct v4l2_mbus_framefmt *fmt;
 
     fmt = &imx219->fmt;
     fmt->code = MEDIA_BUS_FMT_SRGGB10_1X10;
     fmt->colorspace = V4L2_COLORSPACE_SRGB;
     fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
     fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace, fmt->ycbcr_enc);
     fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
     fmt->width = custom_supported_modes[0].width;
     fmt->height = custom_supported_modes[0].height;
     fmt->field = V4L2_FIELD_NONE;
 }
 
 /**
  * @brief Initialize format and crop settings when opening the subdevice.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] fh File handle for the subdevice.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
     struct v4l2_mbus_framefmt *try_fmt = v4l2_subdev_get_try_format(sd, fh->state, 0);
     struct v4l2_rect *try_crop;
 
     mutex_lock(&imx219->mutex);
 
     try_fmt->width = custom_supported_modes[0].width;
     try_fmt->height = custom_supported_modes[0].height;
     try_fmt->code = custom_imx219_get_format_code(imx219, MEDIA_BUS_FMT_SRGGB10_1X10);
     try_fmt->field = V4L2_FIELD_NONE;
 
     try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
     try_crop->top = CUSTOM_IMX219_PIXEL_ARRAY_TOP;
     try_crop->left = CUSTOM_IMX219_PIXEL_ARRAY_LEFT;
     try_crop->width = CUSTOM_IMX219_PIXEL_ARRAY_WIDTH;
     try_crop->height = CUSTOM_IMX219_PIXEL_ARRAY_HEIGHT;
 
     mutex_unlock(&imx219->mutex);
     return 0;
 }
 
 /**
  * @brief Set a V4L2 control value and update sensor registers.
  * @param[in] ctrl Pointer to V4L2 control.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_set_ctrl(struct v4l2_ctrl *ctrl)
 {
     struct custom_imx219 *imx219 = container_of(ctrl->handler, struct custom_imx219, ctrl_handler);
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     int ret;
 
     if (ctrl->id == V4L2_CID_VBLANK) {
         int exposure_max, exposure_def;
         exposure_max = imx219->mode->height + ctrl->val - 4;
         exposure_def = (exposure_max < CUSTOM_IMX219_EXPOSURE_DEFAULT) ?
                        exposure_max : CUSTOM_IMX219_EXPOSURE_DEFAULT;
         __v4l2_ctrl_modify_range(imx219->exposure, imx219->exposure->minimum,
                                  exposure_max, imx219->exposure->step, exposure_def);
     }
 
     if (pm_runtime_get_if_in_use(&client->dev) == 0) {
         return 0;
     }
 
     switch (ctrl->id) {
     case V4L2_CID_ANALOGUE_GAIN:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_ANALOG_GAIN,
                                      CUSTOM_IMX219_REG_VALUE_08BIT, ctrl->val);
         break;
     case V4L2_CID_EXPOSURE:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_EXPOSURE,
                                      CUSTOM_IMX219_REG_VALUE_16BIT, ctrl->val);
         break;
     case V4L2_CID_DIGITAL_GAIN:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_DIGITAL_GAIN,
                                      CUSTOM_IMX219_REG_VALUE_16BIT, ctrl->val);
         break;
     case V4L2_CID_TEST_PATTERN:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_TEST_PATTERN,
                                      CUSTOM_IMX219_REG_VALUE_16BIT,
                                      custom_imx219_test_pattern_val[ctrl->val]);
         break;
     case V4L2_CID_HFLIP:
     case V4L2_CID_VFLIP:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_ORIENTATION, 1,
                                      imx219->hflip->val | (imx219->vflip->val << 1));
         break;
     case V4L2_CID_VBLANK:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_VTS,
                                      CUSTOM_IMX219_REG_VALUE_16BIT,
                                      imx219->mode->height + ctrl->val);
         break;
     case V4L2_CID_TEST_PATTERN_RED:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_TESTP_RED,
                                      CUSTOM_IMX219_REG_VALUE_16BIT, ctrl->val);
         break;
     case V4L2_CID_TEST_PATTERN_GREENR:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_TESTP_GREENR,
                                      CUSTOM_IMX219_REG_VALUE_16BIT, ctrl->val);
         break;
     case V4L2_CID_TEST_PATTERN_BLUE:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_TESTP_BLUE,
                                      CUSTOM_IMX219_REG_VALUE_16BIT, ctrl->val);
         break;
     case V4L2_CID_TEST_PATTERN_GREENB:
         ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_TESTP_GREENB,
                                      CUSTOM_IMX219_REG_VALUE_16BIT, ctrl->val);
         break;
     default:
         dev_info(&client->dev, "ctrl(id:0x%x,val:0x%x) is not handled\n",
                  ctrl->id, ctrl->val);
         ret = -EINVAL;
         break;
     }
 
     pm_runtime_put(&client->dev);
     return ret;
 }
 
 /**
  * @brief V4L2 control operations structure.
  */
 static const struct v4l2_ctrl_ops custom_imx219_ctrl_ops = {
     .s_ctrl = custom_imx219_set_ctrl,
 };
 
 /**
  * @brief Enumerate supported media bus codes.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] sd_state Subdevice state.
  * @param[in,out] code Media bus code enumeration.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_enum_mbus_code(struct v4l2_subdev *sd,
                                        struct v4l2_subdev_state *sd_state,
                                        struct v4l2_subdev_mbus_code_enum *code)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
 
     if (code->index >= (ARRAY_SIZE(custom_codes) / 4)) {
         return -EINVAL;
     }
 
     mutex_lock(&imx219->mutex);
     code->code = custom_imx219_get_format_code(imx219, custom_codes[code->index * 4]);
     mutex_unlock(&imx219->mutex);
 
     return 0;
 }
 
 /**
  * @brief Enumerate supported frame sizes for a given format.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] sd_state Subdevice state.
  * @param[in,out] fse Frame size enumeration.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_enum_frame_size(struct v4l2_subdev *sd,
                                         struct v4l2_subdev_state *sd_state,
                                         struct v4l2_subdev_frame_size_enum *fse)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
     u32 code;
 
     if (fse->index >= ARRAY_SIZE(custom_supported_modes)) {
         return -EINVAL;
     }
 
     mutex_lock(&imx219->mutex);
     code = custom_imx219_get_format_code(imx219, fse->code);
     mutex_unlock(&imx219->mutex);
     if (fse->code != code) {
         return -EINVAL;
     }
 
     fse->min_width = custom_supported_modes[fse->index].width;
     fse->max_width = fse->min_width;
     fse->min_height = custom_supported_modes[fse->index].height;
     fse->max_height = fse->min_height;
 
     return 0;
 }
 
 /**
  * @brief Reset colorspace settings for a format.
  * @param[in,out] fmt Pointer to media bus frame format.
  */
 static void custom_imx219_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
 {
     fmt->colorspace = V4L2_COLORSPACE_SRGB;
     fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
     fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true, fmt->colorspace, fmt->ycbcr_enc);
     fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
 }
 
 /**
  * @brief Update pad format based on the current mode.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @param[in] mode Pointer to sensor mode.
  * @param[in,out] fmt Pointer to subdevice format.
  */
 static void custom_imx219_update_pad_format(struct custom_imx219 *imx219,
                                            const struct custom_imx219_mode *mode,
                                            struct v4l2_subdev_format *fmt)
 {
     fmt->format.width = mode->width;
     fmt->format.height = mode->height;
     fmt->format.field = V4L2_FIELD_NONE;
     custom_imx219_reset_colorspace(&fmt->format);
 }
 
 /**
  * @brief Get the current pad format.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] sd_state Subdevice state.
  * @param[in,out] fmt Pointer to subdevice format.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_get_pad_format(struct v4l2_subdev *sd,
                                        struct v4l2_subdev_state *sd_state,
                                        struct v4l2_subdev_format *fmt)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
 
     mutex_lock(&imx219->mutex);
     if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
         struct v4l2_mbus_framefmt *try_fmt =
             v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
         try_fmt->code = custom_imx219_get_format_code(imx219, try_fmt->code);
         fmt->format = *try_fmt;
     } else {
         custom_imx219_update_pad_format(imx219, imx219->mode, fmt);
         fmt->format.code = custom_imx219_get_format_code(imx219, imx219->fmt.code);
     }
     mutex_unlock(&imx219->mutex);
 
     return 0;
 }
 
 /**
  * @brief Set the pad format and update sensor mode.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] sd_state Subdevice state.
  * @param[in,out] fmt Pointer to subdevice format.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_set_pad_format(struct v4l2_subdev *sd,
                                        struct v4l2_subdev_state *sd_state,
                                        struct v4l2_subdev_format *fmt)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
     const struct custom_imx219_mode *mode;
     struct v4l2_mbus_framefmt *framefmt;
     int exposure_max, exposure_def, hblank;
     unsigned int i;
 
     mutex_lock(&imx219->mutex);
 
     for (i = 0; i < ARRAY_SIZE(custom_codes); i++) {
         if (custom_codes[i] == fmt->format.code) {
             break;
         }
     }
     if (i >= ARRAY_SIZE(custom_codes)) {
         i = 0;
     }
 
     fmt->format.code = custom_imx219_get_format_code(imx219, custom_codes[i]);
 
     mode = v4l2_find_nearest_size(custom_supported_modes,
                                   ARRAY_SIZE(custom_supported_modes),
                                   width, height,
                                   fmt->format.width, fmt->format.height);
     custom_imx219_update_pad_format(imx219, mode, fmt);
 
     if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
         framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
         *framefmt = fmt->format;
     } else if (imx219->mode != mode || imx219->fmt.code != fmt->format.code) {
         imx219->fmt = fmt->format;
         imx219->mode = mode;
         __v4l2_ctrl_modify_range(imx219->vblank, CUSTOM_IMX219_VBLANK_MIN,
                                  CUSTOM_IMX219_VTS_MAX - mode->height, 1,
                                  mode->vts_def - mode->height);
         __v4l2_ctrl_s_ctrl(imx219->vblank, mode->vts_def - mode->height);
         exposure_max = mode->vts_def - 4;
         exposure_def = (exposure_max < CUSTOM_IMX219_EXPOSURE_DEFAULT) ?
                        exposure_max : CUSTOM_IMX219_EXPOSURE_DEFAULT;
         __v4l2_ctrl_modify_range(imx219->exposure, imx219->exposure->minimum,
                                  exposure_max, imx219->exposure->step, exposure_def);
         hblank = CUSTOM_IMX219_PPL_DEFAULT - mode->width;
         __v4l2_ctrl_modify_range(imx219->hblank, hblank, hblank, 1, hblank);
     }
 
     mutex_unlock(&imx219->mutex);
     return 0;
 }
 
 /**
  * @brief Configure binning mode for the sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_set_binning(struct custom_imx219 *imx219)
 {
     if (!imx219->mode->binning) {
         return custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_BINNING_MODE,
                                       CUSTOM_IMX219_REG_VALUE_16BIT,
                                       CUSTOM_IMX219_BINNING_NONE);
     }
 
     switch (imx219->fmt.code) {
     case MEDIA_BUS_FMT_SRGGB8_1X8:
     case MEDIA_BUS_FMT_SGRBG8_1X8:
     case MEDIA_BUS_FMT_SGBRG8_1X8:
     case MEDIA_BUS_FMT_SBGGR8_1X8:
         return custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_BINNING_MODE,
                                       CUSTOM_IMX219_REG_VALUE_16BIT,
                                       CUSTOM_IMX219_BINNING_2X2_ANALOG);
     case MEDIA_BUS_FMT_SRGGB10_1X10:
     case MEDIA_BUS_FMT_SGRBG10_1X10:
     case MEDIA_BUS_FMT_SGBRG10_1X10:
     case MEDIA_BUS_FMT_SBGGR10_1X10:
         return custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_BINNING_MODE,
                                       CUSTOM_IMX219_REG_VALUE_16BIT,
                                       CUSTOM_IMX219_BINNING_2X2);
     }
 
     return -EINVAL;
 }
 
 /**
  * @brief Get the crop rectangle for a pad.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @param[in] sd_state Subdevice state.
  * @param[in] pad Pad number.
  * @param[in] which Format type (try or active).
  * @return Pointer to crop rectangle, or NULL if invalid.
  */
 static const struct v4l2_rect *
 __custom_imx219_get_pad_crop(struct custom_imx219 *imx219,
                              struct v4l2_subdev_state *sd_state,
                              unsigned int pad, enum v4l2_subdev_format_whence which)
 {
     switch (which) {
     case V4L2_SUBDEV_FORMAT_TRY:
         return v4l2_subdev_get_try_crop(&imx219->sd, sd_state, pad);
     case V4L2_SUBDEV_FORMAT_ACTIVE:
         return &imx219->mode->crop;
     }
     return NULL;
 }
 
 /**
  * @brief Get selection information (crop, native size, etc.).
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] sd_state Subdevice state.
  * @param[in,out] sel Pointer to selection structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_get_selection(struct v4l2_subdev *sd,
                                       struct v4l2_subdev_state *sd_state,
                                       struct v4l2_subdev_selection *sel)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
 
     switch (sel->target) {
     case V4L2_SEL_TGT_CROP:
         mutex_lock(&imx219->mutex);
         sel->r = *__custom_imx219_get_pad_crop(imx219, sd_state, sel->pad, sel->which);
         mutex_unlock(&imx219->mutex);
         return 0;
 
     case V4L2_SEL_TGT_NATIVE_SIZE:
         sel->r.top = 0;
         sel->r.left = 0;
         sel->r.width = CUSTOM_IMX219_NATIVE_WIDTH;
         sel->r.height = CUSTOM_IMX219_NATIVE_HEIGHT;
         return 0;
 
     case V4L2_SEL_TGT_CROP_DEFAULT:
     case V4L2_SEL_TGT_CROP_BOUNDS:
         sel->r.top = CUSTOM_IMX219_PIXEL_ARRAY_TOP;
         sel->r.left = CUSTOM_IMX219_PIXEL_ARRAY_LEFT;
         sel->r.width = CUSTOM_IMX219_PIXEL_ARRAY_WIDTH;
         sel->r.height = CUSTOM_IMX219_PIXEL_ARRAY_HEIGHT;
         return 0;
     }
     return -EINVAL;
 }
 
 /**
  * @brief Get frame descriptor for the sensor.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] pad Pad number.
  * @param[in,out] fd Pointer to frame descriptor.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
                                        struct v4l2_mbus_frame_desc *fd)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
     const struct custom_imx219_mode *mode = imx219->mode;
     u32 bpp;
     int ret = 0;
 
     if (pad != 0) {
         return -EINVAL;
     }
 
     mutex_lock(&imx219->mutex);
 
     memset(fd, 0, sizeof(*fd));
     fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
 
     if (imx219->fmt.code == MEDIA_BUS_FMT_SRGGB10_1X10 ||
         imx219->fmt.code == MEDIA_BUS_FMT_SGRBG10_1X10 ||
         imx219->fmt.code == MEDIA_BUS_FMT_SGBRG10_1X10 ||
         imx219->fmt.code == MEDIA_BUS_FMT_SBGGR10_1X10) {
         bpp = 10;
     } else {
         bpp = 8;
     }
 
     fd->entry[fd->num_entries].stream = 0;
     fd->entry[fd->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
     fd->entry[fd->num_entries].length = (mode->width * mode->height * bpp) / 8;
     fd->entry[fd->num_entries].pixelcode = imx219->fmt.code;
     fd->entry[fd->num_entries].bus.csi2.vc = 0;
     if (bpp == 8) {
         fd->entry[fd->num_entries].bus.csi2.dt = 0x2a; /* SRGGB8 */
     } else {
         fd->entry[fd->num_entries].bus.csi2.dt = 0x2b; /* SRGGB10 */
     }
     fd->num_entries++;
 
     mutex_unlock(&imx219->mutex);
     return ret;
 }
 
 /**
  * @brief Start streaming on the sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_start_streaming(struct custom_imx219 *imx219)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     const struct custom_imx219_reg_list *reg_list;
     int ret;
 
     ret = pm_runtime_resume_and_get(&client->dev);
     if (ret < 0) {
         return ret;
     }
 
     ret = custom_imx219_write_regs(imx219, custom_imx219_common_regs,
                                   ARRAY_SIZE(custom_imx219_common_regs));
     if (ret) {
         dev_err(&client->dev, "%s failed to send common regs\n", __func__);
         goto err_rpm_put;
     }
 
     if (imx219->fmt.code == MEDIA_BUS_FMT_SRGGB8_1X8 ||
         imx219->fmt.code == MEDIA_BUS_FMT_SGRBG8_1X8 ||
         imx219->fmt.code == MEDIA_BUS_FMT_SGBRG8_1X8 ||
         imx219->fmt.code == MEDIA_BUS_FMT_SBGGR8_1X8) {
         ret = custom_imx219_write_regs(imx219, custom_raw8_framefmt_regs,
                                       ARRAY_SIZE(custom_raw8_framefmt_regs));
     } else {
         ret = custom_imx219_write_regs(imx219, custom_raw10_framefmt_regs,
                                       ARRAY_SIZE(custom_raw10_framefmt_regs));
     }
     if (ret) {
         dev_err(&client->dev, "%s failed to set frame format\n", __func__);
         goto err_rpm_put;
     }
 
     reg_list = &imx219->mode->reg_list;
     ret = custom_imx219_write_regs(imx219, reg_list->regs, reg_list->num_of_regs);
     if (ret) {
         dev_err(&client->dev, "%s failed to set mode\n", __func__);
         goto err_rpm_put;
     }
 
     ret = custom_imx219_set_binning(imx219);
     if (ret) {
         dev_err(&client->dev, "%s failed to set binning\n", __func__);
         goto err_rpm_put;
     }
 
     ret = __v4l2_ctrl_handler_setup(imx219->sd.ctrl_handler);
     if (ret) {
         dev_err(&client->dev, "%s failed to apply controls\n", __func__);
         goto err_rpm_put;
     }
 
     ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_MODE_SELECT,
                                  CUSTOM_IMX219_REG_VALUE_08BIT,
                                  CUSTOM_IMX219_MODE_STREAMING);
     if (ret) {
         dev_err(&client->dev, "%s failed to start streaming\n", __func__);
         goto err_rpm_put;
     }
 
     __v4l2_ctrl_grab(imx219->vflip, true);
     __v4l2_ctrl_grab(imx219->hflip, true);
 
     return 0;
 
 err_rpm_put:
     pm_runtime_put(&client->dev);
     return ret;
 }
 
 /**
  * @brief Stop streaming on the sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  */
 static void custom_imx219_stop_streaming(struct custom_imx219 *imx219)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     int ret;
 
     ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_MODE_SELECT,
                                  CUSTOM_IMX219_REG_VALUE_08BIT,
                                  CUSTOM_IMX219_MODE_STANDBY);
     if (ret) {
         dev_err(&client->dev, "%s failed to stop streaming: %d\n", __func__, ret);
     }
 
     __v4l2_ctrl_grab(imx219->vflip, false);
     __v4l2_ctrl_grab(imx219->hflip, false);
 
     pm_runtime_put(&client->dev);
 }
 
 /**
  * @brief Set streaming state for the subdevice.
  * @param[in] sd Pointer to V4L2 subdevice.
  * @param[in] enable Non-zero to start streaming, zero to stop.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_set_stream(struct v4l2_subdev *sd, int enable)
 {
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
     int ret = 0;
 
     mutex_lock(&imx219->mutex);
     if (imx219->streaming == enable) {
         mutex_unlock(&imx219->mutex);
         return 0;
     }
 
     if (enable) {
         ret = custom_imx219_start_streaming(imx219);
         if (ret) {
             goto err_unlock;
         }
     } else {
         custom_imx219_stop_streaming(imx219);
     }
 
     imx219->streaming = enable;
     mutex_unlock(&imx219->mutex);
     return ret;
 
 err_unlock:
     mutex_unlock(&imx219->mutex);
     return ret;
 }
 
 /**
  * @brief Power on the sensor.
  * @param[in] dev Pointer to device structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_power_on(struct device *dev)
 {
     struct v4l2_subdev *sd = dev_get_drvdata(dev);
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
     int ret;
 
     ret = regulator_bulk_enable(CUSTOM_IMX219_NUM_SUPPLIES, imx219->supplies);
     if (ret) {
         dev_err(dev, "%s: failed to enable regulators\n", __func__);
         return ret;
     }
 
     ret = clk_prepare_enable(imx219->xclk);
     if (ret) {
         dev_err(dev, "%s: failed to enable clock\n", __func__);
         goto reg_off;
     }
 
     gpiod_set_value_cansleep(imx219->reset_gpio, 1);
     usleep_range(CUSTOM_IMX219_XCLR_MIN_DELAY_US,
                  CUSTOM_IMX219_XCLR_MIN_DELAY_US + CUSTOM_IMX219_XCLR_DELAY_RANGE_US);
 
     return 0;
 
 reg_off:
     regulator_bulk_disable(CUSTOM_IMX219_NUM_SUPPLIES, imx219->supplies);
     return ret;
 }
 
 /**
  * @brief Power off the sensor.
  * @param[in] dev Pointer to device structure.
  * @return 0 on success.
  */
 static int custom_imx219_power_off(struct device *dev)
 {
     struct v4l2_subdev *sd = dev_get_drvdata(dev);
     struct custom_imx219 *imx219 = to_custom_imx219(sd);
 
     gpiod_set_value_cansleep(imx219->reset_gpio, 0);
     regulator_bulk_disable(CUSTOM_IMX219_NUM_SUPPLIES, imx219->supplies);
     clk_disable_unprepare(imx219->xclk);
 
     return 0;
 }
 
 /**
  * @brief Acquire regulators for the sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_get_regulators(struct custom_imx219 *imx219)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     unsigned int i;
 
     for (i = 0; i < CUSTOM_IMX219_NUM_SUPPLIES; i++) {
         imx219->supplies[i].supply = custom_imx219_supply_name[i];
     }
 
     return devm_regulator_bulk_get(&client->dev, CUSTOM_IMX219_NUM_SUPPLIES, imx219->supplies);
 }
 
 /**
  * @brief Verify the sensor's chip ID.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_identify_module(struct custom_imx219 *imx219)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     int ret;
     u32 val;
 
     ret = custom_imx219_read_reg(imx219, CUSTOM_IMX219_REG_CHIP_ID,
                                 CUSTOM_IMX219_REG_VALUE_16BIT, &val);
     if (ret) {
         dev_err(&client->dev, "failed to read chip id %x\n", CUSTOM_IMX219_CHIP_ID);
         return ret;
     }
 
     if (val != CUSTOM_IMX219_CHIP_ID) {
         dev_err(&client->dev, "chip id mismatch: %x!=%x\n", CUSTOM_IMX219_CHIP_ID, val);
         return -EIO;
     }
 
     return 0;
 }
 
 /**
  * @brief Handle async subdevice binding.
  * @param[in] notifier Pointer to async notifier.
  * @param[in] subdev Pointer to bound subdevice.
  * @param[in] asd Pointer to async subdevice.
  * @return 0 on success.
  */
 static int custom_imx219_async_bound(struct v4l2_async_notifier *notifier,
                                     struct v4l2_subdev *subdev,
                                     struct v4l2_async_subdev *asd)
 {
     struct custom_imx219 *imx219 = container_of(notifier, struct custom_imx219, notifier);
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
 
     dev_info(&client->dev, "Async subdev bound successfully to %s\n", subdev->name);
     return 0;
 }
 
 /**
  * @brief Handle async subdevice unbinding.
  * @param[in] notifier Pointer to async notifier.
  * @param[in] subdev Pointer to unbound subdevice.
  * @param[in] asd Pointer to async subdevice.
  */
 static void custom_imx219_async_unbind(struct v4l2_async_notifier *notifier,
                                       struct v4l2_subdev *subdev,
                                       struct v4l2_async_subdev *asd)
 {
     struct custom_imx219 *imx219 = container_of(notifier, struct custom_imx219, notifier);
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
 
     dev_info(&client->dev, "Async subdev unbound from %s\n", subdev->name);
 }
 
 /**
  * @brief Async notifier operations structure.
  */
 static const struct v4l2_async_notifier_operations custom_imx219_async_ops = {
     .bound = custom_imx219_async_bound,
     .unbind = custom_imx219_async_unbind,
 };
 
 /**
  * @brief V4L2 subdevice core operations.
  */
 static const struct v4l2_subdev_core_ops custom_imx219_core_ops = {
     .subscribe_event = v4l2_ctrl_subdev_subscribe_event,
     .unsubscribe_event = v4l2_event_subdev_unsubscribe,
 };
 
 /**
  * @brief V4L2 subdevice video operations.
  */
 static const struct v4l2_subdev_video_ops custom_imx219_video_ops = {
     .s_stream = custom_imx219_set_stream,
 };
 
 /**
  * @brief V4L2 subdevice pad operations.
  */
 static const struct v4l2_subdev_pad_ops custom_imx219_pad_ops = {
     .enum_mbus_code = custom_imx219_enum_mbus_code,
     .get_fmt = custom_imx219_get_pad_format,
     .set_fmt = custom_imx219_set_pad_format,
     .enum_frame_size = custom_imx219_enum_frame_size,
     .get_selection = custom_imx219_get_selection,
     .get_frame_desc = custom_imx219_get_frame_desc,
 };
 
 /**
  * @brief V4L2 subdevice operations.
  */
 static const struct v4l2_subdev_ops custom_imx219_subdev_ops = {
     .core = &custom_imx219_core_ops,
     .video = &custom_imx219_video_ops,
     .pad = &custom_imx219_pad_ops,
 };
 
 /**
  * @brief V4L2 subdevice internal operations.
  */
 static const struct v4l2_subdev_internal_ops custom_imx219_internal_ops = {
     .open = custom_imx219_open,
 };
 
 /**
  * @brief Initialize V4L2 controls for the sensor.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_init_controls(struct custom_imx219 *imx219)
 {
     struct i2c_client *client = v4l2_get_subdevdata(&imx219->sd);
     struct v4l2_ctrl_handler *ctrl_hdlr = &imx219->ctrl_handler;
     unsigned int height = imx219->mode->height;
     int exposure_max, exposure_def, hblank;
     int ret;
 
     ret = v4l2_ctrl_handler_init(ctrl_hdlr, 12);
     if (ret) {
         return ret;
     }
 
     mutex_init(&imx219->mutex);
     ctrl_hdlr->lock = &imx219->mutex;
 
     imx219->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                           V4L2_CID_PIXEL_RATE,
                                           CUSTOM_IMX219_PIXEL_RATE,
                                           CUSTOM_IMX219_PIXEL_RATE, 1,
                                           CUSTOM_IMX219_PIXEL_RATE);
 
     imx219->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                               V4L2_CID_LINK_FREQ,
                                               ARRAY_SIZE(custom_imx219_link_freq_menu) - 1, 0,
                                               custom_imx219_link_freq_menu);
     if (imx219->link_freq) {
         imx219->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
     }
 
     imx219->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                       V4L2_CID_VBLANK, CUSTOM_IMX219_VBLANK_MIN,
                                       CUSTOM_IMX219_VTS_MAX - height, 1,
                                       imx219->mode->vts_def - height);
     hblank = CUSTOM_IMX219_PPL_DEFAULT - imx219->mode->width;
     imx219->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                       V4L2_CID_HBLANK, hblank, hblank, 1, hblank);
     if (imx219->hblank) {
         imx219->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
     }
     exposure_max = imx219->mode->vts_def - 4;
     exposure_def = (exposure_max < CUSTOM_IMX219_EXPOSURE_DEFAULT) ?
                    exposure_max : CUSTOM_IMX219_EXPOSURE_DEFAULT;
     imx219->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                         V4L2_CID_EXPOSURE,
                                         CUSTOM_IMX219_EXPOSURE_MIN, exposure_max,
                                         CUSTOM_IMX219_EXPOSURE_STEP, exposure_def);
 
     v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                       CUSTOM_IMX219_ANA_GAIN_MIN, CUSTOM_IMX219_ANA_GAIN_MAX,
                       CUSTOM_IMX219_ANA_GAIN_STEP, CUSTOM_IMX219_ANA_GAIN_DEFAULT);
 
     v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
                       CUSTOM_IMX219_DGTL_GAIN_MIN, CUSTOM_IMX219_DGTL_GAIN_MAX,
                       CUSTOM_IMX219_DGTL_GAIN_STEP, CUSTOM_IMX219_DGTL_GAIN_DEFAULT);
 
     imx219->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                      V4L2_CID_HFLIP, 0, 1, 1, 0);
     if (imx219->hflip) {
         imx219->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
     }
 
     imx219->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                      V4L2_CID_VFLIP, 0, 1, 1, 0);
     if (imx219->vflip) {
         imx219->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
     }
 
     v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &custom_imx219_ctrl_ops,
                                 V4L2_CID_TEST_PATTERN,
                                 ARRAY_SIZE(custom_imx219_test_pattern_menu) - 1,
                                 0, 0, custom_imx219_test_pattern_menu);
 
     if (ctrl_hdlr->error) {
         ret = ctrl_hdlr->error;
         dev_err(&client->dev, "%s control init failed (%d)\n", __func__, ret);
         v4l2_ctrl_handler_free(ctrl_hdlr);
         mutex_destroy(&imx219->mutex);
         return ret;
     }
 
     imx219->sd.ctrl_handler = ctrl_hdlr;
     return 0;
 }
 
 /**
  * @brief Free V4L2 controls and mutex.
  * @param[in] imx219 Pointer to custom_imx219 structure.
  */
 static void custom_imx219_free_controls(struct custom_imx219 *imx219)
 {
     v4l2_ctrl_handler_free(imx219->sd.ctrl_handler);
     mutex_destroy(&imx219->mutex);
 }
 
 /**
  * @brief Check hardware configuration from device tree.
  * @param[in] dev Pointer to device structure.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_check_hwcfg(struct device *dev)
 {
     struct fwnode_handle *endpoint;
     struct v4l2_fwnode_endpoint ep_cfg = { .bus_type = V4L2_MBUS_CSI2_DPHY };
     int ret = -EINVAL;
 
     endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
     if (!endpoint) {
         dev_err(dev, "endpoint node not found\n");
         return -EINVAL;
     }
 
     if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
         dev_err(dev, "could not parse endpoint\n");
         goto error_out;
     }
 
     if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
         dev_err(dev, "only 2 data lanes are currently supported\n");
         goto error_out;
     }
 
     if (!ep_cfg.nr_of_link_frequencies) {
         dev_err(dev, "link-frequency property not found in DT\n");
         goto error_out;
     }
 
     if (ep_cfg.nr_of_link_frequencies != 1 ||
         ep_cfg.link_frequencies[0] != CUSTOM_IMX219_DEFAULT_LINK_FREQ) {
         dev_err(dev, "Link frequency not supported: %lld\n",
                 ep_cfg.link_frequencies[0]);
         goto error_out;
     }
 
     ret = 0;
 
 error_out:
     v4l2_fwnode_endpoint_free(&ep_cfg);
     fwnode_handle_put(endpoint);
     return ret;
 }
 
 /**
  * @brief Probe function to initialize the IMX219 sensor driver.
  * @param[in] client Pointer to I2C client.
  * @return 0 on success, negative error code on failure.
  */
 static int custom_imx219_probe(struct i2c_client *client)
 {
     struct device *dev = &client->dev;
     struct custom_imx219 *imx219;
     struct fwnode_handle *fwnode;
     int ret;
 
     dev_info(dev, "Starting probe for custom_imx219 on bus %d, addr 0x%02x\n",
              client->adapter->nr, client->addr);
 
     imx219 = devm_kzalloc(dev, sizeof(*imx219), GFP_KERNEL);
     if (!imx219) {
         dev_err(dev, "Failed to allocate memory for imx219\n");
         return -ENOMEM;
     }
     dev_info(dev, "Memory allocated for imx219\n");
 
     v4l2_i2c_subdev_init(&imx219->sd, client, &custom_imx219_subdev_ops);
     dev_set_drvdata(dev, &imx219->sd);
 
     dev_info(dev, "Checking hardware configuration\n");
     if (custom_imx219_check_hwcfg(dev)) {
         dev_err(dev, "Hardware configuration check failed\n");
         return -EINVAL;
     }
 
     imx219->xclk = devm_clk_get(dev, NULL);
     if (IS_ERR(imx219->xclk)) {
         dev_err(dev, "Failed to get xclk: %ld\n", PTR_ERR(imx219->xclk));
         return PTR_ERR(imx219->xclk);
     }
     dev_info(dev, "Clock retrieved successfully\n");
 
     imx219->xclk_freq = clk_get_rate(imx219->xclk);
     if (imx219->xclk_freq != CUSTOM_IMX219_XCLK_FREQ) {
         dev_err(dev, "xclk frequency mismatch: got %d Hz, expected %d Hz\n",
                 imx219->xclk_freq, CUSTOM_IMX219_XCLK_FREQ);
         return -EINVAL;
     }
     dev_info(dev, "Clock frequency verified: %d Hz\n", imx219->xclk_freq);
 
     ret = custom_imx219_get_regulators(imx219);
     if (ret) {
         dev_err(dev, "Failed to get regulators: %d\n", ret);
         return ret;
     }
     dev_info(dev, "Regulators acquired\n");
 
     imx219->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
     if (IS_ERR(imx219->reset_gpio)) {
         dev_err(dev, "Failed to get reset GPIO: %ld\n", PTR_ERR(imx219->reset_gpio));
         return PTR_ERR(imx219->reset_gpio);
     }
     dev_info(dev, "Reset GPIO acquired\n");
 
     ret = custom_imx219_power_on(dev);
     if (ret) {
         dev_err(dev, "Failed to power on: %d\n", ret);
         return ret;
     }
     dev_info(dev, "Device powered on\n");
 
     ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_MODE_SELECT,
                                  CUSTOM_IMX219_REG_VALUE_08BIT,
                                  CUSTOM_IMX219_MODE_STREAMING);
     if (ret) {
         dev_err(dev, "Failed to start streaming for init: %d\n", ret);
         goto error_power_off;
     }
     usleep_range(100, 110);
 
     ret = custom_imx219_write_reg(imx219, CUSTOM_IMX219_REG_MODE_SELECT,
                                  CUSTOM_IMX219_REG_VALUE_08BIT,
                                  CUSTOM_IMX219_MODE_STANDBY);
     if (ret) {
         dev_err(dev, "Failed to stop streaming for init: %d\n", ret);
         goto error_power_off;
     }
     usleep_range(100, 110);
 
     ret = custom_imx219_identify_module(imx219);
     if (ret) {
         dev_err(dev, "Failed to identify module: %d\n", ret);
         goto error_power_off;
     }
     dev_info(dev, "Module identified successfully\n");
 
     imx219->mode = &custom_supported_modes[1]; /* Default to 1920x1080 */
     custom_imx219_set_default_format(imx219);
 
     ret = custom_imx219_init_controls(imx219);
     if (ret) {
         dev_err(dev, "Failed to init controls: %d\n", ret);
         goto error_power_off;
     }
     dev_info(dev, "Controls initialized\n");
 
     imx219->sd.internal_ops = &custom_imx219_internal_ops;
     imx219->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
     imx219->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
 
     imx219->pad.flags = MEDIA_PAD_FL_SOURCE;
     ret = media_entity_pads_init(&imx219->sd.entity, 1, &imx219->pad);
     if (ret) {
         dev_err(dev, "Failed to init media entity pads: %d\n", ret);
         goto error_handler_free;
     }
     dev_info(dev, "Media entity pads initialized\n");
 
     fwnode = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
     if (!fwnode) {
         dev_err(dev, "Failed to get endpoint fwnode\n");
         ret = -EINVAL;
         goto error_media_entity;
     }
     dev_info(dev, "Endpoint fwnode retrieved\n");
 
     v4l2_async_nf_init(&imx219->notifier);
     imx219->notifier.ops = &custom_imx219_async_ops;
 
     imx219->asd = v4l2_async_nf_add_fwnode(&imx219->notifier, fwnode,
                                            struct v4l2_async_subdev);
     if (IS_ERR(imx219->asd)) {
         ret = PTR_ERR(imx219->asd);
         if (ret == -EEXIST) {
             dev_warn(dev, "Async fwnode already registered, checking conflict\n");
             fwnode_handle_put(fwnode);
         } else {
             dev_err(dev, "Failed to add async fwnode: %d\n", ret);
             fwnode_handle_put(fwnode);
             goto error_media_entity;
         }
     } else {
         dev_info(dev, "Async subdev added to notifier\n");
     }
 
     snprintf(imx219->sd.name, sizeof(imx219->sd.name), "custom_imx219 %d-00%02x",
              client->adapter->nr, client->addr);
     dev_info(dev, "Registering subdev '%s'\n", imx219->sd.name);
 
     ret = v4l2_async_register_subdev_sensor(&imx219->sd);
     if (ret < 0) {
         dev_err(dev, "Failed to register sensor sub-device: %d\n", ret);
         goto error_notifier_cleanup;
     }
     dev_info(dev, "Sensor sub-device registered\n");
 
     pm_runtime_set_active(dev);
     pm_runtime_enable(dev);
     pm_runtime_idle(dev);
 
     dev_info(dev, "custom_imx219 probed successfully: %dx%d\n",
              imx219->fmt.width, imx219->fmt.height);
     return 0;
 
 error_notifier_cleanup:
     v4l2_async_nf_cleanup(&imx219->notifier);
 error_media_entity:
     media_entity_cleanup(&imx219->sd.entity);
 error_handler_free:
     custom_imx219_free_controls(imx219);
 error_power_off:
     custom_imx219_power_off(dev);
     return ret;
 }
 
/**
 * @brief Remove function to clean up the IMX219 sensor driver.
 * @param[in] client Pointer to the I2C client structure.
 *
 * This function is called when the I2C driver is removed or the device is
 * unbound. It unregisters the V4L2 subdevice, cleans up the async notifier,
 * releases media entity resources, frees V4L2 controls, and disables power
 * management. If the device is not already suspended, it powers off the sensor
 * and sets the power management state to suspended.
 */
static void custom_imx219_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct custom_imx219 *imx219 = to_custom_imx219(sd);

    v4l2_async_unregister_subdev(sd);
    v4l2_async_nf_cleanup(&imx219->notifier);
    media_entity_cleanup(&sd->entity);
    custom_imx219_free_controls(imx219);

    pm_runtime_disable(&client->dev);
    if (!pm_runtime_status_suspended(&client->dev))
        custom_imx219_power_off(&client->dev);
    pm_runtime_set_suspended(&client->dev);
}

/**
 * @brief Device tree compatible string table for the IMX219 sensor.
 *
 * This table defines the compatible string used to match the driver with the
 * device tree node for the Sony IMX219 sensor. It ensures the driver is probed
 * for devices with the "sony,imx219" compatible property.
 */
static const struct of_device_id custom_imx219_dt_ids[] = {
    { .compatible = "sony,imx219" },
    { }
};
MODULE_DEVICE_TABLE(of, custom_imx219_dt_ids);

/**
 * @brief Power management operations for the IMX219 sensor.
 *
 * This structure defines the power management operations for system sleep and
 * runtime power management. It uses forced suspend/resume for system sleep and
 * custom power on/off functions for runtime power management, ensuring proper
 * power control of the sensor.
 */
static const struct dev_pm_ops custom_imx219_pm_ops = {
    SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
    SET_RUNTIME_PM_OPS(custom_imx219_power_off, custom_imx219_power_on, NULL)
};

/**
 * @brief I2C driver structure for the IMX219 sensor.
 *
 * This structure defines the I2C driver for the IMX219 sensor, including the
 * driver name, device tree matching table, power management operations, and
 * probe/remove functions. It is used to register the driver with the I2C
 * subsystem.
 */
static struct i2c_driver custom_imx219_i2c_driver = {
    .driver = {
        .name = "imx219",
        .of_match_table = custom_imx219_dt_ids,
        .pm = &custom_imx219_pm_ops,
    },
    .probe_new = custom_imx219_probe,
    .remove = custom_imx219_remove,
};

/**
 * @brief Register the IMX219 I2C driver.
 *
 * This macro registers the I2C driver with the kernel, enabling automatic probing
 * and removal of the driver when matching I2C devices are detected or removed.
 */
module_i2c_driver(custom_imx219_i2c_driver);

/**
 * @brief Module author information.
 */
MODULE_AUTHOR("RAM");

/**
 * @brief Module description.
 */
MODULE_DESCRIPTION("Custom IMX219 sensor driver");

/**
 * @brief Module license information.
 */
MODULE_LICENSE("GPL v2");
