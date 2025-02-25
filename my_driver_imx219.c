#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#define IMX219_I2C_ADDR 0x10
#define IMX219_REG_MODE_SELECT 0x0100
#define IMX219_MODE_STREAMING 0x01
#define IMX219_MODE_STANDBY 0x00
#define IMX219_REG_CHIP_ID 0x0000
#define IMX219_CHIP_ID 0x0219

struct imx219 {
    struct v4l2_subdev sd;
    struct media_pad pad;
    struct v4l2_ctrl_handler ctrl_handler;
    struct vb2_queue queue;
    struct mutex lock;
    struct clk *xclk;
    struct regulator_bulk_data supplies[3];
    struct gpio_desc *reset_gpio;
    struct i2c_client *client;
    struct v4l2_mbus_framefmt format;
};

/* Read from I2C register */
static int imx219_read_reg(struct i2c_client *client, u16 reg, u8 *val) {
    u8 data[2] = {reg >> 8, reg & 0xFF};
    struct i2c_msg msgs[2] = {
        { client->addr, 0, 2, data },
        { client->addr, I2C_M_RD, 1, val }
    };
    return i2c_transfer(client->adapter, msgs, 2);
}

/* Write to I2C register */
static int imx219_write_reg(struct i2c_client *client, u16 reg, u8 val) {
    u8 data[3] = {reg >> 8, reg & 0xFF, val};
    struct i2c_msg msg = { client->addr, 0, 3, data };
    return i2c_transfer(client->adapter, &msg, 1);
}

/* Set the camera to streaming mode */
static int imx219_start_streaming(struct imx219 *sensor) {
    return imx219_write_reg(sensor->client, IMX219_REG_MODE_SELECT, IMX219_MODE_STREAMING);
}

/* Stop the camera streaming */
static void imx219_stop_streaming(struct imx219 *sensor) {
    imx219_write_reg(sensor->client, IMX219_REG_MODE_SELECT, IMX219_MODE_STANDBY);
}

/* Set format */
static int imx219_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
                             struct v4l2_subdev_format *fmt) {
    struct imx219 *sensor = container_of(sd, struct imx219, sd);
    sensor->format = fmt->format;
    return 0;
}

/* Probe function */
static int imx219_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct imx219 *sensor;
    u8 chip_id;

    sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    sensor->client = client;
    v4l2_i2c_subdev_init(&sensor->sd, client, NULL);
    mutex_init(&sensor->lock);

    imx219_read_reg(client, IMX219_REG_CHIP_ID, &chip_id);
    if (chip_id != IMX219_CHIP_ID) {
        dev_err(&client->dev, "IMX219 chip ID mismatch\n");
        return -ENODEV;
    }

    return 0;
}

/* Remove function */
static int imx219_remove(struct i2c_client *client) {
    return 0;
}

/* I2C Driver */
static struct i2c_driver imx219_driver = {
    .driver = {
        .name = "imx219",
    },
    .probe = imx219_probe,
    .remove = imx219_remove,
};

module_i2c_driver(imx219_driver);
MODULE_LICENSE("GPL");
