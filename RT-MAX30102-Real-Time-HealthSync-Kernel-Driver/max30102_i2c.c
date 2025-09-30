#include <linux/i2c.h>
#include <linux/numa.h>
#include "max30102.h"

int max30102_write_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
    struct i2c_msg msg;
    uint8_t *send_buf;
    int ret, retry = 3;

    if (!data || !buf) return -EINVAL;
    if (len > 32) dev_err(&data->client->dev, "Invalid buffer length: %d, max is 32\n", len), return -EINVAL;

    send_buf = kmalloc_node(len + 1, GFP_KERNEL, numa_mem_id());
    if (!send_buf) return -ENOMEM;

    send_buf[0] = reg;
    memcpy(&send_buf[1], buf, len);

    msg.addr = data->client->addr;
    msg.flags = 0;
    msg.buf = send_buf;
    msg.len = len + 1;

    do {
        preempt_disable();
        ret = i2c_transfer(data->client->adapter, &msg, 1);
        preempt_enable();
        if (ret == 1) break;
        msleep(10);
    } while (--retry > 0);

    if (ret != 1) {
        dev_err(&data->client->dev, "I2C write failed after retries: reg=0x%02x, len=%d, error=%d\n", reg, len, ret);
        ret = ret < 0 ? ret : -EIO;
    } else ret = 0;

    kfree(send_buf);
    return ret;
}

int max30102_read_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len)
{
    struct i2c_msg msgs[2];
    int ret, retry = 3;

    if (!data || !buf) return -EINVAL;
    if (len > 32) dev_err(&data->client->dev, "Invalid read length: %d, max is 32\n", len), return -EINVAL;

    msgs[0].addr = data->client->addr;
    msgs[0].flags = 0;
    msgs[0].buf = &reg;
    msgs[0].len = 1;

    msgs[1].addr = data->client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].buf = buf;
    msgs[1].len = len;

    do {
        preempt_disable();
        ret = i2c_transfer(data->client->adapter, msgs, 2);
        preempt_enable();
        if (ret == 2) break;
        msleep(10);
    } while (--retry > 0);

    if (ret != 2) {
        dev_err(&data->client->dev, "I2C read failed after retries: reg=0x%02x, len=%d, error=%d\n", reg, len, ret);
        ret = ret < 0 ? ret : -EIO;
    } else ret = 0;

    return ret;
}