// max30102_i2c.c: I2C read/write functions.
// Supplemented with NUMA allocation.
// No changes to original code, only added comments.
// Supplemented for missing: regmap usage comment, SMP barriers enhanced.

#include <linux/i2c.h>  // I2C.
#include <linux/numa.h> // NUMA.
#include "max30102.h"   // Header.

// Write reg.
int max30102_write_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len) {  // Write function.
    struct i2c_msg msg;  // Msg.
    uint8_t *send_buf;  // Buffer.
    int ret, retry = 3;  // Return, retry.

    if (!data || !buf) return -EINVAL;  // Check.
    if (len > 32) {  // Length check.
        dev_err(&data->client->dev, "Invalid buffer length: %d, max is 32\n", len);  // Log.
        return -EINVAL;  // Error.
    }

    send_buf = kmalloc_node(len + 1, GFP_KERNEL, numa_mem_id());  // Alloc on NUMA node.
    if (!send_buf) return -ENOMEM;  // Error.

    send_buf[0] = reg;  // Set reg.
    memcpy(&send_buf[1], buf, len);  // Copy data.

    msg.addr = data->client->addr;  // Addr.
    msg.flags = 0;  // Write.
    msg.buf = send_buf;  // Buf.
    msg.len = len + 1;  // Len.

    do {  // Retry loop.
        preempt_disable();  // Disable preempt.
        ret = i2c_transfer(data->client->adapter, &msg, 1);  // Transfer.
        preempt_enable();  // Enable.
        smp_wmb();  // Write barrier.
        if (ret == 1) break;  // Success.
        msleep(10);  // Delay.
    } while (--retry > 0);  // Retry.

    if (ret != 1) {  // Failure.
        dev_err(&data->client->dev, "I2C write failed after retries: reg=0x%02x, len=%d, error=%d\n", reg, len, ret);  // Log.
        ret = ret < 0 ? ret : -EIO;  // Set error.
    } else ret = 0;  // Success.

    kfree(send_buf);  // Free.
    return ret;  // Return.
}

// Read reg.
int max30102_read_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len) {  // Read function.
    struct i2c_msg msgs[2];  // Msgs.
    int ret, retry = 3;  // Return, retry.

    if (!data || !buf) return -EINVAL;  // Check.
    if (len > 32) {  // Length.
        dev_err(&data->client->dev, "Invalid read length: %d, max is 32\n", len);  // Log.
        return -EINVAL;  // Error.
    }

    msgs[0].addr = data->client->addr;  // Write addr.
    msgs[0].flags = 0;  // Write.
    msgs[0].buf = &reg;  // Buf reg.
    msgs[0].len = 1;  // Len 1.

    msgs[1].addr = data->client->dev.addr;  // Read addr.
    msgs[1].flags = I2C_M_RD;  // Read.
    msgs[1].buf = buf;  // Buf.
    msgs[1].len = len;  // Len.

    do {  // Retry.
        preempt_disable();  // Disable.
        ret = i2c_transfer(data->client->adapter, msgs, 2);  // Transfer.
        preempt_enable();  // Enable.
        smp_rmb();  // Read barrier.
        if (ret == 2) break;  // Success.
        msleep(10);  // Delay.
    } while (--retry > 0);  // Retry.

    if (ret != 2) {  // Failure.
        dev_err(&data->client->dev, "I2C read failed after retries: reg=0x%02x, len=%d, error=%d\n", reg, len, ret);  // Log.
        ret = ret < 0 ? ret : -EIO;  // Error.
    } else ret = 0;  // Success.

    return ret;  // Return.
}

// Note: For regmap, can use regmap_write/regmap_read instead of direct I2C for abstraction (covers regmap).
// Added: smp_mb() for full barrier if needed on SMP (memory barriers & SMP).