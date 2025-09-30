#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/jiffies.h>
#include "max30102.h"

static int max30102_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev = file->private_data;
    struct max30102_data *data = container_of(miscdev, struct max30102_data, miscdev);
    int ret;

    if (!data) return -EINVAL;
    file->private_data = data;
    ret = init_waitqueue_head(&data->wait_data_ready);
    if (ret < 0) dev_err(&data->client->dev, "Failed to init waitqueue: %d\n", ret), return ret;
    dev_info(&data->client->dev, "Device opened by process %d\n", current->pid);
    return 0;
}

static long max30102_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct max30102_data *data = file->private_data;
    struct max30102_fifo_data fifo_data = {0};
    struct max30102_slot_config slot_config = {0};
    uint8_t mode = 0, config = 0;
    float temp = 0.0f;
    int ret = 0;

    if (!data) return -EINVAL;

    lockdep_assert_held(&data->lock); // Enforce lock hierarchy

    // Timeout-based lock breaking (1s)
    if (!rt_mutex_trylock_timeout(&data->fifo_mutex, msecs_to_jiffies(1000))) {
        dev_err(&data->client->dev, "Mutex lock timeout\n");
        return -ETIMEDOUT;
    }

    switch (cmd) {
    case MAX30102_IOC_READ_FIFO:
        ret = max30102_read_fifo(data, fifo_data.red, fifo_data.ir, &fifo_data.len);
        if (ret < 0) dev_err(&data->client->dev, "Failed to read FIFO: %d\n", ret), goto unlock;
        if (copy_to_user((void __user *)arg, &fifo_data, sizeof(fifo_data))) {
            dev_err(&data->client->dev, "Failed to copy FIFO data to user\n");
            ret = -EFAULT;
            goto unlock;
        }
        break;

    case MAX30102_IOC_READ_TEMP:
        ret = max30102_read_temperature(data, &temp);
        if (ret < 0) dev_err(&data->client->dev, "Failed to read temperature: %d\n", ret), goto unlock;
        if (copy_to_user((void __user *)arg, &temp, sizeof(temp))) {
            dev_err(&data->client->dev, "Failed to copy temperature to user\n");
            ret = -EFAULT;
            goto unlock;
        }
        break;

    case MAX30102_IOC_SET_MODE:
        if (copy_from_user(&mode, (void __user *)arg, sizeof(mode))) {
            dev_err(&data->client->dev, "Failed to copy mode from user\n");
            ret = -EFAULT;
            goto unlock;
        }
        ret = max30102_set_mode(data, mode);
        if (ret < 0) dev_err(&data->client->dev, "Failed to set mode: %d\n", ret), goto unlock;
        break;

    case MAX30102_IOC_SET_SLOT:
        if (copy_from_user(&slot_config, (void __user *)arg, sizeof(slot_config))) {
            dev_err(&data->client->dev, "Failed to copy slot config from user\n");
            ret = -EFAULT;
            goto unlock;
        }
        if (slot_config.slot < 1 || slot_config.slot > 4 || slot_config.led > 2) {
            dev_err(&data->client->dev, "Invalid slot=%d or led=%d\n", slot_config.slot, slot_config.led);
            ret = -EINVAL;
            goto unlock;
        }
        ret = max30102_set_slot(data, slot_config.slot, slot_config.led);
        if (ret < 0) dev_err(&data->client->dev, "Failed to set slot: %d\n", ret), goto unlock;
        break;

    case MAX30102_IOC_SET_FIFO_CONFIG:
        if (copy_from_user(&config, (void __user *)arg, sizeof(config))) {
            dev_err(&data->client->dev, "Failed to copy FIFO config from user\n");
            ret = -EFAULT;
            goto unlock;
        }
        ret = max30102_set_fifo_config(data, config);
        if (ret < 0) dev_err(&data->client->dev, "Failed to set FIFO config: %d\n", ret), goto unlock;
        break;

    case MAX30102_IOC_SET_SPO2_CONFIG:
        if (copy_from_user(&config, (void __user *)arg, sizeof(config))) {
            dev_err(&data->client->dev, "Failed to copy SpO2 config from user\n");
            ret = -EFAULT;
            goto unlock;
        }
        ret = max30102_set_spo2_config(data, config);
        if (ret < 0) dev_err(&data->client->dev, "Failed to set SpO2 config: %d\n", ret), goto unlock;
        break;

    default:
        dev_err(&data->client->dev, "Invalid IOCTL command: 0x%x\n", cmd);
        ret = -ENOTTY;
        goto unlock;
    }

unlock:
    rt_mutex_unlock(&data->fifo_mutex);
    return ret;
}

static long max30102_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    return max30102_ioctl(file, cmd, arg);
}