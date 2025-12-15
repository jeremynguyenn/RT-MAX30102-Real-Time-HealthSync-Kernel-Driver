// max30102_ioctl.c: This file handles ioctl operations for the MAX30102 sensor driver.
// It includes file opening, ioctl commands for reading FIFO, temperature, setting modes, etc.
// It is supplemented with additional validation and error handling for completeness.
// No changes to original code, only added comments and minor supplements for clarity where truncated.

#include <linux/uaccess.h>  // Include for copy_to_user and copy_from_user functions.
#include <linux/compat.h>   // Include for compatibility ioctl support.
#include <linux/jiffies.h>  // Include for timeout calculations using jiffies.
#include "max30102.h"       // Include the main header for MAX30102 definitions.

// Open function: Initializes the device file when opened by a user process.
static int max30102_open(struct inode *inode, struct file *file)  // Function signature for open operation.
{
    struct miscdevice *miscdev = file->private_data;  // Retrieve the miscdevice from file private data.
    struct max30102_data *data = container_of(miscdev, struct max30102_data, miscdev);  // Get the max30102_data struct from miscdevice.
    int ret;  // Variable to store return values.

    if (!data) return -EINVAL;  // Check if data is null, return invalid argument error.

    file->private_data = data;  // Set the file's private data to the sensor data struct.

    ret = init_waitqueue_head(&data->wait_data_ready);  // Initialize the wait queue for data ready events.
    if (ret < 0) {  // Check if initialization failed.
        dev_err(&data->client->dev, "Failed to init waitqueue: %d\n", ret);  // Log error message.
        return ret;  // Return the error code.
    }

    dev_info(&data->client->dev, "Device opened by process %d\n", current->pid);  // Log info about the opening process.

    return 0;  // Return success.
}

// IOCTL function: Handles various control commands from user space.
static long max30102_ioctl(struct file *file, unsigned int cmd, unsigned long arg)  // Function signature for ioctl.
{
    struct max30102_data *data = file->private_data;  // Retrieve sensor data from file private data.
    struct max30102_fifo_data fifo_data = {0};  // Initialize FIFO data structure to zero.
    struct max30102_slot_config slot_config = {0};  // Initialize slot config structure to zero.
    uint8_t mode = 0, config = 0;  // Initialize mode and config variables to zero.
    float temp = 0.0f;  // Initialize temperature variable to zero.
    int ret = 0;  // Initialize return value to zero.

    if (!data) return -EINVAL;  // Check if data is null, return invalid argument.

    lockdep_assert_held(&data->lock);  // Assert that the lock is held for debugging.

    // Try to acquire the mutex with a 1-second timeout.
    if (!rt_mutex_trylock_timeout(&data->fifo_mutex, msecs_to_jiffies(1000))) {  // Attempt to lock mutex with timeout.
        dev_err(&data->client->dev, "Mutex lock timeout\n");  // Log timeout error.
        return -ETIMEDOUT;  // Return timeout error.
    }

    // Validate user pointer for argument.
    if (!access_ok((void __user *)arg, sizeof(uint8_t))) return -EFAULT;  // Check if user space memory is accessible.

    switch (cmd) {  // Switch based on the ioctl command.
    case MAX30102_IOC_READ_FIFO:  // Case for reading FIFO data.
        ret = max30102_read_fifo(data, fifo_data.red, fifo_data.ir, &fifo_data.len);  // Call function to read FIFO.
        if (ret < 0) {  // Check if read failed.
            dev_err(&data->client->dev, "Failed to read FIFO: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock section.
        }
        if (copy_to_user((void __user *)arg, &fifo_data, sizeof(fifo_data))) {  // Copy data to user space.
            dev_err(&data->client->dev, "Failed to copy FIFO data to user\n");  // Log copy failure.
            ret = -EFAULT;  // Set fault error.
            goto unlock;  // Jump to unlock.
        }
        break;  // End of case.

    case MAX30102_IOC_READ_TEMP:  // Case for reading temperature.
        ret = max30102_read_temperature(data, &temp);  // Call function to read temperature.
        if (ret < 0) {  // Check if read failed.
            dev_err(&data->client->dev, "Failed to read temperature: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }
        if (copy_to_user((void __user *)arg, &temp, sizeof(temp))) {  // Copy temperature to user space.
            dev_err(&data->client->dev, "Failed to copy temperature to user\n");  // Log copy failure.
            ret = -EFAULT;  // Set fault error.
            goto unlock;  // Jump to unlock.
        }
        break;  // End of case.

    case MAX30102_IOC_SET_MODE:  // Case for setting mode.
        if (copy_from_user(&mode, (void __user *)arg, sizeof(mode))) {  // Copy mode from user space.
            dev_err(&data->client->dev, "Failed to copy mode from user\n");  // Log copy failure.
            ret = -EFAULT;  // Set fault error.
            goto unlock;  // Jump to unlock.
        }
        ret = max30102_set_mode(data, mode);  // Call function to set mode.
        if (ret < 0) {  // Check if set failed.
            dev_err(&data->client->dev, "Failed to set mode: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }
        break;  // End of case.

    case MAX30102_IOC_SET_SLOT:  // Case for setting slot configuration.
        if (copy_from_user(&slot_config, (void __user *)arg, sizeof(slot_config))) {  // Copy slot config from user.
            dev_err(&data->client->dev, "Failed to copy slot config from user\n");  // Log copy failure.
            ret = -EFAULT;  // Set fault error.
            goto unlock;  // Jump to unlock.
        }
        if (slot_config.slot < 1 || slot_config.slot > 4 || slot_config.led > 2) {  // Validate slot and led values.
            dev_err(&data->client->dev, "Invalid slot=%d or led=%d\n", slot_config.slot, slot_config.led);  // Log invalid values.
            ret = -EINVAL;  // Set invalid argument error.
            goto unlock;  // Jump to unlock.
        }
        ret = max30102_set_slot(data, slot_config.slot, slot_config.led);  // Call function to set slot.
        if (ret < 0) {  // Check if set failed.
            dev_err(&data->client->dev, "Failed to set slot: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }
        break;  // End of case.

    case MAX30102_IOC_SET_FIFO_CONFIG:  // Case for setting FIFO config.
        if (copy_from_user(&config, (void __user *)arg, sizeof(config))) {  // Copy config from user.
            dev_err(&data->client->dev, "Failed to copy FIFO config from user\n");  // Log copy failure.
            ret = -EFAULT;  // Set fault error.
            goto unlock;  // Jump to unlock.
        }
        ret = max30102_set_fifo_config(data, config);  // Call function to set FIFO config.
        if (ret < 0) {  // Check if set failed.
            dev_err(&data->client->dev, "Failed to set FIFO config: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }
        break;  // End of case.

    case MAX30102_IOC_SET_SPO2_CONFIG:  // Case for setting SpO2 config.
        if (copy_from_user(&config, (void __user *)arg, sizeof(config))) {  // Copy config from user.
            dev_err(&data->client->dev, "Failed to copy SpO2 config from user\n");  // Log copy failure.
            ret = -EFAULT;  // Set fault error.
            goto unlock;  // Jump to unlock.
        }
        ret = max30102_set_spo2_config(data, config);  // Call function to set SpO2 config.
        if (ret < 0) {  // Check if set failed.
            dev_err(&data->client->dev, "Failed to set SpO2 config: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }
        break;  // End of case.

    case MAX30102_IOC_SET_NETLINK:  // Case for setting netlink (demo).
        // Send a test message via netlink.
        struct sk_buff *skb = nlmsg_new(NLMSG_LENGTH(0), GFP_KERNEL);  // Allocate sk_buff for netlink message.
        if (!skb) {  // Check if allocation failed.
            ret = -ENOMEM;  // Set memory error.
            goto unlock;  // Jump to unlock.
        }
        nlmsg_put(skb, 0, 0, NLMSG_DONE, 0, 0);  // Put the netlink message header.
        NETLINK_CB(skb).dst_group = 1;  // Set destination group to 1.
        netlink_broadcast(data->nl_sock, skb, 0, 1, GFP_KERNEL);  // Broadcast the message.
        break;  // End of case.

    default:  // Default case for invalid commands.
        dev_err(&data->client->dev, "Invalid IOCTL command: 0x%x\n", cmd);  // Log invalid command.
        ret = -ENOTTY;  // Set not a tty error.
        goto unlock;  // Jump to unlock.
    }

unlock:  // Label for unlocking the mutex.
    rt_mutex_unlock(&data->fifo_mutex);  // Unlock the FIFO mutex.
    return ret;  // Return the result code.
}

// Compat IOCTL: Handles 32-bit compatibility ioctl calls.
static long max30102_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)  // Function signature for compat ioctl.
{
    return max30102_ioctl(file, cmd, arg);  // Call the regular ioctl function.
}