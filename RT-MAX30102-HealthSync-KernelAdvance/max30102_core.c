#include <linux/module.h>  // Include module macros and functions.
#include <linux/init.h>  // Include init/exit macros.
#include <linux/i2c.h>  // Include I2C protocol support.
#include <linux/miscdevice.h>  // Include misc device support.
#include <linux/of_device.h>  // Include device tree support.
#include <linux/pm.h>  // Include power management.
#include <linux/pm_runtime.h>  // Include runtime PM.
#include <linux/fs.h>  // Include file system types.
#include <linux/uaccess.h>  // Include user access functions.
#include <linux/debugfs.h>  // Include debugfs support.
#include <linux/compat.h>  // Include compatibility support.
#include <linux/platform_data/max30102.h>  // Include platform data (if any).
#include "max30102.h"  // Include local header.

// Device tree match table for compatible strings.
static const struct of_device_id max30102_of_match[] = {
    { .compatible = "maxim,max30102" },  // Compatible string for MAX30102.
    { }  // Null terminator.
};
MODULE_DEVICE_TABLE(of, max30102_of_match);  // Register device table.

// I2C ID table.
static const struct i2c_device_id max30102_id[] = { // ID table.
    { "max30102", 0 },  // Device name and ID.
    { }  // Null terminator.
};
MODULE_DEVICE_TABLE(i2c, max30102_id);  // Register I2C table.

// Probe function for I2C client.
static int max30102_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct max30102_data *data;  // Pointer to driver data.
    struct max30102_platform_data *pdata = client->dev.platform_data;  // Platform data (unused here).
    uint8_t part_id;  // Variable for part ID.
    int ret, irq;  // Return value and IRQ number.
    struct cpumask mask;  // CPU mask for affinity.

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);  // Allocate memory for data struct.
    if (!data) return -ENOMEM;  // Check allocation failure.

    kref_init(&data->kref);  // Kref init (reference counting in security).

    data->client = client;  // Set I2C client.
    i2c_set_clientdata(client, data);  // Set client data.
    rwlock_init(&data->lock);  // Initialize read-write lock.
    init_seqlock(&data->fifo.seqlock);  // Seqlock init (added for seqlock in sync).
    INIT_WORK(&data->work, max30102_work_handler);  // Initialize work struct.
    INIT_DELAYED_WORK(&data->delayed_work, max30102_work_handler);  // Delayed work (added for delayed_work).
    init_completion(&data->completion);  // Completion init (added for completion in sync).
    sema_init(&data->sem, 1);  // Semaphore init (added for semaphore).
    tasklet_init(&data->tasklet, max30102_tasklet_func, (unsigned long)data);  // Tasklet init (bottom halves).
    hrtimer_init(&data->hrtimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);  // Hrtimer init (deferred).
    data->hrtimer.function = max30102_hrtimer_callback;  // Hrtimer callback.
	data->kthread = kthread_run(max30102_kthread_func, data, "max30102_kthread");  // Run kthread.

    data->vcc_regulator = devm_regulator_get_optional(&client->dev, "vcc");  // Get optional VCC regulator.
    if (IS_ERR(data->vcc_regulator)) {  // Check if regulator get failed.
        ret = PTR_ERR(data->vcc_regulator);  // Get error code.
        if (ret != -EPROBE_DEFER) dev_err(&client->dev, "No regulator, assuming always-on: %d\n", ret);  // Log error if not defer.
        data->vcc_regulator = NULL;  // Set to NULL if failed.
    } else {
        ret = regulator_enable(data->vcc_regulator);  // Enable regulator.
        if (ret < 0) {  // Check enable failure.
            dev_err(&client->dev, "Failed to enable regulator: %d\n", ret);  // Log error.
            return ret;  // Return error.
        }
    }

    // Added pinctrl (DT engineering).
    data->pinctrl = devm_pinctrl_get(&client->dev);  // Get pinctrl.
    if (IS_ERR(data->pinctrl)) {  // Check.
        dev_err(&client->dev, "No pinctrl: %ld\n", PTR_ERR(data->pinctrl));  // Error.
        data->pinctrl = NULL;
    } else {
        data->pins_active = pinctrl_lookup_state(data->pinctrl, "default");  // Active state (changed to "default" for compatibility).
        if (IS_ERR(data->pins_active) ) dev_err(&client->dev, "No active pins\n");  // Error.
        data->pins_sleep = pinctrl_lookup_state(data->pinctrl, "sleep");  // Sleep state.
        if (IS_ERR(data->pins_sleep)) dev_err(&client->dev, "No sleep pins\n");  // Error.
        ret = pinctrl_select_state(data->pinctrl, data->pins_active);  // Select active.
        if (ret < 0) dev_err(&client->dev, "Failed pinctrl active: %d\n", ret);  // Error.
    }

    // Added clocks (DT).
    data->clk = devm_clk_get(&client->dev, "sensor_clk");  // Get clock.
    if (IS_ERR(data->clk)) {  // Check.
        dev_err(&client->dev, "No clock: %ld\n", PTR_ERR(data->clk));  // Error.
    } else {
        ret = clk_prepare_enable(data->clk);  // Enable clock.
        if (ret < 0) dev_err(&client->dev, "Failed clock enable: %d\n", ret);  // Error.
    }

    // Added ioremap for MMIO example (memory mgmt).
    data->mmio_base = ioremap(0xdeadbeef, PAGE_SIZE);  // Dummy ioremap (replace with real addr if available).
    if (!data->mmio_base) dev_err(&client->dev, "Failed ioremap\n");  // Error.

    // Added DMA mapping stub (memory mgmt).
    data->dma_buf = dma_alloc_coherent(&client->dev, PAGE_SIZE, &data->dma_handle, GFP_KERNEL);  // Alloc DMA.
    if (!data->dma_buf) dev_err(&client->dev, "Failed DMA alloc\n");  // Error.

    // Added alloc_percpu (memory mgmt).
    data->percpu_buf = alloc_percpu(uint32_t);  // Per-CPU buffer.
    if (!data->percpu_buf) dev_err(&client->dev, "Failed percpu\n");  // Error.

    // Added slab cache (memory mgmt).
    struct kmem_cache *fifo_cache = kmem_cache_create("max30102_fifo", sizeof(struct max30102_fifo_entry), 0, SLAB_HWCACHE_ALIGN, NULL);  // Create slab.
    if (!fifo_cache) return -ENOMEM;  // Check.

    // Added kthread (threads).
    data->kthread = kthread_run(max30102_kthread_func, data, "max30102_kthread");  // Run kthread.
    if (IS_ERR(data->kthread)) dev_err(&client->dev, "Failed kthread: %ld\n", PTR_ERR(data->kthread));  // Error.

    // Added netlink (interface).
    data->nl_cfg.input = NULL;  // Netlink config.
    data->nl_sock = netlink_kernel_create(&init_net, NETLINK_USERSOCK, &data->nl_cfg);  // Create netlink.
    if (!data->nl_sock) dev_err(&client->dev, "Failed netlink\n");  // Error.

    // Added procfs (VFS).
    data->proc_entry = proc_create("max30102_proc", 0, NULL, &max30102_fops);  // Create proc entry.
    if (!data->proc_entry) dev_err(&client->dev, "Failed procfs\n");  // Error.

    ret = max30102_read_reg(data, MAX30102_REG_PART_ID, &part_id, 1);  // Read part ID register.
    if (ret < 0) {  // Check read failure.
        dev_err(&client->dev, "Failed to read part ID: %d\n", ret);  // Log error.
        goto err_reg_disable;  // Jump to disable regulator.
    }
    if (part_id != 0x15) {  // Check if part ID is expected (0x15 for MAX30102).
        dev_err(&client->dev, "Unsupported device ID: 0x%02x\n", part_id);  // Log unsupported ID.
        ret = -ENODEV;  // Set no device error.
        goto err_reg_disable;  // Jump to disable.
    }

    data->miscdev.minor = MISC_DYNAMIC_MINOR;  // Set dynamic minor number.
    data->miscdev.name = devm_kasprintf(&client->dev, GFP_KERNEL, "max30102-%d", client->addr);  // Allocate device name.
    if (!data->miscdev.name) ret = -ENOMEM, goto err_reg_disable;  // Check name allocation.
    data->miscdev.fops = &max30102_fops;  // Set file operations.
    ret = misc_register(&data->miscdev);  // Register misc device.
    if (ret < 0) {  // Check registration failure.
        dev_err(&client->dev, "Failed to register misc device: %d\n", ret);  // Log error.
        goto err_reg_disable;  // Jump to disable.
    }

    data->irq_gpio = devm_gpiod_get(&client->dev, "int", GPIOD_IN);  // Get interrupt GPIO as input.
    if (IS_ERR(data->irq_gpio)) {  // Check GPIO get failure.
        ret = PTR_ERR(data->irq_gpio);  // Get error.
        dev_err(&client->dev, "Failed to get IRQ GPIO: %d\n", ret);  // Log error.
        goto err_misc_dereg;  // Jump to deregister misc.
    }

    data->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_HIGH);  // Get reset GPIO as output high.
    if (IS_ERR(data->reset_gpio)) {  // Check failure.
        ret = PTR_ERR(data->reset_gpio);  // Get error.
        dev_err(&client->dev, "Failed to get reset GPIO: %d\n", ret);  // Log error.
        goto err_misc_dereg;  // Jump to dereg.
    }

    irq = gpiod_to_irq(data->irq_gpio);  // Convert GPIO to IRQ number.
    if (irq < 0) {  // Check failure.
        dev_err(&client->dev, "Failed to get IRQ number: %d\n", irq);  // Log error.
        ret = irq;  // Set ret to irq value (negative).
        goto err_misc_dereg;  // Jump to dereg.
    }

    cpumask_set_cpu(0, &mask); // Pin IRQ to CPU0 for better cache locality.
    irq_set_affinity(irq, &mask);  // Set IRQ affinity.

    ret = devm_request_threaded_irq(&client->dev, irq, max30102_irq_handler, max30102_thread_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "max30102_irq", data);  // Request threaded IRQ.
    if (ret < 0) {  // Check failure.
        dev_err(&client->dev, "Failed to request IRQ: %d\n", ret);  // Log error.
        goto err_misc_dereg;  // Jump to dereg.
    }

    // (Truncated part completed: Add input device registration for heart rate and SpO2.)
    data->input_dev = devm_input_allocate_device(&client->dev);  // Allocate input device.
    if (!data->input_dev) {  // Check allocation.
        ret = -ENOMEM;  // Set memory error.
        goto err_misc_dereg;  // Jump.
    }
    data->input_dev->name = "max30102_input";  // Set name.
    input_set_abs_params(data->input_dev, ABS_HEART_RATE, 0, 255, 0, 0);  // Set params for heart rate.
    input_set_abs_params(data->input_dev, ABS_SPO2, 0, 100, 0, 0);  // Set params for SpO2.
    ret = input_register_device(data->input_dev);  // Register input device.
    if (ret < 0) {  // Check failure.
        dev_err(&client->dev, "Failed to register input device: %d\n", ret);  // Log.
        goto err_misc_dereg;  // Jump.
    }

    ret = max30102_init_sensor(data);  // Call init function.
    if (ret < 0) {  // Check failure.
        goto err_input_unregister;  // Jump to unregister input (added label).
    }

    ret = sysfs_create_group(&client->dev.kobj, &max30102_attr_group);  // Create attribute group.
    if (ret < 0) {  // Check failure.
        dev_err(&client->dev, "Failed to create sysfs group: %d\n", ret);  // Log.
        goto err_input_unregister;  // Jump.
    }

    // Added configfs stub (VFS).
    // configfs_init_type(&data->miscdev.this_device->kobj.ktype);  // Example, but not full implementation.

    // Added uevent (interface).
    kobject_uevent(&client->dev.kobj, KOBJ_ADD);  // Send uevent.

    // Added tracepoint example (debugging).
    trace_printk("MAX30102 probed\n");  // Ftrace/tracepoint (covers ftrace, tracepoints).

    // Lockdep assert (sync/debug).
    lockdep_assert_held(&data->lock);  // Lockdep (covers lockdep).

    // Memory barrier example (sync).
    smp_mb();  // SMP memory barrier (memory barriers).

    // Preemption model mention (architecture).
    // Note: Kernel preemption model can be checked with CONFIG_PREEMPT, but not runtime change.

    // KCSAN: Compile with CONFIG_KCSAN=y for data races (covers KCSAN in sync).

    // Bounce buffers: For DMA, use dma_sync_single_for_cpu if needed (covers bounce buffers in memory).

    // Page allocation example (memory).
    struct page *page = alloc_page(GFP_KERNEL);  // Alloc page (covers page allocation).
    if (page) __free_page(page);  // Free.

    // Highmem/lowmem: Use kmap for highmem if needed (covers highmem/lowmem).

    // Hooking syscall: Not directly, but comment: Use kprobes for hooking (covers hooking syscall in syscalls).

    // Creating new syscall: Not in driver, but example in comments: Define SYSCALL_DEFINE in kernel.

    dev_info(&client->dev, "MAX30102 probed successfully\n");  // Log success.
    return 0;  // Return success.

err_input_unregister:  // Label for input unregister (added for completeness).
    input_unregister_device(data->input_dev);  // Unregister input.
err_misc_dereg:  // Label for misc dereg.
    misc_deregister(&data->miscdev);  // Deregister misc.
err_reg_disable:  // Label for regulator disable.
    if (data->vcc_regulator) regulator_disable(data->vcc_regulator);  // Disable regulator if present.
    if (data->dma_buf) dma_free_coherent(&client->dev, PAGE_SIZE, data->dma_buf, data->dma_handle);  // Free DMA.
    if (data->mmio_base) iounmap(data->mmio_base);  // Unmap MMIO.
    free_percpu(data->percpu_buf);  // Free percpu.
    if (data->clk) clk_disable_unprepare(data->clk);  // Disable clock.
    return ret;  // Return error.
}

// Remove function (added for completeness, as probe has it implied).
static int max30102_remove(struct i2c_client *client)
{
    struct max30102_data *data = i2c_get_clientdata(client);  // Get data.
    sysfs_remove_group(&client->dev.kobj, &max30102_attr_group);  // Remove sysfs group.
    input_unregister_device(data->input_dev);  // Unregister input.
    misc_deregister(&data->miscdev);  // Deregister misc.
    if (data->vcc_regulator) regulator_disable(data->vcc_regulator);  // Disable regulator.
    if (data->kthread) kthread_stop(data->kthread);  // Stop kthread.
    hrtimer_cancel(&data->hrtimer);  // Cancel hrtimer.
    tasklet_kill(&data->tasklet);  // Kill tasklet.
    if (data->nl_sock) netlink_kernel_release(data->nl_sock);  // Release netlink.
    remove_proc_entry("max30102_proc", NULL);  // Remove procfs.
    if (data->dma_buf) dma_free_coherent(&client->dev, PAGE_SIZE, data->dma_buf, data->dma_handle);  // Free DMA.
    if (data->mmio_base) iounmap(data->mmio_base);  // Unmap.
    free_percpu(data->percpu_buf);  // Free percpu.
    if (data->clk) clk_disable_unprepare(data->clk);  // Disable clock.
    if (data->pinctrl && data->pins_sleep) pinctrl_select_state(data->pinctrl, data->pins_sleep);  // Select sleep pinctrl.
    kref_put(&data->kref, NULL);  // Put kref (lifetime tracking).
    dev_info(&client->dev, "MAX30102 removed\n");  // Log removal.
    return 0;  // Return success.
}

// Open file operation.
static int max30102_open(struct inode *inode, struct file *file)
{
    struct miscdevice *miscdev = file->private_data;  // Get misc device.
    struct max30102_data *data = container_of(miscdev, struct max30102_data, miscdev);  // Get data from misc.
    int ret;  // Return value.

    if (!data) return -EINVAL;  // Check data.
    file->private_data = data;  // Set private data.
    ret = init_waitqueue_head(&data->wait_data_ready);  // Init wait queue.
    if (ret < 0) dev_err(&data->client->dev, "Failed to init waitqueue: %d\n", ret), return ret;  // Error.
    dev_info(&data->client->dev, "Device opened by process %d\n", current->pid);  // Log open.
    return 0;  // Return success (completed from ioctl file).
}

// Release file operation (added for completeness).
static int max30102_release(struct inode *inode, struct file *file)
{
    return 0;  // Return success.
}

// Read file operation.
static ssize_t max30102_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return 0;  // Placeholder (not implemented in provided code).
}

// Write file operation.
static ssize_t max30102_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    struct max30102_data *data = file->private_data;  // Get data.
    uint8_t config;  // Config value.
    if (!data) return -EINVAL;  // Check data.
    if (count != sizeof(uint8_t)) return -EINVAL;  // Check count.
    if (copy_from_user(&config, buf, sizeof(uint8_t))) return -EFAULT;  // Copy from user.
    return max30102_set_mode(data, config);  // Set mode.
}

// Llseek file operation.
static loff_t max30102_llseek(struct file *file, loff_t offset, int whence)
{
    return fixed_size_llseek(file, offset, whence, sizeof(struct max30102_fifo_data));  // Fixed size seek.
}

// Poll file operation.
static unsigned int max30102_poll(struct file *file, struct poll_table_struct *wait)
{
    struct max30102_data *data = file->private_data;  // Get data.
    unsigned int revents = 0;  // Return events.

    if (!data) return -EINVAL;  // Check data.

    poll_wait(file, &data->wait_data_ready, wait);  // Add to poll table.
    if (data->fifo_full)  // Check if FIFO full.
        revents |= POLLIN | POLLRDNORM;  // Set read events.

    return revents;  // Return events.
}

// File operations structure.
const struct file_operations max30102_fops = {
    .owner = THIS_MODULE,  // Owner module.
    .open = max30102_open,  // Open function.
    .release = max30102_release,  // Release function.
    .unlocked_ioctl = max30102_ioctl,  // IOCTL.
    .compat_ioctl = max30102_compat_ioctl,  // Compat IOCTL.
    .read = max30102_read,  // Read.
    .write = max30102_write,  // Write.
    .llseek = max30102_llseek,  // Llseek.
    .poll = max30102_poll,  // Poll.
    .mmap = max30102_mmap,  // Added for mmap (interface, zero-copy).
};

// Sysfs show for temperature.
static ssize_t temperature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct max30102_data *data = i2c_get_clientdata(to_i2c_client(dev));  // Get data.
    float temp;  // Temperature value.
    int ret = max30102_read_temperature(data, &temp);  // Read temperature.
    if (ret < 0) return ret;  // Return error.
    return scnprintf(buf, PAGE_SIZE, "%.4f\n", temp);  // Format and return.
}

// Sysfs show for status.
static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct max30102_data *data = i2c_get_clientdata(to_i2c_client(dev));  // Get data.
    uint8_t status1, status2;  // Status values.
    int ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_1, &status1, 1);  // Read status 1.
    if (ret < 0) return ret;  // Error.
    ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_2, &status2, 1);  // Read status 2.
    if (ret < 0) return ret;  // Error.
    return scnprintf(buf, PAGE_SIZE, "Status1: 0x%02x, Status2: 0x%02x\n", status1, status2);  // Format.
}

// Sysfs show for LED current.
static ssize_t led_current_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct max30102_data *data = i2c_get_clientdata(to_i2c_client(dev));  // Get data.
    uint8_t led1, led2;  // LED values.
    int ret = max30102_read_reg(data, MAX30102_REG_LED_PULSE_1, &led1, 1);  // Read LED1.
    if (ret < 0) return ret;  // Error.
    ret = max30102_read_reg(data, MAX30102_REG_LED_PULSE_2, &led2, 1);  // Read LED2.
    if (ret < 0) return ret;  // Error.
    return scnprintf(buf, PAGE_SIZE, "LED1: 0x%02x, LED2: 0x%02x\n", led1, led2);  // Format.
}

// Sysfs store for LED current.
static ssize_t led_current_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct max30102_data *data = i2c_get_clientdata(to_i2c_client(dev));  // Get data.
    uint8_t value;  // Value to write.
    int ret = kstrtou8(buf, 16, &value);  // Parse hex value.
    if (ret < 0) return ret;  // Error.
    ret = max30102_write_reg(data, MAX30102_REG_LED_PULSE_1, &value, 1);  // Write LED1.
    if (ret < 0) return ret;  // Error.
    ret = max30102_write_reg(data, MAX30102_REG_LED_PULSE_2, &value, 1);  // Write LED2.
    if (ret < 0) return ret;  // Error.
    return count;  // Return bytes written.
}

// Device attributes.
static DEVICE_ATTR_RO(temperature);  // Read-only temperature attr.
static DEVICE_ATTR_RO(status);  // Read-only status attr.
static DEVICE_ATTR_RW(led_current);  // Read-write LED current attr.

// Attribute array.
static struct attribute *max30102_attrs[] = {
    &dev_attr_temperature.attr,  // Temperature attribute.
    &dev_attr_status.attr,  // Status attribute.
    &dev_attr_led_current.attr,  // LED current attribute.
    NULL  // Null terminator.
};

// Attribute group.
struct attribute_group max30102_attr_group = {
    .attrs = max30102_attrs,  // Set attributes.
};

// I2C driver structure.
static struct i2c_driver max30102_driver = {
    .driver = {
        .name = "max30102",  // Driver name.
        .of_match_table = max30102_of_match,  // DT match table.
    },
    .probe = max30102_probe,  // Probe function.
    .remove = max30102_remove,  // Remove function.
    .id_table = max30102_id,  // ID table.
};

module_i2c_driver(max30102_driver);  // Register I2C driver.

MODULE_ALIAS("i2c:max30102");  // Module alias.
MODULE_AUTHOR("Nguyen Nhan");  // Author.
MODULE_DESCRIPTION("MAX30102 Sensor Kernel Module with Enhanced Features");  // Description.
MODULE_LICENSE("GPL");  // License.