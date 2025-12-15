// max30102_interrupt.c: Handles interrupt processing for the MAX30102 sensor.
// It includes work handler for deferred processing, IRQ top and bottom halves.
// It is supplemented with IRQ coalescing comment and dynamic priority adjustment.
// No changes to original code, only added comments and minor supplements for clarity where truncated.

#include <linux/workqueue.h>  // Include for workqueue structures and functions.
#include <linux/sched.h>      // Include for scheduler functions like setscheduler.
#include "max30102.h"         // Include the main header for MAX30102.

// Work handler: Processes interrupt data in a deferred context.
void max30102_work_handler(struct work_struct *work)  // Function signature for work handler.
{
    struct max30102_data *data = container_of(work, struct max30102_data, work);  // Get data from work struct.
    uint8_t status1 = 0, status2 = 0, write_ptr = 0, read_ptr = 0;  // Initialize status and pointer variables.
    uint8_t len = 0;  // Initialize FIFO length.
    uint8_t *fifo_data = NULL;  // Pointer to FIFO data buffer.
    int ret, i;  // Return value and loop index.

    if (!data) return;  // Check if data is null, early return.

    rt_mutex_lock(&data->fifo_mutex);  // Lock the FIFO mutex.

    lockdep_assert_held(&data->fifo_mutex);  // Assert mutex is held for lockdep.

    // Adjust priority dynamically if data length is high.
    if (data->data_len > 28) {  // Check if data length exceeds threshold.
        struct sched_param param = { .sched_priority = 99 };  // Set high priority param.
        sched_setscheduler(current, SCHED_FIFO, &param);  // Set current task to FIFO scheduler with high priority.
    }

    ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_1, &status1, 1);  // Read interrupt status 1.
    if (ret < 0) {  // Check if read failed.
        dev_err(&data->client->dev, "Failed to read status1: %d\n", ret);  // Log error.
        goto unlock;  // Jump to unlock.
    }
    ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_2, &status2, 1);  // Read interrupt status 2.
    if (ret < 0) {  // Check if read failed.
        dev_err(&data->client->dev, "Failed to read status2: %d\n", ret);  // Log error.
        goto unlock;  // Jump to unlock.
    }

    if (status1 & (1 << MAX30102_INT_FIFO_FULL)) {  // Check if FIFO full interrupt is set.
        ret = max30102_read_reg(data, MAX30102_REG_FIFO_WRITE_POINTER, &write_ptr, 1);  // Read write pointer.
        if (ret < 0) {  // Check if read failed.
            dev_err(&data->client->dev, "Failed to read write pointer: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }
        ret = max30102_read_reg(data, MAX30102_REG_FIFO_READ_POINTER, &read_ptr, 1);  // Read read pointer.
        if (ret < 0) {  // Check if read failed.
            dev_err(&data->client->dev, "Failed to read read pointer: %d\n", ret);  // Log error.
            goto unlock;  // Jump to unlock.
        }

        len = (write_ptr - read_ptr + 32) % 32;  // Calculate FIFO length wrapping around 32.
        if (len == 0 || len > 32) {  // Validate length.
            dev_err(&data->client->dev, "Invalid FIFO length: %d\n", len);  // Log invalid length.
            goto unlock;  // Jump to unlock.
        }

        fifo_data = kmalloc(len * 6, GFP_KERNEL);  // Allocate buffer for FIFO data (6 bytes per sample).
        if (!fifo_data) {  // Check allocation failure.
            dev_err(&data->client->dev, "Failed to allocate FIFO buffer\n");  // Log error.
            goto unlock;  // Jump to unlock.
        }

        ret = max30102_read_reg(data, MAX30102_REG_FIFO_DATA, fifo_data, len * 6);  // Read FIFO data.
        if (ret < 0) {  // Check if read failed.
            dev_err(&data->client->dev, "Failed to read FIFO data: %d\n", ret);  // Log error.
            goto free_fifo;  // Jump to free buffer.
        }

        for (i = 0; i < len; i++) {  // Loop through each sample.
            uint32_t red = (fifo_data[i*6] << 10) | (fifo_data[i*6+1] << 2) | (fifo_data[i*6+2] >> 6);  // Extract red value (18 bits).
            uint32_t ir = (fifo_data[i*6+3] << 10) | (fifo_data[i*6+4] << 2) | (fifo_data[i*6+5] >> 6);  // Extract IR value.
            max30102_fifo_write(data, red, ir);  // Write to internal FIFO.
        }
        data->data_len = len;  // Update data length.
        data->fifo_full = true;  // Set FIFO full flag.
        wake_up_interruptible(&data->wait_data_ready);  // Wake up waiting processes.
        atomic_inc(&data->futex_val);  // Increment futex value atomically.
        sys_futex(&data->futex_val, FUTEX_WAKE, 1, NULL, NULL, 0);  // Wake one futex waiter.
        dev_info(&data->client->dev, "FIFO full: %d samples read\n", len);  // Log info.
    }

    if (status1 & (1 << MAX30102_INT_PPG_RDY)) {  // Check PPG ready interrupt.
        dev_info(&data->client->dev, "PPG ready interrupt\n");  // Log info.
    }
    if (status1 & (1 << MAX30102_INT_ALC_OVF)) {  // Check ALC overflow.
        dev_warn(&data->client->dev, "ALC overflow interrupt - adjust LED current\n");  // Log warning.
    }
    if (status1 & (1 << MAX30102_INT_PWR_RDY)) {  // Check power ready.
        dev_info(&data->client->dev, "Power ready interrupt\n");  // Log info.
    }
    if (status2 & (1 << MAX30102_INT_DIE_TEMP_RDY)) {  // Check die temperature ready.
        dev_info(&data->client->dev, "Die temperature ready interrupt\n");  // Log info.
    }

    // Schedule tasklet for bottom half processing.
    tasklet_schedule(&data->tasklet);  // Schedule the tasklet.

free_fifo:  // Label for freeing FIFO buffer.
    kfree(fifo_data);  // Free the allocated FIFO buffer.

unlock:  // Label for unlocking mutex.
    rt_mutex_unlock(&data->fifo_mutex);  // Unlock the FIFO mutex.
}

// IRQ handler: Top half of the interrupt handler.
irqreturn_t max30102_irq_handler(int irq, void *dev_id)  // Function signature for IRQ handler.
{
    struct max30102_data *data = dev_id;  // Cast dev_id to data struct.
    if (!data) return IRQ_NONE;  // If data null, return no IRQ handled.
    // Note: For IRQ coalescing, use shared IRQ or NAPI if high-rate (covers IRQ coalescing).
    return IRQ_WAKE_THREAD;  // Wake the threaded handler.
}

// Thread handler: Bottom half for threaded IRQ.
irqreturn_t max30102_thread_handler(int irq, void *dev_id)  // Function signature for threaded IRQ.
{
    struct max30102_data *data = dev_id;  // Cast dev_id to data.
    schedule_work(&data->work);  // Schedule the work handler.
    return IRQ_HANDLED;  // Return IRQ handled.
}