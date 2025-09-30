#include <linux/delay.h>
#include <linux/rcupdate.h>
#include <linux/math64.h>
#include "max30102.h"

void max30102_rcu_free(struct rcu_head *head)
{
    struct max30102_fifo_entry *entry = container_of(head, struct max30102_fifo_entry, rcu);
    kfree(entry);
}

static int max30102_clear_fifo(struct max30102_data *data)
{
    uint8_t value = 0x00;
    int ret;

    if (!data) return -EINVAL;

    ret = max30102_write_reg(data, MAX30102_REG_FIFO_WRITE_POINTER, &value, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to clear FIFO write pointer: %d\n", ret), return ret;
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_READ_POINTER, &value, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to clear FIFO read pointer: %d\n", ret), return ret;
    ret = max30102_write_reg(data, MAX30102_REG_OVERFLOW_COUNTER, &value, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to clear FIFO overflow counter: %d\n", ret), return ret;
    atomic_set(&data->fifo.head, 0);
    atomic_set(&data->fifo.tail, 0);
    return 0;
}

static uint32_t calculate_mean(uint32_t *arr, uint8_t len) {
    uint64_t sum = 0;
    int i;
    for (i = 0; i < len; i++) sum += arr[i];
    return (uint32_t)div64_u64(sum, len);
}

static uint32_t calculate_stddev(uint32_t *arr, uint8_t len, uint32_t mean) {
    uint64_t variance = 0;
    int i;
    for (i = 0; i < len; i++) {
        int32_t diff = arr[i] - mean;
        variance += diff * diff;
    }
    variance = div64_u64(variance, len);
    return (uint32_t)int_sqrt(variance);
}

static uint32_t find_min(uint32_t *arr, uint8_t len) {
    uint32_t min_val = UINT32_MAX;
    int i;
    for (i = 0; i < len; i++) if (arr[i] < min_val) min_val = arr[i];
    return min_val;
}

static uint32_t find_max(uint32_t *arr, uint8_t len) {
    uint32_t max_val = 0;
    int i;
    for (i = 0; i < len; i++) if (arr[i] > max_val) max_val = arr[i];
    return max_val;
}

static void max30102_fifo_write(struct max30102_data *data, uint32_t red, uint32_t ir)
{
    int tail = atomic_read(&data->fifo.tail);
    int next_tail = (tail + 1) % 32;
    if (next_tail != atomic_read(&data->fifo.head)) {
        data->fifo.entries[tail].red = red;
        data->fifo.entries[tail].ir = ir;
        atomic_cmpxchg(&data->fifo.tail, tail, next_tail);
    } else {
        dev_warn(&data->client->dev, "FIFO full, dropping sample\n");
    }
}

int max30102_read_fifo(struct max30102_data *data, uint32_t *red, uint32_t *ir, uint8_t *len)
{
    int head, tail, idx, i;
    uint8_t ovf_counter;
    int ret;

    if (!data || !red || !ir || !len) return -EINVAL;

    ret = max30102_read_reg(data, MAX30102_REG_OVERFLOW_COUNTER, &ovf_counter, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to read overflow counter: %d\n", ret), return ret;
    if (ovf_counter > 0) {
        dev_warn(&data->client->dev, "FIFO overflow: %d samples lost\n", ovf_counter);
        uint8_t config;
        ret = max30102_read_reg(data, MAX30102_REG_SPO2_CONFIG, &config, 1);
        if (ret < 0) return ret;
        config = (config & ~0x1C) | (0x04 << 2);
        ret = max30102_set_spo2_config(data, config);
        if (ret < 0) return ret;
    }

    read_lock(&data->lock);
    idx = srcu_read_lock(&data->fifo_srcu);
    head = atomic_read(&data->fifo.head);
    tail = atomic_read(&data->fifo.tail);
    *len = (tail >= head) ? (tail - head) : (32 + tail - head);

    if (*len == 0) {
        dev_dbg(&data->client->dev, "No FIFO data available\n");
        srcu_read_unlock(&data->fifo_srcu, idx);
        read_unlock(&data->lock);
        return -ENODATA;
    }

    for (i = 0; i < *len; i++) {
        int pos = (head + i) % 32;
        struct max30102_fifo_entry *entry = &data->fifo.entries[pos];
        red[i] = entry->red;
        ir[i] = entry->ir;
    }
    atomic_set(&data->fifo.head, tail);

    data->rcu_gp_seq = rcu_get_gp_seq();
    dev_info(&data->client->dev, "RCU grace period: %llu\n", data->rcu_gp_seq);

    srcu_read_unlock(&data->fifo_srcu, idx);
    read_unlock(&data->lock);

    ret = max30102_clear_fifo(data);
    if (ret < 0) return ret;

    if (data->input_dev && *len > 10) {
        uint32_t ir_mean = calculate_mean(ir, *len);
        uint32_t ir_stddev = calculate_stddev(ir, *len, ir_mean);
        uint32_t threshold = ir_mean + ir_stddev / 2;
        int peak_count = 0;
        uint64_t total_interval = 0;
        int last_peak = -1;

        for (i = 1; i < *len - 1; i++) {
            if (ir[i] > threshold && ir[i] > ir[i-1] && ir[i] > ir[i+1]) {
                if (last_peak >= 0) {
                    total_interval += (i - last_peak);
                    peak_count++;
                }
                last_peak = i;
            }
        }

        int heart_rate = 0;
        if (peak_count > 0) {
            uint64_t avg_interval = total_interval / peak_count;
            heart_rate = 60 * 100 / avg_interval;
        }

        uint32_t red_mean = calculate_mean(red, *len);
        uint32_t ac_red = find_max(red, *len) - find_min(red, *len);
        uint32_t ac_ir = find_max(ir, *len) - find_min(ir, *len);
        double ratio = (ac_red * 1.0 / red_mean) / (ac_ir * 1.0 / ir_mean);
        int spo2 = (int)(-45.0 * ratio * ratio + 30.0 * ratio + 94.0);
        if (spo2 < 0) spo2 = 0;
        if (spo2 > 100) spo2 = 100;

        if (heart_rate > 30 && heart_rate < 220 && spo2 > 50 && spo2 <= 100) {
            input_report_abs(data->input_dev, ABS_HEART_RATE, heart_rate);
            input_report_abs(data->input_dev, ABS_SPO2, spo2);
            input_sync(data->input_dev);
            dev_info(&data->client->dev, "Calculated HR: %d bpm, SpO2: %d%%\n", heart_rate, spo2);
        } else {
            dev_warn(&data->client->dev, "Invalid HR/SpO2 calculation, skipping report\n");
        }
    }

    return 0;
}

int max30102_read_temperature(struct max30102_data *data, float *temp)
{
    uint8_t temp_int, temp_frac, status;
    int ret, timeout = 10;

    if (!data || !temp) return -EINVAL;

    ret = max30102_write_reg(data, MAX30102_REG_DIE_TEMP_CONFIG, &(uint8_t){MAX30102_TEMP_START}, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to start temperature measurement: %d\n", ret), return ret;

    do {
        msleep(10);
        ret = max30102_read_reg(data, MAX30102_REG_INTERRUPT_STATUS_2, &status, 1);
        if (ret < 0) return ret;
        timeout--;
    } while (!(status & (1 << MAX30102_INT_DIE_TEMP_RDY)) && timeout > 0);

    if (timeout <= 0) dev_err(&data->client->dev, "Temperature measurement timeout\n"), return -ETIMEDOUT;

    ret = max30102_read_reg(data, MAX30102_REG_DIE_TEMP_INTEGER, &temp_int, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to read temperature integer: %d\n", ret), return ret;

    ret = max30102_read_reg(data, MAX30102_REG_DIE_TEMP_FRACTION, &temp_frac, 1);
    if (ret < 0) dev_err(&data->client->dev, "Failed to read temperature fraction: %d\n", ret), return ret;

    *temp = (int8_t)temp_int + (temp_frac * 0.0625);
    atomic_inc(&data->temp_ready);
    sys_futex(&data->temp_ready, FUTEX_WAKE, 1, NULL, NULL, 0);
    return 0;
}