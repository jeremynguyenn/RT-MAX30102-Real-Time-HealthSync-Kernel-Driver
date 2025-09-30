#ifndef MAX30102_H
#define MAX30102_H

#include <linux/i2c.h>
#include <linux/rwlock.h>
#include <linux/workqueue.h>
#include <linux/gpio/consumer.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/debugfs.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/rtmutex.h>
#include <linux/srcu.h>
#include <linux/futex.h>

/* MAX30102 Register Definitions */
#define MAX30102_ADDRESS                0x57
#define MAX30102_REG_INTERRUPT_STATUS_1 0x00
#define MAX30102_REG_INTERRUPT_STATUS_2 0x01
#define MAX30102_REG_INTERRUPT_ENABLE_1 0x02
#define MAX30102_REG_INTERRUPT_ENABLE_2 0x03
#define MAX30102_REG_FIFO_WRITE_POINTER 0x04
#define MAX30102_REG_OVERFLOW_COUNTER   0x05
#define MAX30102_REG_FIFO_READ_POINTER  0x06
#define MAX30102_REG_FIFO_DATA          0x07
#define MAX30102_REG_FIFO_CONFIG        0x08
#define MAX30102_REG_MODE_CONFIG        0x09
#define MAX30102_REG_SPO2_CONFIG        0x0A
#define MAX30102_REG_LED_PULSE_1        0x0C
#define MAX30102_REG_LED_PULSE_2        0x0D
#define MAX30102_REG_MULTI_LED_MODE_1   0x11
#define MAX30102_REG_MULTI_LED_MODE_2   0x12
#define MAX30102_REG_DIE_TEMP_INTEGER   0x1F
#define MAX30102_REG_DIE_TEMP_FRACTION  0x20
#define MAX30102_REG_DIE_TEMP_CONFIG    0x21
#define MAX30102_REG_REVISION_ID        0xFE
#define MAX30102_REG_PART_ID            0xFF

/* Magic Number Macros */
#define MAX30102_FIFO_SMP_AVE_8         0x80
#define MAX30102_MODE_SPO2              0x03
#define MAX30102_SPO2_CONFIG_DEFAULT    0x47
#define MAX30102_LED_PULSE_DEFAULT      0x1F
#define MAX30102_SLOT1_RED              0x01
#define MAX30102_SLOT2_IR               0x02
#define MAX30102_INT_ENABLE_FIFO_PPG    0xC0
#define MAX30102_RESET_SOFT             0x40
#define MAX30102_RESET_HARD_LOW         0
#define MAX30102_RESET_HARD_HIGH        1
#define MAX30102_TEMP_START             0x01

/* Interrupt Status Types */
enum max30102_interrupt_status {
    MAX30102_INT_FIFO_FULL    = 7,
    MAX30102_INT_PPG_RDY      = 6,
    MAX30102_INT_ALC_OVF      = 5,
    MAX30102_INT_PWR_RDY      = 0,
    MAX30102_INT_DIE_TEMP_RDY = 1,
};

/* Sample Averaging Options */
enum max30102_smp_ave {
    SMP_AVE_1 = 0,
    SMP_AVE_2 = 1,
    SMP_AVE_4 = 2,
    SMP_AVE_8 = 3,
    SMP_AVE_16 = 4,
    SMP_AVE_32 = 5,
};

/* Input Event Codes (Custom) */
#define ABS_HEART_RATE  0x3F
#define ABS_SPO2        0x40

/* IOCTL Definitions */
#define MAX30102_IOC_MAGIC 'k'
#define MAX30102_IOC_READ_FIFO      _IOR(MAX30102_IOC_MAGIC, 0, struct max30102_fifo_data)
#define MAX30102_IOC_READ_TEMP      _IOR(MAX30102_IOC_MAGIC, 1, float)
#define MAX30102_IOC_SET_MODE       _IOW(MAX30102_IOC_MAGIC, 2, uint8_t)
#define MAX30102_IOC_SET_SLOT       _IOW(MAX30102_IOC_MAGIC, 3, struct max30102_slot_config)
#define MAX30102_IOC_SET_FIFO_CONFIG _IOW(MAX30102_IOC_MAGIC, 4, uint8_t)
#define MAX30102_IOC_SET_SPO2_CONFIG _IOW(MAX30102_IOC_MAGIC, 5, uint8_t)

struct max30102_fifo_entry {
    uint32_t red;
    uint32_t ir;
    struct rcu_head rcu;
};

struct max30102_fifo {
    struct max30102_fifo_entry entries[32];
    atomic_t head;
    atomic_t tail;
};

struct max30102_fifo_data {
    uint32_t red[32];
    uint32_t ir[32];
    uint8_t len;
};

struct max30102_slot_config {
    uint8_t slot;
    uint8_t led;
};

struct max30102_data {
    struct i2c_client *client;
    rwlock_t lock;
    struct work_struct work;
    struct gpio_desc *irq_gpio;
    struct gpio_desc *reset_gpio;
    struct miscdevice miscdev;
    struct input_dev *input_dev;
    struct regulator *vcc_regulator;
    struct device *hwmon_dev;
    struct max30102_fifo fifo;
    uint32_t red_data[32];
    uint32_t ir_data[32];
    uint8_t data_len;
    bool fifo_full;
    wait_queue_head_t wait_data_ready;
    struct dentry *debug_dir;
    struct rt_mutex fifo_mutex;
    struct srcu_struct fifo_srcu;
    atomic_t futex_val;
    atomic_t temp_ready;
    u64 rcu_gp_seq;
};

extern const struct file_operations max30102_fops;
extern void max30102_work_handler(struct work_struct *work);
extern irqreturn_t max30102_irq_handler(int irq, void *dev_id);
extern int max30102_write_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len);
extern int max30102_read_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len);
extern int max30102_init_sensor(struct max30102_data *data);
extern int max30102_set_mode(struct max30102_data *data, uint8_t mode);
extern int max30102_set_slot(struct max30102_data *data, uint8_t slot, uint8_t led);
extern int max30102_set_interrupt(struct max30102_data *data, uint8_t interrupt, bool enable);
extern int max30102_read_fifo(struct max30102_data *data, uint32_t *red, uint32_t *ir, uint8_t *len);
extern int max30102_read_temperature(struct max30102_data *data, float *temp);
extern int max30102_set_fifo_config(struct max30102_data *data, uint8_t config);
extern int max30102_set_spo2_config(struct max30102_data *data, uint8_t config);
extern void max30102_rcu_free(struct rcu_head *head);

#endif