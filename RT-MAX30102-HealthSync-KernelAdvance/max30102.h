// max30102.h: Header file for MAX30102.
// Includes all necessary includes and declarations.
// Supplemented with missing includes like configfs if needed, but not directly.
// No changes to original code, only added comments.

#ifndef MAX30102_H  // Header guard.
#define MAX30102_H  // Define guard.

#include <linux/i2c.h>         // I2C client.
#include <linux/rwlock.h>      // Read-write lock.
#include <linux/workqueue.h>   // Workqueue.
#include <linux/gpio/consumer.h>  // GPIO consumer.
#include <linux/miscdevice.h>  // Misc device.
#include <linux/device.h>      // Device core.
#include <linux/wait.h>        // Wait queue.
#include <linux/debugfs.h>     // Debugfs.
#include <linux/input.h>       // Input subsystem.
#include <linux/regulator/consumer.h>  // Regulators.
#include <linux/hwmon.h>       // Hwmon.
#include <linux/hwmon-sysfs.h> // Hwmon sysfs.
#include <linux/rtmutex.h>     // RT mutex.
#include <linux/srcu.h>        // SRCU.
#include <linux/futex.h>       // Futex.
#include <linux/seqlock.h>     // Seqlock.
#include <linux/slab.h>        // Slab allocator.
#include <linux/kthread.h>     // Kthread.
#include <linux/hrtimer.h>     // Hrtimer.
#include <linux/kref.h>        // Kref.
#include <linux/dma-mapping.h> // DMA mapping.
#include <linux/pinctrl/consumer.h>  // Pinctrl.
#include <linux/clk.h>         // Clocks.
#include <linux/proc_fs.h>     // Procfs.
#include <linux/completion.h>  // Completion.
#include <linux/semaphore.h>   // Semaphore.
#include <linux/netlink.h>     // Netlink.
#include <linux/eventfd.h>     // Eventfd.
#include <linux/uaccess.h>     // User access.
#include <linux/mm.h>          // Memory management.
#include <linux/vmalloc.h>     // Vmalloc.
#include <linux/highmem.h>     // Highmem.
#include <linux/ioport.h>      // I/O port.
#include <linux/tracepoint.h>  // Tracepoints.
#include <linux/lockdep.h>     // Lockdep.
#include <uapi/linux/netlink.h>  // UAPI netlink.

// Address and register definitions.
#define MAX30102_ADDRESS 0x57  // I2C address.
#define MAX30102_REG_INTERRUPT_STATUS_1 0x00  // Interrupt status 1.
#define MAX30102_REG_INTERRUPT_STATUS_2 0x01  // Interrupt status 2.
#define MAX30102_REG_INTERRUPT_ENABLE_1 0x02  // Enable 1.
#define MAX30102_REG_INTERRUPT_ENABLE_2 0x03  // Enable 2.
#define MAX30102_REG_FIFO_WRITE_POINTER 0x04  // Write pointer.
#define MAX30102_REG_OVERFLOW_COUNTER 0x05  // Overflow counter.
#define MAX30102_REG_FIFO_READ_POINTER 0x06  // Read pointer.
#define MAX30102_REG_FIFO_DATA 0x07  // FIFO data.
#define MAX30102_REG_FIFO_CONFIG 0x08  // FIFO config.
#define MAX30102_REG_MODE_CONFIG 0x09  // Mode config.
#define MAX30102_REG_SPO2_CONFIG 0x0A  // SpO2 config.
#define MAX30102_REG_LED_PULSE_1 0x0C  // LED pulse 1.
#define MAX30102_REG_LED_PULSE_2 0x0D  // LED pulse 2.
#define MAX30102_REG_MULTI_LED_MODE_1 0x11  // Multi LED mode 1.
#define MAX30102_REG_MULTI_LED_MODE_2 0x12  // Multi LED mode 2.
#define MAX30102_REG_DIE_TEMP_INTEGER 0x1F  // Die temp integer.
#define MAX30102_REG_DIE_TEMP_FRACTION 0x20  // Die temp fraction.
#define MAX30102_REG_DIE_TEMP_CONFIG 0x21  // Die temp config.
#define MAX30102_REG_PART_ID 0xFF  // Part ID.

// Constants.
#define MAX30102_PART_ID 0x15  // Expected part ID.
#define MAX30102_RESET_SOFT 0x40  // Soft reset value.
#define MAX30102_RESET_HARD_LOW 0  // Hard reset low.
#define MAX30102_RESET_HARD_HIGH 1  // Hard reset high.
#define MAX30102_FIFO_SMP_AVE_8 0x03  // Sample average 8.
#define MAX30102_MODE_SPO2 0x03  // SpO2 mode.
#define MAX30102_SPO2_CONFIG_DEFAULT 0x40  // Default SpO2 config.
#define MAX30102_LED_PULSE_DEFAULT 0x7F  // Default LED pulse.
#define MAX30102_SLOT1_RED 0x01  // Slot1 red.
#define MAX30102_TEMP_START 0x01  // Start temp measurement.
#define MAX30102_INT_FIFO_FULL 6  // FIFO full interrupt bit.
#define MAX30102_INT_PPG_RDY 5  // PPG ready bit.
// (Continuation of max30102.h)
#define MAX30102_INT_ALC_OVF 4  // ALC overflow bit.
#define MAX30102_INT_PWR_RDY 0  // Power ready bit.
#define MAX30102_INT_DIE_TEMP_RDY 0  // Die temp ready bit.
#define MAX30102_INT_ENABLE_FIFO_PPG 0xC0  // Enable FIFO and PPG.

// FIFO entry structure.
struct max30102_fifo_entry {  // FIFO entry.
    uint32_t red;  // Red value.
    uint32_t ir;  // IR value.
    struct rcu_head rcu;  // RCU head for free.
};

// FIFO structure.
struct max30102_fifo {  // FIFO struct.
    struct max30102_fifo_entry entries[32];  // Entries array.
    atomic_t head;  // Atomic head.
    atomic_t tail;  // Atomic tail.
    seqlock_t seqlock;  // Seqlock for sync.
};

// FIFO data for ioctl.
struct max30102_fifo_data {  // FIFO data struct.
    uint32_t red[32];  // Red array.
    uint32_t ir[32];  // IR array.
    uint8_t len;  // Length.
};

// Slot config for ioctl.
struct max30102_slot_config {  // Slot config struct.
    uint8_t slot;  // Slot number.
    uint8_t led;  // LED type.
};

// IOCTL commands.
#define MAX30102_IOC_MAGIC 0xA1  // Magic number.
#define MAX30102_IOC_READ_FIFO _IOR(MAX30102_IOC_MAGIC, 1, struct max30102_fifo_data)  // Read FIFO.
#define MAX30102_IOC_READ_TEMP _IOR(MAX30102_IOC_MAGIC, 2, float)  // Read temp.
#define MAX30102_IOC_SET_MODE _IOW(MAX30102_IOC_MAGIC, 3, uint8_t)  // Set mode.
#define MAX30102_IOC_SET_SLOT _IOW(MAX30102_IOC_MAGIC, 4, struct max30102_slot_config)  // Set slot.
#define MAX30102_IOC_SET_FIFO_CONFIG _IOW(MAX30102_IOC_MAGIC, 5, uint8_t)  // Set FIFO config.
#define MAX30102_IOC_SET_SPO2_CONFIG _IOW(MAX30102_IOC_MAGIC, 6, uint8_t)  // Set SpO2 config.
#define MAX30102_IOC_SET_NETLINK _IO(MAX30102_IOC_MAGIC, 7)  // Set netlink demo.

// Data structure.
struct max30102_data {  // Main data struct.
    struct i2c_client *client;  // I2C client.
    struct miscdevice miscdev;  // Misc device.
    struct input_dev *input_dev;  // Input device.
    struct regulator *vcc_regulator;  // VCC regulator.
    struct gpio_desc *irq_gpio;  // IRQ GPIO.
    struct gpio_desc *reset_gpio;  // Reset GPIO.
    rwlock_t lock;  // RW lock.
    rt_mutex_t fifo_mutex;  // RT mutex for FIFO.
    wait_queue_head_t wait_data_ready;  // Wait queue for data ready.
    struct dentry *debugfs_entry;  // Debugfs entry.
    struct max30102_fifo fifo;  // FIFO struct.
    bool fifo_full;  // FIFO full flag.
    uint8_t data_len;  // Data length.
    atomic_t futex_val;  // Futex value.
    atomic_t temp_ready;  // Temp ready atomic.
    struct work_struct work;  // Work struct.
    struct pinctrl *pinctrl;  // Pinctrl.
    struct pinctrl_state *pins_active;  // Active pins.
    struct pinctrl_state *pins_sleep;   // Sleep pins.
    struct clk *clk;              // Clock.
    void __iomem *mmio_base;      // MMIO base.
    void *dma_buf;                // DMA buffer.
    dma_addr_t dma_handle;        // DMA handle.
    struct completion completion; // Completion.
    struct semaphore sem;         // Semaphore.
    struct tasklet_struct tasklet;// Tasklet.
    struct delayed_work delayed_work;  // Delayed work.
    struct hrtimer hrtimer;       // Hrtimer.
    struct task_struct *kthread;  // Kthread.
    struct netlink_kernel_cfg nl_cfg;  // Netlink config.
    struct sock *nl_sock;         // Netlink socket.
    struct proc_dir_entry *proc_entry;  // Proc entry.
    void __percpu *percpu_buf;    // Percpu buffer.
    struct kref kref;             // Kref for reference counting.
};

// Extern declarations.
extern const struct file_operations max30102_fops;  // File ops.
extern void max30102_work_handler(struct work_struct *work);  // Work handler.
extern irqreturn_t max30102_irq_handler(int irq, void *dev_id);  // IRQ handler.
extern irqreturn_t max30102_thread_handler(int irq, void *dev_id);  // Threaded IRQ.
extern int max30102_write_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len);  // Write reg.
extern int max30102_read_reg(struct max30102_data *data, uint8_t reg, uint8_t *buf, uint16_t len);  // Read reg.
extern int max30102_init_sensor(struct max30102_data *data);  // Init sensor.
extern int max30102_set_mode(struct max30102_data *data, uint8_t mode);  // Set mode.
extern int max30102_set_slot(struct max30102_data *data, uint8_t slot, uint8_t led);  // Set slot.
extern int max30102_set_interrupt(struct max30102_data *data, uint8_t interrupt, bool enable);  // Set interrupt.
extern int max30102_read_fifo(struct max30102_data *data, uint32_t *red, uint32_t *ir, uint8_t *len);  // Read FIFO.
extern int max30102_read_temperature(struct max30102_data *data, float *temp);  // Read temp.
extern int max30102_set_fifo_config(struct max30102_data *data, uint8_t config);  // Set FIFO config.
extern int max30102_set_spo2_config(struct max30102_data *data, uint8_t config);  // Set SpO2 config.
extern void max30102_rcu_free(struct rcu_head *head);  // RCU free.
extern int max30102_mmap(struct file *file, struct vm_area_struct *vma);  // Mmap.
extern int max30102_kthread_func(void *arg);  // Kthread func.
extern enum hrtimer_restart max30102_hrtimer_callback(struct hrtimer *timer);  // Hrtimer callback.
extern void max30102_tasklet_func(unsigned long data);  // Tasklet func.

#endif  // End guard.