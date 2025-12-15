#include <linux/delay.h>  // Include delay functions.
#include "max30102.h"  // Include local header.

// Function to initialize sensor.
int max30102_init_sensor(struct max30102_data *data)
{
    uint8_t value;  // Temporary value.
    int ret;  // Return value.

    if (!data || !data->reset_gpio) return -EINVAL;  // Check arguments.

    gpiod_set_value(data->reset_gpio, MAX30102_RESET_HARD_LOW);  // Set reset low.
    msleep(10);  // Delay 10ms.
    gpiod_set_value(data->reset_gpio, MAX30102_RESET_HARD_HIGH);  // Set reset high.
    msleep(100);  // Delay 100ms.

    value = MAX30102_RESET_SOFT;  // Set soft reset value.
    ret = max30102_write_reg(data, MAX30102_REG_MODE_CONFIG, &value, 1);  // Write soft reset.
    if (ret < 0) {  // Error.
        dev_err(&data->client->dev, "Software reset failed: %d\n", ret);  // Log.
        return ret;  // Return.
    }
    msleep(100);  // Delay after reset.

    value = 0x00;  // Zero value.
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_WRITE_POINTER, &value, 1);  // Clear write ptr.
    if (ret < 0) return ret;  // Error.
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_READ_POINTER, &value, 1);  // Clear read ptr.
    if (ret < 0) return ret;  // Error.
    ret = max30102_write_reg(data, MAX30102_REG_OVERFLOW_COUNTER, &value, 1);  // Clear overflow.
    if (ret < 0) return ret;  // Error.

    value = MAX30102_FIFO_SMP_AVE_8;  // Set sample averaging.
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_CONFIG, &value, 1);  // Write FIFO config.
    if (ret < 0) return ret;  // Error.

    value = MAX30102_MODE_SPO2;  // Set SpO2 mode.
    ret = max30102_write_reg(data, MAX30102_REG_MODE_CONFIG, &value, 1);  // Write mode.
    if (ret < 0) return ret;  // Error.

    value = MAX30102_SPO2_CONFIG_DEFAULT;  // Default SpO2 config.
    ret = max30102_write_reg(data, MAX30102_REG_SPO2_CONFIG, &value, 1);  // Write SpO2 config.
    if (ret < 0) return ret;  // Error.

    value = MAX30102_LED_PULSE_DEFAULT;  // Default LED pulse.
    ret = max30102_write_reg(data, MAX30102_REG_LED_PULSE_1, &value, 1);  // Write LED1.
    if (ret < 0) return ret;  // Error.
    ret = max30102_write_reg(data, MAX30102_REG_LED_PULSE_2, &value, 1);  // Write LED2.
    if (ret < 0) return ret;  // Error.

    value = MAX30102_SLOT1_RED;  // Slot1 red.
    ret = max30102_write_reg(data, MAX30102_REG_MULTI_LED_MODE_1, &value, 1);  // Write multi LED1.
    if (ret < 0) return ret;  // Error.
    value = MAX30102_SLOT2_IR;  // Slot2 IR.
    ret = max30102_write_reg(data, MAX30102_REG_MULTI_LED_MODE_2, &value, 1);  // Write multi LED2.
    if (ret < 0) return ret;  // Error.

    value = MAX30102_INT_ENABLE_FIFO_PPG;  // Enable FIFO PPG interrupt.
    ret = max30102_write_reg(data, MAX30102_REG_INTERRUPT_ENABLE_1, &value, 1);  // Write enable1.
    if (ret < 0) return ret;  // Error.

    return 0;  // Success.
}

// Function to set mode.
int max30102_set_mode(struct max30102_data *data, uint8_t mode)
{
    if (!data) return -EINVAL;  // Check data.
    if (mode != 0x02 && mode != 0x03 && mode != 0x07) dev_err(&data->client->dev, "Invalid mode: 0x%02x\n", mode), return -EINVAL;  // Validate mode.
    return max30102_write_reg(data, MAX30102_REG_MODE_CONFIG, &mode, 1);  // Write mode.
}

// Function to set slot.
int max30102_set_slot(struct max30102_data *data, uint8_t slot, uint8_t led)
{
    uint8_t reg, shift, value, current;  // Variables.
    int ret;  // Return.

    if (!data) return -EINVAL;  // Check data.
    if (slot < 1 || slot > 4 || led > 3) dev_err(&data->client->dev, "Invalid slot=%d or led=%d\n", slot, led), return -EINVAL;  // Validate.
    reg = (slot <= 2) ? MAX30102_REG_MULTI_LED_MODE_1 : MAX30102_REG_MULTI_LED_MODE_2;  // Select reg.
    shift = (slot % 2 == 1) ? 0 : 4;  // Calculate shift.
    ret = max30102_read_reg(data, reg, &current, 1);  // Read current.
    if (ret < 0) return ret;  // Error.
    value = (current & ~(0x07 << shift)) | (led << shift);  // Modify value.
    return max30102_write_reg(data, reg, &value, 1);  // Write back.
}

// Function to set interrupt.
int max30102_set_interrupt(struct max30102_data *data, uint8_t interrupt, bool enable)
{
    uint8_t reg, value, mask;  // Variables.
    int ret;  // Return.

    if (!data) return -EINVAL;  // Check.
    if (interrupt > MAX30102_INT_DIE_TEMP_RDY && interrupt != MAX30102_INT_FIFO_FULL &&
        interrupt != MAX30102_INT_PPG_RDY && interrupt != MAX30102_INT_ALC_OVF &&
        interrupt != MAX30102_INT_PWR_RDY) dev_err(&data->client->dev, "Invalid interrupt type: %d\n", interrupt), return -EINVAL;  // Validate type.

    reg = (interrupt == MAX30102_INT_DIE_TEMP_RDY) ? MAX30102_REG_INTERRUPT_ENABLE_2 : MAX30102_REG_INTERRUPT_ENABLE_1;  // Select reg.
    mask = 1 << interrupt;  // Create mask.

    ret = max30102_read_reg(data, reg, &value, 1);  // Read current.
    if (ret < 0) return ret;  // Error.

    value = enable ? (value | mask) : (value & ~mask);  // Set or clear bit.
    return max30102_write_reg(data, reg, &value, 1);  // Write back.
}

// Function to set FIFO config.
int max30102_set_fifo_config(struct max30102_data *data, uint8_t config)
{
    if (!data) return -EINVAL;  // Check.
    if (config & ~0xFF) dev_err(&data->client->dev, "Invalid FIFO config: 0x%02x\n", config), return -EINVAL;  // Validate.
    return max30102_write_reg(data, MAX30102_REG_FIFO_CONFIG, &config, 1);  // Write.
}

// Function to set SpO2 config.
int max30102_set_spo2_config(struct max30102_data *data, uint8_t config)
{
    uint8_t pw, sr;  // Pulse width and sample rate.

    if (!data) return -EINVAL;  // Check.
    if (config & ~0x7F) dev_err(&data->client->dev, "Invalid SpO2 config: 0x%02x\n", config), return -EINVAL;  // Validate.
    pw = config & 0x03;  // Extract PW.
    sr = (config >> 2) & 0x07;  // Extract SR.
    if ((pw == 0 && sr > 4) || (pw == 1 && sr > 6)) dev_err(&data->client->dev, "Invalid SR/PW combination\n"), return -EINVAL;  // Validate combo.
    return max30102_write_reg(data, MAX30102_REG_SPO2_CONFIG, &config, 1);  // Write.
}