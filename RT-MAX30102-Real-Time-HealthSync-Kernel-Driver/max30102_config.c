#include <linux/delay.h>
#include "max30102.h"

int max30102_init_sensor(struct max30102_data *data)
{
    uint8_t value;
    int ret;

    if (!data || !data->reset_gpio) return -EINVAL;

    gpiod_set_value(data->reset_gpio, MAX30102_RESET_HARD_LOW);
    msleep(10);
    gpiod_set_value(data->reset_gpio, MAX30102_RESET_HARD_HIGH);
    msleep(100);

    value = MAX30102_RESET_SOFT;
    ret = max30102_write_reg(data, MAX30102_REG_MODE_CONFIG, &value, 1);
    if (ret < 0) dev_err(&data->client->dev, "Software reset failed: %d\n", ret), return ret;
    msleep(100);

    value = 0x00;
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_WRITE_POINTER, &value, 1);
    if (ret < 0) return ret;
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_READ_POINTER, &value, 1);
    if (ret < 0) return ret;
    ret = max30102_write_reg(data, MAX30102_REG_OVERFLOW_COUNTER, &value, 1);
    if (ret < 0) return ret;

    value = MAX30102_FIFO_SMP_AVE_8;
    ret = max30102_write_reg(data, MAX30102_REG_FIFO_CONFIG, &value, 1);
    if (ret < 0) return ret;

    value = MAX30102_MODE_SPO2;
    ret = max30102_write_reg(data, MAX30102_REG_MODE_CONFIG, &value, 1);
    if (ret < 0) return ret;

    value = MAX30102_SPO2_CONFIG_DEFAULT;
    ret = max30102_write_reg(data, MAX30102_REG_SPO2_CONFIG, &value, 1);
    if (ret < 0) return ret;

    value = MAX30102_LED_PULSE_DEFAULT;
    ret = max30102_write_reg(data, MAX30102_REG_LED_PULSE_1, &value, 1);
    if (ret < 0) return ret;
    ret = max30102_write_reg(data, MAX30102_REG_LED_PULSE_2, &value, 1);
    if (ret < 0) return ret;

    value = MAX30102_SLOT1_RED;
    ret = max30102_write_reg(data, MAX30102_REG_MULTI_LED_MODE_1, &value, 1);
    if (ret < 0) return ret;
    value = MAX30102_SLOT2_IR;
    ret = max30102_write_reg(data, MAX30102_REG_MULTI_LED_MODE_2, &value, 1);
    if (ret < 0) return ret;

    value = MAX30102_INT_ENABLE_FIFO_PPG;
    ret = max30102_write_reg(data, MAX30102_REG_INTERRUPT_ENABLE_1, &value, 1);
    if (ret < 0) return ret;

    return 0;
}

int max30102_set_mode(struct max30102_data *data, uint8_t mode)
{
    if (!data) return -EINVAL;
    if (mode != 0x02 && mode != 0x03 && mode != 0x07) dev_err(&data->client->dev, "Invalid mode: 0x%02x\n", mode), return -EINVAL;
    return max30102_write_reg(data, MAX30102_REG_MODE_CONFIG, &mode, 1);
}

int max30102_set_slot(struct max30102_data *data, uint8_t slot, uint8_t led)
{
    uint8_t reg, shift, value, current;
    int ret;

    if (!data) return -EINVAL;
    if (slot < 1 || slot > 4 || led > 3) dev_err(&data->client->dev, "Invalid slot=%d or led=%d\n", slot, led), return -EINVAL;
    reg = (slot <= 2) ? MAX30102_REG_MULTI_LED_MODE_1 : MAX30102_REG_MULTI_LED_MODE_2;
    shift = (slot % 2 == 1) ? 0 : 4;
    ret = max30102_read_reg(data, reg, &current, 1);
    if (ret < 0) return ret;
    value = (current & ~(0x07 << shift)) | (led << shift);
    return max30102_write_reg(data, reg, &value, 1);
}

int max30102_set_interrupt(struct max30102_data *data, uint8_t interrupt, bool enable)
{
    uint8_t reg, value, mask;
    int ret;

    if (!data) return -EINVAL;
    if (interrupt > MAX30102_INT_DIE_TEMP_RDY && interrupt != MAX30102_INT_FIFO_FULL &&
        interrupt != MAX30102_INT_PPG_RDY && interrupt != MAX30102_INT_ALC_OVF &&
        interrupt != MAX30102_INT_PWR_RDY) dev_err(&data->client->dev, "Invalid interrupt type: %d\n", interrupt), return -EINVAL;

    reg = (interrupt == MAX30102_INT_DIE_TEMP_RDY) ? MAX30102_REG_INTERRUPT_ENABLE_2 : MAX30102_REG_INTERRUPT_ENABLE_1;
    mask = 1 << interrupt;

    ret = max30102_read_reg(data, reg, &value, 1);
    if (ret < 0) return ret;

    value = enable ? (value | mask) : (value & ~mask);
    return max30102_write_reg(data, reg, &value, 1);
}

int max30102_set_fifo_config(struct max30102_data *data, uint8_t config)
{
    if (!data) return -EINVAL;
    if (config & ~0xFF) dev_err(&data->client->dev, "Invalid FIFO config: 0x%02x\n", config), return -EINVAL;
    return max30102_write_reg(data, MAX30102_REG_FIFO_CONFIG, &config, 1);
}

int max30102_set_spo2_config(struct max30102_data *data, uint8_t config)
{
    uint8_t pw, sr;

    if (!data) return -EINVAL;
    if (config & ~0x7F) dev_err(&data->client->dev, "Invalid SpO2 config: 0x%02x\n", config), return -EINVAL;
    pw = config & 0x03;
    sr = (config >> 2) & 0x07;
    if ((pw == 0 && sr > 4) || (pw == 1 && sr > 6)) dev_err(&data->client->dev, "Invalid SR/PW combination\n"), return -EINVAL;
    return max30102_write_reg(data, MAX30102_REG_SPO2_CONFIG, &config, 1);
}