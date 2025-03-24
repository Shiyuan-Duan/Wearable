//ads1299.c

#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>



#include <ncs_version.h>
#if NCS_VERSION_NUMBER >= 0x20600
#include <zephyr/internal/syscall_handler.h>
#else
#include <zephyr/syscall_handler.h>
#endif


#include "ads1299.h"


#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')

#define SPIOP (SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)
// #define SPIOP (SPI_WORD_SET(8))



LOG_MODULE_REGISTER(ADS1299, LOG_LEVEL_DBG);

struct ads1299_data
{
    uint8_t chip_id;
    struct k_sem drdy_sem;            // Semaphore for DRDY signaling
    struct gpio_callback drdy_cb;     // GPIO callback structure for DRDY

};
struct ads1299_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec drdy;
    struct gpio_dt_spec pwdn;
    struct gpio_dt_spec reset;
    struct gpio_dt_spec start;
};

static struct gpio_callback drdy_cb_data;


// This function writes a value of one byte to a register in the ADS1299
static int ads1299_write_register(const struct device *dev, uint8_t reg, uint8_t data)
{
    int err;
    const struct ads1299_config *cfg = dev->config;
    uint8_t tx_buf[3];

    tx_buf[0] = 0x40 | reg;
    tx_buf[1] = 0x00;
    tx_buf[2] = data;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("spi_write_dt() failed, err %d", err);
        return err;
    }

    return 0;

}

static int ads1299_send_command(const struct device *dev, uint8_t command)
{
    int err;
    const struct ads1299_config *cfg = dev->config;
    uint8_t tx_buf[1];

    tx_buf[0] = command;

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("spi_write_dt() failed, err %d", err);
        return err;
    }

    return 0;
}


// This function reads a value of one byte from a register in the ADS1299
static int ads1299_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, int size)
{
    int err;
    const int total_length = 2 + size; // 2 command bytes + data bytes
    uint8_t tx_buffer[total_length];
    uint8_t rx_buffer[total_length];

    // Form the RREG command
    tx_buffer[0] = 0x20 | reg;      // RREG command with register address
    tx_buffer[1] = size - 1;        // Number of registers to read minus one

    // Fill in dummy bytes to clock out the data
    memset(&tx_buffer[2], 0x00, size);


    struct spi_buf tx_spi_buf = {.buf = tx_buffer, .len = total_length};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buffer, .len = total_length};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count = 1};

    const struct ads1299_config *cfg = dev->config;

    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }

    // Extract the received data starting after the command bytes
    memcpy(data, &rx_buffer[2], size);


    return 0;
}

static int _ads1299_check_id(const struct device *dev, uint8_t *chip_id)
{
    int err;
    err = ads1299_read_reg(dev, 0x01, chip_id, 1);
    if (err < 0) {
        LOG_ERR("Error reading chip ID\n");
        return err;
    }
    LOG_INF("Chip ID: 0x%02x\n", *chip_id);
    return 0;
}

static int _ads1299_reset(const struct device *dev)
{
    k_msleep(100); // Small delay to ensure command is processed
    int err;
    const struct ads1299_config *cfg = dev->config;
    err = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
    err = ads1299_send_command(dev, 0x06);
    if (err < 0) {
        LOG_ERR("Error sending RESET command\n");
        return err;
    }
    k_msleep(100); // Small delay to ensure command is processedf
    return 0;
}

static int _ads1299_start(const struct device *dev)
{
    int err;
    err = ads1299_send_command(dev, 0x08);
    if (err < 0) {
        LOG_ERR("Error sending START command\n");
        return err;
    }
    // LOG_INF("Sent start command\n");
    return 0;
}

static int _ads1299_stop(const struct device *dev)
{
    int err;
    err = ads1299_send_command(dev, 0x0A);
    if (err < 0) {
        LOG_ERR("Error sending STOP command\n");
        return err;
    }
    return 0;
}
static int _ads1299_read_data(const struct device *dev, uint8_t *read_buffer, uint8_t read_size)
{
    struct ads1299_data *data = dev->data;
    const struct ads1299_config *cfg = dev->config;
    int err;

    /* Wait for the semaphore to be released by the DRDY interrupt */
    k_sem_take(&data->drdy_sem, K_FOREVER);

    /* Allocate a buffer of dummy bytes to send */
    uint8_t dummy_buffer[read_size];
    memset(dummy_buffer, 0x00, read_size);

    struct spi_buf tx_spi_buf = {
        .buf = dummy_buffer, // Send dummy bytes
        .len = read_size      // Length equal to data to read
    };
    struct spi_buf_set tx_spi = {
        .buffers = &tx_spi_buf,
        .count = 1
    };
    struct spi_buf rx_spi_buf = {
        .buf = read_buffer,
        .len = read_size
    };
    struct spi_buf_set rx_spi = {
        .buffers = &rx_spi_buf,
        .count = 1
    };

    /* Perform SPI transaction to read data */
    err = spi_transceive_dt(&cfg->spi, &tx_spi, &rx_spi);
    if (err < 0) {
        LOG_ERR("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }

    // printk("Reading data!\n");
    return 0;
}
// static int _ads1299_read_data(const struct device *dev, uint8_t *read_buffer, uint8_t read_size)
// {
//     struct ads1299_data *data = dev->data;
//     const struct ads1299_config *cfg = dev->config;
//     int err;

//     /* Wait for the semaphore to be released by the DRDY interrupt */
//     k_sem_take(&data->drdy_sem, K_FOREVER);

//     /* Perform a direct SPI read of the data frame */
//     struct spi_buf tx_spi_buf = {
//         .buf = NULL, // No data to send; SPI read
//         .len = 0      // No transmission
//     };
//     struct spi_buf_set tx_spi = {
//         .buffers = &tx_spi_buf,
//         .count = 1
//     };
//     struct spi_buf rx_spi_buf = {
//         .buf = read_buffer,
//         .len = read_size
//     };
//     struct spi_buf_set rx_spi = {
//         .buffers = &rx_spi_buf,
//         .count = 1
//     };

//     /* Perform SPI transaction to read data */
//     err = spi_transceive_dt(&cfg->spi, &tx_spi, &rx_spi);
//     if (err < 0) {
//         printk("spi_transceive_dt() failed, err: %d\n", err);
//         return err;
//     }

//     printk("Reading data!\n");
//     return 0;
// }

static int _ads1299_wakeup(const struct device *dev)
{
    int err;
    LOG_INF("Waking up ADS1299");
    err = ads1299_send_command(dev, 0x02);
    if (err < 0) {
        LOG_ERR("Error sending WAKEUP command\n");
        return err;
    }

    return 0;
}

static int _ads1299_standby(const struct device *dev)
{
    int err;
    err = ads1299_send_command(dev, 0x04);
    if (err < 0) {
        LOG_ERR("Error sending STANDBY command\n");
        return err;
    }
    return 0;

}

static int turn_on_electrodes(const struct device *dev)
{

    int err = 0;
    for(int i = 0; i < 8; i++)
    {   
        uint8_t reg = 0x05 + i;
        uint8_t data = 0x00;

        // err = ads1299_read_reg(dev, reg, &data, 1);
        // LOG_ERR("Data first read from register 0x%02x: 0x%02x\n", reg, data);

        data &= ~0x07; // Clear bits [0:2]
        err = ads1299_write_register(dev, reg, data);
        if (err < 0) {
            LOG_ERR("Error writing register\n");
            return err;
        }


        // err = ads1299_read_reg(dev, reg, &data, 1);
        // LOG_ERR("Data after read from register 0x%02x: 0x%02x\n", reg, data);


    }
    return 0;
}



static void drdy_interrupt_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct ads1299_data *data = CONTAINER_OF(cb, struct ads1299_data, drdy_cb);
    // printk("DRDY interrupt triggered\n");
    k_sem_give(&data->drdy_sem); // Release the semaphore
}

static int config_sample_rate(const struct device *dev)
{
    int err;
    uint8_t data = 0x00;
    err = ads1299_read_reg(dev, 0x01, &data, 1);
    if(CONFIG_ADS1299_SAMPLE_RATE == 250){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x06;  // Set bits [2:0] to 110
    }else if(CONFIG_ADS1299_SAMPLE_RATE == 500){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x05;  // Set bits [2:0] to 101
    }else if(CONFIG_ADS1299_SAMPLE_RATE == 1000){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x04;  // Set bits [2:0] to 100
    }else if(CONFIG_ADS1299_SAMPLE_RATE == 2000){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x03;  // Set bits [2:0] to 011
    }else if(CONFIG_ADS1299_SAMPLE_RATE == 4000){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x02;  // Set bits [2:0] to 010
    }else if(CONFIG_ADS1299_SAMPLE_RATE == 8000){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x01;  // Set bits [2:0] to 001
    }else if(CONFIG_ADS1299_SAMPLE_RATE == 16000){
        data &= ~0x07; // Clear bits [2:0]
        data |= 0x00;  // Set bits [2:0] to 000
    }else{
        return -1;
    }                                                                
    err = ads1299_write_register(dev, 0x01, data);
    if (err < 0) {
        LOG_ERR("Error writing register\n");
        return err;
    }
    return 0;
}

static int _configure_bias(const struct device *dev)
{
    const struct ads1299_config *cfg = dev->config;
    int err;

    // Set the bias sense positive and negative
    err = ads1299_write_register(dev, REG_BIAS_SENSP, 0x00);
    if (err < 0) {
        LOG_ERR("Error writing register\n");
        return err;
    }
    err = ads1299_write_register(dev, REG_BIAS_SENSN, 0xFF);

    if (err < 0) {
        LOG_ERR("Error writing register\n");
        return err;
    }

    return 0;
}
static int init_gpios(const struct device *dev)
{
    struct ads1299_data *data = dev->data;
    const struct ads1299_config *cfg = dev->config;
    int err;

    /* Ensure all GPIO devices are ready */
    if (!device_is_ready(cfg->drdy.port)) {
        LOG_ERR("DRDY GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->pwdn.port)) {
        LOG_ERR("PWDN GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->reset.port)) {
        LOG_ERR("RESET GPIO device not ready\n");
        return -ENODEV;
    }
    if (!device_is_ready(cfg->start.port)) {
        LOG_ERR("START GPIO device not ready\n");
        return -ENODEV;
    }

    /* Configure DRDY pin as input with pull-up and interrupt on falling edge */
    err = gpio_pin_configure_dt(&cfg->drdy, GPIO_INPUT | GPIO_PULL_UP);
    if (err < 0) {
        LOG_ERR("Error configuring DRDY pin\n");
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&cfg->drdy, GPIO_INT_EDGE_FALLING);
    if (err < 0) {
        LOG_ERR("Error configuring DRDY interrupt\n");
        return err;
    }

    /* Initialize the GPIO callback */
    gpio_init_callback(&data->drdy_cb, drdy_interrupt_callback, BIT(cfg->drdy.pin));
    err = gpio_add_callback(cfg->drdy.port, &data->drdy_cb);
    if (err < 0) {
        LOG_ERR("Error adding DRDY callback\n");
        return err;
    }

    /* Configure PWDN pin as output and set it inactive (high) */
    err = gpio_pin_configure_dt(&cfg->pwdn, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        LOG_ERR("Error configuring PWDN pin\n");
        return err;
    }

    /* Configure RESET pin as output and set it inactive (high) */
    err = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        LOG_ERR("Error configuring RESET pin\n");
        return err;
    }

    /* Configure START pin as output and set it inactive (low) initially */
    err = gpio_pin_configure_dt(&cfg->start, GPIO_OUTPUT_INACTIVE);
    if (err < 0) {
        LOG_ERR("Error configuring START pin\n");
        return err;
    }

    // LOG_INF("GPIOs initialized successfully\n");
    return 0;
}

static int init_ads1299(const struct device *dev)
{
    int err;
    struct ads1299_data *data = dev->data;
    const struct ads1299_config *cfg = dev->config;

    /* Initialize GPIOs */
    err = init_gpios(dev);
    if (err < 0) {
        LOG_ERR("Error initializing GPIOs\n");
        return err;
    }

    err = _ads1299_reset(dev);
    if(err < 0)
    {
        LOG_ERR("Error resetting device\n");
        return err;
    }

    /* Initialize the semaphore with an initial count of 0 and a maximum count of 1 */
    k_sem_init(&data->drdy_sem, 0, 1);

    k_msleep(100); // Allow time for the device to power up

    /* The PWDN and RESET pins are already set to inactive (high) during GPIO initialization */

    /* Send SDATAC command to stop continuous data mode */
    err = ads1299_send_command(dev, 0x11); // SDATAC command
    if (err < 0) {
        LOG_ERR("Error sending SDATAC command\n");
        return err;
    }
    k_msleep(1); // Small delay to ensure command is processed

    /* Send WAKEUP command if the device is in standby */
    err = ads1299_send_command(dev, 0x02); // WAKEUP command
    if (err < 0) {
        LOG_ERR("Error sending WAKEUP command\n");
        return err;
    }
    k_msleep(1);

    /* Read Device ID to verify communication */
    uint8_t chip_id = 0x00;
    err = _ads1299_check_id(dev, &chip_id);
    if (err < 0) {
        LOG_ERR("Error checking device ID\n");
        return err;
    }


    // LOG_INF("ADS1299 driver initialized successfully\n");

    uint8_t srs_data = 0x00;
    err = ads1299_read_reg(dev, 0x01, &srs_data, 1);
    printk("Data read from register 0x01: 0x%02x\n", srs_data);
    // Reset device



    // check 0x05 - 0x0C register after reset
    // for (int i = 0x05; i <= 0x0C; i++)
    // {
    //     uint8_t data = 0x00;
    //     err = ads1299_read_reg(dev, i, &data, 1);
    //     LOG_INF("Data read from register 0x%02x: 0x%02x\n", i, data);
    // }

    
    uint8_t config3 = 0x00;
    err = ads1299_read_reg(dev, 0x03, &config3, 1);
    config3 |= BIT(7); // Set BIT7 to enable internal reference
    config3 |= BIT(3); // Use interal bias ref.
    config3 |= BIT(4); // Set BIT4 to enable bias measurement
    config3 |= BIT(2); // Set BIT2 to enable bias measurement
    err = ads1299_write_register(dev, 0x03, config3);
    turn_on_electrodes(dev);

    _configure_bias(dev);

    // Enable SRB1 reference
    uint8_t misc1_reg = 0x15;
    uint8_t misc1_data = 0x00 | BIT(5);
    err = ads1299_write_register(dev, misc1_reg, misc1_data);
    if (err < 0) {
        printk("Error writing register\n");
        return err;
    }

    // Set sample rate
    // err = config_sample_rate(dev);


    /* Send RDATAC command to start continuous data mode */
    err = ads1299_send_command(dev, 0x10); // RDATAC command
    if (err < 0) {
        printk("Error sending RDATAC command\n");
        return err;
    }
    k_msleep(1); // Small delay to ensure command is processed

    /* Start data conversions by setting the START pin high */
    err = gpio_pin_set_dt(&cfg->start, 1);
    if (err < 0) {
        printk("Error setting START pin high\n");
        return err;
    }

    printk("ADS1299 started data conversions\n");

    // Everything is ready, put to standby
    err = _ads1299_standby(dev);
    return 0;
}



static const struct ads1299_driver_api_funcs ads1299_api = {
    .check_id = _ads1299_check_id,
    .start = _ads1299_start,
    .stop = _ads1299_stop,
    .wakeup = _ads1299_wakeup,
    .standby = _ads1299_standby,
    .reset = _ads1299_reset,
    .read_data = _ads1299_read_data,
};




#define ADS1299_CONFIG_SPI(inst) \
{ \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0), \
    .drdy = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios), \
    .pwdn = GPIO_DT_SPEC_INST_GET(inst, pwdn_gpios), \
    .reset = GPIO_DT_SPEC_INST_GET(inst, reset_gpios), \
    .start = GPIO_DT_SPEC_INST_GET(inst, start_gpios), \
}

#define INST_DRDY_GPIO_SPEC(idx)                              \
    static const struct gpio_dt_spec drdy_##idx =          \
        GPIO_DT_SPEC_GET(DT_DRV_INST(idx), drdy_gpios);

#define ADS1299_DEFINE(inst)                                    \
    static struct ads1299_data ads1299_data_##inst;            \
    static const struct ads1299_config ads1299_config_##inst = \
        ADS1299_CONFIG_SPI(inst);                              \
    INST_DRDY_GPIO_SPEC(inst)                                  \
    DEVICE_DT_INST_DEFINE(inst,                                \
                init_ads1299,                                   \
                NULL,                                           \
                &ads1299_data_##inst,                          \
                &ads1299_config_##inst,                        \
                POST_KERNEL,                                    \
                CONFIG_MAX30003_INIT_PRIORITY,                  \
                &ads1299_api);

/* Instantiate the driver for each device tree node with status "okay" */
DT_INST_FOREACH_STATUS_OKAY(ADS1299_DEFINE)