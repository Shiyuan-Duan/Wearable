// max30001.c

#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>



#include <ncs_version.h>
#if NCS_VERSION_NUMBER >= 0x20600
#include <zephyr/internal/syscall_handler.h>
#else
#include <zephyr/syscall_handler.h>
#endif


#include "max30001.h"


#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')
#define SPIOP	SPI_WORD_SET(8)

#define WRITE_BIT 0x00
#define READ_BIT 0x01


LOG_MODULE_REGISTER(MAX30001, LOG_LEVEL_DBG);

static int _check_id(const struct device *dev)
{
    int err;
    uint8_t INFO_REG = 0x0F;

    uint8_t tx_buffer[4];
    tx_buffer[0] = (INFO_REG << 1) | READ_BIT;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    uint8_t rx_buffer[4];

    struct spi_buf tx_spi_buf = {
        .buf = (void *)&tx_buffer,
        .len = 4
    };
    struct spi_buf rx_spi_buf = {
        .buf = rx_buffer,
        .len = 4
    };
    struct spi_buf_set tx_spi = {
        .buffers = &tx_spi_buf,
        .count = 1
    };
    struct spi_buf_set rx_spi = {
        .buffers = &rx_spi_buf,
        .count = 1
    };

    const struct max30001_config *cfg = dev->config;

    err = spi_transceive_dt(&cfg->spi, &tx_spi, &rx_spi);

    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }
    for (int i = 0; i < 4; i++)
    {
        LOG_INF("Data read from register 0x%02x: 0x%02x\n", INFO_REG, rx_buffer[i]);
    }
    


    return 0;
}

static int _read_status(const struct device *dev, uint8_t *data)
{
    int err;
    const struct max30001_config *cfg = dev->config;
    uint8_t tx_buffer[4];
    tx_buffer[0] = (0x01 << 1) | READ_BIT;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;

    uint8_t rx_buffer[4];

    struct spi_buf tx_spi_buf = {
        .buf = (void *)&tx_buffer,
        .len = 4
    };
    struct spi_buf rx_spi_buf = {
        .buf = rx_buffer,
        .len = 4
    };
    struct spi_buf_set tx_spi = {
        .buffers = &tx_spi_buf,
        .count = 1
    };
    struct spi_buf_set rx_spi = {
        .buffers = &rx_spi_buf,
        .count = 1
    };

    err = spi_transceive_dt(&cfg->spi, &tx_spi, &rx_spi);

    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }

    for (int i = 0; i < 4; i++)
    {
        data[i] = rx_buffer[i];
    }

    return 0;

}

static void ecg_fifo_cb(const struct device *port, 
                        struct gpio_callback *cb, 
                        gpio_port_pins_t pins)
{
    struct max30001_data *data = CONTAINER_OF(cb, struct max30001_data, inta_cb);
    k_sem_give(&data->inta_sem);
    LOG_INF("ECG FIFO callback triggered!");
}

static void bioz_fifo_cb(const struct device *port, 
                         struct gpio_callback *cb, 
                         gpio_port_pins_t pins)
{
    struct max30001_data *data = CONTAINER_OF(cb, struct max30001_data, intb_cb);
    k_sem_give(&data->intb_sem);
    LOG_INF("BIOZ FIFO callback triggered!");
}

static int _init_gpios(const struct device *dev)
{
    const struct max30001_config *cfg = dev->config;
    const struct max30001_data *data = dev->data;
    int err;

    /* Configure DRDY pin as input with pull-up and interrupt on falling edge */
    err = gpio_pin_configure_dt(&cfg->inta, GPIO_INPUT | GPIO_PULL_UP);
    if(err < 0)
    {
        printk("Error configuring interrupt pin\n");
        return err;
    }
    err = gpio_pin_configure_dt(&cfg->intb, GPIO_INPUT | GPIO_PULL_UP);
    if (err < 0)
    {
        printk("Error configuring interrupt pin\n");
        return err;
    }
    int val = gpio_pin_get_dt(&cfg->inta);
    LOG_INF("INTA pin read after config: %d", val);
    if (err < 0) {
        LOG_ERR("Error configuring interrupt pin\n");
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&cfg->inta, GPIO_INT_EDGE_FALLING);
    err = gpio_pin_interrupt_configure_dt(&cfg->intb, GPIO_INT_EDGE_FALLING);
    if (err < 0) {
        LOG_ERR("Error configuring interrupt interrupt\n");
        return err;
    }

    /* Initialize the GPIO callback */
    gpio_init_callback(&data->inta_cb, ecg_fifo_cb, BIT(cfg->inta.pin));
    gpio_init_callback(&data->intb_cb, bioz_fifo_cb, BIT(cfg->intb.pin));
    err = gpio_add_callback(cfg->inta.port, &data->inta_cb);
    err = gpio_add_callback(cfg->intb.port, &data->intb_cb);
    printk("Added interrupt callback\n");
    printk("INTA PIN: %d\n", cfg->inta.pin);
    printk("INTB PIN: %d\n", cfg->intb.pin);

    printk("INTA PORT: %p\n", cfg->inta.port);
    printk("INTB PORT: %p\n", cfg->intb.port);
    if (err < 0) {
        LOG_ERR("Error adding interrupt callback\n");
        return err;
    }

    return 0;
}
static int _reset_device(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;
    uint8_t tx_buffer[4];
    tx_buffer[0] = (0x08 << 1) | WRITE_BIT;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;


    struct spi_buf tx_spi_buf = {
        .buf = (void *)&tx_buffer,
        .len = 4
    };
    struct spi_buf_set tx_spi = {
        .buffers = &tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;
}

static int _resync_device(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;
    uint8_t tx_buffer[4];
    tx_buffer[0] = (0x09 << 1) | WRITE_BIT;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;

    struct spi_buf tx_spi_buf = {
        .buf = (void *)&tx_buffer,
        .len = 4
    };
    struct spi_buf_set tx_spi = {
        .buffers = &tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;
}

static int _reset_fifo(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;
    uint8_t tx_buffer[4];
    tx_buffer[0] = (0x0A << 1) | WRITE_BIT;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;

    struct spi_buf tx_spi_buf = {
        .buf = (void *)&tx_buffer,
        .len = 4
    };
    struct spi_buf_set tx_spi = {
        .buffers = &tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;
}

static int _config_general(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;

    // Configure General
    uint8_t cnfg_gen_tx_buf[4];
    uint8_t cnfg_gen_rx_buf[4];
    cnfg_gen_tx_buf[0] = (0x10 << 1) | READ_BIT;
    cnfg_gen_tx_buf[1] = 0x00;
    cnfg_gen_tx_buf[2] = 0x00;
    cnfg_gen_tx_buf[3] = 0x00;

    struct spi_buf cnfg_gen_tx_spi_buf = {
        .buf = (void *)&cnfg_gen_tx_buf,
        .len = 4
    };
    struct spi_buf cnfg_gen_rx_spi_buf = {
        .buf = (void *)&cnfg_gen_rx_buf,
        .len = 4
    };
    struct spi_buf_set cnfg_gen_tx_spi = {
        .buffers = &cnfg_gen_tx_spi_buf,
        .count = 1
    };
    struct spi_buf_set cnfg_gen_rx_spi = {
        .buffers = &cnfg_gen_rx_spi_buf,
        .count = 1
    };

    err = spi_transceive_dt(&cfg->spi, &cnfg_gen_tx_spi, &cnfg_gen_rx_spi);

    if (err < 0)
    {
        printk("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }
    uint8_t cnfg_gen_default[3];
    memcpy(cnfg_gen_default, &cnfg_gen_rx_buf[1], 3);


    cnfg_gen_tx_buf[0] = (0x10 << 1) | WRITE_BIT;
    cnfg_gen_tx_buf[1] = (cnfg_gen_default[0] | BIT(3))| BIT(2);
    cnfg_gen_tx_buf[2] = cnfg_gen_default[1];
    cnfg_gen_tx_buf[3] = cnfg_gen_default[2];



    err = spi_write_dt(&cfg->spi, &cnfg_gen_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;
}

static int _enable_ecg(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;
    const struct max30001_data *data = dev->data;
    // Enable ECG
    uint8_t cnfg_mux_tx_buf[4];
    cnfg_mux_tx_buf[0] = (0x14 << 1) | WRITE_BIT;
    cnfg_mux_tx_buf[1] = 0x00;
    cnfg_mux_tx_buf[2] = 0x00;
    cnfg_mux_tx_buf[3] = 0x00;

    struct spi_buf cnfg_mux_tx_spi_buf = {
        .buf = (void *)&cnfg_mux_tx_buf,
        .len = 4
    };

    struct spi_buf_set cnfg_mux_tx_spi = {
        .buffers = &cnfg_mux_tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &cnfg_mux_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    uint8_t cnfg_mux_rx_buf[4];
    cnfg_mux_rx_buf[0] = (0x14 << 1) | READ_BIT;
    cnfg_mux_rx_buf[1] = 0x00;
    cnfg_mux_rx_buf[2] = 0x00;
    cnfg_mux_rx_buf[3] = 0x00;

    struct spi_buf cnfg_mux_rx_spi_buf = {
        .buf = (void *)&cnfg_mux_rx_buf,
        .len = 4
    };

    struct spi_buf_set cnfg_mux_rx_spi = {
        .buffers = &cnfg_mux_rx_spi_buf,
        .count = 1
    };

    err = spi_transceive_dt(&cfg->spi, &cnfg_mux_tx_spi, &cnfg_mux_rx_spi);

    if (err < 0)
    {
        printk("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }   

    for(int i = 0; i < 4; i++)
    {
        printk("CNFG_MUX register %d: 0x%02x\n", i, cnfg_mux_rx_buf[i]);
    }


    return 0;
}

static int _config_ecg(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;

    uint8_t cnfg_ecg_tx_buf[4];
    cnfg_ecg_tx_buf[0] = (0x15 << 1) | WRITE_BIT;
    cnfg_ecg_tx_buf[1] = 0x01; // 0b00000001 512 Hz, 40 Gain
    cnfg_ecg_tx_buf[2] = 0x50; // 0b01010000 0.5 Hz HPF, 40 Hz LPF
    cnfg_ecg_tx_buf[3] = 0x00;

    struct spi_buf cnfg_ecg_tx_spi_buf = {
        .buf = (void *)&cnfg_ecg_tx_buf,
        .len = 4
    };

    struct spi_buf_set cnfg_ecg_tx_spi = {
        .buffers = &cnfg_ecg_tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &cnfg_ecg_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;
}

static int _enable_bioz(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;

    uint8_t cnfg_bioz_tx_buf[4];
    cnfg_bioz_tx_buf[0] = (0x17 << 1) | WRITE_BIT;
    cnfg_bioz_tx_buf[1] = 0x00;
    cnfg_bioz_tx_buf[2] = 0x00;
    cnfg_bioz_tx_buf[3] = 0x40;

    struct spi_buf cnfg_bioz_tx_spi_buf = {
        .buf = (void *)&cnfg_bioz_tx_buf,
        .len = 4
    };

    struct spi_buf_set cnfg_bioz_tx_spi = {
        .buffers = &cnfg_bioz_tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &cnfg_bioz_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;


}

static int _config_bioz(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;

    uint8_t cnfg_bioz_tx_buf[4];
    cnfg_bioz_tx_buf[0] = (0x18 << 1) | WRITE_BIT;
    cnfg_bioz_tx_buf[1] = 0x25; // 0b00100101 64 sps, 800 Hz HPF, 20 Gain
    cnfg_bioz_tx_buf[2] = 0x18;
    cnfg_bioz_tx_buf[3] = 0x10; 

    struct spi_buf cnfg_bioz_tx_spi_buf = {
        .buf = (void *)&cnfg_bioz_tx_buf,
        .len = 4
    };

    struct spi_buf_set cnfg_bioz_tx_spi = {
        .buffers = &cnfg_bioz_tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &cnfg_bioz_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    
    return 0;
}
static int _read_efifo(const struct device *dev, uint8_t *data)
{
    int err;
    
    const struct max30001_config *cfg = dev->config;
    struct max30001_data *dev_data = dev->data;

    k_sem_take(&dev_data->inta_sem, K_FOREVER);
    // For now just reset fifo
    _reset_fifo(dev);

    return 0;
}

static int _read_bfifo(const struct device *dev, uint8_t *data)
{
    int err;
    const struct max30001_config *cfg = dev->config;
    struct max30001_data *dev_data = dev->data;

    k_sem_take(&dev_data->intb_sem, K_FOREVER);
    // For now just reset fifo
    _reset_fifo(dev);

    return 0;
}
static int _config_interrupts(const struct device *dev)
{
    int err;
    const struct max30001_config *cfg = dev->config;

    // Enabling Interrupts
    err = _init_gpios(dev);

    uint8_t en_int_tx_buf[4];
    en_int_tx_buf[0] = (0x02 << 1) | WRITE_BIT;
    en_int_tx_buf[1] = 0x00 | BIT(7);
    en_int_tx_buf[2] = 0x00;
    en_int_tx_buf[3] = 0x03;

    struct spi_buf en_int_tx_spi_buf = {
        .buf = (void *)&en_int_tx_buf,
        .len = 4
    };
    struct spi_buf_set en_int_tx_spi = {
        .buffers = &en_int_tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &en_int_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    uint8_t en_intb_tx_buf[4];
    en_intb_tx_buf[0] = (0x03 << 1) | WRITE_BIT;
    en_intb_tx_buf[1] = 0x00 | BIT(3);
    en_intb_tx_buf[2] = 0x00;
    en_intb_tx_buf[3] = 0x03;

    struct spi_buf en_intb_tx_spi_buf = {
        .buf = (void *)&en_intb_tx_buf,
        .len = 4
    };
    struct spi_buf_set en_intb_tx_spi = {
        .buffers = &en_intb_tx_spi_buf,
        .count = 1
    };

    err = spi_write_dt(&cfg->spi, &en_intb_tx_spi);

    if (err < 0)
    {
        printk("spi_write_dt() failed, err: %d\n", err);
        return err;
    }

    // Manage Interrupts
    // Default ECG FIFO water mark: 16
    // Default BIOZ FIFO water mark: 4
    // Good!

    return 0;





}
void init_max30001(const struct device *dev)
{
    printk("Initializing MAX30001\n");
    const struct max30001_config *cfg = dev->config;
    struct max30001_data *data = dev->data;
    data->chip_id = 0;

    // Configure Status

    _read_status(dev, data->status);
    for (int i = 0; i < 3; i++)
    {
        printk("Status register %d: 0x%02x\n", i, data->status[i]);
    }
    _config_interrupts(dev);
    _config_general(dev);

    _config_ecg(dev);
    _enable_ecg(dev);

    _config_bioz(dev);
    _enable_bioz(dev);

    k_sem_init(&data->inta_sem, 0, 1);
    k_sem_init(&data->intb_sem, 0, 1);
    //
    _check_id(dev);
    printk("MAX30001 initialized\n");
}


static const struct max30001_driver_api_funcs max30001_api = {
    .check_id = _check_id,
    .read_status = _read_status,
    .reset_device = _reset_device,
    .resync_device = _resync_device,
    .reset_fifo = _reset_fifo,
    .read_efifo = _read_efifo,
    .read_bfifo = _read_bfifo,
};



#define MAX30001_CONFIG_SPI(inst) \
{ \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0), \
    .inta = GPIO_DT_SPEC_INST_GET(inst, int1_gpios), \
    .intb = GPIO_DT_SPEC_INST_GET(inst, int2_gpios), \
}

#define MAX30001_DEFINE(inst)						\
    static struct max30001_data max30001_data_##inst;			\
    static const struct max30001_config max30001_config_##inst = MAX30001_CONFIG_SPI(inst);	\
    DEVICE_DT_INST_DEFINE(inst,			\
                init_max30001,							\
                NULL,							\
                &max30001_data_##inst,	\
                &max30001_config_##inst,\
                POST_KERNEL, \
                CONFIG_MAX30001_INIT_PRIORITY, \
                &max30001_api);


DT_INST_FOREACH_STATUS_OKAY(MAX30001_DEFINE)


