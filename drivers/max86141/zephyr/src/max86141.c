//MAX86141.c

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

#include "max86141.h"


#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')
#define SPIOP	SPI_WORD_SET(8)


LOG_MODULE_REGISTER(MAX86141, LOG_LEVEL_DBG);


struct max86141_config
{
    struct spi_dt_spec spi;
};



static int max86141_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, int size)
{
    int err;
    uint8_t tx_buffer[3]; 
    
    tx_buffer[0] = reg;
    tx_buffer[1] = 0x80;
    tx_buffer[2] = 0x00;




    struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = sizeof(tx_buffer)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = data, .len = size};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count=1};

    const struct max86141_config *cfg = dev->config;
    
    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d\n", err);
        return err;
    }

    return 0;

}



static int max86141_write_reg(const struct device *dev, uint8_t reg, uint8_t *values, size_t size)
{
    int err;
    const struct max86141_config *cfg = dev->config;
    

    /* Bit7 is 0 for the write command */
    uint8_t tx_buf[size+2];
    tx_buf[0] = reg;
    tx_buf[1] = 0x00;
    memcpy(&tx_buf[2], values, size);

    struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
    if (err < 0) {
        LOG_ERR("spi_write_dt() failed, err %d\n", err);
        return err;
    }

    return 0;
}

static int read_interrupt_status(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, INT_STATUS1, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading interrupt status 1\n");
        return err;
    }
    LOG_INF("Status of interrupt 1:\n");
    LOG_INF("data[0]:0x%x, data[1]:0x%x, data[2]:0x%x", data[0], data[1], data[2]);

    err = max86141_read_reg(dev, INT_STATUS2, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading interrupt status 2\n");
        return err;
    }
    LOG_INF("Status of interrupt 2:\n");
    LOG_INF("data[0]:0x%x, data[1]:0x%x, data[2]:0x%x", data[0], data[1], data[2]);
    return 0;
}
static int interrupt1_enable(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, INT1_EN, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading int1 en\n");
        return err;
    }
    data[2] |= BIT(7); // enable A_FULL interrupt
    LOG_INF("Enabling interrupt 1\n");


    err = max86141_write_reg(dev, INT1_EN, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to int1 en\n");
    }
    return err;

}

static int read_overflow_counter(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, OVERFLOW_COUNTER, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading over flow counter\n");
    }

    LOG_INF("Over flow count: %x\n", data[2]);
    return 0;
}

static int read_fifo_data_counter(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, FIFO_DATA_COUNTER, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading fifo data counter\n");
    }

    LOG_INF("fifo data count: %d\n", data[2]);
    return 0;
}

static int config_fifo1(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, FIFO_CONFIG1, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading fifo config1\n");
    }

    LOG_INF("fifo config 1 : %x\n", data[2]);
    // TODO: Config fifo
    data[2] = 0x60; // 32 entries

    err = max86141_write_reg(dev, FIFO_CONFIG1, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to fifo config1\n");
    }



    // Start config fifo 2


    uint8_t fifo2_data[3];
    err = max86141_read_reg(dev, FIFO_CONFIG2, fifo2_data, sizeof(fifo2_data));

    if (err < 0){
        LOG_ERR("Error reading fifo config2\n");
    }

    fifo2_data[2] = fifo2_data[2] | BIT(3); // SET FIFO_STAT_CLR to 1, clear A_FULL interrupt on fifo read;
    // fifo2_data[2] = fifo2_data[2] | BIT(2); // A_FULL triggers once 
    return 0;
}

static int flush_fifo(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, FIFO_CONFIG2, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading fifo config2\n");
    }



    data[2] = data[2] | BIT(4); // flush, self clearing bit


    err = max86141_write_reg(dev, FIFO_CONFIG2, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to fifo config2\n");
    }
    return 0;
}
static int max86141_reset(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, SYS_CTRL, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading sys ctrl\n");
    }



    data[2] = data[2] | BIT(0); // set 0th bit to 1 


    err = max86141_write_reg(dev, SYS_CTRL, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to resetting max86141\n");
    }
    return 0;
}

static int ppg_sync_ctrl(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, PPG_SYNC_CTRL, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading PPG_SYNC_CTRL\n");
    }



    // data[2] = data[2] | BIT(0); // gpio ctrl = 0001 refer to datasheet pg. 66 https://www.analog.com/media/en/technical-documentation/data-sheets/max86140-max86141.pdf



    err = max86141_write_reg(dev, PPG_SYNC_CTRL, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to PPG_SYNC_CTRL\n");
    }
    return 0;
}
static int shutdown_max86141(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, SYS_CTRL, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading sys ctrl\n");
    }
    data[2] = data[2] | BIT(1); // set 1st bit to 1 
    err = max86141_write_reg(dev, SYS_CTRL, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to shutting down max86141\n");
    }
    return 0;
}

static int turn_on_max86141(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, SYS_CTRL, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading sys ctrl\n");
    }
    data[2] = data[2] & ~BIT(1); // set 1st bit to 0 
    err = max86141_write_reg(dev, SYS_CTRL, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to turning on max86141\n");
    }
    return 0;
}

static int ppg_config(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, PPG_CONFIG1, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading PPG_CONFIG1\n");
    }

    // Modify data here to ppg config 1
    // data[2] = (data[2] | BIT(5) | BIT(4) | BIT(3) | BIT(2)); // ADC Full scale range 32768
    data[2] = (data[2] | BIT(5) | BIT(3)); // ADC Full scale range 16384


    data[2] = (data[2] & ~(BIT(0) | BIT(1))); // ppg_int = 14.8 uS

    err = max86141_write_reg(dev, PPG_CONFIG1, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to PPG_CONFIG1\n");
    }

    uint8_t config2_data[3];
    err = max86141_read_reg(dev, PPG_CONFIG2, config2_data, sizeof(config2_data));

    if (err < 0){
        LOG_ERR("Error reading PPG_CONFIG2\n");
    }

    // Modify data here to ppg config 2
    config2_data[2] = (config2_data[2] | BIT(0) | BIT(1) | BIT(2    )); // Take average of 128 samples
    // config2_data[2] = ((config2_data[2] | BIT(1)) | BIT(2)); // Take average of 64 samples
    // config2_data[2] = (config2_data[2] | BIT(1)); // Take average of 4 samples

    // config2_data[2] = (config2_data[2] | BIT(3)) | BIT(6); // SR 100Hz, 2 pulses persample

    err = max86141_write_reg(dev, PPG_CONFIG2, &config2_data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to PPG_CONFIG2\n");
    }

    uint8_t config3_data[3];
    err = max86141_read_reg(dev, PPG_CONFIG3, config3_data, sizeof(config3_data));
    if (err < 0){
        LOG_ERR("Error reading PPG_CONFIG3\n");
    }
    // set config3 register here

    config3_data[2] = (config3_data[2] | BIT(0)); // Enable burst
    // config3_data[2] = (config3_data[2] | BIT(1) | BIT(2 )); // Burst rate = 256Hz
    config3_data[2] = (config3_data[2] | BIT(1)); // Burst rate = 84Hz

    err = max86141_write_reg(dev, PPG_CONFIG3, &config3_data[2], 1);

    if (err < 0){
        LOG_ERR("Error writing to PPG_CONFIG3\n");  
    }



    return 0;

}

static int config_pd_bias(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, PD_BIAS, data, sizeof(data));

    if (err < 0){
        LOG_ERR("Error reading PD_BIAS\n");
    }

    // set pd bias to 0-65pF
    data[2] = (data[2] | BIT(0)) | BIT(4);



    err = max86141_write_reg(dev, PD_BIAS, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to PD_BIAS\n");
    }
}

static int config_led_seq(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, LED_SEQ1, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_SEQ1\n");
    }
    // Modify data here to configure LED_SEQ1

    data[2] = data[2] | BIT(0) | BIT(5);
    err = max86141_write_reg(dev, LED_SEQ1, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_SEQ1\n");
    }
    
    err = max86141_read_reg(dev, LED_SEQ2, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_SEQ2\n");
    }
    // Modify data here to configure LED_SEQ2
    data[2] |= (BIT(7) | BIT(4) | BIT(1) | BIT(0));
    err = max86141_write_reg(dev, LED_SEQ2, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_SEQ2\n");
    }
    
    err = max86141_read_reg(dev, LED_SEQ3, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_SEQ3\n");
    }
    // Modify data here to configure LED_SEQ3
    err = max86141_write_reg(dev, LED_SEQ3, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_SEQ3\n");
    }
    
    return 0;
}

static int set_led_pa_registers(const struct device *dev)
{
    int err;
    uint8_t data[3];
    
    // Set LED_PA1 register
    err = max86141_read_reg(dev, LED_PA1, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_PA1\n");
    }
    data[2] = 0xFF; // Set all bits to 1
    err = max86141_write_reg(dev, LED_PA1, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_PA1\n");
    }
    
    // Set LED_PA2 register
    err = max86141_read_reg(dev, LED_PA2, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_PA2\n");
    }
    data[2] = 0x50; // config red led current
    err = max86141_write_reg(dev, LED_PA2, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_PA2\n");
    }
    
    // Set LED_PA3 register
    err = max86141_read_reg(dev, LED_PA3, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_PA3\n");
    }
    data[2] = 0xFF; // Set all bits to 1
    err = max86141_write_reg(dev, LED_PA3, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_PA3\n");
    }
    // Set PILOT_PA register
    err = max86141_read_reg(dev, PILOT_PA, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading PILOT_PA\n");
    }
    data[2] = 0x00;
    err = max86141_write_reg(dev, PILOT_PA, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to PILOT_PA\n");
    }
    return 0;
}
static int set_led_rge1_registers(const struct device *dev)
{
    int err;
    uint8_t data[3];
    
    // Read LED_RGE1 register
    err = max86141_read_reg(dev, LED_RGE1, data, sizeof(data));
    if (err < 0){
        LOG_ERR("Error reading LED_RGE1\n");
    }
    
    // Set bits 5,4,3,2,1,0 to 1
    // data[2] |= 0x3F; // set all leds to 124mA
    data[2] |= data[2] | BIT(0); // set led 1 to 62mA
    data[2] |= data[2] | BIT(5) | BIT(4); // set led 3 to 124mA
    data[2] |= data[2] | BIT(3) | BIT(2); // set led 2 to 124mA
    // data[2] |= data[2] | BIT()
    
    // Write back to LED_RGE1 register
    err = max86141_write_reg(dev, LED_RGE1, &data[2], 1);
    if (err < 0){
        LOG_ERR("Error writing to LED_RGE1\n");
    }
    
    return 0;
}




static int read_part_id(const struct device *dev)
{
    int err;
    uint8_t data[3];
    err = max86141_read_reg(dev, PART_ID, &data, sizeof(data));
    if (err < 0) {
        LOG_ERR("Error reading PART_ID\n");
        return err;
    }
    LOG_INF("PART_ID: 0x%x\n", data[2]);
    return 0;
}

static int read_fifo(const struct device *dev, uint8_t *buffer)
{
    int err;
    int fifo_byte_size = 128 * 3;
    uint8_t data[fifo_byte_size+2];
    err = max86141_read_reg(dev, FIFO_DATA_READ, data, sizeof(data));
    if (err < 0) {
        LOG_ERR("Error reading FIFO_DATA\n");
        return err;
    }
    memcpy(buffer, &data[2], fifo_byte_size);

    return 0;
}


static int init_max86141(const struct device *dev)
{


    LOG_INF("Initializing MAX86141\n");
    max86141_reset(dev);
    k_msleep(10);
    shutdown_max86141(dev);

    read_part_id(dev);
    read_interrupt_status(dev);
    interrupt1_enable(dev);


    
    ppg_config(dev);
    config_pd_bias(dev);
    set_led_rge1_registers(dev);
    set_led_pa_registers(dev);
    config_fifo1(dev);
    ppg_sync_ctrl(dev);
    config_led_seq(dev);

    // turn_on_max86141(dev);



    

 
    
    
    return 0;
}

static const struct max86141_driver_api_funcs max86141_api = {
    .read_status = read_interrupt_status,
    .flush_fifo = flush_fifo,
    .read_fifo = read_fifo,
    .count_fifo = read_fifo_data_counter,
    .turn_on = turn_on_max86141,
    .turn_off = shutdown_max86141

};
#define MAX86141_CONFIG_SPI(inst) {.spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0)}

#define MAX86141_DEFINE(inst)						\
    static struct max86141_data max86141_data_##inst;			\
    static const struct max86141_config max86141_config_##inst = MAX86141_CONFIG_SPI(inst);	\
    DEVICE_DT_INST_DEFINE(inst,			\
                init_max86141,							\
                NULL,							\
                &max86141_data_##inst,	\
                &max86141_config_##inst,\
                POST_KERNEL, \
                CONFIG_MAX86141_INIT_PRIORITY, \
                &max86141_api);

/* STEP 5.2 - Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(MAX86141_DEFINE)