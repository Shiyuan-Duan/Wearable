

#define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')
#define SPIOP	SPI_WORD_SET(8)

#include "w25m02gw.h"


LOG_MODULE_REGISTER(W25M02GW, LOG_LEVEL_DBG);



struct w25m02gw_config
{
    struct spi_dt_spec spi;
    struct gpio_dt_spec hold;
    struct gpio_dt_spec wp;
};

struct w25m02gw_data
{

    uint8_t chip_id[3];
    uint16_t num_of_blocks;
    uint16_t current_block; 
    uint8_t current_page;


};

void construct_page_address(uint16_t block_number, uint8_t page_number, uint8_t * address) {
    // Ensure block number and page number are within valid ranges
    if (block_number > 1023 || page_number > 63) {
        address[0] = 0xFF;
        address[1] = 0xFF;
        LOG_ERR("Invalid block or page number");
        return;
    }


    uint16_t full_address = (block_number << 6) | (page_number & 0x3F);


    address[0] = (full_address >> 8) & 0xFF;  // High byte
    address[1] = full_address & 0xFF;         // Low byte
}

static int _uint16to8(uint16_t val, uint8_t *buf)
{
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
    return 0;
}



static int w25m02gw_spi_transceive(const struct device *dev, uint8_t * tx_buf, size_t tx_size, uint8_t * rx_buf, int rx_size)
{
    int err;

    struct spi_buf tx_spi_buf = {.buf = (void *)tx_buf, .len = tx_size};
    struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
    struct spi_buf rx_spi_buf = {.buf = rx_buf, .len = rx_size};
    struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count=1};

    const struct w25m02gw_config *cfg = dev->config;
    
    err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
    if (err < 0) {
        printk("spi_transceive_dt() failed, err: %d", err);
        return err;
    }
    // for (int i = 0; i < tx_size; i++) {
    //     printk("tx Data[%d]: %x\n", i, tx_buf[i]);
    //     k_msleep(10);
    // }   
    // for (int i = 0; i < rx_size; i++) {
    //     printk("rx Data[%d]: %x\n", i, rx_buf[i]);
    //     k_msleep(10);
    // }


    return 0;

}

static int _w25m02gw_read_status_register(const struct device *dev, uint8_t reg, uint8_t *data)
{
    int err;
    uint8_t tx_buffer[2]; 
    uint8_t rx_buffer[3];

    tx_buffer[0] = 0x0F;
    tx_buffer[1] = reg;
    err = w25m02gw_spi_transceive(dev, tx_buffer, 2, rx_buffer, 3);
    if (err < 0) {
        LOG_ERR("Error reading register");
        return err;
    }
    memcpy(data, &rx_buffer[2], 1);

    return 0;
}

static int _w25m02gw_write_status_register(const struct device *dev, uint8_t reg, uint8_t data)
{
    int err;
    uint8_t tx_buffer[3]; 
    uint8_t rx_buffer[3];

    tx_buffer[0] = 0x1F;
    tx_buffer[1] = reg;
    tx_buffer[2] = data;
    err = w25m02gw_spi_transceive(dev, tx_buffer, 3, rx_buffer, 3);
    if (err < 0) {
        LOG_ERR("Error writing register");
        return err;
    }

    return 0;
}






static int init_gpios(const struct device *dev)
{
    int err;
    const struct w25m02gw_config *cfg = dev->config;


    
    err = gpio_pin_configure_dt(&cfg->hold, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Error configuring HOLD GPIO");
        return err;
    }

    err = gpio_pin_configure_dt(&cfg->wp, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("Error configuring WP GPIO");
        return err;
    }

    return 0;
}
static int _wait_until_ready(const struct device *dev)
{
    int err;
    uint8_t status_reg;

    do {
        // Read the status register, specifically Status Register-1 (address 0xC0)
        err = _w25m02gw_read_status_register(dev, 0xC0, &status_reg);
        if (err < 0) {
            LOG_ERR("Error reading status register");
            return err;
        }

        // Check the BUSY bit (bit 0); loop while BUSY bit is set (1)
    } while ((status_reg & BIT(0)) == 1);

    return 0;  // Device is ready
}

static int _reset_device(const struct device *dev)
{
    int err;

    err = _wait_until_ready(dev);
    if (err < 0) {
        LOG_ERR("Error waiting for device to be ready");
        return err;
    }

    uint8_t spi_tx_data[1];
    spi_tx_data[0] = 0xff;
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 1, NULL, 0);
    if (err < 0) {
        LOG_ERR("Error sending reset command");
        return err;
    }



    return 0;
}






static int _write_enable(const struct device *dev)
{
    int err;
    // pull gpio inactive
    const struct w25m02gw_config *cfg = dev->config;

    // Step 1: Pull the /WP pin high to allow writes
    err = gpio_pin_set_dt(&cfg->wp, 1);  // Set WP pin to high (inactive)
    if (err) {
        LOG_ERR("Error setting WP pin high");
        return err;
    }

    // issue command
    uint8_t spi_tx_data[1];
    spi_tx_data[0] = INSTRU_WRITE_ENABLE;
    err = w25m02gw_spi_transceive(dev, spi_tx_data, 1, NULL, 0);
    if (err < 0) {
        LOG_ERR("Error writing enable");
        return err;
    }

    // check WEL bit, if not set, return error
    uint8_t status_reg;
    err = _w25m02gw_read_status_register(dev, 0xC0, &status_reg);
    if (err < 0) {
        LOG_ERR("Error reading status register");
        return err;
    }
    if((status_reg & BIT(1)) == 0){
        LOG_ERR("Write enable bit not set");
        return -1;
    }
    return 0;
}

static int _w25m02gw_erase_block(const struct device *dev, uint16_t block_number)
{
    int err;
    err = _wait_until_ready(dev);
    err = _write_enable(dev);
    uint8_t tx_buffer[4] = {0xD8, 0x00, 0x00, 0x00};
    construct_page_address(block_number, 0, &tx_buffer[2]);
    err = w25m02gw_spi_transceive(dev, tx_buffer, 4, NULL, 0);
    if (err < 0) {
        LOG_ERR("Error erasing block");
        return err;
    }
    err = _wait_until_ready(dev);

    uint8_t status_reg;
    err = _w25m02gw_read_status_register(dev, 0xC0, &status_reg);
    if(status_reg & BIT(2) == 1){
        LOG_ERR("Erase failed");
        return -1;
    }
    return 0;
}

static int _configure_status_registers(const struct device *dev)
{
    int err;
    err = _wait_until_ready(dev);

    // Configure status register 1
    uint8_t status_reg1 = 0x00;
    err = _w25m02gw_write_status_register(dev, 0xA0, status_reg1);
    if (err < 0) {
        LOG_ERR("Error writing status register 1");
        return err;
    }

    // Configure status register 2
    uint8_t status_reg2 = 0x18;
    err = _w25m02gw_write_status_register(dev, 0xB0, status_reg2);
    if (err < 0) {
        LOG_ERR("Error writing status register 2");
        return err;
    }


    
    return 0;
}
static int _w25m02gw_load_program_data(const struct device *dev, uint16_t column_addr, uint8_t * buffer, size_t buffer_size)
{

    int err;
    err = _wait_until_ready(dev);
    err = _write_enable(dev);

    uint8_t tx_buffer[buffer_size + 3];
    tx_buffer[0] = 0x02;
    _uint16to8(column_addr, &tx_buffer[1]);
    memcpy(&tx_buffer[3], buffer, buffer_size);

    err = w25m02gw_spi_transceive(dev, tx_buffer, buffer_size + 3, NULL, 0);
    if (err < 0) {
        LOG_ERR("Error loading program data");
        return err;
    }
    return 0;
}

// static int _w25m02gw_load_program_data(const struct device *dev, uint16_t column_addr, uint8_t *buffer, size_t buffer_size)
// {
//     int err;
//     uint8_t tx_buffer[512 + 3]; // Maximum chunk size plus 3 bytes for command and address
//     size_t offset = 0;
//     const size_t PAGE_SIZE = 2048; // Define the page size

//     while (offset < buffer_size) {
//         // Wait until the device is ready and enable write operations
//         err = _wait_until_ready(dev);
//         if (err < 0) {
//             LOG_ERR("Device not ready");
//             return err;
//         }

//         err = _write_enable(dev);
//         if (err < 0) {
//             LOG_ERR("Write enable failed");
//             return err;
//         }

//         // Calculate the current column address
//         uint16_t current_column_addr = column_addr + offset;

//         // Calculate how many bytes can be written without crossing a page boundary
//         size_t bytes_remaining_in_buffer = buffer_size - offset;
//         size_t bytes_remaining_in_page = PAGE_SIZE - (current_column_addr % PAGE_SIZE);
//         size_t chunk_size = bytes_remaining_in_buffer;

//         if (chunk_size > 512) {
//             chunk_size = 512;
//         }
//         if (chunk_size > bytes_remaining_in_page) {
//             chunk_size = bytes_remaining_in_page;
//         }

//         // Prepare the transmission buffer
//         tx_buffer[0] = 0x02; // Write command
//         _uint16to8(current_column_addr, &tx_buffer[1]); // Convert address to bytes
//         memcpy(&tx_buffer[3], &buffer[offset], chunk_size); // Copy data chunk

//         // Perform the SPI transaction
//         err = w25m02gw_spi_transceive(dev, tx_buffer, chunk_size + 3, NULL, 0);
//         if (err < 0) {
//             LOG_ERR("Error loading program data at offset %zu", offset);
//             return err;
//         }

//         // Move to the next chunk
//         offset += chunk_size;
//     }

//     return 0;
// }


static int _page_data_read(const struct device *dev, uint16_t block_addr, uint8_t page_addr)
{
    int err;
    err = _wait_until_ready(dev);
    uint8_t tx_buffer[4];
    tx_buffer[0] = 0x13;
    tx_buffer[1] = 0x00;
    construct_page_address(block_addr, page_addr, &tx_buffer[2]);
    err = w25m02gw_spi_transceive(dev, tx_buffer, 4, NULL, 0);
    if (err < 0) {
        LOG_ERR("Error reading page data");
        return err;
    }
    _wait_until_ready(dev);
    return 0;
}

static int _w25m02gw_write(const struct device *dev, uint16_t block_addr, uint8_t page_addr, uint8_t * buffer, size_t buffer_size)
{
    int err;
    err = _wait_until_ready(dev);
    
    err = _w25m02gw_load_program_data(dev, 0, buffer, buffer_size);
    if (err < 0) {
        LOG_ERR("Error loading program data");
        return err;
    }

    err = _wait_until_ready(dev);

    uint8_t execute_cmd[4];
    execute_cmd[0] = 0x10;
    execute_cmd[1] = 0x00;
    construct_page_address(block_addr, page_addr, &execute_cmd[2]);
    err = w25m02gw_spi_transceive(dev, execute_cmd, 4, NULL, 0);
    if (err < 0) {
        LOG_ERR("Error executing program");
        return err;
    }

    err = _wait_until_ready(dev);

    uint8_t status_reg;
    err = _w25m02gw_read_status_register(dev, 0xC0, &status_reg);
    if (err < 0) {
        LOG_ERR("Error reading status register");
        return err;
    }
    if(status_reg & BIT(3) == 1){
        LOG_ERR("Program failed");
        return -1;
    }
    return 0;
}

static int _w25m02gw_read(const struct device *dev, uint16_t block_addr, uint8_t page_addr, uint16_t column_addr, uint8_t * output, size_t output_size)
{   
    int err;
    if (output_size > 2048) {
        LOG_ERR("Output size exceeds 2048 bytes");
        return -EINVAL;
    }
    err = _wait_until_ready(dev);
    err = _page_data_read(dev, block_addr, page_addr);
    if (err < 0) {
        LOG_ERR("Error reading page data");
        return err;
    }
    uint8_t temp_buffer[2048 + 4];
    uint8_t tx_buffer[4];
    tx_buffer[0] = 0x03;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00; // assuming column address is 0 and always read 2048 bytes
    tx_buffer[3] = 0x00;

    err = w25m02gw_spi_transceive(dev, tx_buffer, 4, temp_buffer, 2048 + 4);
    if (err < 0) {
        LOG_ERR("Error reading page data");
        return err;
    }
    memcpy(output, &temp_buffer[4], output_size);

    return 0;
}


static int init_device_data(const struct device *dev)
{
    struct w25m02gw_data *data = dev->data;
    data->current_block = 0;
    data->current_page = 0;
    return 0;
}


static int init_w25m02gw(const struct device *dev)
{

    int err = 0;


    k_msleep(100);
    _reset_device(dev);
    k_msleep(1); // Small delay to ensure command is processed
    uint8_t status_reg[3] = {0};
    _configure_status_registers(dev);
    k_msleep(100); // Small delay to ensure command is processed
    
    _w25m02gw_read_status_register(dev, 0xA0, &status_reg[0]);
    _w25m02gw_read_status_register(dev, 0xB0, &status_reg[1]);
    _w25m02gw_read_status_register(dev, 0xC0, &status_reg[2]);
    for(int i = 0; i < 3; i++)
    {
        printk("Status Register %d: 0x%02x\n", i, status_reg[i]);
    }   


    return err;

}




static const struct w25m02gw_driver_api_funcs w25m02gw_api = {
    .read= _w25m02gw_read,
    .write = _w25m02gw_write,
    .erase = _w25m02gw_erase_block,

};




#define INST_HOLD_GPIO_SPEC(idx)                              \
    static const struct gpio_dt_spec hold_##idx = \
        GPIO_DT_SPEC_GET(DT_DRV_INST(idx), hold_gpios);\


#define INST_WP_GPIO_SPEC(idx)                              \
    static const struct gpio_dt_spec wp_##idx = \
        GPIO_DT_SPEC_GET(DT_DRV_INST(idx), wp_gpios);\


#define W25M02GW_CONFIG(inst) \
    INST_HOLD_GPIO_SPEC(inst) \
    INST_WP_GPIO_SPEC(inst) \
    static const struct w25m02gw_config w25m02gw_config_##inst = { \
    .spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0), \
    .hold = hold_##inst, \
    .wp = wp_##inst, \
    }	\
    


#define W25M02GW_DEFINE(inst)						\
    static struct w25m02gw_data w25m02gw_data_##inst;			\
    W25M02GW_CONFIG(inst);						\
    DEVICE_DT_INST_DEFINE(inst,			\
                init_w25m02gw,							\
                NULL,							\
                &w25m02gw_data_##inst,	\
                &w25m02gw_config_##inst,\
                POST_KERNEL, \
                CONFIG_W25M02GW_INIT_PRIORITY, \
                &w25m02gw_api);

/* STEP 5.2 - Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(W25M02GW_DEFINE)
