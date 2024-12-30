    //max30003.c
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


    #include "max30003.h"


    #define PRINT_BIT(val, n) ((val & BIT(n)) ? '1' : '0')
    #define SPIOP	SPI_WORD_SET(8)


    LOG_MODULE_REGISTER(MAX30003, LOG_LEVEL_DBG);


    struct max30003_config
    {
        struct spi_dt_spec spi;
    };



    static int max30003_read_reg(const struct device *dev, uint8_t reg, uint8_t *data, int size)
    {
        int err;
        uint8_t tx_buffer[4]; 
        tx_buffer[0] = (reg << 1) | 0x01;
        tx_buffer[1] = 0x00;
        tx_buffer[2] = 0x00;
        tx_buffer[3] = 0x00;



        struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = sizeof(tx_buffer)};
        struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
        struct spi_buf rx_spi_buf = {.buf = data, .len = size};
        struct spi_buf_set rx_spi_buf_set = {.buffers = &rx_spi_buf, .count=1};

        const struct max30003_config *cfg = dev->config;
        
        err = spi_transceive_dt(&cfg->spi, &tx_spi_buf_set, &rx_spi_buf_set);
        if (err < 0) {
            printk("spi_transceive_dt() failed, err: %d", err);
            return err;
        }

        return 0;

    }



    static int max30003_write_reg(const struct device *dev, uint8_t reg, uint8_t *values, size_t size)
    {
        int err;
        const struct max30003_config *cfg = dev->config;
        

        /* Bit7 is 0 for the write command */
        uint8_t tx_buf[size+1];
        tx_buf[0] = (reg << 1);
        memcpy(&tx_buf[1], values, size);

        struct spi_buf tx_spi_buf = {.buf = tx_buf, .len = sizeof(tx_buf)};
        struct spi_buf_set tx_spi_buf_set = {.buffers = &tx_spi_buf, .count = 1};
        err = spi_write_dt(&cfg->spi, &tx_spi_buf_set);
        if (err < 0) {
            LOG_ERR("spi_write_dt() failed, err %d", err);
            return err;
        }

        return 0;
    }


    static int fifo_rst(const struct device *dev)
    {
        int err;
        uint8_t data[3];
        data[0] = 0x0;
        data[1] = 0x0;
        data[2] = 0x0;
        err = max30003_write_reg(dev, FIFO_RST, data, 3);
        if (err<0){
            LOG_ERR("fifo rst failed with err: %d\n", err);
        }
        return err;

    }
    static int check_info(const struct device *dev)
    {
        int err;
        uint8_t data[4];
        err = max30003_read_reg(dev, INFO, data, sizeof(data));

        if (err<0){
            LOG_ERR("Error reading max30003 reg\n");
            return err;
        }

        if ((data[1]>>4) != 0b0101)
        {
            LOG_ERR("Error communicating to MAX30003, expecting 0b0101 but got 0x%x\n", data[1]>>4);
            return -1;
        }
        LOG_INF("INFO REVID: 0x%x\n", data[1]);
        return 0;
    }
    static int read_status(const struct device *dev)
    {
        int err;
        uint8_t data[4];
        err = max30003_read_reg(dev, STATUS, data, sizeof(data));
        if (err < 0){
            LOG_ERR("Error print status\n");
            return err;
        }

        struct max30003_data *dev_data = dev->data;
        memcpy(dev_data->status, &data[1], 3);

        // LOG_INF("Printing status:\n");
        // LOG_INF("EINT : %c\n", PRINT_BIT(data[1], 7));

        return 0;
    }
    static int print_status(const struct device *dev)
    {
        int err;
        uint8_t data[4];
        err = max30003_read_reg(dev, STATUS, data, sizeof(data));
        if (err < 0){
            LOG_ERR("Error print status\n");
            return err;
        }
        LOG_INF("Printing status:\n");
        LOG_INF("EINT : %c\n", PRINT_BIT(data[1], 7));
        LOG_INF("EOVF : %c\n", PRINT_BIT(data[1], 6));
        LOG_INF("FSTINT : %c\n", PRINT_BIT(data[1], 5));
        LOG_INF("DCLOFF_INT : %c\n", PRINT_BIT(data[1], 4));
        LOG_INF("********************\n");
        LOG_INF("LONINT : %c\n", PRINT_BIT(data[2], 3));
        LOG_INF("RRINT : %c\n", PRINT_BIT(data[2], 2));
        LOG_INF("SAMP : %c\n", PRINT_BIT(data[2], 1));
        LOG_INF("PLLINT : %c\n", PRINT_BIT(data[2], 0));



        return 0;
    }

    static int print_cnfg(const struct device *dev)
    {
        int err;
        uint8_t data[4];
        err = max30003_read_reg(dev, CNFG_GEN, data, sizeof(data));
        if (err < 0){
            LOG_ERR("Error print cnfg\n");
            return err;
        }
        LOG_INF("Printing configs:\n");
        LOG_INF("data[0]:0x%x, data[1]:0x%x, data[2]:0x%x, data[3]:0x%x\n", data[0], data[1], data[2], data[3]);
        LOG_INF("EN_ULP_LON : %c%c\n", PRINT_BIT(data[1], 7),PRINT_BIT(data[1], 6));
        LOG_INF("FMSTR : %c%c\n", PRINT_BIT(data[1], 5), PRINT_BIT(data[1], 4));
        LOG_INF("EN_ECG : %c\n", PRINT_BIT(data[1], 3));


        LOG_INF("RBIASV: %c%c\n", PRINT_BIT(data[3], 3), PRINT_BIT(data[3], 2));
        LOG_INF("********************\n");


        return 0;
    }

    static int max30003_reset(const struct device *dev)
    {
        int err;
        uint8_t data[3] = {0, 0, 0};
        err = max30003_write_reg(dev, SW_RST, data, 3);
        if (err<0){
            LOG_ERR("Error in writing to SW_RST\n");
        }
        return 0;
    }

    static int enable_interrupt(const struct device *dev)
    {
        int err;
        uint8_t en_int_data[4];
        err = max30003_read_reg(dev, EN_INT, en_int_data, sizeof(en_int_data));

        en_int_data[1] = en_int_data[1] | BIT(7);

        err = max30003_write_reg(dev, EN_INT, &en_int_data[1], 3);

        if (err<0){
            LOG_ERR("Something went wrong in enable interrupt!\n");
        }
        return err;
    }

    static int config_ecg(const struct device *dev)
    {
        int err;
        uint8_t cnfg_ecg_data[4];
        err = max30003_read_reg(dev, CNFG_ECG, cnfg_ecg_data, sizeof(cnfg_ecg_data));


        cnfg_ecg_data[1] = 0x0; // rate = 512 Hz
        cnfg_ecg_data[1] = (cnfg_ecg_data[1] | BIT(1)) | BIT(0); // gain = 11 (160V/V) 

        cnfg_ecg_data[2] = cnfg_ecg_data[2] | BIT(5); // high pass = 100Hz





        err = max30003_write_reg(dev, CNFG_ECG, &cnfg_ecg_data[1], 3);

        if (err<0){
            LOG_ERR("Something went wrong in config ecg!\n");
        }
        return err;
    }

    static int config_mux(const struct device *dev)
    {
        int err;
        uint8_t cnfg_mux_data[4];
        err = max30003_read_reg(dev, CNFG_MUX, cnfg_mux_data, sizeof(cnfg_mux_data));
        printk("config mux before %x\n", cnfg_mux_data[1]);
        cnfg_mux_data[1] = 0x0;

        err = max30003_write_reg(dev, CNFG_MUX, &cnfg_mux_data[1], 3);
        
        err = max30003_read_reg(dev, CNFG_MUX, cnfg_mux_data, sizeof(cnfg_mux_data));
        printk("config mux after %x\n", cnfg_mux_data[1]);
        if (err<0){
            LOG_ERR("Something went wrong in config ecg!\n");
        }
        return err;
    }

    static int fifo_read(const struct device *dev, uint8_t *buf)
    {   
        read_status(dev);
        int err;
        const size_t read_size = 32*3+1; // 32 words (potential maximum) + 1 extra for alignment or control
        uint8_t ecg_data[read_size]; // Adjust size if needed based on actual maximum ECG words expected

        err = max30003_read_reg(dev, ECG_BURST, ecg_data, sizeof(ecg_data));
        if (err < 0) {
            LOG_ERR("Something went wrong in burst read ecg: %d", err);
            return err;
        }

        // Assuming each ECG word is 18 bits and packed into 3 bytes
        struct max30003_data *data = dev->data;
        memset(data->ecg_data, 0, sizeof(data->ecg_data));
        int index = 1;
        int ecg_index = 0;
        while (index < read_size - 3) { // Ensure there's a full word to read
            // Combine three bytes into a single 32-bit integer
            int32_t raw_value = (ecg_data[index] << 16) | (ecg_data[index+1] << 8) | ecg_data[index+2];
            index += 3;

            // Extract ETag and value from the combined data
            uint8_t etag = (raw_value >> 3) & 0x07; // ETag is in the top 3 bits of the shifted value
            int32_t ecg_value = (raw_value>>6) & 0x3FFFF; // ECG value is the lower 18 bits

            // Check and perform sign extension if the value is negative
            if (ecg_value & BIT(17)) { // Check the sign bit (21st bit) 
                ecg_value |= 0xFFFC0000; // Sign extend to 32 bits
            } else {
                ecg_value &= 0x0003FFFF; // Ensure the value is only 18 bits
            }

            
            
            // Handle different ETags based on expected behavior
            if (etag == 0x02) { // Last valid sample indication (example tag)
                break; // Exit if this is the end of the data of interest
            } else if (etag == 0x06) { // FIFO Overread
                break;
            } else if (etag == 0x07) { // FIFO Overflow
                fifo_rst(dev);
                return 0; // Indicate an overflow error
            }

            // TODO: Read data into device ecg data
            if (ecg_index < 32) {
                data->ecg_data[ecg_index++] = ecg_value;
                // printk("ECG Value before: %d\n", ecg_value);
            }
        }
        // printk("Read %d samples from fifo\n", ecg_index);

        read_status(dev);

        return 0; // Return success
    }
    // static int fifo_read(const struct device *dev, uint8_t *buf)
    // {
    //     int err;
    //     const size_t read_size = 32*3+1; // 32 words (potential maximum) + 1 extra for alignment or control
    //     uint8_t ecg_data[read_size]; // Adjust size if needed based on actual maximum ECG words expected

    //     err = max30003_read_reg(dev, ECG_BURST, ecg_data, sizeof(ecg_data));
    //     if (err < 0) {
    //         LOG_ERR("Something went wrong in burst read ecg: %d", err);
    //         return err;
    //     }

    //     // Assuming each ECG word is 18 bits and packed into 3 bytes
    //     int index = 1;
    //     while (index < read_size - 3) { // Ensure there's a full word to read
    //         // Combine three bytes into a single 32-bit integer
    //         uint32_t raw_value = (ecg_data[index] << 16) | (ecg_data[index+1] << 8) | ecg_data[index+2];
    //         uint8_t etag;
    //         index += 3;

    //         uint32_t ecg_sampe = (raw_value >> 8) & 0xFFFF;
    //         etag = (raw_value >> 3) & 0x07;

    //         printk("ETag: %u, ECG Value: %d\n", etag, ecg_sampe >> 6); // Shift right to get the proper ECG value
            
    //         // Handle different ETags based on expected behavior
    //         if (etag == 0x02) { // Last valid sample indication (example tag)
    //             break; // Exit if this is the end of the data of interest
    //         } else if (etag == 0x06) { // FIFO Overread
    //             break;
    //         } else if (etag == 0x07) { // FIFO Overflow
    //             fifo_rst(dev);
    //             return 0; // Indicate an overflow error
    //         }
    //     }

    //     return 0; // Return success
    // }
        //   ecgSample = (ecgFIFO >> 8) & 0xFFFF; // Isolate voltage data
        //   ETAG = (ecgFIFO >> 3) & ETAG_BITS;

    static int manage_interrupts(const struct device *dev)
    {
        int err;
        uint8_t mngr_int_data[4];
        err = max30003_read_reg(dev, MNGR_INT, mngr_int_data, sizeof(mngr_int_data));

        // mngr_int_data[1] = 0x18; // b'00011000 4 samples
        mngr_int_data[1] = 0xF0; // b'11110000 30 samples



        err = max30003_write_reg(dev, MNGR_INT, &mngr_int_data[1], 3);

        if (err<0){
            LOG_ERR("Something went wrong in mngr int!\n");
        }
        return err;
    }

    static int synch_ecg(const struct device *dev)
    {
        int err;
        uint8_t sync_data[3];
        sync_data[0] = 0x0;
        sync_data[1] = 0x0;
        sync_data[2] = 0x0;
        err = max30003_write_reg(dev, SYNCH, sync_data, 3);
        if (err<0){
            LOG_ERR("synch ecg failed with err: %d\n", err);
        }
        return err;

    }



    static int init_max30003(const struct device *dev)
    {   

        int err;
        max30003_reset(dev);
        synch_ecg(dev);
        check_info(dev);
        // print_status(dev);
        // print_cnfg(dev);


        
        uint8_t cnfg_gen_buf[4];
        err = max30003_read_reg(dev, CNFG_GEN, cnfg_gen_buf, sizeof(cnfg_gen_buf));
        if (err < 0){
            LOG_ERR("Error reading cnfg_gen\n");
            return err;
        }


        // CNFG_GEN: Enable ECG
        cnfg_gen_buf[1] = cnfg_gen_buf[1] | BIT(3);
        // CNFG_GEN: FMSTR (Default 00)

        // CNFG_GEN: EN_RBIAS
        cnfg_gen_buf[3] = cnfg_gen_buf[3] | BIT(4);

        // CNFG_GEN: RBIASP/N
        cnfg_gen_buf[3] = (cnfg_gen_buf[3] | BIT(0)) | BIT(1);


        max30003_write_reg(dev, CNFG_GEN, &cnfg_gen_buf[1], 3);


        err = manage_interrupts(dev);
        err = enable_interrupt(dev);
        
        LOG_INF("Enable interrupt status:%d\n", err);

        err = config_ecg(dev);
        err = config_mux(dev);



        print_cnfg(dev);


        printk("Initializing MAX30003!\n");
        return 0;
    }



    static const struct max30003_driver_api_funcs max30003_api = {
        .print_cnfg = print_cnfg,
        .synch_ecg = synch_ecg,
        .fifo_read = fifo_read,
        .read_status = read_status,
    };


#define MAX30003_CONFIG_SPI(inst) {.spi = SPI_DT_SPEC_INST_GET(inst, SPIOP, 0)}

#define MAX30003_DEFINE(inst)						\
    static struct max30003_data max30003_data_##inst;			\
    static const struct max30003_config max30003_config_##inst = MAX30003_CONFIG_SPI(inst);	\
    DEVICE_DT_INST_DEFINE(inst,			\
                init_max30003,							\
                NULL,							\
                &max30003_data_##inst,	\
                &max30003_config_##inst,\
                POST_KERNEL, \
                CONFIG_MAX30003_INIT_PRIORITY, \
                &max30003_api);

/* STEP 5.2 - Create the struct device for every status "okay" node in the devicetree */
DT_INST_FOREACH_STATUS_OKAY(MAX30003_DEFINE)




