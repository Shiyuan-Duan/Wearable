#ifndef W25M02GW_H
#define W25M02GW_H

#define DT_DRV_COMPAT dsy_w25m02gw

#include <zephyr/device.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nordic-nrf-gpio.h>



#include <ncs_version.h>
#if NCS_VERSION_NUMBER >= 0x20600
#include <zephyr/internal/syscall_handler.h>
#else
#include <zephyr/syscall_handler.h>
#endif


#define INSTRU_W25M02GW_ID 0x9F
#define INSTRU_W25M02GW_READ_LUT 0xA5
#define INSTRU_W25M02GW_WRITE_LUT 0xA1
#define INSTRU_PAGE_DATA_READ 0x13
#define INSTRU_READ_BUF 0x03
#define INSTRU_PROGRAM_BUF 0x02
#define INSTRU_PROGRAM_EXECUTE 0x10
#define INSTRU_WRITE_ENABLE 0x06
#define INSTRU_ERASE_BLOCK 0xD8
#define INSTRU_READ_STATUS 0x05 
#define INSTRU_WRITE_STATUS 0x01


#define STATUS3_REG 0xC0

#define MAX_PAGE_SIZE 2048

// typedef int  (*max30003_api_print_cnfg)(const struct device * dev);
// typedef int  (*max30003_api_synch_ecg)(const struct device * dev);
// typedef int  (*max30003_api_fifo_read)(const struct device * dev, uint8_t *buf);
// typedef int  (*max30003_api_read_status)(const struct device * dev);

typedef int (*w25m02gw_api_read)(const struct device *dev, uint16_t block_addr, uint8_t page_addr, uint16_t column_addr, uint8_t * output, size_t output_size);
typedef int (*w25m02gw_api_write)(const struct device *dev, uint16_t block_addr, uint8_t page_addr, uint8_t * buffer, size_t buffer_size);
typedef int (*w25m02gw_api_erase)(const struct device *dev, uint16_t block_addr);
typedef int (*w25m02gw_api_set_opmode)(const struct device *dev, uint8_t *opmode);
struct w25m02gw_driver_api_funcs {
    w25m02gw_api_set_opmode set_opmode;
    w25m02gw_api_read read;
    w25m02gw_api_write write;
    w25m02gw_api_erase erase;
};

__syscall     int        w25m02gw_read(const struct device * dev, uint16_t block_addr, uint8_t page_addr, uint16_t column_addr, uint8_t * output, size_t output_size);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline int z_impl_w25m02gw_read(const struct device * dev, uint16_t block_addr, uint8_t page_addr, uint16_t column_addr, uint8_t * output, size_t output_size)
{
    const struct w25m02gw_driver_api_funcs *api = dev->api;

    __ASSERT(api->read, "Callback pointer should not be NULL");

    return api->read(dev, block_addr, page_addr, column_addr, output, output_size);
}


__syscall     int        w25m02gw_write(const struct device * dev, uint16_t block_addr, uint8_t page_addr, uint8_t * buffer, size_t buffer_size);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline int z_impl_w25m02gw_write(const struct device * dev, uint16_t block_addr, uint8_t page_addr, uint8_t * buffer, size_t buffer_size)
{
    const struct w25m02gw_driver_api_funcs *api = dev->api;

    __ASSERT(api->write, "Callback pointer should not be NULL");

    return api->write(dev, block_addr, page_addr, buffer, buffer_size);

}

__syscall     int        w25m02gw_erase_block(const struct device * dev, uint16_t block_addr);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline int z_impl_w25m02gw_erase_block(const struct device * dev, uint16_t block_addr)
{
    const struct w25m02gw_driver_api_funcs *api = dev->api;

    __ASSERT(api->erase, "Callback pointer should not be NULL");

    return api->erase(dev, block_addr);
}

#include <syscalls/w25m02gw.h>




#endif // W25M02GW_H