#ifndef MAX30001_H
#define MAX30001_H



#define DT_DRV_COMPAT maxim_max30001
#include <zephyr/device.h>

struct max30001_config
{
    struct spi_dt_spec spi;
    struct gpio_dt_spec inta;
    struct gpio_dt_spec intb;
};


struct max30001_data
{
    uint8_t chip_id;
    uint8_t status[3];
    struct gpio_callback inta_cb;
    struct gpio_callback intb_cb;
    struct k_sem inta_sem; 
    struct k_sem intb_sem; 
};



// typedef int  (*ads1299_api_check_id)(const struct device * dev, uint8_t *chip_id);
// typedef int  (*ads1299_api_sample)(const struct device * dev, uint8_t *read_buffer);
// typedef int  (*ads1299_api_start)(const struct device * dev);
// typedef int  (*ads1299_api_stop)(const struct device * dev);

typedef int (*max30001_api_check_id)(const struct device *dev);
typedef int (*max30001_api_read_status)(const struct device *dev, uint8_t *data);
typedef int (*max30001_api_reset_device)(const struct device *dev);
typedef int (*max30001_api_resync_device)(const struct device *dev);
typedef int (*max30001_api_reset_fifo)(const struct device *dev);
typedef int (*max30001_api_read_efifo)(const struct device *dev, uint8_t *data);
typedef int (*max30001_api_read_bfifo)(const struct device *dev, uint8_t *data);

struct max30001_driver_api_funcs {
    max30001_api_check_id check_id;
    max30001_api_read_status read_status;
    max30001_api_reset_device reset_device;
    max30001_api_resync_device resync_device;
    max30001_api_reset_fifo reset_fifo;
    max30001_api_read_efifo read_efifo;
    max30001_api_read_bfifo read_bfifo;
};

__syscall void max30001_read_status(const struct device *dev, uint8_t *data);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline void z_impl_max30001_read_status(const struct device *dev, uint8_t *data)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->read_status, "Callback pointer should not be NULL");

    api->read_status(dev, data);
}

__syscall void max30001_check_id(const struct device *dev);

static inline void z_impl_max30001_check_id(const struct device *dev)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->check_id, "Callback pointer should not be NULL");

    api->check_id(dev);
}

__syscall void max30001_reset_device(const struct device *dev);

static inline void z_impl_max30001_reset_device(const struct device *dev)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->reset_device, "Callback pointer should not be NULL");

    api->reset_device(dev);
}

__syscall void max30001_resync_device(const struct device *dev);

static inline void z_impl_max30001_resync_device(const struct device *dev)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->resync_device, "Callback pointer should not be NULL");

    api->resync_device(dev);
}


__syscall void max30001_reset_fifo(const struct device *dev);

static inline void z_impl_max30001_reset_fifo(const struct device *dev)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->reset_fifo, "Callback pointer should not be NULL");

    api->reset_fifo(dev);
}


__syscall void max30001_read_efifo(const struct device *dev, uint8_t *data);


static inline void z_impl_max30001_read_efifo(const struct device *dev, uint8_t *data)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->read_efifo, "Callback pointer should not be NULL");

    api->read_efifo(dev, data);
}


__syscall void max30001_read_bfifo(const struct device *dev, uint8_t *data);


static inline void z_impl_max30001_read_bfifo(const struct device *dev, uint8_t *data)
{
    const struct max30001_driver_api_funcs *api = dev->api;

    __ASSERT(api->read_bfifo, "Callback pointer should not be NULL");

    api->read_bfifo(dev, data);
}

#include <syscalls/max30001.h>


#endif