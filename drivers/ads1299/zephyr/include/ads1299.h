#ifndef MAX30003_H_
#define MAX30003_H_

#define DT_DRV_COMPAT ti_ads1299
#include <zephyr/device.h>


#define REG_ID 0x00
#define REG_CONFIG1 0x01
#define REG_CONFIG2 0x02
#define REG_CONFIG3 0x03
#define REG_LOFF 0x04
#define REG_CH1SET 0x05
#define REG_CH2SET 0x06
#define REG_CH3SET 0x07
#define REG_CH4SET 0x08
#define REG_CH5SET 0x09
#define REG_CH6SET 0x0A
#define REG_CH7SET 0x0B
#define REG_CH8SET 0x0C
#define REG_BIAS_SENSP 0x0D
#define REG_BIAS_SENSN 0x0E
#define REG_LOFF_SENSP 0x0F
#define REG_LOFF_SENSN 0x10
#define REG_LOFF_FLIP 0x11
#define REG_LOFF_STATP 0x12
#define REG_LOFF_STATN 0x13
#define REG_GPIO 0x14
#define REG_MISC1 0x15
#define REG_MISC2 0x16
#define REG_CONFIG4 0x17




typedef int  (*ads1299_api_check_id)(const struct device * dev, uint8_t *chip_id);
typedef int  (*ads1299_api_sample)(const struct device * dev, uint8_t *read_buffer);
typedef int  (*ads1299_api_start)(const struct device * dev);
typedef int  (*ads1299_api_stop)(const struct device * dev);
typedef int  (*ads1299_api_wakeup)(const struct device * dev);
typedef int  (*ads1299_api_standby)(const struct device * dev);
typedef int  (*ads1299_api_reset)(const struct device * dev);
typedef int  (*ads1299_api_read_data)(const struct device * dev, uint8_t *read_buffer, uint8_t read_size);


/* STEP 4.2 - Define a struct to have a member for each typedef you defined in Part 1 */
struct ads1299_driver_api_funcs {
    ads1299_api_check_id check_id;
    ads1299_api_sample sample;
    ads1299_api_start start;
    ads1299_api_stop stop;
    ads1299_api_wakeup wakeup;
    ads1299_api_standby standby;
    ads1299_api_reset reset;
    ads1299_api_read_data read_data;
};

__syscall     void        ads1299_check_id(const struct device * dev, uint8_t *chip_id);
static inline void z_impl_ads1299_check_id(const struct device * dev, uint8_t *chip_id)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->check_id, "Callback pointer should not be NULL");

    api->check_id(dev, chip_id);
}

__syscall     void        ads1299_sample(const struct device * dev, uint8_t *read_buffer);
static inline void z_impl_ads1299_sample(const struct device * dev, uint8_t *read_buffer)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->sample, "Callback pointer should not be NULL");

    api->sample(dev, read_buffer);
}

__syscall     void        ads1299_start(const struct device * dev);
static inline void z_impl_ads1299_start(const struct device * dev)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->start, "Callback pointer should not be NULL");

    api->start(dev);
}

__syscall     void        ads1299_stop(const struct device * dev);
static inline void z_impl_ads1299_stop(const struct device * dev)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->stop, "Callback pointer should not be NULL");

    api->stop(dev);
}


__syscall     void        ads1299_wakeup(const struct device * dev);
static inline void z_impl_ads1299_wakeup(const struct device * dev)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->wakeup, "Callback pointer should not be NULL");

    api->wakeup(dev);
}

__syscall     void        ads1299_standby(const struct device * dev);
static inline void z_impl_ads1299_standby(const struct device * dev)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->standby, "Callback pointer should not be NULL");

    api->standby(dev);
}

__syscall     void        ads1299_reset(const struct device * dev);
static inline void z_impl_ads1299_reset(const struct device * dev)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->reset, "Callback pointer should not be NULL");

    api->reset(dev);
}

__syscall     int        ads1299_read_data(const struct device * dev, uint8_t *read_buffer, uint8_t read_size); 
static inline int z_impl_ads1299_read_data(const struct device * dev, uint8_t *read_buffer, uint8_t read_size)
{
    const struct ads1299_driver_api_funcs *api = dev->api;

    __ASSERT(api->read_data, "Callback pointer should not be NULL");

    return api->read_data(dev, read_buffer, read_size);
}





#include <syscalls/ads1299.h>

#endif