#ifndef MAX86141_H_
#define MAX86141_H_

#define DT_DRV_COMPAT dsy_max86141
#include <zephyr/device.h>


#define INT_STATUS1         0x00
#define INT_STATUS2         0X01
#define INT1_EN             0x02
#define INT2_EN             0x03
#define FIFO_WRITE_PTR      0x04
#define FIFO_READ_PTR       0x05
#define OVERFLOW_COUNTER    0x06
#define FIFO_DATA_COUNTER   0x07
#define FIFO_DATA_READ      0x08
#define FIFO_CONFIG1        0x09
#define FIFO_CONFIG2        0x0A
#define SYS_CTRL            0x0D
#define PPG_SYNC_CTRL       0x10

#define PPG_CONFIG1         0x11
#define PPG_CONFIG2         0x12
#define PPG_CONFIG3         0x13

#define PD_BIAS             0x15
#define CONFIG_PF           0x16

#define LED_SEQ1            0x20
#define LED_SEQ2            0x21
#define LED_SEQ3            0x22


#define LED_PA1              0x23
#define LED_PA2              0x24
#define LED_PA3              0x25
#define PILOT_PA             0x29

#define LED_RGE1             0x2A
#define PART_ID             0xFF



typedef int  (*max86141_api_read_status)(const struct device * dev);
typedef int  (*max86141_api_read_fifo)(const struct device * dev, uint8_t *buf);
typedef int (*max86141_api_flush_fifo)(const struct device * dev);
typedef int (*max86141_api_read_fifo)(const struct device * dev, uint8_t *buf);
typedef int (*max86141_api_count_fifo)(const struct device * dev);
typedef int (*max86141_api_turn_on)(const struct device * dev);
typedef int (*max86141_api_turn_off)(const struct device * dev);

/* STEP 4.2 - Define a struct to have a member for each typedef you defined in Part 1 */
struct max86141_driver_api_funcs {
    max86141_api_read_status read_status;
    max86141_api_flush_fifo flush_fifo;
    max86141_api_read_fifo read_fifo;
    max86141_api_count_fifo count_fifo;
    max86141_api_turn_on turn_on;
    max86141_api_turn_off turn_off;
};

struct max86141_data
{
    uint8_t chip_id;
    uint8_t status[3];
    uint8_t fifo_data[128*3];
    int led1[128];
    int led2[128];
    int led3[128];
    uint8_t led1_ptr;
    uint8_t led2_ptr;
    uint8_t led3_ptr;

};


__syscall     void        max86141_read_status(const struct device * dev);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline void z_impl_max86141_read_status(const struct device * dev)
{
	const struct max86141_driver_api_funcs *api = dev->api;
    __ASSERT(api->read_status, "Callback pointer should not be NULL");
    api->read_status(dev);

}

__syscall     void        max86141_flush_fifo(const struct device * dev);

static inline void z_impl_max86141_flush_fifo(const struct device * dev)
{
    const struct max86141_driver_api_funcs *api = dev->api;
    __ASSERT(api->flush_fifo, "Callback pointer should not be NULL");
    api->flush_fifo(dev);


}

__syscall     void        max86141_read_fifo(const struct device * dev, uint8_t *buf);

static inline void z_impl_max86141_read_fifo(const struct device * dev, uint8_t *buf)
{
    const struct max86141_driver_api_funcs *api = dev->api;
    __ASSERT(api->read_fifo, "Callback pointer should not be NULL");
    api->read_fifo(dev, buf);

}

__syscall     void        max86141_count_fifo(const struct device * dev);

static inline void z_impl_max86141_count_fifo(const struct device * dev)
{
    const struct max86141_driver_api_funcs *api = dev->api;
    __ASSERT(api->count_fifo, "Callback pointer should not be NULL");
    api->count_fifo(dev);

}

__syscall     void        max86141_turn_on(const struct device * dev);

static inline void z_impl_max86141_turn_on(const struct device * dev)
{
    const struct max86141_driver_api_funcs *api = dev->api;
    __ASSERT(api->turn_on, "Callback pointer should not be NULL");
    api->turn_on(dev);

}

__syscall     void        max86141_turn_off(const struct device * dev);

static inline void z_impl_max86141_turn_off(const struct device * dev)
{
    const struct max86141_driver_api_funcs *api = dev->api;
    __ASSERT(api->turn_off, "Callback pointer should not be NULL");
    api->turn_off(dev);

}






#include <syscalls/max86141.h>


#endif /*MAX86141_H_*/