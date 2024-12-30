#ifndef MAX30003_H_
#define MAX30003_H_

#define DT_DRV_COMPAT dsy_max30003
#include <zephyr/device.h>

// Define reg addr
#define STATUS 0x01
#define EN_INT 0x02
#define EN_INT2 0x03
#define MNGR_INT 0x04
#define SW_RST 0x08
#define SYNCH 0x09
#define FIFO_RST 0x0A
#define CNFG_MUX 0x14
#define CNFG_ECG 0x15

#define CNFG_RTOR1 0x1D


#define ECG_BURST 0x20
#define INFO  0x0F
#define CNFG_GEN 0x10


typedef int  (*max30003_api_print_cnfg)(const struct device * dev);
typedef int  (*max30003_api_synch_ecg)(const struct device * dev);
typedef int  (*max30003_api_fifo_read)(const struct device * dev, uint8_t *buf);
typedef int  (*max30003_api_read_status)(const struct device * dev);

/* STEP 4.2 - Define a struct to have a member for each typedef you defined in Part 1 */
struct max30003_driver_api_funcs {
    max30003_api_print_cnfg print_cnfg;
	max30003_api_synch_ecg synch_ecg;
	max30003_api_fifo_read fifo_read;
	max30003_api_read_status read_status;
};

struct max30003_data
{
    uint8_t chip_id;
    int32_t ecg_data[32];
	uint8_t status[3];

};

__syscall     void        max30003_read_status(const struct device * dev);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline void z_impl_max30003_read_status(const struct device * dev)
{
	const struct max30003_driver_api_funcs *api = dev->api;

	__ASSERT(api->print_cnfg, "Callback pointer should not be NULL");

	api->read_status(dev);
}

__syscall     void        max30003_print_cnfg(const struct device * dev);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline void z_impl_max30003_print_cnfg(const struct device * dev)
{
	const struct max30003_driver_api_funcs *api = dev->api;

	__ASSERT(api->print_cnfg, "Callback pointer should not be NULL");

	api->print_cnfg(dev);
}

__syscall     void        max30003_synch_ecg(const struct device * dev);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline void z_impl_max30003_synch_ecg(const struct device * dev)
{
	const struct max30003_driver_api_funcs *api = dev->api;

	__ASSERT(api->synch_ecg, "Callback pointer should not be NULL");

	api->synch_ecg(dev);
}

__syscall     void        max30003_fifo_read(const struct device * dev, uint8_t * buf);

/* STEP 4.4 - Implement the Z_impl_* translation function to call the device driver API for this feature */
static inline void z_impl_max30003_fifo_read(const struct device * dev, uint8_t * buf)
{
	const struct max30003_driver_api_funcs *api = dev->api;

	__ASSERT(api->fifo_read, "Callback pointer should not be NULL");

	api->fifo_read(dev, buf);
}




#include <syscalls/max30003.h>

#endif /*MAX30003_H_*/