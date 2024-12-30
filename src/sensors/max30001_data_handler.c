#if CONFIG_MAX30001
#include "max30001_data_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "w25m02gw.h"
#include "ble_sensor_control.h"
#include "max30001.h"
LOG_MODULE_REGISTER(MAX30001_DATA_HANDLER, LOG_LEVEL_DBG);
const struct device *max30001_dev = DEVICE_DT_GET(DT_NODELABEL(max30001));
static int init_data_handler(void)
{
    LOG_INF("Initializing data handler: %d\n", CONFIG_MAX30001);
    while(1)
    {
        LOG_INF("Data handler running\n");
        max30001_read_efifo(max30001_dev, NULL);
        max30001_read_bfifo(max30001_dev, NULL);
        k_yield();
    }

    return 0;
}


K_THREAD_DEFINE(max30001_data_init_thread, 2048, init_data_handler, NULL, NULL, NULL, 1, 0, 0);
// K_THREAD_DEFINE(ads1299_producer_thread, 4096, ads1299_data_producer, NULL, NULL, NULL, 2, 0, 0);
// K_THREAD_DEFINE(ads1299_streamer_thread, 4096, ble_stream_thread, NULL, NULL, NULL, 3, 0, 0);
// K_THREAD_DEFINE(ads1299_flash_thread, 4096, flash_thread, NULL, NULL, NULL, 3, 0, 0);

#endif


