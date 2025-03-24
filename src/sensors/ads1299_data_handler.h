#ifndef ADS1299_DATA_HANDLER_H
#define ADS1299_DATA_HANDLER_H

#if CONFIG_ADS1299
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ads1299.h"


#define ADS1299_DATA_SIZE 24
#define BLE_BUFFER_SIZE 60

int disk_write_die_pointer;
int disk_write_block_pointer;
int disk_write_page_pointer;
int disk_write_offset;



struct k_fifo ads1299_data_fifo;
struct k_sem fifo_ready_sem;
struct k_fifo ble_stream_fifo;




struct ble_stream_data {
    void *fifo_reserved;
    int32_t channel_data[8];
};

struct ads1299_fifo_data_item_t {
    void *fifo_reserved;
    uint8_t data[24];
};
void stream_buffer(uint8_t *data, int size, int channel);
#endif /* ADS1299_DATA_HANDLER_H */
#endif


