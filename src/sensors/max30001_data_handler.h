#if CONFIG_MAX30001
#ifndef MAX30001_DATA_HANDLER_H
#define MAX30001_DATA_HANDLER_H
#include <zephyr/types.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
int disk_write_die_pointer;
int disk_write_block_pointer;
int disk_write_page_pointer;
int disk_write_offset;

int disk_read_die_pointer;
int disk_read_block_pointer;
int disk_read_page_pointer;
int disk_read_offset;
struct k_fifo max30001_ecg_data_fifo;
struct k_fifo max30001_bioz_data_fifo;
struct k_fifo ble_stream_fifo;




struct ble_stream_data {
    void *fifo_reserved;
    int32_t ecg_data[8];
    int32_t bioz_data;
};

struct max30001_ecg_fifo_data_item_t {
    void *fifo_reserved;
    int32_t data;
};

struct max30001_bioz_fifo_data_item_t {
    void *fifo_reserved;
    int32_t data;
};

void stream_buffer(uint8_t *data, int size, int channel);
// int reset_flash_pointers(void)

#endif /* MAX30001_DATA_HANDLER_H */
#endif /* MAX30001_DATA_HANDLER_H */
