#if CONFIG_MAX30001
#include "max30001_data_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include "w25m02gw.h"
#include "ble_sensor_control.h"
#include "max30001.h"

#define ECG_FIFO_SIZE 32*3
#define BIOZ_FIFO_SIZE 8*3
#define ECG_FIFO_COUNT 32
#define BIOZ_FIFO_COUNT 8

#define FLASH_PAGE_SIZE 2048
#define FLASH_NUM_BLOCKS 1024
#define FLASH_NUM_PAGES 64
#define NUM_ECG_BIOZ_DATA_PAIR_PER_PAGE 56
LOG_MODULE_REGISTER(MAX30001_DATA_HANDLER, LOG_LEVEL_DBG);

K_MEM_SLAB_DEFINE(ecg_fifo_data_slab, 8, 128, 4);
K_MEM_SLAB_DEFINE(bioz_fifo_data_slab, 8, 64, 4);
K_MEM_SLAB_DEFINE(ble_stream_slab, 40, 250, 4);

struct k_sem ecg_download_sem;
struct k_sem bioz_download_sem;


const struct device *max30001_dev = DEVICE_DT_GET(DT_NODELABEL(max30001));
const struct device *w25m02gw_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));
static int32_t ecg_fifo[ECG_FIFO_COUNT];
static int32_t bioz_fifo[BIOZ_FIFO_COUNT];
static int32_t flash_buffer[512]; // 2048 / 4

static uint8_t max30001_download_start;
static uint8_t max30001_sensor_start;

uint32_t block_to_download = 0;

static int write_pointer_buffer[4];
#define META_PARTITION	storage_partition
#define META_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(META_PARTITION)
#define META_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(META_PARTITION)
#define FLASH_PAGE_SIZE   4096

const struct device *flash_dev = META_PARTITION_DEVICE;

void save_flash_pointers(void)
{
    // flash_erase(flash_dev, META_PARTITION_OFFSET, FLASH_PAGE_SIZE);
    // write_pointer_buffer[0] = disk_write_block_pointer;
    // write_pointer_buffer[1] = disk_write_page_pointer;
    // write_pointer_buffer[2] = disk_write_offset;
    // write_pointer_buffer[3] = 0;
    // flash_write(flash_dev, META_PARTITION_OFFSET, write_pointer_buffer, 16);
}

void read_flash_pointers(void)
{

    // flash_read(flash_dev, META_PARTITION_OFFSET, write_pointer_buffer, 16);
    // disk_write_block_pointer = write_pointer_buffer[0];
    // disk_write_page_pointer = write_pointer_buffer[1];
    // disk_write_offset = write_pointer_buffer[2];
}

void stream_buffer(uint8_t *data, int size, int channel)
{
    const int max_chunk_size = 128 ;

    // Loop through the buffer and stream in chunks
    for (int offset = 0; offset < size; offset += max_chunk_size)
    {
        // Calculate the size of the current chunk
        int chunk_size = (size - offset > max_chunk_size) ? max_chunk_size : (size - offset);

        // Stream the current chunk
        stream_sensor_data(channel, data + offset, chunk_size);

    }
}
static int process_raw_ecg_fifo(uint8_t *ecg_buffer, int32_t *ecg_fifo, int size)
{
    /*
     * For each 24-bit sample (3 bytes):
     *   - Bits [23:6] => 18 bits of ECG data
     *   - Bits [5:3]  => ETAG
     *   - Bits [2:0]  => PTAG
     */

    int sample_count = size / 3;
    for (int i = 0; i < sample_count; i++)
    {
        /* Reassemble 24 bits from three bytes */
        uint32_t raw = ((uint32_t)ecg_buffer[i * 3 + 0] << 16) |
                       ((uint32_t)ecg_buffer[i * 3 + 1] << 8)  |
                        (uint32_t)ecg_buffer[i * 3 + 2];

        /* Extract ETAG (bits [5:3]) and PTAG (bits [2:0]) */
        uint8_t etag = (raw >> 3) & 0x07;
        uint8_t ptag = (raw >> 0) & 0x07;

        /* Extract the 18-bit ECG sample (bits [23:6]) */
        int32_t ecg_data = (raw >> 6) & 0x3FFFF; /* 18 bits */

        /* Sign-extend from 18 bits to 32 bits if necessary */
        if (ecg_data & (1 << 17)) {
            /* If bit 17 (the top of the 18-bit data) is 1, extend sign */
            ecg_data |= 0xFFFC0000; 
        }

        /*
         * Next, interpret ETAG & PTAG as needed. The tables (like Table 48 in 
         * the MAX30001 datasheet) show which tags are valid, overflow, end-of-file, etc.
         * 
         * Example usage:
         *   - If ETAG = 000 => Valid Sample
         *   - If ETAG = 001 => “Fast Mode Sample” (voltage not valid, but time-stamp is).
         *   - If ETAG = 010 => Last Valid Sample (EOF)
         *   - ...
         * 
         * You might conditionally store or skip the sample based on ETAG. 
         * For simplicity, we store them blindly here.
         */

        ecg_fifo[i] = ecg_data;
    }

    return sample_count;
}

static int process_raw_bioz_fifo(uint8_t *bioz_buffer, int32_t *bioz_fifo, int size)
{
    /*
     * For each 24-bit sample (3 bytes):
     *   - Bits [23:4] => 20 bits of BioZ data
     *   - Bit  3      => always 0
     *   - Bits [2:0]  => BTAG
     */

    int sample_count = size / 3;
    for (int i = 0; i < sample_count; i++)
    {
        /* Reassemble 24 bits from three bytes */
        uint32_t raw = ((uint32_t)bioz_buffer[i * 3 + 0] << 16) |
                       ((uint32_t)bioz_buffer[i * 3 + 1] << 8)  |
                        (uint32_t)bioz_buffer[i * 3 + 2];

        /* Extract BTAG (bits [2:0]) */
        uint8_t btag = raw & 0x07;

        /* Extract the 20-bit BioZ sample (bits [23:4]) */
        int32_t bioz_data = (raw >> 4) & 0xFFFFF; /* 20 bits */

        /* Sign-extend from 20 bits to 32 bits if necessary */
        if (bioz_data & (1 << 19)) {
            /* If bit 19 is 1, extend sign for 20-bit data */
            bioz_data |= 0xFFF00000; 
        }

        /*
         * Similar to ECG, you can check BTAG for validity:
         *   - 000 => Valid Sample, 001 => Over/Under Range, etc.
         * 
         * For simplicity, we simply store all samples here. You can add logic
         * to discard invalid ones or handle overflow, lead-off detection, etc.
         */

        bioz_fifo[i] = bioz_data;
    }

    return sample_count;
}
void stread_buffer(uint8_t *data, int size, int channel)
{

}
static int data_producer(void)
{
    
}
int reset_flash_pointers(void)
{
    disk_write_die_pointer = 0;
    disk_write_block_pointer = 0;
    disk_write_page_pointer = 0;
    disk_write_offset = 0;

    save_flash_pointers();

    disk_read_die_pointer = 0;
    disk_read_block_pointer = 0;
    disk_read_page_pointer = 0;
    disk_read_offset = 0;

    w25m02gw_erase_block(w25m02gw_dev, 0);
    return 0;
}
static int toggle_sensor(uint8_t enable)
{
    max30001_sensor_start = enable;
    printk("Toggled sensor to %d\n", enable);
    printk("max30001_sensor_start_value: %d\n", max30001_sensor_start);
    if(enable){
        uint8_t test_buffer[4] = {enable, 0x02, 0x03, 0x04};
        stream_sensor_data(DOWNLOAD, test_buffer, 4);
        reset_flash_pointers();
        max30001_download_start = 0;
    }
    return 0;

}

int retrieve_flash(void)
{
    disk_write_block_pointer = 1023;
    disk_write_page_pointer = 63;
    disk_write_offset = 0;
}
static int sensor_data_download_cb(uint8_t sw)
{
    printk("Sensor data download sw: %d\n", sw);
    max30001_download_start = sw;
    if(sw){
        disk_read_die_pointer = 0;
        disk_read_block_pointer = 0;
        disk_read_page_pointer = 0;
        disk_read_offset = 0;
    }
    return 0;
}

int sensor_update_download_block_cb(uint32_t block)
{
    printk("Block recieved here: %d\n", block);
    block_to_download = block;
    k_sem_give(&ecg_download_sem);
    k_sem_give(&bioz_download_sem);
    return 0;
}
size_t read_flashed_data_size(void)
{
    return disk_write_page_pointer * 2048 + disk_write_block_pointer * 2048 * 64;
}
static int t_ecg_data_producer(void)
{
    uint8_t ecg_buffer[ECG_FIFO_SIZE];
    int err;
    LOG_INF("Initializing data handler: %d\n", CONFIG_MAX30001);
    printk("max30001_sensor_start_value: %d\n", max30001_sensor_start);
    while(1)
    {
        if(max30001_sensor_start){
            LOG_INF("Data handler running\n");
            max30001_read_efifo(max30001_dev, ecg_buffer, ECG_FIFO_SIZE);
            process_raw_ecg_fifo(ecg_buffer, ecg_fifo, ECG_FIFO_SIZE);

            for (int i = 0; i < ECG_FIFO_COUNT; i++)
            {
                struct max30001_ecg_fifo_data_item_t *data_item;
                // printk("Requesting memory\n");
                err = k_mem_slab_alloc(&ecg_fifo_data_slab, (void **)&data_item, K_FOREVER);
                // printk("Memory allocated\n");
                data_item->data = ecg_fifo[i];

                k_fifo_put(&max30001_ecg_data_fifo, data_item);

            }
            stream_sensor_data(CHANNEL1, ecg_fifo, ECG_FIFO_COUNT*4);

            
        }else{
            k_msleep(1);
        }

        k_yield();

    }
    return 0;
}
static int t_bioz_data_producer(void)
{
    uint8_t bioz_buffer[BIOZ_FIFO_SIZE];
    int err;
    while(1)
    {
        if(max30001_sensor_start){
            uint8_t test_buffer[4] = {0x01, 0x02, 0x03, 0x04};
            
            max30001_read_bfifo(max30001_dev, bioz_buffer, BIOZ_FIFO_SIZE);
            stream_sensor_data(DOWNLOAD, test_buffer, 4);
            process_raw_bioz_fifo(bioz_buffer, bioz_fifo, BIOZ_FIFO_SIZE);
            for (int i = 0; i < BIOZ_FIFO_COUNT; i++)
            {
                struct max30001_bioz_fifo_data_item_t *data_item;
                err = k_mem_slab_alloc(&bioz_fifo_data_slab, (void **)&data_item, K_FOREVER);
                data_item->data = bioz_fifo[i];
                k_fifo_put(&max30001_bioz_data_fifo, data_item);
            }

            stream_sensor_data(CHANNEL2, bioz_fifo, BIOZ_FIFO_COUNT*4);
            k_msleep(125);
        }else{
            k_msleep(1);
        }
        
        k_yield();

    }
    return 0;

}
static int t_max30001_ecg_flasher(void)
{
    int32_t ecg_data_flasher_fifo[512];
    int ecg_data_flasher_fifo_index = 0;

    while(1)
    {
        struct max30001_ecg_fifo_data_item_t *ecg_data_item = k_fifo_get(&max30001_ecg_data_fifo, K_FOREVER);
        if (ecg_data_item)
        {

            ecg_data_flasher_fifo[ecg_data_flasher_fifo_index] = ecg_data_item->data;
            k_mem_slab_free(&ecg_fifo_data_slab, (void *)ecg_data_item);
            ecg_data_flasher_fifo_index++;

        }

        if (ecg_data_flasher_fifo_index >= 512)
        {
            w25m02gw_write(w25m02gw_dev, disk_write_block_pointer, disk_write_page_pointer, ecg_data_flasher_fifo, 2048);
            ecg_data_flasher_fifo_index = 0;
            disk_write_page_pointer++;
            if (disk_write_page_pointer >= FLASH_NUM_PAGES)
            {
                disk_write_page_pointer = 0;
                disk_write_block_pointer++;
                w25m02gw_erase_block(w25m02gw_dev, disk_write_block_pointer);
            }

            save_flash_pointers();
        }
    }
}

static int t_max30001_bioz_flasher(void)
{
    int32_t bioz_data_flasher_fifo[512];
    int bioz_data_flasher_fifo_index = 0;

    while(1)
    {
        struct max30001_bioz_fifo_data_item_t *bioz_data_item = k_fifo_get(&max30001_bioz_data_fifo, K_FOREVER);
        if (bioz_data_item)
        {
            // memcpy(&bioz_data_flasher_fifo[bioz_data_flasher_fifo_index], bioz_data_item->data, 4);
            bioz_data_flasher_fifo[bioz_data_flasher_fifo_index] = bioz_data_item->data;
            k_mem_slab_free(&bioz_fifo_data_slab, (void *)bioz_data_item);
            bioz_data_flasher_fifo_index++;
        }

        if (bioz_data_flasher_fifo_index >= 512)
        {
            w25m02gw_write(w25m02gw_dev, disk_write_block_pointer, disk_write_page_pointer, bioz_data_flasher_fifo, 2048);
            bioz_data_flasher_fifo_index = 0;
            disk_write_page_pointer++;
            if (disk_write_page_pointer >= FLASH_NUM_PAGES)
            {
                disk_write_page_pointer = 0;
                disk_write_block_pointer++;
                w25m02gw_erase_block(w25m02gw_dev, disk_write_block_pointer);
            }

            save_flash_pointers();
        }
    }
}

static int t_download_ecg_thread()
{
    uint8_t ecg_buffer[2048];
    int err;
    while(1)
    {
        k_sem_take(&ecg_download_sem, K_FOREVER);
        for (int _page = 0 ; _page < 64; _page++)
        {   
            printk("Downloading from block: %d, page: %d\n", block_to_download, _page);
            w25m02gw_read(w25m02gw_dev, block_to_download, _page, 0, ecg_buffer, 2048);
            printk("First data: %d\n", ecg_buffer[0]);  
            stream_buffer(ecg_buffer, 2048, DOWNLOAD);
        }
        k_yield();
    }
    return 0;
}

static int t_download_bioz_thread()
{
    uint8_t bioz_buffer[2048];
    int err;

    while(1)
    {
        k_sem_take(&bioz_download_sem, K_FOREVER);
        for (int _page = 0 ; _page < 64; _page++)
        {   
            printk("Downloading from block: %d, page: %d\n", block_to_download, _page);
            w25m02gw_read(w25m02gw_dev, block_to_download, _page, 0, bioz_buffer, 2048);
            printk("First data: %d\n", bioz_buffer[0]);  
            stream_buffer(bioz_buffer, 2048, DOWNLOAD);
        }
        k_yield();
    }
    return 0;
}

static int init_max30001_data_handler(void)
{
    max30001_download_start = 0;
    max30001_sensor_start = 0;
    k_sem_init(&ecg_download_sem, 0, 1);
    k_sem_init(&bioz_download_sem, 0, 1);
    read_flash_pointers();
    k_fifo_init(&max30001_ecg_data_fifo);
    k_fifo_init(&max30001_bioz_data_fifo);
    const struct ble_sensor_ctrl_cb callbacks = {
        .sensor_switch_cb = toggle_sensor,
        // .sensor_read_data_cb = sensor_read_data_cb,
        .sensor_data_download_cb = sensor_data_download_cb,
        .sensor_update_download_block_cb = sensor_update_download_block_cb,
        .sensor_read_fifo_size_cb = read_flashed_data_size,
        .retrieve_flash_cb = retrieve_flash,
    };
    register_ble_cb(&callbacks);



}


K_THREAD_DEFINE(max30001_data_init_thread, 4096, init_max30001_data_handler, NULL, NULL, NULL, 1, 0, 0);
#if CONFIG_MAX30001_ECG
K_THREAD_DEFINE(max30001_ecg_data_producer_thread, 4096, t_ecg_data_producer, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(max30001_ecg_flash_thread, 4096*4, t_max30001_ecg_flasher, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(max30001_ecg_download_thread, 4096, t_download_ecg_thread, NULL, NULL, NULL, 3, 0, 0);
#endif

#if CONFIG_MAX30001_BIOZ
K_THREAD_DEFINE(max30001_bioz_data_producer_thread, 1024, t_bioz_data_producer, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(max30001_bioz_flash_thread, 4096*4, t_max30001_bioz_flasher, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(max30001_bioz_download_thread, 4096, t_download_bioz_thread, NULL, NULL, NULL, 3, 0, 0);
#endif



#endif


