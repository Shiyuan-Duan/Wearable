#if CONFIG_ADS1299
#include "ads1299_data_handler.h"
#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/logging/log.h>

#include "w25m02gw.h"
#include "ble_sensor_control.h"
LOG_MODULE_REGISTER(ADS1299_DATA_MON, LOG_LEVEL_DBG);


K_MEM_SLAB_DEFINE(ads1299_fifo_data_slab, 28, 100, 4);
K_MEM_SLAB_DEFINE(ble_stream_slab, sizeof(struct ble_stream_data), 250, 4);


static int ads1299_disk_read_block_pointer = 0;
static int ads1299_disk_read_page_pointer = 0;
static int ads1299_disk_read_offset = 0;
static int ads1299_sensor_state = 0;
static int ads1299_block_to_download = 0;
static uint8_t ads1299_download_start = 0;

struct k_sem ads1299_download_sem;

static uint8_t ads1299_read_cb_data_buffer[2048];

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



// helper functions
void stream_buffer(uint8_t *data, int size, int channel)
{
    const int max_chunk_size = 120 ;

    // Loop through the buffer and stream in chunks
    for (int offset = 0; offset < size; offset += max_chunk_size)
    {
        // Calculate the size of the current chunk
        int chunk_size = (size - offset > max_chunk_size) ? max_chunk_size : (size - offset);

        // Stream the current chunk
        stream_sensor_data(channel, data + offset, chunk_size);

    }
}

int retrieve_flash(void)
{
    disk_write_block_pointer = 1023;
    disk_write_page_pointer = 63;
    disk_write_offset = 0;
}

int sensor_update_download_block_cb(uint32_t block)
{
    printk("Block recieved here: %d\n", block);
    ads1299_block_to_download = block;
    k_sem_give(&ads1299_download_sem);

    return 0;
}


static void process_data(uint8_t *data_buffer, int32_t *channel_data, uint8_t *stripped_data, uint32_t *data_status)
{
    uint32_t status = 0;

    // Combine the 3 status bytes into a 24-bit status word
    status = (data_buffer[0] << 16) | (data_buffer[1] << 8) | data_buffer[2];
    memcpy(data_status, &status, sizeof(status));

    memcpy(stripped_data, &data_buffer[3], 24);

    for (int i = 0; i < 8; i++) {
        // Combine three bytes into a 24-bit signed integer (two's complement)
        channel_data[i] = (data_buffer[3 + (i * 3)] << 16) |
                          (data_buffer[4 + (i * 3)] << 8) |
                          (data_buffer[5 + (i * 3)]);

        // Sign extension for negative values
        if (channel_data[i] & 0x800000) {
            channel_data[i] |= 0xFF000000;
        }
    }
}
size_t read_flashed_data_size(void)
{
    return disk_write_block_pointer;
}
// size_t read_flashed_data_size(void)
// {
//     // Constants defining the sizes
//     const size_t DATA_BYTES_PER_ITEM = 24;         // Each data item is 24 bytes
//     const size_t DATA_ITEMS_PER_PAGE = 85;         // 2040 / 24
//     const size_t DATA_BYTES_PER_PAGE = 2040;       // Data bytes per page
//     const size_t PAGES_PER_BLOCK = 64;             // Number of pages per block
//     const size_t DATA_BYTES_PER_BLOCK = DATA_BYTES_PER_PAGE * PAGES_PER_BLOCK;

//     // Calculate total bytes written
//     size_t total_bytes_written = ((size_t)disk_write_block_pointer * DATA_BYTES_PER_BLOCK) +
//                                  ((size_t)disk_write_page_pointer * DATA_BYTES_PER_PAGE) +
//                                  (size_t)disk_write_offset;

//     // Calculate total bytes read
//     size_t total_bytes_read = ((size_t)ads1299_disk_read_block_pointer * DATA_BYTES_PER_BLOCK) +
//                               ((size_t)ads1299_disk_read_page_pointer * DATA_BYTES_PER_PAGE) +
//                               (size_t)ads1299_disk_read_offset;

//     // Calculate available data size
//     if (total_bytes_written >= total_bytes_read)
//     {
//         return total_bytes_written;
//     }
//     else
//     {
//         // Handle wraparound if necessary
//         // Assuming the flash size is large enough, we might need to account for wraparound
//         // For simplicity, return 0 if total_bytes_written is less than total_bytes_read
//         return total_bytes_written;
//     }
// }

static int toggle_sensor(uint8_t enable)
{
    const struct device *ads1299_dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));
    const struct device *w25m02gw_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));
    if (enable) {
        // Code to enable the sensor
        if (ads1299_sensor_state == 1) {
            LOG_INF("ADS1299 sensor already enabled");
            return 0;
        }

                
        disk_write_page_pointer = 0;
        disk_write_block_pointer = 0;
        disk_write_offset = 0;
        save_flash_pointers();  
        ads1299_disk_read_block_pointer = 0;
        ads1299_disk_read_page_pointer = 0;
        ads1299_disk_read_offset = 0;
        w25m02gw_erase_block(w25m02gw_dev, 0);
        ads1299_wakeup(ads1299_dev);


        
        ads1299_sensor_state = 1;
        LOG_INF("ADS1299 sensor enabled");
    } else {
        // Code to disable the sensor
        if (ads1299_sensor_state == 0) {
            LOG_INF("ADS1299 sensor already disabled");
            return 0;
        }
        ads1299_standby(ads1299_dev);
        LOG_INF("ADS1299 sensor disabled");
        ads1299_sensor_state = 0;
    }
    return 0;
}

static int sensor_data_download_cb(uint8_t sw)
{
    ads1299_download_start = sw;
    if(sw){
        ads1299_disk_read_block_pointer = 0;
        ads1299_disk_read_page_pointer = 0;
        ads1299_disk_read_offset = 0;

    }
    printk("ads1299_download_start: %d\n", ads1299_download_start);

    return 0;
}


static int sensor_read_data_cb(uint8_t *data)
{
    const struct device *ads1299_dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));
    const struct device *w25m02gw_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));
    
    uint8_t num_data_to_copy = 0;
    // Read a new flash page if at the beginning of a page
    if (ads1299_disk_read_offset == 0)
    {
        int ret = w25m02gw_read(w25m02gw_dev, ads1299_disk_read_block_pointer, ads1299_disk_read_page_pointer, 0, ads1299_read_cb_data_buffer, 2048);
        if (ret < 0) {
            LOG_ERR("Failed to read flash data");
            return 0;
        }
    }

    size_t bytes_remaining = 2040 - ads1299_disk_read_offset;

    if (bytes_remaining >= 240)
    {
        num_data_to_copy = 10; // 10 data points * 24 bytes = 240 bytes
    }
    else
    {
        num_data_to_copy = bytes_remaining / 24; // 5 data points * 24 bytes = 120 bytes
    }

    if (ads1299_disk_read_offset + (num_data_to_copy * 24) > 2040)
    {
        LOG_ERR("Attempting to read beyond buffer");
        return 0;
    }



    memcpy(data, &ads1299_read_cb_data_buffer[ads1299_disk_read_offset], num_data_to_copy * 24);
    // stream_sensor_data(GLUCOSE, data, num_data_to_copy * 24);
    ads1299_disk_read_offset += (num_data_to_copy * 24);


    if (ads1299_disk_read_offset >= 2040)
    {
        ads1299_disk_read_offset = 0;
        ads1299_disk_read_page_pointer++;
        if (ads1299_disk_read_page_pointer >= 64)
        {
            ads1299_disk_read_page_pointer = 0;
            ads1299_disk_read_block_pointer++;
        }
    }

    return num_data_to_copy;
}




// define consumer thread



// define producer thread
void ads1299_data_producer(void)
{
    const struct device *ads1299_dev = DEVICE_DT_GET(DT_NODELABEL(ads1299));
    while(1)
    {
        uint8_t ads1299_data[27];
        uint8_t ads1299_stripped_data[24];
        uint32_t ads1299_status;
        int32_t ads1299_channel_data[8];
        int ret = 0;
        ads1299_read_data(ads1299_dev, ads1299_data, 27);
        LOG_INF("Data: %d", ads1299_data[0]);

        process_data(ads1299_data, ads1299_channel_data, ads1299_stripped_data, &ads1299_status);

        struct ads1299_fifo_data_item_t *data_item;

        ret = k_mem_slab_alloc(&ads1299_fifo_data_slab, (void **)&data_item, K_FOREVER);

        memcpy(data_item->data, ads1299_stripped_data, sizeof(ads1299_stripped_data));

        struct ble_stream_data *stream_data;

        ret = k_mem_slab_alloc(&ble_stream_slab, (void **)&stream_data, K_FOREVER);

        // Copy channel data
        memcpy(stream_data->channel_data, ads1299_channel_data, sizeof(ads1299_channel_data));


        k_fifo_put(&ads1299_data_fifo, data_item);
        k_fifo_put(&ble_stream_fifo, stream_data);

        k_yield();
        LOG_INF("Data produced");

    }
}
////////////////////////////////////////////////////////////////////////
//////////////////////////STREAM THREAD/////////////////////////////////
////////////////////////////////////////////////////////////////////////
static int32_t channel_buffers[8][BLE_BUFFER_SIZE] = {0};
void ble_stream_thread(void)
{
    LOG_INF("BLE stream thread started");

    // Buffers to accumulate data
    
    int buffer_index = 0;
    int counter = 0;
    while (1)
    {
        // Wait indefinitely for data
        struct ble_stream_data *stream_data = k_fifo_get(&ble_stream_fifo, K_FOREVER);
        LOG_INF("Stream Data: %d", stream_data->channel_data[0]);

        if (stream_data)
        {
            // Accumulate data into buffers
            for (int ch = 0; ch < 8; ch++)
            {
                channel_buffers[ch][buffer_index] = stream_data->channel_data[ch];

            }

            buffer_index++;

            // Free the allocated memory back to the slab


            k_mem_slab_free(&ble_stream_slab, (void *)stream_data);
            // printk("Data: %d\n", stream_data->channel_data[0]); 
            uint32_t data = stream_data->channel_data[0];






            // When buffer_index reaches BLE_BUFFER_SIZE, send data over BLE
            if (buffer_index >= BLE_BUFFER_SIZE)
            {
                // Stream data over BLE for each channel
                for (int ch = 0; ch < 8; ch++)
                {
                    int ret = stream_sensor_data(ch, channel_buffers[ch], BLE_BUFFER_SIZE * sizeof(int32_t));
                    printk("Streaming channel %d data\n", ch + 1);
                    if (ret < 0)
                    {
                        LOG_ERR("Failed to stream CHANNEL%d data: %d", ch + 1, ret);
                    }
                }

                // Reset buffer_index
                buffer_index = 0;

            }

        

        }
        k_yield();
    }
}

////////////////////////////////////////////////////////////////////////
//////////////////////////FLASH THREAD//////////////////////////////////
////////////////////////////////////////////////////////////////////////
static uint8_t flash_data_buffer[2048] = {0};

void flash_thread(void)
{

    disk_write_die_pointer = 0;
    disk_write_block_pointer = 0;
    disk_write_page_pointer = 0;
    disk_write_offset = 0;
    

    const struct device *w25m02gw_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));



    while(1)
    {
        struct ads1299_fifo_data_item_t *data_item = k_fifo_get(&ads1299_data_fifo, K_FOREVER);

        if(data_item)
        {   
            memcpy(&flash_data_buffer[disk_write_offset], data_item->data, 24);
            k_mem_slab_free(&ads1299_fifo_data_slab, (void *)data_item);
            disk_write_offset += 24;
        }
        if((disk_write_block_pointer + disk_write_page_pointer + disk_write_offset) == 0) 
        {
            w25m02gw_erase_block(w25m02gw_dev, disk_write_block_pointer);
        }

        if(disk_write_offset >= 2040)
        {   
            int err = w25m02gw_write(w25m02gw_dev, disk_write_block_pointer, disk_write_page_pointer, flash_data_buffer, 2048);
            disk_write_page_pointer++;
            if(disk_write_page_pointer >= 64)
            {
                disk_write_page_pointer = 0;
                disk_write_block_pointer++;
                w25m02gw_erase_block(w25m02gw_dev, disk_write_block_pointer);
            }

            disk_write_offset = 0;

            save_flash_pointers();
        }
        k_yield();
    }
}

void init_data_handler(void)
{
    printk("Initializing data handler: %d\n", CONFIG_ADS1299);
    k_sem_init(&ads1299_download_sem, 0, 1);
    k_fifo_init(&ads1299_data_fifo);
    k_fifo_init(&ble_stream_fifo);
    read_flash_pointers();
    const struct ble_sensor_ctrl_cb callbacks = {
        .sensor_switch_cb = toggle_sensor,
        .sensor_update_download_block_cb = sensor_update_download_block_cb,
        .sensor_data_download_cb = sensor_data_download_cb,
        .sensor_read_fifo_size_cb = read_flashed_data_size,
        .retrieve_flash_cb = retrieve_flash,
    };
    register_ble_cb(&callbacks);
}

int t_ads1299_download_thread(void)
{
    const struct device *w25m02gw_dev = DEVICE_DT_GET(DT_NODELABEL(w25m02gw));
    uint8_t ads1299_buffer[2040];
    int err;

    while(1){
        k_sem_take(&ads1299_download_sem, K_FOREVER);

        for (int _page = 0 ; _page < 64; _page++)
        {   
            printk("Downloading from block: %d, page: %d\n", ads1299_block_to_download, _page);
            w25m02gw_read(w25m02gw_dev, ads1299_block_to_download, _page, 0, ads1299_buffer, 2040);
            printk("First data: %d\n", ads1299_buffer[0]);  
            stream_buffer(ads1299_buffer, 2040, DOWNLOAD);
        }

        k_yield();
    }

}

// static int t_download_bioz_thread()
// {
//     uint8_t bioz_buffer[2048];
//     int err;
//     while(1)
//     {
//         if(max30001_download_start){
//             printk("Downloading BIOZ data read pointer: %d, %d, %d and write pointer: %d, %d, %d\n", disk_read_block_pointer, disk_read_page_pointer, disk_read_offset, disk_write_block_pointer, disk_write_page_pointer, disk_write_offset);
//             w25m02gw_read(w25m02gw_dev, disk_read_block_pointer, disk_read_page_pointer, disk_read_offset, bioz_buffer, 2048);
//             stream_buffer(bioz_buffer, 2048, DOWNLOAD);
//             disk_read_page_pointer++;
//             if (disk_read_page_pointer >= FLASH_NUM_PAGES)
//             {
//                 disk_read_page_pointer = 0;
//                 disk_read_block_pointer++;
//             }
//             if(disk_read_block_pointer >= disk_write_block_pointer && disk_read_page_pointer >= disk_write_page_pointer){
//                 max30001_download_start = 0;
//                 printk("Download complete\n");
//             }
//         }else{
//             k_msleep(1);
//         }
//         k_yield();
//     }

//     return 0;
// }

K_THREAD_DEFINE(ads1299_data_init_thread, 2048, init_data_handler, NULL, NULL, NULL, 1, 0, 0);
K_THREAD_DEFINE(ads1299_producer_thread, 4096, ads1299_data_producer, NULL, NULL, NULL, 2, 0, 0);
K_THREAD_DEFINE(ads1299_streamer_thread, 4096, ble_stream_thread, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(ads1299_download_thread, 4096*4, t_ads1299_download_thread, NULL, NULL, NULL, 3, 0, 0);
K_THREAD_DEFINE(ads1299_flash_thread, 4096, flash_thread, NULL, NULL, NULL, 3, 0, 0);


#endif


