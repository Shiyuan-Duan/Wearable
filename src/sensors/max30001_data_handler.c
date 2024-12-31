// #if CONFIG_MAX30001
#include "max30001_data_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include "w25m02gw.h"
#include "ble_sensor_control.h"
#include "max30001.h"

#define ECG_FIFO_SIZE 32*3
#define BIOZ_FIFO_SIZE 8*3
#define ECG_FIFO_COUNT 32
#define BIOZ_FIFO_COUNT 8

LOG_MODULE_REGISTER(MAX30001_DATA_HANDLER, LOG_LEVEL_DBG);
const struct device *max30001_dev = DEVICE_DT_GET(DT_NODELABEL(max30001));
static int32_t ecg_fifo[ECG_FIFO_COUNT];
static int32_t bioz_fifo[BIOZ_FIFO_COUNT];
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
static int init_data_handler(void)
{
    uint8_t ecg_buffer[ECG_FIFO_SIZE];
    uint8_t bioz_buffer[BIOZ_FIFO_SIZE];


    LOG_INF("Initializing data handler: %d\n", CONFIG_MAX30001);
    while(1)
    {
        LOG_INF("Data handler running\n");
        max30001_read_efifo(max30001_dev, ecg_buffer, ECG_FIFO_SIZE);
        process_raw_ecg_fifo(ecg_buffer, ecg_fifo, ECG_FIFO_SIZE);
        // max30001_read_bfifo(max30001_dev, bioz_buffer, BIOZ_FIFO_SIZE);
        // process_raw_bioz_fifo(bioz_buffer, bioz_fifo, BIOZ_FIFO_SIZE);

        stream_sensor_data(CHANNEL1, ecg_fifo, ECG_FIFO_COUNT*4);
        stream_sensor_data(CHANNEL2, bioz_fifo, BIOZ_FIFO_COUNT*4);

        k_yield();
    }

    return 0;
}


K_THREAD_DEFINE(max30001_data_init_thread, 2048, init_data_handler, NULL, NULL, NULL, 1, 0, 0);
// K_THREAD_DEFINE(ads1299_producer_thread, 4096, ads1299_data_producer, NULL, NULL, NULL, 2, 0, 0);
// K_THREAD_DEFINE(ads1299_streamer_thread, 4096, ble_stream_thread, NULL, NULL, NULL, 3, 0, 0);
// K_THREAD_DEFINE(ads1299_flash_thread, 4096, flash_thread, NULL, NULL, NULL, 3, 0, 0);

// #endif


