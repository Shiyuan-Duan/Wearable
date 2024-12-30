#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

#include "ble_sensor_control.h"
#define PRIO 17

#define MIN_BATTERY_VOLTAGE 385
#define MAX_BATTERY_VOLTAGE 535
#define SAMPLE_INTERVAL_MS 10
#define AVERAGE_INTERVAL_MS 10000

/* STEP 3.2 - Define a variable of type adc_dt_spec for each channel */
static const struct adc_dt_spec adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));
static const struct gpio_dt_spec batt_mon_en = GPIO_DT_SPEC_GET(DT_NODELABEL(batt_mon_en), gpios);

LOG_MODULE_REGISTER(ADC_MON, LOG_LEVEL_DBG);

int batt_mon() {
    int err;
    uint32_t sample_count = 0;
    int32_t sample_sum = 0;
    uint32_t sample_time = 0;
	uint8_t on_start = 1;
    
    /* STEP 4.1 - Define a variable of type adc_sequence and a buffer of type uint16_t */
    int16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        /* buffer size in bytes, not number of samples */
        .buffer_size = sizeof(buf),
    };

    /* STEP 3.3 - validate that the ADC peripheral (SAADC) is ready */
    if (!adc_is_ready_dt(&adc_channel)) {
        LOG_ERR("ADC controller device %s not ready", adc_channel.dev->name);
        return 0;
    }

    if (!gpio_is_ready_dt(&batt_mon_en)) {
        LOG_ERR("GPIO device %s not ready", batt_mon_en.port->name);
        return 0;
    }

    err = gpio_pin_configure_dt(&batt_mon_en, GPIO_OUTPUT_INACTIVE);

    /* STEP 3.4 - Setup the ADC channel */
    err = adc_channel_setup_dt(&adc_channel);
    if (err < 0) {
        LOG_ERR("Could not setup channel #%d (%d)", 0, err);
        return 0;
    }

    /* STEP 4.2 - Initialize the ADC sequence */
    err = adc_sequence_init_dt(&adc_channel, &sequence);
    if (err < 0) {
        LOG_ERR("Could not initialize sequence");
        return 0;
    }




    while (1) {
        int val_mv;

        err = gpio_pin_set_dt(&batt_mon_en, 1);
        /* STEP 5 - Read a sample from the ADC */
        err = adc_read(adc_channel.dev, &sequence);
        if (err < 0) {
            LOG_ERR("Could not read (%d)", err);
            gpio_pin_set_dt(&batt_mon_en, 0);
            k_sleep(K_MSEC(SAMPLE_INTERVAL_MS));
            continue;
        }

        val_mv = (int)buf;

        /* STEP 6 - Convert raw value to mV */
        err = adc_raw_to_millivolts_dt(&adc_channel, &val_mv);
        if (err < 0) {
            LOG_WRN(" (value in mV not available)");
        }

        gpio_pin_set_dt(&batt_mon_en, 0);
        
        /* Accumulate samples for averaging */
        sample_sum += val_mv;
        sample_count++;
        sample_time += SAMPLE_INTERVAL_MS;

        if (sample_time >= AVERAGE_INTERVAL_MS) {
            int avg_mv = sample_sum / sample_count;
            uint8_t battery_level = (avg_mv - MIN_BATTERY_VOLTAGE) * 100 / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE);
            LOG_INF("Average Battery Level: %d", battery_level);
            state_set_battery_level(battery_level);

            /* Reset counters */
            sample_sum = 0;
            sample_count = 0;
            sample_time = 0;
        }

        k_sleep(K_MSEC(SAMPLE_INTERVAL_MS));
    }

    return 0;
}

K_THREAD_DEFINE(batt_mon_t, 1024, batt_mon, NULL, NULL, NULL, PRIO, 0, 0);

