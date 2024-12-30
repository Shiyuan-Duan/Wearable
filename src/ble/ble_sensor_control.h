#ifndef BLE_SENSOR_CONTROL_H_
#define BLE_SENSOR_CONTROL_H_

#include <zephyr/types.h>

enum SensorType {
	CHANNEL1,
	CHANNEL2,
	CHANNEL3,
	CHANNEL4,
	CHANNEL5,
	CHANNEL6,
	CHANNEL7,
	CHANNEL8,
	HR,
	SPO2,
	TEMP,
	DOWNLOAD
};



typedef int (*sensor_switch_cb_t) (uint8_t value);
typedef int (*sensor_data_download_cb_t) (uint8_t value);
typedef int (*sensor_add_event_cb_t) (uint16_t event_number);
typedef int (*sensor_read_data_cb_t) (uint8_t * data);
typedef size_t (*sensor_read_fifo_size_cb_t) (void);


struct ble_sensor_ctrl_cb  {
	sensor_switch_cb_t sensor_switch_cb;
	sensor_data_download_cb_t sensor_data_download_cb;
	sensor_add_event_cb_t sensor_add_event_cb;
	sensor_read_data_cb_t sensor_read_data_cb;
	sensor_read_fifo_size_cb_t sensor_read_fifo_size_cb;
};



int register_ble_cb(struct ble_sensor_ctrl_cb *callbacks);
int stream_sensor_data(enum SensorType sensor_type, uint32_t *sensor_value, ssize_t size);

/// @brief // BLE Control Function Header
int state_set_battery_level(uint8_t level);
int state_get_battery_level(void);
int state_set_has_data(int has_data);
int state_get_has_data(void);
int state_set_is_charging(uint8_t is_charging);
int state_get_is_charging(void);
int state_set_last_recording_name(char *name);
int state_get_last_recording_name(char *name);
int state_set_last_recording_start_time(uint64_t start_time);
uint64_t state_get_last_recording_start_time(void);



#endif /* BLE_SENSOR_CONTROL_H_ */