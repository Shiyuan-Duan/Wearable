#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/reboot.h>

#include "ble_sensor_control.h"
#include "ble.h"
#define PRIO 7

LOG_MODULE_REGISTER(BLE_SENSOR_CONTROL);

/// @brief // Global variables
static bool stream_sensor_channel1_enabled;
static bool stream_sensor_channel2_enabled;
static bool stream_sensor_channel3_enabled;
static bool stream_sensor_channel4_enabled;
static bool stream_sensor_channel5_enabled;
static bool stream_sensor_channel6_enabled;
static bool stream_sensor_channel7_enabled;
static bool stream_sensor_channel8_enabled;
static bool stream_sensor_hr_enabled;
static bool stream_sensor_spo2_enabled;
static bool stream_sensor_temp_enabled;
static bool stream_sensor_download_enabled;

/// @brief // State Variables
static uint8_t _state_is_sensor_on = 0;
static bool _has_data = false;
static uint8_t is_charging = 0;
static uint8_t _state_battery_level = 0;
static char _recording_name[32] = "NI_recording";
static uint64_t _recording_start_time = 0;

// LAST RECORDING
static char _last_recording_name[32] = "NI_recording";
static uint64_t _last_recording_start_time = 0;
static uint32_t _last_recording_size = 0;

static uint8_t _ads1299_data[240] = {0};

static size_t sensor_fifo_size = 0;





/// @brief // Global Callbacks
static struct ble_sensor_ctrl_cb cb;

/// @brief // Attribute Callbacks
// #region Attribute Callbacks

static void ble_ccc_stream_channel1_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel1_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel2_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel2_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel3_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel3_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel4_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel4_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel5_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel5_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel6_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel6_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel7_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel7_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_channel8_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_channel8_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_hr_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_hr_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_spo2_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_spo2_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_temp_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_temp_enabled = (value == BT_GATT_CCC_NOTIFY);
}
static void ble_ccc_stream_download_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	stream_sensor_download_enabled = (value == BT_GATT_CCC_NOTIFY);
}


static ssize_t attr_cb_write_sensor_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value;
	memcpy(&value, buf, sizeof(uint8_t));

	if(value == 1){
		printk("Sensor is turning on\n");
		printk("Is sensor switch cb registered? %d\n", cb.sensor_switch_cb != NULL);
		printk("Sensor switch cb address: %p\n", cb.sensor_switch_cb);
		cb.sensor_switch_cb(value);

		printk("Did I get to here?\n");
	}else{
		cb.sensor_switch_cb(value);
	}

	_state_is_sensor_on = value;



	return len;
}
static ssize_t attr_cb_read_sensor_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
    uint8_t sensor_sw_value = _state_is_sensor_on; 

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &sensor_sw_value, sizeof(sensor_sw_value));
}

static ssize_t attr_cb_read_battery_level(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	uint8_t battery_level = _state_battery_level; 

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &battery_level, sizeof(battery_level));
}

static ssize_t attr_cb_write_sensor_data_download_sw(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t *)buf);

	if(value == 1){
		cb.sensor_data_download_cb(1);
	}else{
		cb.sensor_data_download_cb(0);
	}

	return len;
}

static ssize_t attr_cb_bt_write_reset(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint8_t value = *((uint8_t *)buf);

	if (!value) {
		sys_reboot(SYS_REBOOT_COLD);
		printk("Resetting\n");
	}

	return len;
}

static ssize_t attr_cb_read_recording_name(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, _recording_name, sizeof(_recording_name));
}
static ssize_t attr_cb_bt_write_reset_recording_name(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

	memcpy(_recording_name, buf, MIN(len, 32));
	_recording_name[len] = '\0';

	return len;
}

static ssize_t attr_cb_read_recording_start_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &_recording_start_time, sizeof(_recording_start_time));
}
static ssize_t attr_cb_bt_write_reset_recording_start_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{

	if(len > 8){
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(&_recording_start_time, buf, len); 

	return len;
}

static ssize_t attr_cb_bt_add_event(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (len >= 2) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint16_t value = *((uint16_t *)buf);

	cb.sensor_add_event_cb(value);

	return len;
}

static ssize_t attr_cb_read_last_recording_name(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, _last_recording_name, sizeof(_last_recording_name));
}
static ssize_t attr_cb_read_last_recording_start_time(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &_last_recording_start_time, sizeof(_last_recording_start_time));
}
static ssize_t attr_cb_read_last_recording_size(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &_last_recording_size, sizeof(_last_recording_size));
}

static ssize_t attr_cb_read_data(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	printk("Trying to call cb\n");
	cb.sensor_read_data_cb(_ads1299_data);
	printk("Triggered cb\n");
	return bt_gatt_attr_read(conn, attr, buf, len, offset, _ads1299_data, sizeof(_ads1299_data));
}

static ssize_t attr_cb_read_fifo_size(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset) {
	size_t fifo_size = cb.sensor_read_fifo_size_cb(); 

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &fifo_size, sizeof(fifo_size));
}

// #endregion


/// @brief // BLE Attribute table declaration
// #region Attribute Table
BT_GATT_SERVICE_DEFINE(
	exg_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL1, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel1_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL2, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel2_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL3, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel3_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL4, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel4_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL5, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel5_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL6, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel6_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL7, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel7_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_CHANNEL8, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_channel8_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_HR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_hr_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_SPO2, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_spo2_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_TEMP, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_temp_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_DOWNLOAD, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE, NULL,
			       NULL, NULL),
	BT_GATT_CCC(ble_ccc_stream_download_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),


	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_SW, BT_GATT_CHRC_WRITE|BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, attr_cb_read_sensor_sw, attr_cb_write_sensor_sw, &_state_is_sensor_on),
	BT_GATT_CHARACTERISTIC(BT_UUID_RESET, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, attr_cb_bt_write_reset, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_STREAM_SW, BT_GATT_CHRC_WRITE|BT_GATT_CHRC_READ, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, NULL, NULL, NULL), // Not used, consider changing to other state manager.
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_DOWNLOAD_DATA_SW, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, attr_cb_write_sensor_data_download_sw, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_BATTERY_LEVEL, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, attr_cb_read_battery_level, NULL, &_state_battery_level),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_RECORDING_START_TIME, BT_GATT_CHRC_READ|BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ|BT_GATT_PERM_WRITE, attr_cb_read_recording_start_time, attr_cb_bt_write_reset_recording_start_time, &_recording_start_time),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_NEW_RECORDING_NAME, BT_GATT_CHRC_READ|BT_GATT_CHRC_WRITE, BT_GATT_PERM_READ|BT_GATT_PERM_WRITE, attr_cb_read_recording_name, attr_cb_bt_write_reset_recording_name, &_recording_name),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_ADD_EVENT, BT_GATT_CHRC_WRITE, BT_GATT_PERM_WRITE, NULL, attr_cb_bt_add_event, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LAST_RECORDING_NAME, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, attr_cb_read_last_recording_name, NULL, &_last_recording_name),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LAST_RECORDING_START_TIME, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, attr_cb_read_last_recording_start_time, NULL, &_last_recording_start_time),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_LAST_RECORDING_SIZE, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, attr_cb_read_last_recording_size, NULL, &_last_recording_size),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_READ_DATA, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, attr_cb_read_data, NULL, _ads1299_data),
	BT_GATT_CHARACTERISTIC(BT_UUID_SENSOR_FLASH_FIFO_SIZE, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, attr_cb_read_fifo_size, NULL, &sensor_fifo_size),
);


// #endregion

/// @brief // BLE Control Function Implementation
int stream_sensor_data(enum SensorType sensor_type, uint32_t *sensor_value, ssize_t size)
{
	struct bt_gatt_attr *attr;
	switch (sensor_type) {
		case CHANNEL1:
			attr = &exg_service.attrs[1];
			if (!stream_sensor_channel1_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL2:
			attr = &exg_service.attrs[4];
			if (!stream_sensor_channel2_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL3:
			attr = &exg_service.attrs[7];
			if (!stream_sensor_channel3_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL4:
			attr = &exg_service.attrs[10];
			if (!stream_sensor_channel4_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL5:
			attr = &exg_service.attrs[13];
			if (!stream_sensor_channel5_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL6:
			attr = &exg_service.attrs[16];
			if (!stream_sensor_channel6_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL7:
			attr = &exg_service.attrs[19];
			if (!stream_sensor_channel7_enabled) {
				return -EACCES;
			}
			break;
		case CHANNEL8:
			attr = &exg_service.attrs[22];
			if (!stream_sensor_channel8_enabled) {
				return -EACCES;
			}
			break;
		case HR:
			attr = &exg_service.attrs[25];
			if (!stream_sensor_hr_enabled) {
				return -EACCES;
			}
			break;
		case SPO2:
			attr = &exg_service.attrs[28];
			if (!stream_sensor_spo2_enabled) {
				return -EACCES;
			}
			break;
		case TEMP:
			attr = &exg_service.attrs[31];
			if (!stream_sensor_temp_enabled) {
				return -EACCES;
			}
			break;
		case DOWNLOAD:
			attr = &exg_service.attrs[34];
			if (!stream_sensor_download_enabled) {
				return -EACCES;
			}
			break;
		
		default:
			return -EINVAL;
	}
	
	return bt_gatt_notify(NULL, attr, sensor_value, size);
}

int register_ble_cb(struct ble_sensor_ctrl_cb *callbacks){
    cb.sensor_switch_cb = callbacks->sensor_switch_cb;
    cb.sensor_data_download_cb = callbacks->sensor_data_download_cb;
	cb.sensor_add_event_cb = callbacks->sensor_add_event_cb;
	cb.sensor_read_data_cb = callbacks->sensor_read_data_cb;
	cb.sensor_read_fifo_size_cb = callbacks->sensor_read_fifo_size_cb;
    return 0;
};

int state_set_battery_level(uint8_t level){
	_state_battery_level = level;
	LOG_INF("Battery level is set to %d", level);
	return 0;
}

int state_get_battery_level(void){
	return _state_battery_level;
}

int state_set_has_data(int has_data){
	_has_data = has_data;
	return 0;
}

int state_get_has_data(void){
	return _has_data;
}

int state_set_is_charging(uint8_t is_charging){
	is_charging = is_charging;
	return 0;
}

int state_get_is_charging(void){
	return is_charging;
}

int state_set_last_recording_name(char *name){
	memcpy(_last_recording_name, name, strlen(name));
	return 0;
}

int state_get_last_recording_name(char *name){
	memcpy(name, _last_recording_name, strlen(_last_recording_name));
	return 0;
}

int state_set_last_recording_start_time(uint64_t start_time){
	_last_recording_start_time = start_time;
	return 0;
}

uint64_t state_get_last_recording_start_time(void){
	return _last_recording_start_time;
}

int state_set_last_recording_size(uint32_t size){
	_last_recording_size = size;
	return 0;
}

uint32_t state_get_last_recording_size(void){
	return _last_recording_size;
}

