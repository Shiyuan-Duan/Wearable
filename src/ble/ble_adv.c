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
#include "ble.h"
#define PRIO 7

LOG_MODULE_REGISTER(BLE_ADV);

static struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
        BT_GAP_ADV_SLOW_INT_MIN,
        BT_GAP_ADV_SLOW_INT_MAX,
	NULL); /* Set to NULL for undirected advertising */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_VAL),
};

static struct bt_gatt_exchange_params exchange_params;

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (err) {
        printk("MTU exchange failed (err %u)\n", err);
    } else {
        uint16_t mtu = bt_gatt_get_mtu(conn);
        printk("MTU exchange successful, MTU size: %u\n", mtu);
    }
}


static void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
		return;
	}

    exchange_params.func = mtu_exchange_cb;

    // Request MTU exchange
    int ret = bt_gatt_exchange_mtu(conn, &exchange_params);
    if (ret) {
        printk("MTU exchange request failed (err %d)\n", ret);
    } else {
        printk("MTU exchange requested\n");
    }

	struct bt_conn_info info;
	ret = bt_conn_get_info(conn, &info);
	if (ret == 0) {
        printk("Connection Interval: %u units (%u ms)\n",
               info.le.interval, (info.le.interval * 1.25));
        printk("Latency: %u\n", info.le.latency);
        printk("Timeout: %u ms\n", info.le.timeout * 10);
    } else {
        printk("Failed to get connection info (err %d)\n", ret);
    }


	// struct bt_conn_le_phy_info phy_info;
    // ret = bt_conn_le_phy_get(conn, &phy_info);
    // if (ret == 0) {
    //     printk("TX PHY: %s, RX PHY: %s\n",
    //            (phy_info.tx_phy == BT_GAP_LE_PHY_1M) ? "1M" :
    //            (phy_info.tx_phy == BT_GAP_LE_PHY_2M) ? "2M" :
    //            (phy_info.tx_phy == BT_GAP_LE_PHY_CODED) ? "Coded" : "Unknown",
    //            (phy_info.rx_phy == BT_GAP_LE_PHY_1M) ? "1M" :
    //            (phy_info.rx_phy == BT_GAP_LE_PHY_2M) ? "2M" :
    //            (phy_info.rx_phy == BT_GAP_LE_PHY_CODED) ? "Coded" : "Unknown");
    // } else {
    //     printk("Failed to get PHY info (err %d)\n", ret);
    // }


	ssize_t mtu = bt_gatt_get_mtu(conn);
	printk("MTU: %d\n", mtu);


}

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);


}

struct bt_conn_cb connection_callbacks = {
	.connected = on_connected,
	.disconnected = on_disconnected,
};

int ble_adv_thread_entry(void)
{	

	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	bt_conn_cb_register(&connection_callbacks);

	LOG_INF("Bluetooth initialized\n");
	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)\n", err);
		return -1;
	}

	// stream_sensor_data(1);

	LOG_INF("Advertising successfully started with prioirty: %d\n", PRIO);
	return 0;
}

K_THREAD_DEFINE(ble_t, 1024, ble_adv_thread_entry, NULL, NULL, NULL, PRIO, 0, 0);


