/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zephyr/drivers/gpio.h>
#include <app/drivers/blink.h>
#include <app_version.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#if 0
static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		return;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
	printk("connected to %s\n", addr_str);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

	printk("disconnected %s, reason %u\n", addr_str, reason);

	struct bt_conn_info connection_info;
	int err;

	err = bt_conn_get_info(conn, &connection_info);

	if (err) {
		printk("Failed to get conn info (err %d)\n", err);
		return;
	}

	/* Get the ID of the disconnected advertiser. */
	uint8_t id_current = connection_info.id;

	printk("Advertiser %d disconnected\n", id_current);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};
#endif

int main(void)
{
#if 0
    int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	/* Register connection callbacks. */
	err = bt_conn_cb_register(&conn_callbacks);
	if (err) {
		printk("Conn callback register failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
#endif
    printk("main start\n");
}

