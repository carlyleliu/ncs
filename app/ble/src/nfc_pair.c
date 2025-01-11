
/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>

#include <dk_buttons_and_leds.h>

#include <nfc_t4t_lib.h>
#include <nfc/t4t/ndef_file.h>

#include <nfc/ndef/msg.h>
#include <nfc/ndef/ch.h>
#include <nfc/ndef/ch_msg.h>
#include <nfc/ndef/le_oob_rec.h>
#include <nfc/ndef/le_oob_rec_parser.h>

#include <nfc/tnep/tag.h>
#include <nfc/tnep/ch.h>

#include <zephyr/settings/settings.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define K_POOL_EVENTS_CNT (NFC_TNEP_EVENTS_NUMBER + 1)

#define NDEF_MSG_BUF_SIZE 256
#define AUTH_SC_FLAG 0x08

#define NFC_FIELD_LED DK_LED2
#define CON_STATUS_LED DK_LED1

#define KEY_BOND_REMOVE_MASK DK_BTN4_MSK

#define NFC_NDEF_LE_OOB_REC_PARSER_BUFF_SIZE 150
#define NFC_TNEP_BUFFER_SIZE 1024

#define NFC_PAIR_THREAD_STACK_SIZE 1024
#define NFC_PAIR_THREAD_PRIORITY 5

static struct bt_le_oob oob_local;
static struct k_work adv_work;
static uint8_t conn_cnt;
static uint8_t tk_value[NFC_NDEF_LE_OOB_REC_TK_LEN];
static uint8_t remote_tk_value[NFC_NDEF_LE_OOB_REC_TK_LEN];
static struct bt_le_oob oob_remote;

/* Bonded address queue. */
K_MSGQ_DEFINE(bonds_queue,
	      sizeof(bt_addr_le_t),
	      CONFIG_BT_MAX_PAIRED,
	      4);

static const struct bt_data ad[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static struct k_poll_signal pair_signal;
static struct k_poll_event events[K_POOL_EVENTS_CNT];
static uint8_t tnep_buffer[NFC_TNEP_BUFFER_SIZE];
static uint8_t tnep_swap_buffer[NFC_TNEP_BUFFER_SIZE];
static bool use_remote_tk;
static bool adv_permission;

int nfc_pair_thread_entry_point(void *, void *, void *);

LOG_MODULE_REGISTER(nfc_pair_thread, LOG_LEVEL_INF);
K_THREAD_DEFINE(nfc_pair_tid, NFC_PAIR_THREAD_STACK_SIZE,
                nfc_pair_thread_entry_point, NULL, NULL, NULL,
                NFC_PAIR_THREAD_PRIORITY, 0, 0);

static int tk_value_generate(void)
{
	int err;

	err = bt_rand(tk_value, sizeof(tk_value));
	if (err) {
		LOG_ERR("Random TK value generation failed: %d", err);
	}

	return err;
}

static void pair_key_generate_init(void)
{
	k_poll_signal_init(&pair_signal);
	k_poll_event_init(&events[NFC_TNEP_EVENTS_NUMBER],
			  K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &pair_signal);
}

static int paring_key_generate(void)
{
	int err;

	LOG_INF("Generating new pairing keys");

	err = bt_le_oob_get_local(BT_ID_DEFAULT, &oob_local);
	if (err) {
		LOG_ERR("Error while fetching local OOB data: %d", err);
	}

	return tk_value_generate();
}

static void paring_key_process(void)
{
	int err;

	if (events[NFC_TNEP_EVENTS_NUMBER].state == K_POLL_STATE_SIGNALED) {
		err = paring_key_generate();
		if (err) {
			LOG_ERR("Pairing key generation error: %d", err);
		}

		k_poll_signal_reset(events[NFC_TNEP_EVENTS_NUMBER].signal);
		events[NFC_TNEP_EVENTS_NUMBER].state = K_POLL_STATE_NOT_READY;
	}
}

static void bond_find(const struct bt_bond_info *info, void *user_data)
{
	int err;
	struct bt_conn *conn;

	/* Filter already connected peers. */
	conn = bt_conn_lookup_addr_le(BT_ID_DEFAULT, &info->addr);
	if (conn) {
		bt_conn_unref(conn);
		return;
	}

	err = k_msgq_put(&bonds_queue, (void *) &info->addr, K_NO_WAIT);
	if (err) {
		LOG_ERR("No space in the queue for the bond");
	}
}

static void advertising_start(void)
{
	k_msgq_purge(&bonds_queue);
	bt_foreach_bond(BT_ID_DEFAULT, bond_find, NULL);

	k_work_submit(&adv_work);
}

/**
 * @brief Callback function for handling NFC events.
 */
static void nfc_callback(void *context,
			 nfc_t4t_event_t event,
			 const uint8_t *data,
			 size_t data_length,
			 uint32_t flags)
{
	ARG_UNUSED(context);
	ARG_UNUSED(data);
	ARG_UNUSED(flags);

    LOG_INF("%s event:%d", __func__, event);
	switch (event) {
	case NFC_T4T_EVENT_FIELD_ON:
		nfc_tnep_tag_on_selected();
		adv_permission = true;
		break;

	case NFC_T4T_EVENT_FIELD_OFF:
		nfc_tnep_tag_on_selected();
		break;

	case NFC_T4T_EVENT_NDEF_READ:
		if (adv_permission) {
			advertising_start();
			adv_permission = false;
		}
		break;

	case NFC_T4T_EVENT_NDEF_UPDATED:
		if (data_length > 0) {
			nfc_tnep_tag_rx_msg_indicate(nfc_t4t_ndef_file_msg_get(data),
						     data_length);
		}

	default:
		break;
	}
}

static void advertising_continue(void)
{
	struct bt_le_adv_param adv_param;
	bt_addr_le_t addr;

	if (!k_msgq_get(&bonds_queue, &addr, K_NO_WAIT)) {
		char addr_buf[BT_ADDR_LE_STR_LEN];

		adv_param = *BT_LE_ADV_CONN_DIR(&addr);
		adv_param.options |= BT_LE_ADV_OPT_DIR_ADDR_RPA;

		int err = bt_le_adv_start(&adv_param, NULL, 0, NULL, 0);

		if (err) {
			LOG_ERR("Directed advertising failed to start");
			return;
		}

		bt_addr_le_to_str(&addr, addr_buf, BT_ADDR_LE_STR_LEN);
		LOG_INF("Direct advertising to %s started", addr_buf);
	} else {
		int err;

		adv_param = *BT_LE_ADV_CONN;
		adv_param.options |= BT_LE_ADV_OPT_ONE_TIME;
		err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);

		/* Re-enabling advertising when it is already active
		 * is not an error here.
		 */
		if (err && (err != -EALREADY)) {
			LOG_ERR("Advertising failed to start (err %d)", err);
			return;
		}

		LOG_INF("Regular advertising started");
	}
}

static void adv_handler(struct k_work *work)
{
	advertising_continue();
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void lesc_oob_data_set(struct bt_conn *conn,
			      struct bt_conn_oob_info *oob_info)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN];
	struct bt_conn_info info;

	err = bt_conn_get_info(conn, &info);
	if (err) {
		return;
	}

	struct bt_le_oob_sc_data *oob_data_local =
		oob_info->lesc.oob_config != BT_CONN_OOB_REMOTE_ONLY
					  ? &oob_local.le_sc_data
					  : NULL;
	struct bt_le_oob_sc_data *oob_data_remote =
		oob_info->lesc.oob_config != BT_CONN_OOB_LOCAL_ONLY
					  ? &oob_remote.le_sc_data
					  : NULL;

	if (oob_data_remote &&
	    bt_addr_le_cmp(info.le.remote, &oob_remote.addr)) {
		bt_addr_le_to_str(info.le.remote, addr, sizeof(addr));
		LOG_INF("No OOB data available for remote %s", addr);
		bt_conn_auth_cancel(conn);
		return;
	}

	if (oob_data_local &&
	    bt_addr_le_cmp(info.le.local, &oob_local.addr)) {
		bt_addr_le_to_str(info.le.local, addr, sizeof(addr));
		LOG_INF("No OOB data available for local %s", addr);
		bt_conn_auth_cancel(conn);
		return;
	}

	err = bt_le_oob_set_sc_data(conn, oob_data_local, oob_data_remote);
	if (err) {
		LOG_ERR("Error while setting OOB data: %d", err);
	}
}

static void legacy_tk_value_set(struct bt_conn *conn)
{
	int err;
	const uint8_t *tk = use_remote_tk ? remote_tk_value : tk_value;

	err = bt_le_oob_set_legacy_tk(conn, tk);
	if (err) {
		LOG_ERR("TK value set error: %d", err);
	}

	use_remote_tk = false;
}

static void auth_oob_data_request(struct bt_conn *conn,
				  struct bt_conn_oob_info *info)
{
	if (info->type == BT_CONN_OOB_LE_SC) {
		LOG_INF("LESC OOB data requested");
		lesc_oob_data_set(conn, info);
	}

	if (info->type == BT_CONN_OOB_LE_LEGACY) {
		LOG_INF("Legacy TK value requested");
		legacy_tk_value_set(conn);
	}
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);

	k_poll_signal_raise(&pair_signal, 0);
	bt_le_oob_set_sc_flag(false);
	bt_le_oob_set_legacy_flag(false);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d %s", addr, reason,
	       bt_security_err_to_str(reason));

	k_poll_signal_raise(&pair_signal, 0);
	bt_le_oob_set_sc_flag(false);
	bt_le_oob_set_legacy_flag(false);
}

static enum bt_security_err pairing_accept(struct bt_conn *conn,
				const struct bt_conn_pairing_feat *const feat)
{
	if (feat->oob_data_flag && (!(feat->auth_req & AUTH_SC_FLAG))) {
		bt_le_oob_set_legacy_flag(true);
	}

	return BT_SECURITY_ERR_SUCCESS;

}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
	.oob_data_request = auth_oob_data_request,
	.pairing_accept = pairing_accept,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		if (err == BT_HCI_ERR_ADV_TIMEOUT) {
			LOG_INF("Direct advertising to %s timed out", addr);
			k_work_submit(&adv_work);
		} else {
			LOG_INF("Failed to connect to %s 0x%02x %s", addr, err,
			       bt_hci_err_to_str(err));
		}

		return;
	}

	conn_cnt++;

	LOG_INF("Connected %s", addr);
	dk_set_led_on(CON_STATUS_LED);

	if (conn_cnt < CONFIG_BT_MAX_CONN) {
		advertising_start();
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	conn_cnt--;

	if (!conn_cnt) {
		dk_set_led_off(CON_STATUS_LED);
	}

	LOG_INF("Disconnected from %s, reason 0x%02x %s", addr, reason, bt_hci_err_to_str(reason));
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		LOG_INF("Security changed: %s level %u", addr, level);
	} else {
		LOG_INF("Security failed: %s level %u err %d %s", addr, level, err,
		       bt_security_err_to_str(err));
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};


/** .. include_startingpoint_pair_msg_rst */
static int tnep_initial_msg_encode(struct nfc_ndef_msg_desc *msg)
{
	int err;
	struct nfc_ndef_ch_msg_records ch_records;
	static struct nfc_ndef_le_oob_rec_payload_desc rec_payload;

	NFC_NDEF_LE_OOB_RECORD_DESC_DEF(oob_rec, '0', &rec_payload);
	NFC_NDEF_CH_AC_RECORD_DESC_DEF(oob_ac, NFC_AC_CPS_ACTIVE, 1, "0", 0);
	NFC_NDEF_CH_HS_RECORD_DESC_DEF(hs_rec, NFC_NDEF_CH_MSG_MAJOR_VER,
				       NFC_NDEF_CH_MSG_MINOR_VER, 1);

	memset(&rec_payload, 0, sizeof(rec_payload));

	rec_payload.addr = &oob_local.addr;
	rec_payload.le_sc_data = &oob_local.le_sc_data;
	rec_payload.tk_value = tk_value;
	rec_payload.local_name = bt_get_name();
	rec_payload.le_role = NFC_NDEF_LE_OOB_REC_LE_ROLE(
		NFC_NDEF_LE_OOB_REC_LE_ROLE_PERIPH_ONLY);
	rec_payload.appearance = NFC_NDEF_LE_OOB_REC_APPEARANCE(
		CONFIG_BT_DEVICE_APPEARANCE);
	rec_payload.flags = NFC_NDEF_LE_OOB_REC_FLAGS(BT_LE_AD_NO_BREDR);

	ch_records.ac = &NFC_NDEF_CH_AC_RECORD_DESC(oob_ac);
	ch_records.carrier = &NFC_NDEF_LE_OOB_RECORD_DESC(oob_rec);
	ch_records.cnt = 1;

	err =  nfc_ndef_ch_msg_hs_create(msg,
					 &NFC_NDEF_CH_RECORD_DESC(hs_rec),
					 &ch_records);
	if (err) {
		return err;
	}

	return nfc_tnep_initial_msg_encode(msg, NULL, 0);
}
/** .. include_endpoint_pair_msg_rst */

static int check_oob_carrier(const struct nfc_tnep_ch_record *ch_record,
			     const struct nfc_ndef_record_desc **oob_data)
{
	const struct nfc_ndef_ch_ac_rec *ac_rec = NULL;

	for (size_t i = 0; i < ch_record->count; i++) {
		if (nfc_ndef_le_oob_rec_check(ch_record->carrier[i])) {
			*oob_data = ch_record->carrier[i];
		}
	}

	if (!oob_data) {
		LOG_ERR("Connection Handover Requester not supporting OOB BLE");
		return -EINVAL;
	}

	/* Look for the corresponding Alternative Carrier Record. */
	for (size_t i = 0; i < ch_record->count; i++) {
		if (((*oob_data)->id_length == ch_record->ac[i].carrier_data_ref.length) &&
		    (memcmp((*oob_data)->id,
			    ch_record->ac[i].carrier_data_ref.data,
			    (*oob_data)->id_length) == 0)) {
			ac_rec = &ch_record->ac[i];
		}
	}

	if (!ac_rec) {
		LOG_ERR("No Alternative Carrier Record for OOB LE carrier");
		return -EINVAL;
	}

	/* Check carrier state */
	if ((ac_rec->cps != NFC_AC_CPS_ACTIVE) &&
	    (ac_rec->cps != NFC_AC_CPS_ACTIVATING)) {
		LOG_ERR("LE OBB Carrier inactive");
		return -EINVAL;
	}

	return 0;
}

static int oob_le_data_handle(const struct nfc_ndef_record_desc *rec,
			      bool request)
{
	int err;
	const struct nfc_ndef_le_oob_rec_payload_desc *oob;
	uint8_t desc_buf[NFC_NDEF_LE_OOB_REC_PARSER_BUFF_SIZE];
	uint32_t desc_buf_len = sizeof(desc_buf);

	err = nfc_ndef_le_oob_rec_parse(rec, desc_buf,
					&desc_buf_len);
	if (err) {
		LOG_ERR("Error during NDEF LE OOB Record parsing, err: %d.", err);
	}

	oob = (struct nfc_ndef_le_oob_rec_payload_desc *) desc_buf;

	nfc_ndef_le_oob_rec_printout(oob);

	if ((*oob->le_role != NFC_NDEF_LE_OOB_REC_LE_ROLE_CENTRAL_ONLY) &&
	    (*oob->le_role != NFC_NDEF_LE_OOB_REC_LE_ROLE_CENTRAL_PREFFERED)) {
		LOG_ERR("Unsupported Device LE Role");
		return -EINVAL;
	}

	if (oob->le_sc_data) {
		bt_le_oob_set_sc_flag(true);
		oob_remote.le_sc_data = *oob->le_sc_data;
		bt_addr_le_copy(&oob_remote.addr, oob->addr);
	}

	if (oob->tk_value) {
		bt_le_oob_set_legacy_flag(true);
		memcpy(remote_tk_value, oob->tk_value, sizeof(remote_tk_value));
		use_remote_tk = request;
	}

	advertising_start();

	return 0;
}

/** .. include_startingpoint_nfc_tnep_ch_tag_rst */
static int carrier_prepare(void)
{
	static struct nfc_ndef_le_oob_rec_payload_desc rec_payload;

	NFC_NDEF_LE_OOB_RECORD_DESC_DEF(oob_rec, '0', &rec_payload);
	NFC_NDEF_CH_AC_RECORD_DESC_DEF(oob_ac, NFC_AC_CPS_ACTIVE, 1, "0", 0);

	memset(&rec_payload, 0, sizeof(rec_payload));

	rec_payload.addr = &oob_local.addr;
	rec_payload.le_sc_data = &oob_local.le_sc_data;
	rec_payload.tk_value = tk_value;
	rec_payload.local_name = bt_get_name();
	rec_payload.le_role = NFC_NDEF_LE_OOB_REC_LE_ROLE(
		NFC_NDEF_LE_OOB_REC_LE_ROLE_PERIPH_ONLY);
	rec_payload.appearance = NFC_NDEF_LE_OOB_REC_APPEARANCE(
		CONFIG_BT_DEVICE_APPEARANCE);
	rec_payload.flags = NFC_NDEF_LE_OOB_REC_FLAGS(BT_LE_AD_NO_BREDR);

	return nfc_tnep_ch_carrier_set(&NFC_NDEF_CH_AC_RECORD_DESC(oob_ac),
				       &NFC_NDEF_LE_OOB_RECORD_DESC(oob_rec),
				       1);
}

#if defined(CONFIG_NFC_TAG_CH_REQUESTER)
static int tnep_ch_request_prepare(void)
{
	bt_le_adv_stop();
	return carrier_prepare();
}

static int tnep_ch_select_received(const struct nfc_tnep_ch_record *ch_select,
				   bool inactive)
{
	int err;
	const struct nfc_ndef_record_desc *oob_data = NULL;

	if (!ch_select->count) {
		return -EINVAL;
	}

    LOG_INF("%s", __func__);

	/* All alternative carrier are inactive */
	if (inactive) {
		/* Try send request again. */
		return carrier_prepare();
	}

	err = check_oob_carrier(ch_select, &oob_data);
	if (err) {
		return err;
	}

	err = oob_le_data_handle(oob_data, false);
	if (err) {
		return err;
	}

	return 0;

}
#endif /* defined(CONFIG_NFC_TAG_CH_REQUESTER) */

static int tnep_ch_request_received(const struct nfc_tnep_ch_request *ch_req)
{
	int err;
	const struct nfc_ndef_record_desc *oob_data = NULL;

	if (!ch_req->ch_record.count) {
		return -EINVAL;
	}

	err = check_oob_carrier(&ch_req->ch_record, &oob_data);
	if (err) {
		return err;
	}

	bt_le_adv_stop();

	err = oob_le_data_handle(oob_data, true);
	if (err) {
		return err;
	}

	return carrier_prepare();
}

static struct nfc_tnep_ch_cb ch_cb = {
#if defined(CONFIG_NFC_TAG_CH_REQUESTER)
	.request_msg_prepare = tnep_ch_request_prepare,
	.select_msg_recv = tnep_ch_select_received,
#endif
	.request_msg_recv = tnep_ch_request_received
};
/** .. include_endpoint_nfc_tnep_ch_tag_rst */

static void nfc_init(void)
{
	int err;

	/* TNEP init */
	err = nfc_tnep_tag_tx_msg_buffer_register(tnep_buffer, tnep_swap_buffer, sizeof(tnep_buffer));
	if (err) {
		LOG_ERR("Cannot register tnep buffer, err: %d", err);
		return;
	}

	err = nfc_tnep_tag_init(events, NFC_TNEP_EVENTS_NUMBER, nfc_t4t_ndef_rwpayload_set);
	if (err) {
		LOG_ERR("Cannot initialize TNEP protocol, err: %d", err);
		return;
	}

	/* Set up NFC */
	err = nfc_t4t_setup(nfc_callback, NULL);
	if (err) {
		LOG_ERR("Cannot setup NFC T4T library!");
		return;
	}

	err = nfc_tnep_tag_initial_msg_create(2, tnep_initial_msg_encode);
	if (err) {
		LOG_ERR("Cannot create initial TNEP message, err: %d", err);
	}

	err = nfc_tnep_ch_service_init(&ch_cb);
	if (err) {
		LOG_ERR("TNEP CH Service init error: %d", err);
		return;
	}

	/* Start sensing NFC field */
	err = nfc_t4t_emulation_start();
	if (err) {
		LOG_ERR("Cannot start emulation!");
		return;
	}

	LOG_INF("NFC configuration done");
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	int err;
	uint32_t buttons = button_state & has_changed;

	if (buttons & KEY_BOND_REMOVE_MASK) {
		err = bt_unpair(BT_ID_DEFAULT, NULL);
		if (err) {
			LOG_ERR("Bond remove failed err: %d", err);
		} else {
			LOG_INF("All bond removed\n");
		}
	}
}

int nfc_pair_thread_entry_point(void *unused1, void *unused2, void *unused3)
{
	int err;

	LOG_INF("Starting Bluetooth NFC Pairing Reference example");

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization info callbacks.");
		return 0;
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return 0;
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = paring_key_generate();
	if (err) {
		return 0;
	}

	k_work_init(&adv_work, adv_handler);
	pair_key_generate_init();
	nfc_init();

	for (;;) {
		k_poll(events, ARRAY_SIZE(events), K_FOREVER);
		nfc_tnep_tag_process();
		paring_key_process();
	}
}

