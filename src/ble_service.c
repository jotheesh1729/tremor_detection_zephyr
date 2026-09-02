#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "ble_service.h"

LOG_MODULE_REGISTER(ble_service, LOG_LEVEL_INF);

/* Custom 128-bit UUIDs, arbitrary (private use, no SIG registration needed
 * for a project like this -- same approach every custom GATT service uses,
 * e.g. Nordic's UART service).
 */
static const struct bt_uuid_128 tremor_svc_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xc9a00000, 0x1fdd, 0x4a7e, 0x9ab0, 0xb1a9c0ab0001));
static const struct bt_uuid_128 severity_chrc_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xc9a00001, 0x1fdd, 0x4a7e, 0x9ab0, 0xb1a9c0ab0001));
static const struct bt_uuid_128 episode_chrc_uuid =
	BT_UUID_INIT_128(BT_UUID_128_ENCODE(0xc9a00002, 0x1fdd, 0x4a7e, 0x9ab0, 0xb1a9c0ab0001));

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	ARG_UNUSED(value);
}

BT_GATT_SERVICE_DEFINE(tremor_svc, BT_GATT_PRIMARY_SERVICE(&tremor_svc_uuid),
			BT_GATT_CHARACTERISTIC(&severity_chrc_uuid.uuid, BT_GATT_CHRC_NOTIFY,
						BT_GATT_PERM_NONE, NULL, NULL, NULL),
			BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
			BT_GATT_CHARACTERISTIC(&episode_chrc_uuid.uuid, BT_GATT_CHRC_NOTIFY,
						BT_GATT_PERM_NONE, NULL, NULL, NULL),
			BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE));

#define SEVERITY_ATTR_IDX 1
#define EPISODE_ATTR_IDX  4

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_128_ENCODE(0xc9a00000, 0x1fdd, 0x4a7e, 0x9ab0,
							       0xb1a9c0ab0001)),
};
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("connection failed: 0x%02x", err);
	} else {
		LOG_INF("connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("disconnected: 0x%02x", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void ble_service_start(void)
{
	int err = bt_enable(NULL);

	if (err) {
		LOG_ERR("bt_enable failed: %d", err);
		return;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("advertising failed to start: %d", err);
		return;
	}

	LOG_INF("advertising started");
}

void ble_service_notify_severity(const struct tremor_window_result *result)
{
	uint8_t buf[4];

	buf[0] = (uint8_t)result->classification;
	buf[1] = (uint8_t)(result->severity * 100.0f);
	sys_put_le16((uint16_t)(result->peak_freq_hz * 10.0f), &buf[2]);

	bt_gatt_notify(NULL, &tremor_svc.attrs[SEVERITY_ATTR_IDX], buf, sizeof(buf));
}

void ble_service_notify_episode(const struct episode_record *record)
{
	uint8_t buf[6];

	buf[0] = (uint8_t)record->type;
	sys_put_le16((uint16_t)(record->duration_ms / 1000), &buf[1]);
	sys_put_le16((uint16_t)(record->dominant_freq_hz * 10.0f), &buf[3]);
	buf[5] = (uint8_t)(record->mean_severity * 100.0f);

	bt_gatt_notify(NULL, &tremor_svc.attrs[EPISODE_ATTR_IDX], buf, sizeof(buf));
}
