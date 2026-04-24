// Copyright 2015-2022 Espressif Systems (Shanghai) PTE LTD
/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */

#include <inttypes.h>
#include "rpc_core.h"
#include "rpc_slave_if.h"
#include "esp_log.h"
#include "esp_hosted_transport.h"

DEFINE_LOG_TAG(rpc_evt);

/* ======================================================================== */
/* HyperFi Custom RPC receive-path backport (ADR-022, 2026-04-23)          */
/* Ported from esp-hosted-mcu v2.12.6 @ f99ee55                             */
/* See hyperfi/docs/adr/ADR-022-custom-rpc-backport-to-esp-hosted-v2013.md  */
/* ======================================================================== */

#define HF_PEER_DATA_TRANSFER         1    /* always on in patched build */
#define HF_MAX_CUSTOM_MSG_HANDLERS    8    /* reasonable default */

#if HF_PEER_DATA_TRANSFER

/* Callback slots (empty slot has callback = NULL, msg_id = -1 is invalid sentinel) */
static struct {
	uint32_t msg_id;
	void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd, size_t data_len_recvd, void *local_context);
	void *local_context;
} hf_custom_callbacks[HF_MAX_CUSTOM_MSG_HANDLERS] = {
	[0 ... (HF_MAX_CUSTOM_MSG_HANDLERS - 1)] = {
		.msg_id = (uint32_t)-1,
		.callback = NULL
	}
};

static void *hf_custom_callbacks_mutex = NULL;

/* Register callback for specific message ID.
 * Pass callback = NULL to deregister. Returns SUCCESS/FAILURE. */
int rpc_evt_register_custom_callback(uint32_t msg_id_exp,
		void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd,
			size_t data_len_recvd, void *local_context),
		void *local_context)
{
	if (msg_id_exp == (uint32_t)-1) {
		ESP_LOGE(TAG, "Invalid message ID 0xFFFFFFFF");
		return FAILURE;
	}

	if (!hf_custom_callbacks_mutex) {
		hf_custom_callbacks_mutex = g_h.funcs->_h_create_mutex();
		if (!hf_custom_callbacks_mutex) {
			ESP_LOGE(TAG, "Failed to create custom callback mutex");
			return FAILURE;
		}
	}

	g_h.funcs->_h_lock_mutex(hf_custom_callbacks_mutex, HOSTED_BLOCK_MAX);

	/* Existing registration? */
	for (int i = 0; i < HF_MAX_CUSTOM_MSG_HANDLERS; i++) {
		if (hf_custom_callbacks[i].msg_id == msg_id_exp) {
			if (callback == NULL) {
				hf_custom_callbacks[i].msg_id = (uint32_t)-1;
				hf_custom_callbacks[i].callback = NULL;
				hf_custom_callbacks[i].local_context = NULL;
				ESP_LOGD(TAG, "Deregistered callback for msg_id %" PRIu32, msg_id_exp);
			} else {
				hf_custom_callbacks[i].callback = callback;
				hf_custom_callbacks[i].local_context = local_context;
				ESP_LOGD(TAG, "Updated callback for msg_id %" PRIu32, msg_id_exp);
			}
			g_h.funcs->_h_unlock_mutex(hf_custom_callbacks_mutex);
			return SUCCESS;
		}
	}

	if (callback == NULL) {
		ESP_LOGD(TAG, "Cannot deregister msg_id %" PRIu32 " - not registered", msg_id_exp);
		g_h.funcs->_h_unlock_mutex(hf_custom_callbacks_mutex);
		return FAILURE;
	}

	/* Empty slot for new registration */
	for (int i = 0; i < HF_MAX_CUSTOM_MSG_HANDLERS; i++) {
		if (hf_custom_callbacks[i].callback == NULL) {
			hf_custom_callbacks[i].msg_id = msg_id_exp;
			hf_custom_callbacks[i].callback = callback;
			hf_custom_callbacks[i].local_context = local_context;
			ESP_LOGD(TAG, "Registered callback for msg_id %" PRIu32, msg_id_exp);
			g_h.funcs->_h_unlock_mutex(hf_custom_callbacks_mutex);
			return SUCCESS;
		}
	}

	ESP_LOGW(TAG, "No space for callback (max %d)", HF_MAX_CUSTOM_MSG_HANDLERS);
	g_h.funcs->_h_unlock_mutex(hf_custom_callbacks_mutex);
	return FAILURE;
}

#endif /* HF_PEER_DATA_TRANSFER */

/* For new RPC event (from ESP to host), add up switch case for your message
 * In general, it is better to subscribe all events or notifications
 * at slave side & selective subscribe the events at host side.
 * This way, all the events reach at host and host will decide
 * if incoming event is expected to be entertained or dropped
 *
 * If you are concerned over battery usage, it is code further could be
 * optimized that only selective events are subscribed at slave and host both sides
 *
 * This function will copy rpc event from `Rpc` into
 * app structure `ctrl_cmd_t`
 * This function is called after
 * 1. Protobuf decoding is successful
 * 2. There is non NULL event callback is available
 **/
int rpc_parse_evt(Rpc *rpc_msg, ctrl_cmd_t *app_ntfy)
{
	if (!rpc_msg || !app_ntfy) {
		ESP_LOGE(TAG, "NULL rpc event or App struct\n");
		goto fail_parse_rpc_msg;
	}

	app_ntfy->msg_type = RPC_TYPE__Event;
	app_ntfy->msg_id = rpc_msg->msg_id;
	app_ntfy->resp_event_status = SUCCESS;

	switch (rpc_msg->msg_id) {

	case RPC_ID__Event_ESPInit: {
		ESP_LOGI(TAG, "EVENT: ESP INIT\n");
		break;
	} case RPC_ID__Event_Heartbeat: {
		ESP_LOGD(TAG, "EVENT: Heartbeat\n");
		RPC_FAIL_ON_NULL(event_heartbeat);
		app_ntfy->u.e_heartbeat.hb_num = rpc_msg->event_heartbeat->hb_num;
		break;
	} case RPC_ID__Event_AP_StaConnected: {
		wifi_event_ap_staconnected_t * p_a = &(app_ntfy->u.e_wifi_ap_staconnected);
		RpcEventAPStaConnected * p_c = rpc_msg->event_ap_sta_connected;

		RPC_FAIL_ON_NULL(event_ap_sta_connected);
		app_ntfy->resp_event_status = p_c->resp;

		if(SUCCESS==app_ntfy->resp_event_status) {
			RPC_FAIL_ON_NULL_PRINT(p_c->mac.data, "NULL mac");
			g_h.funcs->_h_memcpy(p_a->mac, p_c->mac.data, p_c->mac.len);
			ESP_LOGI(TAG, "EVENT: AP ->  sta connected mac[" MACSTR "] (len:%u)",
				MAC2STR(p_a->mac), p_c->mac.len);
		}
		p_a->aid = p_c->aid;
		p_a->is_mesh_child = p_c->is_mesh_child;

		break;
	} case RPC_ID__Event_AP_StaDisconnected: {
		wifi_event_ap_stadisconnected_t * p_a = &(app_ntfy->u.e_wifi_ap_stadisconnected);
		RpcEventAPStaDisconnected * p_c = rpc_msg->event_ap_sta_disconnected;

		ESP_LOGD(TAG, "EVENT: AP ->  sta disconnected");
		RPC_FAIL_ON_NULL(event_ap_sta_disconnected);
		app_ntfy->resp_event_status = p_c->resp;

		if(SUCCESS==app_ntfy->resp_event_status) {
			RPC_FAIL_ON_NULL_PRINT(p_c->mac.data, "NULL mac");
			g_h.funcs->_h_memcpy(p_a->mac, p_c->mac.data, p_c->mac.len);
			ESP_LOGI(TAG, "EVENT: AP ->  sta DISconnected mac[" MACSTR "] (len:%u)",
				MAC2STR(p_a->mac), p_c->mac.len);
		}

		p_a->aid = p_c->aid;
		p_a->is_mesh_child = p_c->is_mesh_child;
		p_a->reason = p_c->reason;

		break;
    } case RPC_ID__Event_WifiEventNoArgs: {
		RPC_FAIL_ON_NULL(event_wifi_event_no_args);
		app_ntfy->resp_event_status = rpc_msg->event_wifi_event_no_args->resp;
        ESP_LOGI(TAG, "Event [0x%lx] received", rpc_msg->event_wifi_event_no_args->event_id);
		app_ntfy->u.e_wifi_simple.wifi_event_id = rpc_msg->event_wifi_event_no_args->event_id;

		switch (rpc_msg->event_wifi_event_no_args->event_id) {
		/* basic events populated, not all */
		case WIFI_EVENT_WIFI_READY:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi Ready");
			break;
		case WIFI_EVENT_SCAN_DONE:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi scan done");
			break;
		case WIFI_EVENT_STA_START:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi Start");
			break;
		case WIFI_EVENT_STA_STOP:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi Stop");
			break;
		case WIFI_EVENT_STA_CONNECTED:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi Connected");
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi Disconnected");
			break;
		case WIFI_EVENT_STA_AUTHMODE_CHANGE:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi AuthMode change");
			break;
		case WIFI_EVENT_AP_START:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi AP Start");
			break;
		case WIFI_EVENT_AP_STOP:
			ESP_LOGI(TAG, "EVT rcvd: Wi-Fi AP stop");
			break;
		}
		break;
    } case RPC_ID__Event_StaScanDone: {
		RpcEventStaScanDone *p_c = rpc_msg->event_sta_scan_done;
		wifi_event_sta_scan_done_t *p_a = &app_ntfy->u.e_wifi_sta_scan_done;
		RPC_FAIL_ON_NULL(event_sta_scan_done);
		app_ntfy->resp_event_status = p_c->resp;
		ESP_LOGI(TAG, "Event Scan Done, %ld items", rpc_msg->event_sta_scan_done->scan_done->number);
		p_a->status = p_c->scan_done->status;
		p_a->number = p_c->scan_done->number;
		p_a->scan_id = p_c->scan_done->scan_id;
		break;
	} case RPC_ID__Event_StaConnected: {
		RPC_FAIL_ON_NULL(event_sta_connected);
		RPC_FAIL_ON_NULL(event_sta_connected->sta_connected);
		WifiEventStaConnected *p_c = rpc_msg->event_sta_connected->sta_connected;
		wifi_event_sta_connected_t *p_a = &app_ntfy->u.e_wifi_sta_connected;
		app_ntfy->resp_event_status = rpc_msg->event_sta_connected->resp;
		if (SUCCESS == app_ntfy->resp_event_status) {
			RPC_FAIL_ON_NULL_PRINT(p_c->ssid.data, "NULL SSID");
			g_h.funcs->_h_memcpy(p_a->ssid, p_c->ssid.data, p_c->ssid.len);
			p_a->ssid_len = p_c->ssid_len;
			RPC_FAIL_ON_NULL_PRINT(p_c->bssid.data, "NULL BSSID");
			g_h.funcs->_h_memcpy(p_a->bssid, p_c->bssid.data, p_c->bssid.len);
			p_a->channel = p_c->channel;
			p_a->authmode = p_c->authmode;
			p_a->aid = p_c->aid;
		}
		break;
	} case RPC_ID__Event_StaDisconnected: {
		RPC_FAIL_ON_NULL(event_sta_disconnected);
		RPC_FAIL_ON_NULL(event_sta_disconnected->sta_disconnected);
		WifiEventStaDisconnected *p_c = rpc_msg->event_sta_disconnected->sta_disconnected;
		wifi_event_sta_disconnected_t *p_a = &app_ntfy->u.e_wifi_sta_disconnected;
		app_ntfy->resp_event_status = rpc_msg->event_sta_connected->resp;
		if (SUCCESS == app_ntfy->resp_event_status) {
			RPC_FAIL_ON_NULL_PRINT(p_c->ssid.data, "NULL SSID");
			g_h.funcs->_h_memcpy(p_a->ssid, p_c->ssid.data, p_c->ssid.len);
			p_a->ssid_len = p_c->ssid_len;
			RPC_FAIL_ON_NULL_PRINT(p_c->bssid.data, "NULL BSSID");
			g_h.funcs->_h_memcpy(p_a->bssid, p_c->bssid.data, p_c->bssid.len);
			p_a->reason = p_c->reason;
			p_a->rssi = p_c->rssi;
		}
		break;
	}
#if HF_PEER_DATA_TRANSFER
	case RPC_ID__Event_CustomRpc: {
		/* HyperFi Custom RPC receive-path backport (ADR-022) */
		RpcEventCustomRpc *p_c = rpc_msg->event_custom_rpc;
		RPC_FAIL_ON_NULL(event_custom_rpc);
		app_ntfy->resp_event_status = p_c->resp;

		uint32_t msg_id = p_c->custom_event_id;
		const uint8_t *payload = p_c->data.data;
		size_t payload_len = p_c->data.len;

		bool callback_found = false;
		void (*cb)(uint32_t, const uint8_t *, size_t, void *) = NULL;
		void *cb_local_context = NULL;

		if (hf_custom_callbacks_mutex) {
			g_h.funcs->_h_lock_mutex(hf_custom_callbacks_mutex, HOSTED_BLOCK_MAX);
			for (int i = 0; i < HF_MAX_CUSTOM_MSG_HANDLERS; i++) {
				if (hf_custom_callbacks[i].msg_id == msg_id && hf_custom_callbacks[i].callback) {
					cb = hf_custom_callbacks[i].callback;
					cb_local_context = hf_custom_callbacks[i].local_context;
					callback_found = true;
					break;
				}
			}
			g_h.funcs->_h_unlock_mutex(hf_custom_callbacks_mutex);
		}

		/* Invoke callback outside mutex to avoid deadlock */
		if (callback_found && cb) {
			cb(msg_id, payload, payload_len, cb_local_context);
		} else {
			ESP_LOGI(TAG, "No callback registered for msg_id %" PRIu32 ", drop", msg_id);
		}
		break;
	}
#endif
	default: {
		ESP_LOGE(TAG, "Invalid/unsupported event[%u] received\n",rpc_msg->msg_id);
		goto fail_parse_rpc_msg;
		break;
	}

	}

	return SUCCESS;

fail_parse_rpc_msg:
	app_ntfy->resp_event_status = FAILURE;
	return FAILURE;
}
