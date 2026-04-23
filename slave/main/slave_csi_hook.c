/*
 * HyperFi Slave CSI Hook
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Pacport Inc.
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_wifi_he_types.h"
#include "esp_timer.h"
#include "esp_hosted_peer_data.h"
#include "slave_csi_hook.h"
#include "slave_diag.h"

static const char *TAG = "csi_hook";

static bool s_csi_hook_initialized = false;

/* Stats (diagnostic — optional, but useful to confirm the hook is alive) */
static uint32_t s_csi_callback_count  = 0;
static uint32_t s_csi_send_ok_count   = 0;
static uint32_t s_csi_send_err_count  = 0;
static uint32_t s_csi_dropped_too_big = 0;

/* Max CSI len we expect per frame. HT20 L-LTF = 106, HT40 up to 226.
 * Total send size = 20 (header) + iq_len ≤ HYPERFI_CSI_MAX_IQ_LEN.
 * Cap at 1024 bytes to stay well inside the 8166-byte Custom RPC ceiling. */
#define HYPERFI_CSI_MAX_IQ_LEN 1024
#define HYPERFI_CSI_WIRE_BUF_SIZE (sizeof(hyperfi_csi_wire_hdr_t) + HYPERFI_CSI_MAX_IQ_LEN)

/* Single serialization buffer. CSI callback is invoked from the Wi-Fi task
 * (serialized by the Wi-Fi stack), so a static buffer is safe here — we never
 * have two in-flight CSI callbacks simultaneously. */
static uint8_t s_csi_wire_buf[HYPERFI_CSI_WIRE_BUF_SIZE];

/* CSI RX callback — fires on Wi-Fi task context each time the hardware
 * captures a CSI frame. We keep it SHORT: just memcpy + send. */
static void hyperfi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;

    if (!info || info->len == 0 || !info->buf) {
        /* Report first null/empty info only, to avoid spam */
        static bool reported_null = false;
        if (!reported_null) {
            reported_null = true;
            hf_slave_diag("CSI cb NULL/empty info (info=%p len=%d buf=%p)",
                          (void*)info,
                          info ? (int)info->len : -1,
                          info ? (void*)info->buf : NULL);
        }
        return;
    }

    s_csi_callback_count++;

    /* First CSI frame — tell host we are alive */
    if (s_csi_callback_count == 1) {
        hf_slave_diag("CSI cb FIRST FRAME: len=%u rssi=%d seq=%u",
                      (unsigned)info->len, info->rx_ctrl.rssi, (unsigned)info->rx_seq);
    }

    /* Cap length to our wire buffer */
    uint16_t iq_len = (info->len > HYPERFI_CSI_MAX_IQ_LEN)
        ? HYPERFI_CSI_MAX_IQ_LEN : (uint16_t)info->len;
    if (info->len > HYPERFI_CSI_MAX_IQ_LEN) {
        s_csi_dropped_too_big++;
    }

    hyperfi_csi_wire_hdr_t *hdr = (hyperfi_csi_wire_hdr_t *)s_csi_wire_buf;
    hdr->timestamp_us = (uint32_t)(esp_timer_get_time() & 0xFFFFFFFFu);
    hdr->rssi         = info->rx_ctrl.rssi;
    hdr->noise_floor  = info->rx_ctrl.noise_floor;
    hdr->seq          = info->rx_seq;
    hdr->len          = iq_len;
    memcpy(hdr->mac, info->mac, 6);
    hdr->bw           = 0;     /* HT20 on C5 for now (ADR-021 §Architecture) */
    memset(hdr->reserved, 0, sizeof(hdr->reserved));

    /* Copy raw IQ buffer after header */
    memcpy(s_csi_wire_buf + sizeof(*hdr), info->buf, iq_len);

    size_t total_len = sizeof(*hdr) + iq_len;

    esp_err_t err = esp_hosted_send_custom_data(HYPERFI_CSI_EVENT_ID,
                                                s_csi_wire_buf, total_len);
    if (err == ESP_OK) {
        s_csi_send_ok_count++;
    } else {
        s_csi_send_err_count++;
        /* Report first send error (and every 100th after that) */
        if (s_csi_send_err_count == 1 ||
            (s_csi_send_err_count & 0x63) == 0) {
            hf_slave_diag("CSI send_custom_data err=0x%x (err_count=%lu)",
                          err, (unsigned long)s_csi_send_err_count);
        }
    }

    /* Heartbeat every 64 callbacks */
    if ((s_csi_callback_count & 0x3F) == 0) {
        hf_slave_diag("CSI hb cb=%lu ok=%lu err=%lu drop_big=%lu",
                      (unsigned long)s_csi_callback_count,
                      (unsigned long)s_csi_send_ok_count,
                      (unsigned long)s_csi_send_err_count,
                      (unsigned long)s_csi_dropped_too_big);
    }
}

esp_err_t slave_csi_hook_init(void)
{
    hf_slave_diag("CSI init ENTER");

    if (s_csi_hook_initialized) {
        ESP_LOGW(TAG, "already initialized, skip");
        hf_slave_diag("CSI init already done, skip");
        return ESP_OK;
    }

    /* CSI config — mirrors c5_rx_5g/main.c post-issue-#18493 fix */
    wifi_csi_acquire_config_t csi_config = {
        .enable                  = 1,
        .acquire_csi_legacy      = 1,
        .acquire_csi_force_lltf  = 1,  /* forces L-LTF demod path (12-bit CSI) */
        .acquire_csi_ht20        = 1,
        .acquire_csi_ht40        = 1,
        .acquire_csi_su          = 1,
        .acquire_csi_mu          = 1,
        .acquire_csi_dcm         = 1,
        .acquire_csi_beamformed  = 1,
        .lltf_bit_mode           = 1,  /* L-LTF 12-bit precision (per QingzhaoYin) */
        .val_scale_cfg           = 0,
        .dump_ack_en             = 1,
    };

    esp_err_t err = esp_wifi_set_csi_rx_cb(hyperfi_csi_rx_cb, NULL);
    hf_slave_diag("set_csi_rx_cb err=0x%x", err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_csi_rx_cb failed: 0x%x", err);
        return err;
    }

    err = esp_wifi_set_csi_config(&csi_config);
    hf_slave_diag("set_csi_config err=0x%x", err);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_set_csi_config failed: 0x%x (using defaults)", err);
        /* non-fatal: defaults may still produce usable CSI */
    }

    err = esp_wifi_set_csi(true);
    hf_slave_diag("set_csi(true) err=0x%x", err);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_csi(true) failed: 0x%x", err);
        return err;
    }

    s_csi_hook_initialized = true;
    ESP_LOGI(TAG, "CSI hook initialized — forwarding to host via event 0x%04X",
             HYPERFI_CSI_EVENT_ID);
    hf_slave_diag("CSI init DONE ok, event_id=0x%04X", HYPERFI_CSI_EVENT_ID);
    return ESP_OK;
}
