/*
 * HyperFi Slave CSI Hook
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Pacport Inc.
 *
 * Registers an esp_wifi CSI callback on the slave (C5) side, serializes each
 * CSI frame into a compact flat buffer, and sends it up to the P4 host via
 * esp_hosted_send_custom_data(HYPERFI_CSI_EVENT_ID, ...).
 *
 * Host-side consumer: WT99P4C5-S1/main/main.cpp using
 *   esp_hosted_register_custom_callback(HYPERFI_CSI_EVENT_ID, cb, ctx)
 *
 * See hyperfi/docs/adr/ADR-021-csi-passthrough-over-esp-hosted.md
 * and     hyperfi/docs/adr/ADR-022-custom-rpc-backport-to-esp-hosted-v2013.md
 */

#ifndef __SLAVE_CSI_HOOK_H
#define __SLAVE_CSI_HOOK_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Custom RPC event ID for HyperFi CSI frames (slave → host).
 * Picked from the high user-reserved range; Espressif core IDs top out at 789
 * (Event_MemMonitor in upstream, Event_Max in our v2.0.13 backport = 790).
 * 0x2001 leaves plenty of headroom and clearly flags "HyperFi" origin.
 */
#define HYPERFI_CSI_EVENT_ID  0x2001

/**
 * On-wire payload layout (flat bytes sent via esp_hosted_send_custom_data):
 *
 *   offset  0:  u32  timestamp_us       (little-endian; esp_timer_get_time() low 32 bits)
 *   offset  4:  i8   rssi
 *   offset  5:  i8   noise_floor
 *   offset  6:  u16  seq                (little-endian; from wifi_csi_info_t.rx_seq)
 *   offset  8:  u16  len                (little-endian; # of bytes in iq_buf[])
 *   offset 10:  u8[6] mac               (source MAC from wifi_csi_info_t)
 *   offset 16:  u8   bw                 (0 = HT20, 1 = HT40 — currently always 0 on C5)
 *   offset 17:  u8[3] reserved           (zero padding to 20-byte header)
 *   offset 20:  u8[len] iq_buf          (raw CSI I/Q data as returned by esp_wifi)
 *
 * Total frame size = 20 + len. For HT20 (len = 106) that's 126 bytes. Well below
 * the 8166-byte Custom RPC payload ceiling documented in esp-hosted-mcu.
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;
    int8_t   rssi;
    int8_t   noise_floor;
    uint16_t seq;
    uint16_t len;
    uint8_t  mac[6];
    uint8_t  bw;
    uint8_t  reserved[3];
    /* iq_buf[] follows, length = .len */
} hyperfi_csi_wire_hdr_t;

/**
 * Initialize the CSI hook:
 *   - Configures wifi_csi_acquire_config_t (force_lltf=1, lltf_bit_mode=1)
 *   - Registers esp_wifi_set_csi_rx_cb() pointing at our internal callback
 *   - Enables CSI via esp_wifi_set_csi(true)
 *
 * Call after esp_wifi_start() succeeds on slave. Safe to call multiple times —
 * subsequent calls are no-ops.
 *
 * @return ESP_OK on success, ESP_FAIL on API error.
 */
esp_err_t slave_csi_hook_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SLAVE_CSI_HOOK_H */
