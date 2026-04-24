/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2026 Pacport Inc. (HyperFi backport for v2.0.13)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * HyperFi Custom RPC receive-path backport (ADR-022, 2026-04-23).
 *
 * This header exposes the RECEIVE half of the esp_hosted Custom RPC API
 * (originally introduced in esp_hosted v2.12.x). It was backported to
 * v2.0.13 because our P4 eco2 + PSRAM constraint pins us to v2.0.13
 * while our slave runs v2.12.6 and sends Rpc_Event_CustomRpc (msg_id 788).
 *
 * Send side (host → slave) is NOT backported — HyperFi's CSI pipeline
 * is slave-push only (slave sends CSI events, host consumes).
 *
 * See: hyperfi/docs/adr/ADR-022-custom-rpc-backport-to-esp-hosted-v2013.md
 */

#ifndef __ESP_HOSTED_MISC_H
#define __ESP_HOSTED_MISC_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Register callback for custom data reception from slave co-processor.
 *
 * @param msg_id_exp    Message ID to listen for (any uint32_t except 0xFFFFFFFF)
 * @param callback      Function called when data with matching msg_id is received
 *                      (NULL to deregister)
 * @param local_context Opaque pointer returned as-is to the callback on every
 *                      invocation. May be NULL. Caller is responsible for ensuring
 *                      the pointed-to object remains valid until deregistered.
 *
 * @return ESP_OK on success, ESP_ERR_* on failure
 *
 * @note Callback runs in RPC RX task context. Keep it fast — copy data and return.
 */
esp_err_t esp_hosted_register_custom_callback(uint32_t msg_id_exp,
    void (*callback)(uint32_t msg_id_recvd, const uint8_t *data_recvd, size_t data_len_recvd, void *local_context),
    void *local_context);

#ifdef __cplusplus
}
#endif

#endif /* __ESP_HOSTED_MISC_H */
