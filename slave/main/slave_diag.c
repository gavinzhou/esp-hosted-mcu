/*
 * HyperFi Slave Diagnostics Channel — implementation
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Pacport Inc.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_log.h"
#include "esp_hosted_peer_data.h"
#include "slave_diag.h"

static const char *TAG = "hf_diag";

/* Drop-if-not-ready counter: useful to notice if diag() is called before
 * ESP-Hosted transport is up. Not exposed — just in-memory. */
static uint32_t s_diag_dropped = 0;

void hf_slave_diag(const char *fmt, ...)
{
    char buf[HYPERFI_SLAVE_DIAG_MAX_LEN];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }
    if (n >= (int)sizeof(buf)) {
        /* Output was truncated by vsnprintf. buf is still null-terminated. */
        n = sizeof(buf) - 1;
    }

    /* Include the trailing null in what we send so host can treat as C-string. */
    size_t send_len = (size_t)n + 1;

    esp_err_t err = esp_hosted_send_custom_data(HYPERFI_SLAVE_DIAG_EVENT_ID,
                                                (uint8_t *)buf, send_len);
    if (err != ESP_OK) {
        s_diag_dropped++;
        /* Don't recurse: use regular ESP_LOG so we don't call ourselves. */
        ESP_LOGW(TAG, "send failed 0x%x (dropped=%lu)",
                 err, (unsigned long)s_diag_dropped);
    }
}
