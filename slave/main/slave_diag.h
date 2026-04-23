/*
 * HyperFi Slave Diagnostics Channel
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2026 Pacport Inc.
 *
 * Provides a slave→host text-message channel over the ESP-Hosted Custom RPC
 * event transport. Allows slave-side code to surface diagnostic information
 * to the P4 host (and thereby to operators / ops tooling) WITHOUT requiring a
 * physical UART debug port on the shipped hardware.
 *
 * Design principle (Gavin CLAUDE.md §2.5 "做正确且难的事情"):
 *   Once HyperFi is deployed in care facilities, the only channel back to the
 *   maintainer is host→network. Slave visibility MUST go through host first.
 *   This diag channel is production-level infrastructure, not debug scaffolding.
 *
 * Usage (slave side):
 *     hf_slave_diag("CSI hook init, wifi_err=0x%x", err);
 *
 * Host side consumer: register_custom_callback(HYPERFI_SLAVE_DIAG_EVENT_ID, cb)
 * and print payload as a C string (it is null-terminated).
 *
 * Wire format: raw UTF-8 text, null-terminated. Max 256 bytes/message.
 *
 * See hyperfi/docs/adr/ADR-022-custom-rpc-backport-to-esp-hosted-v2013.md
 */

#ifndef __SLAVE_DIAG_H
#define __SLAVE_DIAG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Custom RPC event ID for slave diagnostic text messages (slave → host).
 * Companion to HYPERFI_CSI_EVENT_ID (0x2001); picked from the same user-
 * reserved range. One ID per logical channel keeps host-side dispatch trivial.
 */
#define HYPERFI_SLAVE_DIAG_EVENT_ID  0x2002

/** Max diag payload (including trailing null). */
#define HYPERFI_SLAVE_DIAG_MAX_LEN   256

/**
 * Send a printf-formatted diagnostic message from slave to host.
 *
 * Blocking: formats, then calls esp_hosted_send_custom_data(). On the slave
 * Wi-Fi task this is ~sub-millisecond. Safe to call from any FreeRTOS task
 * context. MUST NOT be called from ISR.
 *
 * On host side the message is routed to the registered 0x2002 callback; if
 * none registered, it is silently dropped in host RPC dispatch.
 */
void hf_slave_diag(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif /* __SLAVE_DIAG_H */
