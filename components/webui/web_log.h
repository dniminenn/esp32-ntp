#pragma once
// SPDX-License-Identifier: Unlicense
#include <stddef.h>
#include <stdbool.h>

// Allocates the ring and mirrors ESP_LOG output, in addition to the UART.
// Call only when the saved setting is enabled, after configuration is loaded.
// Returns false if the ring could not be allocated; the log hook stays untouched.
bool weblog_install(void);
bool weblog_active(void);

// Copies the buffered log (oldest first, whole lines only) into out as a
// NUL-terminated string and returns its length.
size_t weblog_snapshot(char* out, size_t cap);
