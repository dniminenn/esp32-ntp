// SPDX-License-Identifier: Unlicense

#include "web_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const size_t RING_SIZE = 8192;
static const size_t LOG_LINE_MAX = 192;

static char* s_ring = nullptr;
static size_t s_head = 0;       // next write position
static bool s_wrapped = false;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t s_orig = nullptr;

// Drops ANSI colour sequences (ESC [ ... m) and carriage returns.
static size_t strip(char* s, size_t n) {
  size_t o = 0;
  for (size_t i = 0; i < n; ++i) {
    if (s[i] == 0x1b && i + 1 < n && s[i + 1] == '[') {
      i += 2;
      while (i < n && !(s[i] >= '@' && s[i] <= '~')) ++i;
      continue;
    }
    if (s[i] != '\r') s[o++] = s[i];
  }
  return o;
}

static int hook(const char* fmt, va_list ap) {
  va_list copy;
  va_copy(copy, ap);
  char line[LOG_LINE_MAX];
  int n = vsnprintf(line, sizeof(line), fmt, copy);
  va_end(copy);

  if (n > 0) {
    bool truncated = (size_t)n >= sizeof(line);
    size_t len = truncated ? sizeof(line) - 1 : (size_t)n;
    len = strip(line, len);
    // Keep the next log entry on a new line even when this one's newline was
    // beyond the formatting buffer.
    if (truncated && len > 0 && line[len - 1] != '\n') line[len - 1] = '\n';
    portENTER_CRITICAL(&s_lock);
    for (size_t i = 0; i < len; ++i) {
      s_ring[s_head] = line[i];
      if (++s_head == RING_SIZE) { s_head = 0; s_wrapped = true; }
    }
    portEXIT_CRITICAL(&s_lock);
  }
  return s_orig ? s_orig(fmt, ap) : n;
}

bool weblog_install(void) {
  if (s_ring) return true;
  s_ring = (char*)malloc(RING_SIZE);
  if (!s_ring) return false;
  s_orig = esp_log_set_vprintf(hook);
  return true;
}

bool weblog_active(void) { return s_ring != nullptr; }

size_t weblog_snapshot(char* out, size_t cap) {
  if (cap == 0) return 0;
  if (!s_ring) { out[0] = '\0'; return 0; }
  size_t n = 0;
  portENTER_CRITICAL(&s_lock);
  size_t head = s_head;
  bool wrapped = s_wrapped;
  size_t start = wrapped ? head : 0;
  size_t total = wrapped ? RING_SIZE : head;
  if (total > cap - 1) { start = (start + total - (cap - 1)) % RING_SIZE; total = cap - 1; }
  for (size_t i = 0; i < total; ++i) out[i] = s_ring[(start + i) % RING_SIZE];
  portEXIT_CRITICAL(&s_lock);
  n = total;
  out[n] = '\0';

  // The oldest line may have been cut in half; start at the next whole line.
  if (wrapped || n == cap - 1) {
    char* nl = (char*)memchr(out, '\n', n);
    if (nl) {
      size_t skip = (size_t)(nl - out) + 1;
      memmove(out, out + skip, n - skip + 1);
      n -= skip;
    }
  }
  return n;
}
