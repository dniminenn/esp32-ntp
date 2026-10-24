// SPDX-License-Identifier: Unlicense

#include "config_store.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static bool append(char* out, size_t capacity, size_t* used, const char* text) {
  size_t n = strlen(text);
  if (n >= capacity - *used) return false;
  memcpy(out + *used, text, n + 1);
  *used += n;
  return true;
}

esp_err_t cfg_export_csv(char* out, size_t capacity, size_t* length) {
  if (length) *length = 0;
  if (!out || !capacity || !length) return ESP_ERR_INVALID_ARG;
  out[0] = '\0';
  size_t used = 0;
  if (!append(out, capacity, &used,
              "key,type,encoding,value\r\n" CFG_NVS_NAMESPACE ",namespace,,\r\n"))
    goto too_small;

  for (int i = 0; i < CFG_COUNT; ++i) {
    const cfg_field_t* f = &g_cfg_fields[i];
    if (!append(out, capacity, &used, f->key)) goto too_small;
    if (f->type == CF_STR || f->type == CF_PASS) {
      if (!append(out, capacity, &used, ",data,string,\"")) goto too_small;
      // Quote every string, preserving commas, line breaks and whitespace;
      // CSV represents a literal quote by doubling it.
      for (const char* p = cfg_str((cfg_id_t)i); *p; ++p) {
        char ch[2] = { *p, '\0' };
        if (*p == '"' && !append(out, capacity, &used, "\"")) goto too_small;
        if (!append(out, capacity, &used, ch)) goto too_small;
      }
      if (!append(out, capacity, &used, "\"\r\n")) goto too_small;
    } else {
      // Booleans and enum indices use the same signed i32 as cfg_commit().
      char value[32];
      snprintf(value, sizeof(value), ",data,i32,%" PRIi32 "\r\n", cfg_int((cfg_id_t)i));
      if (!append(out, capacity, &used, value)) goto too_small;
    }
  }
  *length = used;
  return ESP_OK;

too_small:
  out[0] = '\0';
  return ESP_ERR_NO_MEM;
}
