// SPDX-License-Identifier: Unlicense
#pragma once
#include <stdint.h>

// Receiver quality is a current observation, unlike the clock's holdover
// estimate. Expire it when valid RMC sentences stop arriving.
static inline bool gps_quality_fresh(uint64_t nowUs, uint64_t lastValidRmcUs) {
  return lastValidRmcUs != 0 && nowUs >= lastValidRmcUs &&
         nowUs - lastValidRmcUs < 2500000ULL;
}
