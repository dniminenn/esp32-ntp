#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>

/* 32x8 image as four 8x8 tiles, left to right. Bit 7 of a row byte is the leftmost pixel. */
static const uint8_t kPreSyncLeft[8]   = {0x70,0x88,0x88,0x7f,0x70,0x8f,0x88,0x70};
static const uint8_t kPreSyncMiddle[8] = {0x00,0x00,0x00,0xff,0x00,0xff,0x00,0x00};
static const uint8_t kPreSyncRight[8]  = {0x00,0x00,0x1e,0xe1,0x07,0xe1,0x1e,0x00};
static const uint8_t* const kPreSyncTiles[4] = { kPreSyncLeft, kPreSyncMiddle, kPreSyncMiddle, kPreSyncRight };
