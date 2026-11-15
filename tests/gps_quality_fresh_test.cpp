// SPDX-License-Identifier: Unlicense
#include "../components/gps/gps_quality_fresh.h"
#include <assert.h>

int main() {
  assert(!gps_quality_fresh(0, 0));
  assert(!gps_quality_fresh(1000000, 0));
  assert(gps_quality_fresh(1000000, 1000000));
  assert(gps_quality_fresh(3499999, 1000000));
  assert(!gps_quality_fresh(3500000, 1000000));
  assert(!gps_quality_fresh(4000000, 1000000));
  assert(!gps_quality_fresh(1000000, 2000000));
}
