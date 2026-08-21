#pragma once
// SPDX-License-Identifier: Unlicense
#include <time.h>

/* civil UTC to unix epoch */
static inline time_t civil_to_unix(int year, int month, int day,
                                   int hour, int min, int sec) {
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;
  int JDN = day + (153*m + 2)/5 + 365*y + y/4 - y/100 + y/400 - 32045;
  int days = JDN - 2440588;   /* 1970-01-01 is JDN 2440588 */
  return (time_t)days * 86400 + hour*3600 + min*60 + sec;
}
