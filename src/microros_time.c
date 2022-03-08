#include <time.h>

#include "./config.h"

int clock_gettime(clockid_t unused, struct timespec * tp)
{
  (void)unused;

  extern uint32_t g_ui32SysTickCount;

  uint32_t elapsed_ms = g_ui32SysTickCount * SYSTICK_PERIOD_MS;
  uint32_t elapsed_us = elapsed_ms * 1000;

  tp->tv_sec = elapsed_us / 1000000;
  tp->tv_nsec = (elapsed_us % 1000000) * 1000;

  return 0;
}
