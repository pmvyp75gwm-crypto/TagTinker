#pragma once
#include <cstdint>
static unsigned long g_total_delay_ms = 0;
static inline void delay(unsigned long ms) { g_total_delay_ms += ms; }
