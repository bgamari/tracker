#pragma once

#ifdef SEMIHOSTING
#include <stdio.h>
#define dbg_printf(fmt, ...) printf(fmt, __VA_ARGS__)
#else
#define dbg_printf(fmt, ...)
#endif
