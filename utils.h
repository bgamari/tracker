#pragma once

#include <string.h>

static inline void memzero(void* ptr, unsigned int size)
{
    memset(ptr, 0, size);
}
