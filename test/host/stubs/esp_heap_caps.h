#pragma once
#include <stdlib.h>
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_INTERNAL 2
#define MALLOC_CAP_8BIT 4
#define MALLOC_CAP_DMA 8
#define heap_caps_malloc(n, caps) malloc(n)
#define heap_caps_calloc(c, n, caps) calloc(c, n)
#define heap_caps_free(p) free(p)
