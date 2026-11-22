/* Forced into every file compiled for the PC tests (-include) */
#pragma once
#include <stddef.h>
#include "sdkconfig.h"
size_t strlcpy(char *dst, const char *src, size_t size);
void esp_restart(void);
