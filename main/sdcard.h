#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mount SD card FAT filesystem at /sdcard using 1-bit SDMMC.
 * GPIO38=CMD, GPIO39=CLK, GPIO40=D0.
 * Returns ESP_OK on success, logs error if no card inserted.
 */
esp_err_t sdcard_init(void);

/**
 * Returns true if the SD card is mounted and accessible.
 */
bool sdcard_is_mounted(void);

/**
 * Returns free space on SD card in KB, or -1 if not mounted.
 */
int32_t sdcard_get_free_kb(void);

#ifdef __cplusplus
}
#endif
