#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SDCARD_NONE,        /* no card, or it does not answer */
    SDCARD_NO_FS,       /* a card without a FAT file system the board can mount */
    SDCARD_MOUNTED,
    SDCARD_FORMATTING,
} sdcard_state_t;

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

/** Total size of the file system in KB, or -1 if not mounted */
int32_t sdcard_get_total_kb(void);

sdcard_state_t sdcard_state(void);

/**
 * Format the card (FAT, 16 KB clusters) and mount it: a mounted card, or one
 * that did not mount. Blocks for seconds to tens of seconds; the card counts as
 * not mounted meanwhile. The caller makes sure nothing is using the card.
 */
esp_err_t sdcard_format(void);

#ifdef __cplusplus
}
#endif
