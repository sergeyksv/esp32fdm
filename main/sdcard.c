#include "sdcard.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "ff.h"

#include <string.h>

static const char *TAG = "sdcard";

#define MOUNT_POINT "/sdcard"

/* Freenove ESP32-S3-WROOM SD socket: 1-bit SDMMC */
#define SD_CMD_GPIO  38
#define SD_CLK_GPIO  39
#define SD_D0_GPIO   40

static bool s_mounted = false;

esp_err_t sdcard_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.cmd = SD_CMD_GPIO;
    slot_config.clk = SD_CLK_GPIO;
    slot_config.d0  = SD_D0_GPIO;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    sdmmc_card_t *card = NULL;
    esp_err_t err = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config,
                                             &mount_config, &card);
    if (err != ESP_OK) {
        if (err == ESP_FAIL) {
            ESP_LOGW(TAG, "SD card: mount failed (no FAT filesystem?)");
        } else {
            ESP_LOGW(TAG, "SD card: not detected (err=%s)", esp_err_to_name(err));
        }
        return err;
    }

    s_mounted = true;
    sdmmc_card_print_info(stdout, card);

    int32_t free_kb = sdcard_get_free_kb();
    ESP_LOGI(TAG, "SD card mounted at " MOUNT_POINT " (%ld KB free)",
             (long)free_kb);

    return ESP_OK;
}

bool sdcard_is_mounted(void)
{
    return s_mounted;
}

int32_t sdcard_get_free_kb(void)
{
    if (!s_mounted) return -1;

    FATFS *fs;
    DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) != FR_OK) return -1;

    uint64_t free_bytes = (uint64_t)fre_clust * fs->csize * 512;
    return (int32_t)(free_bytes / 1024);
}

