#include "sdcard.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"
#include "ff.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"

#include <string.h>

static const char *TAG = "sdcard";

#define MOUNT_POINT "/sdcard"

/* Freenove ESP32-S3-WROOM SD socket: 1-bit SDMMC */
#define SD_CMD_GPIO  38
#define SD_CLK_GPIO  39
#define SD_D0_GPIO   40

static volatile sdcard_state_t s_state = SDCARD_NONE;
static volatile bool s_mounted = false;
static sdmmc_card_t *s_card;

static esp_err_t mount(bool format_if_no_fs)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = format_if_no_fs,
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
            s_state = SDCARD_NO_FS;
        } else {
            ESP_LOGW(TAG, "SD card: not detected (err=%s)", esp_err_to_name(err));
            s_state = SDCARD_NONE;
        }
        return err;
    }

    s_card = card;
    s_mounted = true;
    s_state = SDCARD_MOUNTED;
    return ESP_OK;
}

esp_err_t sdcard_init(void)
{
    esp_err_t err = mount(false);
    if (err != ESP_OK) return err;

    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "SD card mounted at " MOUNT_POINT " (%ld KB free)",
             (long)sdcard_get_free_kb());
    return ESP_OK;
}

sdcard_state_t sdcard_state(void)
{
    return s_state;
}

/* A mounted card goes through ESP-IDF's format (unmount, f_mkfs, mount back
 * on the same card handle); one that did not mount (no or a broken file
 * system) is mounted again with format_if_mount_failed. Either way FAT with
 * the allocation unit the board mounts with: FAT32 on any card over 2 GB,
 * also over 32 GB, since only the board reads this card. A file still open
 * across the format is not a memory hazard: its next read or write fails,
 * since FatFs checks the volume's mount id */
esp_err_t sdcard_format(void)
{
    sdcard_state_t was = s_state;
    if (was == SDCARD_FORMATTING) return ESP_ERR_INVALID_STATE;
    s_state = SDCARD_FORMATTING;
    s_mounted = false;

    esp_err_t err;
    if (was == SDCARD_MOUNTED) {
        ESP_LOGW(TAG, "Formatting the SD card");
        err = esp_vfs_fat_sdcard_format(MOUNT_POINT, s_card);
        if (err == ESP_OK) {
            s_mounted = true;
            s_state = SDCARD_MOUNTED;
        } else {
            /* f_mkfs failed (mounted back), no memory (left unmounted), or not
             * mounted back (ESP-IDF released the card): start over */
            if (ff_diskio_get_pdrv_card(s_card) != 0xff) esp_vfs_fat_sdcard_unmount(MOUNT_POINT, s_card);
            s_card = NULL;
            mount(false);
        }
    } else {
        ESP_LOGW(TAG, "Formatting the SD card (it did not mount)");
        err = mount(true);
    }
    ESP_LOGW(TAG, "SD card format: %s, %ld KB free", esp_err_to_name(err),
             (long)sdcard_get_free_kb());
    return err;
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


int32_t sdcard_get_total_kb(void)
{
    if (!s_mounted) return -1;

    FATFS *fs;
    DWORD fre_clust;
    if (f_getfree("0:", &fre_clust, &fs) != FR_OK) return -1;
    return (int32_t)((uint64_t)(fs->n_fatent - 2) * fs->csize * 512 / 1024);
}
