#pragma once

/*
 * Tasks with their stack in PSRAM. Internal RAM is scarce (~85 KB free), PSRAM
 * has megabytes free, so a task here gets a generous stack (8 KB, 12 KB with
 * HTTP or TLS) instead of one cut to its measured use: an overflow costs a
 * reboot, a few spare KB of PSRAM cost nothing. Measured on fdm13 (2026-11-17):
 * a 7 MB G-code scan takes the same time with its stack here as in internal RAM.
 *
 * Flash is fine from such a task: with SPIRAM_FETCH_INSTRUCTIONS and
 * SPIRAM_RODATA (SPIRAM_XIP_FROM_PSRAM) ESP-IDF keeps the cache on during
 * flash reads and writes (spi_flash_os_func_app.c, SPI_FLASH_CACHE_NO_DISABLE),
 * so NVS, LittleFS and esp_partition_read/write work; an hour of NVS and
 * LittleFS writes from two such tasks gave no error. What still disables the
 * cache, and asserts on a PSRAM stack (cache_utils.c), is mapping flash:
 * spi_flash_mmap, esp_partition_mmap, and the OTA functions built on it
 * (esp_ota_end, esp_ota_set_boot_partition, esp_ota_mark_app_valid_*, the
 * first esp_ota_get_running_partition): only ota.c and boot code use them.
 * And no buffer on such a stack may go to DMA (SD card, USB): SDMMC copies it
 * through a bounce buffer a sector at a time.
 *
 * Stays in internal RAM: drivers (USB, camera DMA), the printer task, the
 * SD upload and the print start (whole-file loops at full speed), OTA, httpd.
 *
 * A task made here ends with task_delete(NULL), not vTaskDelete(NULL): its
 * stack must be freed with vTaskDeleteWithCaps(), and if PSRAM could not take
 * it the task was made the ordinary way, which task_delete() tells apart.
 */

#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

static inline BaseType_t task_create_psram(TaskFunction_t fn, const char *name, uint32_t stack,
                                           void *arg, UBaseType_t prio, TaskHandle_t *handle,
                                           BaseType_t core)
{
    BaseType_t r = xTaskCreatePinnedToCoreWithCaps(fn, name, stack, arg, prio, handle, core,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    /* Internal RAM if PSRAM cannot take it: the task still runs */
    if (r != pdPASS) r = xTaskCreatePinnedToCore(fn, name, stack, arg, prio, handle, core);
    return r;
}

/** Deletes a task made by task_create_psram() (NULL: the calling task), or
 *  any other task. A WithCaps task has static buffers, an ordinary one not */
static inline void task_delete(TaskHandle_t task)
{
    StackType_t *stack;
    StaticTask_t *tcb;
    if (xTaskGetStaticBuffers(task, &stack, &tcb) == pdTRUE) vTaskDeleteWithCaps(task);
    else vTaskDelete(task);
}
