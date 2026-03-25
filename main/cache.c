#include "cache.h"

#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <utime.h>
#include <unistd.h>

#include "esp_log.h"

static const char *TAG = "cache";

void cache_touch(const char *path)
{
    struct utimbuf ut = {
        .actime  = time(NULL),
        .modtime = time(NULL),
    };
    utime(path, &ut);
}

void cache_evict_lru(const char *dir_path, int max_entries)
{
    DIR *d = opendir(dir_path);
    if (!d) return;

    /* First pass: count entries */
    int count = 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        if (ent->d_type == DT_REG)
            count++;
    }

    if (count <= max_entries) {
        closedir(d);
        return;
    }

    int to_delete = count - max_entries;
    ESP_LOGI(TAG, "Evicting %d oldest from %s (%d entries, max %d)",
             to_delete, dir_path, count, max_entries);

    /* Delete oldest files one at a time. For small max_entries (~50-100)
     * repeated linear scans are fine and avoid dynamic allocation. */
    while (to_delete > 0) {
        rewinddir(d);
        time_t oldest_mtime = 0;
        char oldest_name[128] = {0};

        while ((ent = readdir(d)) != NULL) {
            if (ent->d_type != DT_REG) continue;

            char fpath[300];
            snprintf(fpath, sizeof(fpath), "%s/%s", dir_path, ent->d_name);
            struct stat st;
            if (stat(fpath, &st) == 0) {
                if (oldest_name[0] == '\0' || st.st_mtime < oldest_mtime) {
                    oldest_mtime = st.st_mtime;
                    strncpy(oldest_name, ent->d_name, sizeof(oldest_name) - 1);
                    oldest_name[sizeof(oldest_name) - 1] = '\0';
                }
            }
        }

        if (oldest_name[0] == '\0') break;

        char del_path[300];
        snprintf(del_path, sizeof(del_path), "%s/%s", dir_path, oldest_name);
        if (unlink(del_path) == 0) {
            ESP_LOGI(TAG, "Evicted: %s", oldest_name);
        }
        to_delete--;
    }

    closedir(d);
}
