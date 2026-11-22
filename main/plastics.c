#include "plastics.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "plastics";

#define NVS_NAMESPACE   "filament"
#define NVS_KEY_LOADED  "loaded"    /* The name, so a reordered table keeps it */

/* Starting points: published values differ by up to 20 C, and the right pull
 * temperature is the one where the plug neither snaps nor stretches. */
const plastic_t PLASTICS[] = {
    { "PLA",   210,  90 },
    { "PETG",  240, 125 },
    { "ABS",   240, 140 },
    { "Nylon", 250, 150 },
};
const int PLASTIC_COUNT = sizeof(PLASTICS) / sizeof(PLASTICS[0]);

int plastic_loaded(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return -1;
    char name[16];
    size_t len = sizeof(name);
    int idx = -1;
    if (nvs_get_str(nvs, NVS_KEY_LOADED, name, &len) == ESP_OK) {
        for (int i = 0; i < PLASTIC_COUNT; i++) {
            if (strcmp(PLASTICS[i].name, name) == 0) idx = i;
        }
    }
    nvs_close(nvs);
    return idx;
}

void plastic_set_loaded(int idx)
{
    if (idx >= PLASTIC_COUNT) return;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    if (idx < 0) nvs_erase_key(nvs, NVS_KEY_LOADED);
    else nvs_set_str(nvs, NVS_KEY_LOADED, PLASTICS[idx].name);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "In the nozzle: %s", idx < 0 ? "unknown" : PLASTICS[idx].name);
}

int plastic_hotter(int a, int b)
{
    if (a < 0 || a >= PLASTIC_COUNT) return b;
    if (b < 0 || b >= PLASTIC_COUNT) return a;
    return PLASTICS[b].print_c > PLASTICS[a].print_c ? b : a;
}

void plastic_script_done(bool completed, int idx)
{
    plastic_set_loaded(completed ? idx : plastic_hotter(plastic_loaded(), idx));
}
