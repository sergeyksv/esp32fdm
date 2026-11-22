#pragma once

/*
 * The board's own LEDs: the single LED (IO1/IO2) and the WS2812 RGB LED
 * (GPIO48), and the light for the camera while a print runs: the RGB LED in
 * white, the printer's own light (a command: M355 on Marlin, the FDM_LIGHT
 * macro on Klipper), both or none, chosen in Settings.  /api/led sets the
 * board's LEDs by hand (no page yet).
 */

#include <stdbool.h>
#include <stdint.h>
#include "cJSON.h"
#include "esp_err.h"
#include "esp_http_server.h"
#include "layout.h"

typedef enum {
    LIGHT_BOARD = 0,        /* The board's RGB LED (the default, as before 2.6) */
    LIGHT_PRINTER,
    LIGHT_BOTH,
    LIGHT_NONE,
} light_mode_t;

void board_led_init(void);

void board_led_set(bool io1, bool io2, uint8_t r, uint8_t g, uint8_t b);

esp_err_t board_led_register_httpd(httpd_handle_t server);

/** The light mode, and the printer's on/off commands ("" = the backend's
 *  default; NULL leaves one as it is).  Saved in NVS */
esp_err_t board_led_set_config(light_mode_t mode, const char *on_cmd, const char *off_cmd);
light_mode_t board_led_mode(void);

void board_led_render_settings(html_buf_t *p);

/* The "light" section of the settings copied between boards */
void board_led_settings_export(cJSON *o);
void board_led_settings_import(const cJSON *o);
