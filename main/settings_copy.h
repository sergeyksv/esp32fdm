#pragma once

/*
 * Copying settings between boards.  GET /api/settings gives every setting that
 * is not this board's own as JSON sections; POST /api/settings applies the
 * sections it is sent.  The settings page loads another board's JSON from the
 * browser (or a file), shows the sections with tick boxes and posts the ones
 * chosen, so the boards never talk to each other.
 *
 * Not copied: the name, Wi-Fi, the Obico link, the backend and Moonraker
 * address, the camera clock (it depends on the camera module), counters,
 * maintenance marks, recent files, the plastic in the nozzle.
 */

#include "esp_err.h"
#include "esp_http_server.h"
#include "layout.h"

/** The printer model, typed by the user: "" if not set */
const char *settings_copy_model(void);

void settings_copy_init(void);

void settings_copy_render_settings(html_buf_t *p);

esp_err_t settings_copy_register_httpd(httpd_handle_t server);
