#pragma once

/*
 * /print: minimal page for controlling the current print from a phone.
 * This is what the notification links to.
 */

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t print_page_register_httpd(httpd_handle_t server);

/** Remember a file started from the SD card, for "Print again" on /print */
void print_page_note_started(const char *filename);
