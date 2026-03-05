#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PRINTER_BACKEND_MARLIN  = 0,
    PRINTER_BACKEND_KLIPPER = 1,
} printer_backend_t;

/**
 * Get the currently active printer backend.
 */
printer_backend_t printer_comm_get_backend(void);

#ifdef __cplusplus
}
#endif
