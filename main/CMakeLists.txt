set(SRCS
    "main.c"
    "wifi.c"
    "camera.c"
    "httpd.c"
    "usb_serial.cpp"
    "sdcard.c"
    "sdcard_httpd.c"
    "printer_comm.c"
    "gcode_scan.c"
    "host_print.c"
    "printer_comm_klipper.c"
    "obico_client.c"
    "rfc2217.c"
    "terminal.c"
    "logbuf.c"
    "cache.c"
    "bedlevel.c"
    "dns_server.c"
    "mdns_service.c"
    "ota.c"
    "hub_client.c"
    "print_page.c"
    "cold_pull.c"
    "filament.c"
    "plastics.c"
    "print_stats.c"
    "maintenance.c"
    "settings_copy.c"
    "board_led.c"
    "nozzle_guard.c"
    "print_checks.c"
    "wifi_page.c"
    "i18n.c"
    "layout.c"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
)

# Translation tables from i18n/<lang>.po (tools/i18n.py, see i18n.h)
idf_build_get_property(python PYTHON)
file(GLOB I18N_PO CONFIGURE_DEPENDS "${COMPONENT_DIR}/../i18n/*.po")
set(I18N_TABLES "${CMAKE_CURRENT_BINARY_DIR}/i18n_tables.c")
add_custom_command(
    OUTPUT "${I18N_TABLES}"
    COMMAND ${python} "${COMPONENT_DIR}/../tools/i18n.py" gen --out "${I18N_TABLES}"
    DEPENDS ${I18N_PO} "${COMPONENT_DIR}/../tools/i18n.py"
    COMMENT "Building translation tables"
    VERBATIM)
target_sources(${COMPONENT_LIB} PRIVATE "${I18N_TABLES}")

# Build stamp (FW_BUILD in version.h): checked on every build, rewritten when it changes
set(BUILD_INFO "${CMAKE_CURRENT_BINARY_DIR}/build_info.c")
add_custom_target(fw_build_stamp
    COMMAND ${python} "${COMPONENT_DIR}/../tools/build_stamp.py" "${BUILD_INFO}"
    BYPRODUCTS "${BUILD_INFO}"
    WORKING_DIRECTORY "${COMPONENT_DIR}/.."
    COMMENT "Build stamp"
    VERBATIM)
target_sources(${COMPONENT_LIB} PRIVATE "${BUILD_INFO}")
add_dependencies(${COMPONENT_LIB} fw_build_stamp)

# Crash report in /logs: logbuf.c wraps ESP-IDF's panic handler to copy the
# reason and backtrace into the log ring before the restart
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_panic_handler")
