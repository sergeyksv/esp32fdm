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
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
)
