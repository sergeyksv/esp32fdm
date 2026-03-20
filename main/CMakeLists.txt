set(SRCS
    "main.c"
    "wifi.c"
    "camera.c"
    "httpd.c"
    "usb_serial.cpp"
    "sdcard.c"
    "sdcard_httpd.c"
    "printer_comm.c"
    "printer_comm_klipper.c"
    "obico_client.c"
    "rfc2217.c"
    "terminal.c"
    "logbuf.c"
    "dns_server.c"
    "mdns_service.c"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
)
