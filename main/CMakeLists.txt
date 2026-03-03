set(SRCS
    "main.c"
    "wifi.c"
    "camera.c"
    "httpd.c"
    "usb_serial.cpp"
)

if(CONFIG_OBICO_ENABLED)
    list(APPEND SRCS "printer_comm.c" "printer_comm_klipper.c" "obico_client.c")
endif()

if(CONFIG_RFC2217_ENABLED)
    list(APPEND SRCS "rfc2217.c")
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
)
