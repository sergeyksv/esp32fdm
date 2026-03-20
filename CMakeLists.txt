cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Merge sdkconfig.local (git-ignored) on top of sdkconfig.defaults for private overrides
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.local")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.local")
endif()

project(esp32fdm)
