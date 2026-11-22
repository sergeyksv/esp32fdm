cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Merge sdkconfig.local (git-ignored) on top of sdkconfig.defaults for private overrides
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig.local")
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults;sdkconfig.local")
endif()

# The app descriptor carries FW_VERSION rather than "git describe": a deferred
# OTA update shows it before the image runs.  Re-run cmake if it changes.
file(STRINGS "${CMAKE_SOURCE_DIR}/main/version.h" FW_VERSION_LINE REGEX "#define FW_VERSION ")
string(REGEX REPLACE ".*\"(.*)\".*" "\\1" PROJECT_VER "${FW_VERSION_LINE}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/main/version.h")

project(esp32fdm)
