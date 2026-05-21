cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(miniproject_mobile)

target_sources(app PRIVATE
    src/main/src/codein.c
    src/usb_device/src/usb_device.c
    src/usb_serial/src/usb_serial.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_device/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_serial/src
)
