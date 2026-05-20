cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(skywatch_controller)

# Per-lib source listing. Layout convention: every lib lives at
#   src/<lib>/src/  (code + private headers)
#   src/<lib>/ref/  (self-reference description.md)
# See resources/organisation/file_layout.md for the full spec.

target_sources(app PRIVATE
    src/main/src/main.c
    src/shell/src/shell.c
    src/usb_device/src/usb_device.c
    src/usb_serial/src/usb_serial.c
    src/usb_handler/src/usb_handler.c
    src/json/src/json_parser.c
    src/ble/src/ble_central.c
    src/bridge/src/bridge.c
    src/aircraft_db/src/aircraft_db.c
    src/db_publisher/src/db_publisher.c
    src/kalman/src/kalman.c
    src/collision/src/collision.c
)

# Each lib's src/ is also its public include dir — keeps the module boundary
# narrow: headers that are not in a sibling lib's src/ are not reachable.
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/shell/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_device/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_serial/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/usb_handler/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/json/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ble/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bridge/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/aircraft_db/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/db_publisher/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kalman/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/collision/src
)
