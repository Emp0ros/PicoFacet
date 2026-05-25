cmake_minimum_required(VERSION 3.12)
project(pfcli C)

set(CMAKE_C_STANDARD 11)

add_executable(pfcli
    main.c
    usb.c
    device.c
    fuses.c
)

target_include_directories(pfcli PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(pfcli PRIVATE _GNU_SOURCE)
target_compile_options(pfcli PRIVATE -Wall -O2)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)
target_include_directories(pfcli PRIVATE ${LIBUSB_INCLUDE_DIRS})
target_link_libraries(pfcli PRIVATE ${LIBUSB_LIBRARIES})
target_link_directories(pfcli PRIVATE ${LIBUSB_LIBRARY_DIRS})
