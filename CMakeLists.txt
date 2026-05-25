cmake_minimum_required(VERSION 3.12)

option(BUILD_PFCLI "Build pfcli tool" ON)

# Platform selection: cmake -DPLATFORM=rp2040 or cmake -DPLATFORM=ch32v305
if(NOT PLATFORM)
    message(FATAL_ERROR "PLATFORM not set. Use -DPLATFORM=rp2040 or -DPLATFORM=ch32v305")
endif()

# Portable sources
set(SHARED_SRCS
    main.c
    mmc/abort.c
    mmc/init.c
    mmc/read.c
    mmc/write.c
    usb_descriptors.c
)

if(PLATFORM STREQUAL "rp2040")
    # Pico SDK
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

    # Project definition
    project(PicoFacet C CXX ASM)

    # Language standards
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    # Use 200 MHz clock for RP2040
    set(PICO_USE_FASTEST_SUPPORTED_CLOCK 1)

    pico_sdk_init()

    add_executable(${PROJECT_NAME}
        ${SHARED_SRCS}
        rp2040/spi_reg.c
        rp2040/usb_dev.c
    )

    target_include_directories(${PROJECT_NAME} PRIVATE
        .
        rp2040
    )

    pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/rp2040/spi.pio)

    target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_dma
        hardware_pio
        hardware_resets
        hardware_irq
        hardware_sync
    )

    pico_add_extra_outputs(${PROJECT_NAME})
    pico_enable_stdio_usb(${PROJECT_NAME} 0)
    pico_enable_stdio_uart(${PROJECT_NAME} 0)

elseif(PLATFORM STREQUAL "ch32v305")
    # RISC-V toolchain
    if(NOT CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE "ch32v305/toolchain.cmake" CACHE FILEPATH "" FORCE)
    endif()

    # Project definition
    project(PicoFacet C CXX ASM)

    # Language standards
    set(CMAKE_C_STANDARD 11)

    # CH32V30x HAL sources
    file(GLOB_RECURSE VENDOR_C_SRCS
        ch32v305/vendor/Core/*.c
        ch32v305/vendor/Debug/*.c
        ch32v305/vendor/Peripheral/src/*.c
        ch32v305/vendor/User/*.c
    )

    add_executable(${PROJECT_NAME}.elf
        ${SHARED_SRCS}
        ch32v305/usb_dev.c
        ch32v305/spi_reg.c
        ch32v305/pins.c
        ${VENDOR_C_SRCS}
        # Startup code
        ch32v305/vendor/Startup/startup_ch32v30x_D8C.S
    )

    target_compile_definitions(${PROJECT_NAME}.elf PRIVATE PLATFORM_CH32V305)

    target_include_directories(${PROJECT_NAME}.elf PRIVATE
        .
        ch32v305
        ch32v305/vendor/Core
        ch32v305/vendor/Debug
        ch32v305/vendor/Peripheral/inc
        ch32v305/vendor/User
    )

    target_link_options(${PROJECT_NAME}.elf PRIVATE
        -T ${CMAKE_CURRENT_SOURCE_DIR}/ch32v305/vendor/Ld/Link.ld
        -Wl,-Map,$<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/picofacet.map
    )

    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
        COMMENT "Generating ${PROJECT_NAME}.hex and ${PROJECT_NAME}.bin"
    )

else()
    message(FATAL_ERROR "Unknown PLATFORM '${PLATFORM}'. Use rp2040 or ch32v305")
endif()

# pfcli — native host tool, built with system compiler via ExternalProject
if(BUILD_PFCLI)
    include(ExternalProject)
    ExternalProject_Add(pfcli
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pfcli
        BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/pfcli
        INSTALL_COMMAND ""
        BUILD_ALWAYS TRUE
    )
endif()
