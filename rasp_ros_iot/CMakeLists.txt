# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rasp_ros_iot C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(rasp_ros_iot rasp_ros_iot.c lib/pico_uart/pico_uart_transport.c lib/oled/ssd1306_i2c.c)

pico_set_program_name(rasp_ros_iot "rasp_ros_iot")
pico_set_program_version(rasp_ros_iot "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rasp_ros_iot 0)
pico_enable_stdio_usb(rasp_ros_iot 1)

target_compile_definitions(rasp_ros_iot PRIVATE
    PICO_UART_ENABLE_CRLF_SUPPORT=0
)

# Add the standard library to the build
target_link_libraries(rasp_ros_iot
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(rasp_ros_iot PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(rasp_ros_iot 
        pico_cyw43_arch_lwip_threadsafe_background
        ${CMAKE_CURRENT_LIST_DIR}/lib/libmicroros/libmicroros.a
        hardware_i2c
        )


target_include_directories(rasp_ros_iot PUBLIC
            lib/libmicroros/include
        )

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

add_compile_definitions(
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)
        

pico_add_extra_outputs(rasp_ros_iot)

