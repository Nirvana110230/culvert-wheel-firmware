cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME inspection_robot_chassis)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Application/Src/app_state_machine.c
    Bsp/Src/rc_capture.c
    Services/Src/rc_input.c
    Protocol/Src/jetson_protocol.c
    Protocol/Src/modbus_rtu.c
    Protocol/Src/modbus_rx_assembler.c
    Protocol/Src/mssd_driver.c
    Bsp/Src/jetson_uart_transport.c
    Bsp/Src/rs485_transport.c
    Bsp/Src/uart_dma_rx_buffer.c
    Bsp/Src/usart2_rs485_adapter.c
    Bsp/Src/uart_hal_callbacks.c
)

set_source_files_properties(
    Application/Src/app_state_machine.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra"
)

set_source_files_properties(
    Bsp/Src/rc_capture.c
    Services/Src/rc_input.c
    Protocol/Src/jetson_protocol.c
    Protocol/Src/modbus_rtu.c
    Protocol/Src/modbus_rx_assembler.c
    Protocol/Src/mssd_driver.c
    Bsp/Src/jetson_uart_transport.c
    Bsp/Src/rs485_transport.c
    Bsp/Src/uart_dma_rx_buffer.c
    Bsp/Src/usart2_rs485_adapter.c
    Bsp/Src/uart_hal_callbacks.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror"
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application/Inc
    Bsp/Inc
    Services/Inc
    Protocol/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
