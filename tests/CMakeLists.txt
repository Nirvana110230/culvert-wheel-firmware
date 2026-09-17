cmake_minimum_required(VERSION 3.22)

project(inspection_robot_chassis_host_tests C)

enable_testing()

add_executable(app_state_machine_tests
    test_app_state_machine.c
    ../Application/Src/app_state_machine.c
)

target_include_directories(app_state_machine_tests PRIVATE
    ../Application/Inc
)

target_compile_features(app_state_machine_tests PRIVATE c_std_11)

if(MSVC)
    target_compile_options(app_state_machine_tests PRIVATE /W4 /WX)
else()
    target_compile_options(app_state_machine_tests PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
    )
endif()

add_test(NAME app_state_machine COMMAND app_state_machine_tests)

add_executable(chassis_kinematics_tests
    test_chassis_kinematics.c
    ../Application/Src/chassis_kinematics.c
)
target_include_directories(chassis_kinematics_tests PRIVATE ../Application/Inc)
target_compile_features(chassis_kinematics_tests PRIVATE c_std_11)
target_compile_options(chassis_kinematics_tests PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME chassis_kinematics COMMAND chassis_kinematics_tests)

add_executable(motion_supervisor_tests
    test_motion_supervisor.c
    ../Application/Src/motion_supervisor.c
)
target_include_directories(motion_supervisor_tests PRIVATE
    ../Application/Inc
    ../Protocol/Inc
    ../Bsp/Inc
)
target_compile_features(motion_supervisor_tests PRIVATE c_std_11)
target_compile_options(motion_supervisor_tests PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME motion_supervisor COMMAND motion_supervisor_tests)

add_executable(chassis_app_tests
    test_chassis_app.c
    ../Application/Src/chassis_app.c
    ../Application/Src/app_state_machine.c
    ../Application/Src/chassis_kinematics.c
    ../Application/Src/motion_supervisor.c
    ../Services/Src/rc_input.c
    ../Protocol/Src/jetson_protocol.c
    ../Protocol/Src/modbus_rtu.c
    ../Protocol/Src/modbus_rx_assembler.c
    ../Protocol/Src/mssd_driver.c
    ../Bsp/Src/jetson_uart_transport.c
    ../Bsp/Src/board_status_outputs.c
    ../Bsp/Src/rs485_transport.c
    ../Bsp/Src/uart_dma_rx_buffer.c
    ../Bsp/Src/usart2_rs485_adapter.c
)
target_include_directories(chassis_app_tests PRIVATE
    stubs
    ../Application/Inc
    ../Services/Inc
    ../Protocol/Inc
    ../Bsp/Inc
)
target_compile_features(chassis_app_tests PRIVATE c_std_11)
target_compile_options(chassis_app_tests PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME chassis_app COMMAND chassis_app_tests)

add_executable(board_status_outputs_tests
    test_board_status_outputs.c
    ../Bsp/Src/board_status_outputs.c
)
target_include_directories(board_status_outputs_tests PRIVATE
    stubs
    ../Application/Inc
    ../Bsp/Inc
)
target_compile_features(board_status_outputs_tests PRIVATE c_std_11)
target_compile_options(board_status_outputs_tests PRIVATE
    -Wall -Wextra -Werror -pedantic
)
add_test(NAME board_status_outputs COMMAND board_status_outputs_tests)

add_executable(chassis_end_to_end_tests
    test_chassis_end_to_end.c
    ../Application/Src/chassis_app.c
    ../Application/Src/app_state_machine.c
    ../Application/Src/chassis_kinematics.c
    ../Application/Src/motion_supervisor.c
    ../Services/Src/rc_input.c
    ../Protocol/Src/jetson_protocol.c
    ../Protocol/Src/modbus_rtu.c
    ../Protocol/Src/modbus_rx_assembler.c
    ../Protocol/Src/mssd_driver.c
    ../Bsp/Src/board_status_outputs.c
    ../Bsp/Src/jetson_uart_transport.c
    ../Bsp/Src/rs485_transport.c
    ../Bsp/Src/uart_dma_rx_buffer.c
    ../Bsp/Src/uart_hal_callbacks.c
    ../Bsp/Src/usart2_rs485_adapter.c
)
target_include_directories(chassis_end_to_end_tests PRIVATE
    stubs
    ../Application/Inc
    ../Services/Inc
    ../Protocol/Inc
    ../Bsp/Inc
)
target_compile_features(chassis_end_to_end_tests PRIVATE c_std_11)
target_compile_options(chassis_end_to_end_tests PRIVATE
    -Wall -Wextra -Werror -pedantic
)
add_test(NAME chassis_end_to_end COMMAND chassis_end_to_end_tests)

add_executable(rc_input_tests
    test_rc_input.c
    ../Services/Src/rc_input.c
)

target_include_directories(rc_input_tests PRIVATE
    ../Services/Inc
    ../Bsp/Inc
)

target_compile_features(rc_input_tests PRIVATE c_std_11)

if(MSVC)
    target_compile_options(rc_input_tests PRIVATE /W4 /WX)
else()
    target_compile_options(rc_input_tests PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
    )
endif()

add_test(NAME rc_input COMMAND rc_input_tests)

foreach(name IN ITEMS jetson_protocol modbus_rtu)
    add_executable(${name}_tests test_${name}.c ../Protocol/Src/${name}.c)
    target_include_directories(${name}_tests PRIVATE ../Protocol/Inc)
    target_compile_features(${name}_tests PRIVATE c_std_11)
    target_compile_options(${name}_tests PRIVATE -Wall -Wextra -Werror -pedantic)
    add_test(NAME ${name} COMMAND ${name}_tests)
endforeach()

add_executable(rs485_transport_tests test_rs485_transport.c ../Bsp/Src/rs485_transport.c)
target_include_directories(rs485_transport_tests PRIVATE ../Bsp/Inc)
target_compile_features(rs485_transport_tests PRIVATE c_std_11)
target_compile_options(rs485_transport_tests PRIVATE -Wall -Wextra -Werror -pedantic)
add_test(NAME rs485_transport COMMAND rs485_transport_tests)

add_executable(mssd_driver_tests
    test_mssd_driver.c
    ../Protocol/Src/mssd_driver.c
    ../Protocol/Src/modbus_rtu.c
    ../Bsp/Src/rs485_transport.c
)

target_include_directories(mssd_driver_tests PRIVATE
    ../Protocol/Inc
    ../Bsp/Inc
)

target_compile_features(mssd_driver_tests PRIVATE c_std_11)
target_compile_options(mssd_driver_tests PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

add_test(NAME mssd_driver COMMAND mssd_driver_tests)

add_executable(modbus_rx_assembler_tests
    test_modbus_rx_assembler.c
    ../Protocol/Src/modbus_rx_assembler.c
    ../Bsp/Src/uart_dma_rx_buffer.c
)

target_include_directories(modbus_rx_assembler_tests PRIVATE
    ../Protocol/Inc
    ../Bsp/Inc
)

target_compile_features(modbus_rx_assembler_tests PRIVATE c_std_11)
target_compile_options(modbus_rx_assembler_tests PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

add_test(NAME modbus_rx_assembler COMMAND modbus_rx_assembler_tests)

add_executable(usart2_rs485_adapter_tests
    test_usart2_rs485_adapter.c
    ../Bsp/Src/usart2_rs485_adapter.c
    ../Bsp/Src/uart_dma_rx_buffer.c
    ../Bsp/Src/uart_hal_callbacks.c
    ../Protocol/Src/modbus_rx_assembler.c
    ../Protocol/Src/mssd_driver.c
    ../Protocol/Src/modbus_rtu.c
    ../Bsp/Src/rs485_transport.c
)

target_include_directories(usart2_rs485_adapter_tests PRIVATE
    stubs
    ../Bsp/Inc
    ../Protocol/Inc
)

target_compile_features(usart2_rs485_adapter_tests PRIVATE c_std_11)
target_compile_options(usart2_rs485_adapter_tests PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

add_test(NAME usart2_rs485_adapter COMMAND usart2_rs485_adapter_tests)

add_executable(jetson_uart_transport_tests
    test_jetson_uart_transport.c
    ../Bsp/Src/jetson_uart_transport.c
    ../Protocol/Src/jetson_protocol.c
)

target_include_directories(jetson_uart_transport_tests PRIVATE
    stubs
    ../Bsp/Inc
    ../Protocol/Inc
)

target_compile_features(jetson_uart_transport_tests PRIVATE c_std_11)
target_compile_options(jetson_uart_transport_tests PRIVATE
    -Wall
    -Wextra
    -Werror
    -pedantic
)

add_test(NAME jetson_uart_transport COMMAND jetson_uart_transport_tests)
