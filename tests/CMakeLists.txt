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
