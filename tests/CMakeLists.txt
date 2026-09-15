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
