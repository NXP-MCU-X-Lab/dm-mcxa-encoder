cmake_minimum_required(VERSION 3.16)
project(encoder_tests C)

set(CMAKE_C_STANDARD 99)
enable_testing()

# Links the real firmware algorithm sources, not copies of them, so results
# describe the shipping code path.
add_executable(encoder_sim
    encoder_sim.c
    ../source/app_encoder.c
    ../source/app_encoder_defaults.c
)
target_include_directories(encoder_sim PRIVATE ../source)
target_compile_options(encoder_sim PRIVATE -Wall -Wextra)
if(NOT MSVC)
    target_link_libraries(encoder_sim m)
endif()

add_test(NAME encoder_highspeed COMMAND encoder_sim highspeed)
# Guards the zero-factory-step property: a board that has never been calibrated
# must converge on its own from the compiled-in defaults.
add_test(NAME encoder_bootstrap COMMAND encoder_sim bootstrap)
# Guards start-up from a bench state: stationary or hand-nudged before real rotation.
add_test(NAME encoder_benchstart COMMAND encoder_sim benchstart)
