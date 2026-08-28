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

# Same idea for the protocol layer: app_tformat.c is included rather than linked,
# with tests/stubs standing in for the MCU headers, so the request state machine
# and frame builder under test are the ones that ship.
add_executable(tformat_sim
    tformat_sim.c
    stubs/mcu_stub.c
)
target_include_directories(tformat_sim PRIVATE stubs ../source)
target_compile_options(tformat_sim PRIVATE -Wall -Wextra)
if(NOT MSVC)
    target_link_libraries(tformat_sim m)
endif()
add_test(NAME tformat_protocol COMMAND tformat_sim)

add_executable(storage_sim
    storage_sim.c
    ../source/app_encoder_storage.c
)
target_compile_definitions(storage_sim PRIVATE ENCODER_STORAGE_HOST_TEST)
target_include_directories(storage_sim PRIVATE ../source)
target_compile_options(storage_sim PRIVATE -Wall -Wextra)
add_test(NAME encoder_storage COMMAND storage_sim)

add_test(NAME encoder_highspeed COMMAND encoder_sim highspeed)
# Guards the zero-factory-step property: a board that has never been calibrated
# must converge on its own from the compiled-in defaults.
add_test(NAME encoder_bootstrap COMMAND encoder_sim bootstrap)
# Guards start-up from a bench state: stationary or hand-nudged before real rotation.
add_test(NAME encoder_benchstart COMMAND encoder_sim benchstart)
