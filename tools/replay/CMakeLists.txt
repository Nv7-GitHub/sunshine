# Cross-platform build for the offline replay harness + sunshine_core unit tests.
# Links the REAL sunshine_core sources (no logic duplicated). Works on
# Windows (MSVC), macOS, and Linux (gcc/clang).
#
#   cmake -B build -S .          # configure (from tools/replay)
#   cmake --build build          # build -> build/replay(.exe) + tests
#   ctest --test-dir build       # run the unit tests
#
# Then e.g.:  ./build/replay LOG.sun > cont.csv
cmake_minimum_required(VERSION 3.16)
project(sunshine_replay C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

get_filename_component(CORE "${CMAKE_CURRENT_SOURCE_DIR}/../../sunshine_core" ABSOLUTE)
set(CORE_SRC
    ${CORE}/src/utils.c ${CORE}/src/kalman.c ${CORE}/src/derot_filter.c
    ${CORE}/src/control.c ${CORE}/src/brain.c)
set(COMPAT ${CMAKE_CURRENT_SOURCE_DIR}/msvc_compat.h)

# Apply the per-target settings the core needs on each toolchain.
function(core_target tgt)
    target_include_directories(${tgt} PRIVATE ${CORE}/include ${CORE}/test)
    if(MSVC)
        # MSVC can't parse GCC's __attribute__((packed)); force-include the shim.
        target_compile_options(${tgt} PRIVATE /FI${COMPAT})
    else()
        target_link_libraries(${tgt} m)   # libm for sinf/atan2f/etc.
    endif()
endfunction()

add_executable(replay replay.c ${CORE_SRC})
core_target(replay)

enable_testing()
foreach(t utils kalman derot_filter control brain)
    add_executable(test_${t} ${CORE}/test/test_${t}.c ${CORE_SRC})
    core_target(test_${t})
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
