cmake_minimum_required(VERSION 3.16)
project(sunshine_core C)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CORE_SOURCES
    src/utils.c
    src/kalman.c
    src/mag_heading.c
    src/control.c
    src/brain.c
)

add_library(sunshine_core STATIC ${CORE_SOURCES})
target_include_directories(sunshine_core PUBLIC include)
target_link_libraries(sunshine_core m)
target_compile_options(sunshine_core PRIVATE -Wall -Wextra -Werror)

enable_testing()
foreach(t utils kalman mag_heading control brain)
    add_executable(test_${t} test/test_${t}.c)
    target_link_libraries(test_${t} sunshine_core)
    add_test(NAME ${t} COMMAND test_${t})
endforeach()
