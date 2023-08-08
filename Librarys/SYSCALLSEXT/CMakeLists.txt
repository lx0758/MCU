cmake_minimum_required(VERSION 3.24)

project(syscallsext C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        syscallsext
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/syscallsext.c
)
target_include_directories(
        syscallsext
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
