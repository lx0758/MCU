cmake_minimum_required(VERSION 3.24)

project(encoder C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        encoder
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/encoder.c
)
target_include_directories(
        encoder
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
