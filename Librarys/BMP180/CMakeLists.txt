cmake_minimum_required(VERSION 3.24)

project(bmp180 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        bmp180
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/bmp_180.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/Src/bmp180.c
)
target_include_directories(
        bmp180
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/Inc
)
