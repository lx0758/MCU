cmake_minimum_required(VERSION 3.24)

project(oled C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        oled
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/oled_iic.c
)
target_include_directories(
        oled
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
