cmake_minimum_required(VERSION 3.24)

project(lcd C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        st7735
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/st7735.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/lcdfonts.c
)
target_include_directories(
        st7735
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
