cmake_minimum_required(VERSION 3.24)

project(anodt C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        anodt
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/ano_dt.c
)
target_include_directories(
        anodt
        PUBLIC
        Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/Inc
)
