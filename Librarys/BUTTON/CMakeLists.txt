cmake_minimum_required(VERSION 3.24)

project(button C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        button
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/button.c
)
target_include_directories(
        button
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
