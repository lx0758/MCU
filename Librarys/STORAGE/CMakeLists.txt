cmake_minimum_required(VERSION 3.24)

project(storage C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        storage
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/eeprom.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/flash.c
)
target_include_directories(
        storage
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
