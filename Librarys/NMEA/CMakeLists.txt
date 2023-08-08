cmake_minimum_required(VERSION 3.24)

project(NMEA C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        nmea
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/nmea_0183.c
)
target_include_directories(
        nmea
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
