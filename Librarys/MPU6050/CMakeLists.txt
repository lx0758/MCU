cmake_minimum_required(VERSION 3.24)

project(MPU6050 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        mpu6050
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/mpu_6050.c
)
target_include_directories(
        mpu6050
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
