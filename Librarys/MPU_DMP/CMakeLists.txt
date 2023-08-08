cmake_minimum_required(VERSION 3.24)

project(MPU_DMP C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        mpudmp
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/mpu_6050.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/Src/inv_mpu.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/Src/inv_mpu_dmp_motion_driver.c
)
target_include_directories(
        mpudmp
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/Inc
)
