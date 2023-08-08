cmake_minimum_required(VERSION 3.24)

project(hmc5883l C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(
        hmc5883l
        STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/hmc_5883l.c
)
target_include_directories(
        hmc5883l
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)
