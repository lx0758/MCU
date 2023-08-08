cmake_minimum_required(VERSION 3.24)

project(MPU_MPL C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_library(libmplmpu STATIC IMPORTED)
set_target_properties(
        libmplmpu
        PROPERTIES
        IMPORTED_LOCATION
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/liblibmplmpu_m4_hardfp.a
)

file(GLOB_RECURSE MPUMPL_SRC_LIST "Lib/*.*" "Src/*.*")
add_library(
        mpumpl
        STATIC
        ${MPUMPL_SRC_LIST}
)
target_include_directories(
        mpumpl
        PUBLIC
        Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/driver/eMPL
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/driver/include
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/driver/stm32L
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/eMPL-hal
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/mllite
        ${CMAKE_CURRENT_SOURCE_DIR}/Lib/mpl
)
target_link_libraries(mpumpl libmplmpu)

# add_definitions(-DEMPL -DEMPL_TARGET_STM32 -DMPU9250 -DSELF_TEST_DEBUG)
