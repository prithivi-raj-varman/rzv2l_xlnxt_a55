cmake_minimum_required(VERSION 3.10)

project(openamp_a55_app C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    main.cpp
    helper.c
    platform_info.c
    rz_rproc.c
    cancomm/CanComm.cpp
)

add_executable(openamp_a55 ${SOURCES})

target_include_directories(openamp_a55
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/libmetal/usr/include
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/open-amp/usr/include
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/sysfsutils/usr/include
)

target_link_directories(openamp_a55
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/libmetal/usr/lib64
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/open-amp/usr/lib64
        ${CMAKE_CURRENT_SOURCE_DIR}/inc/sysfsutils/lib64
)

target_link_libraries(openamp_a55
    PRIVATE
        open_amp
        metal
        sysfs
        pthread
)
