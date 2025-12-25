# cmake_minimum_required(VERSION 3.10)

# project(openamp_a55_app C)

# set(CMAKE_C_STANDARD 99)
# set(CMAKE_C_STANDARD_REQUIRED ON)

# set(SOURCES
#     main.c
#     helper.c
#     platform_info.c
#     rz_rproc.c
# )

# add_executable(openamp_a55 ${SOURCES})

# target_include_directories(openamp_a55
#     PRIVATE
#         ${CMAKE_CURRENT_SOURCE_DIR}
#         ${CMAKE_CURRENT_SOURCE_DIR}/metal
#         ${CMAKE_CURRENT_SOURCE_DIR}/sysfs
# )

# target_link_directories(openamp_a55
#     PRIVATE
#         ${CMAKE_SYSROOT}/usr/lib
# )

# target_link_libraries(openamp_a55
#     PRIVATE
#         open_amp
#         metal
#         pthread
# )

# target_compile_options(openamp_a55
#     PRIVATE
#         -Wall
#         -Wextra
# )

#2nd version
# cmake_minimum_required(VERSION 3.10)

# project(openamp_a55_app C)

# set(CMAKE_C_STANDARD 99)
# set(CMAKE_C_STANDARD_REQUIRED ON)

# set(SOURCES
#     main.c
#     helper.c
#     platform_info.c
#     rz_rproc.c
# )

# add_executable(openamp_a55 ${SOURCES})

# target_include_directories(openamp_a55
#     PRIVATE
#         ${CMAKE_CURRENT_SOURCE_DIR}
#         /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/libmetal/usr/include
#         /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/open-amp/usr/include
# )

# target_link_directories(openamp_a55
#     PRIVATE
#         /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/libmetal/usr/lib64
#         /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/open-amp/usr/lib64
# )

# target_link_libraries(openamp_a55
#     PRIVATE
#         open_amp
#         metal
#         pthread
# )


#3rd version
cmake_minimum_required(VERSION 3.10)

project(openamp_a55_app C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(SOURCES
    main.c
    helper.c
    platform_info.c
    rz_rproc.c
)

add_executable(openamp_a55 ${SOURCES})

target_include_directories(openamp_a55
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/libmetal/usr/include
        /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/open-amp/usr/include
        /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/sysfsutils/usr/include
)

target_link_directories(openamp_a55
    PRIVATE
        /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/libmetal/usr/lib64
        /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/open-amp/usr/lib64
        /media/prithivi/DATA-HDD/rzv_vlp_v3.0.7_update3/build/tmp/sysroots-components/aarch64/sysfsutils/lib64
)

target_link_libraries(openamp_a55
    PRIVATE
        open_amp
        metal
        sysfs
        pthread
)
