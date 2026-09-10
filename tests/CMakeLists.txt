cmake_minimum_required(VERSION 3.20.0)

set(SEP3_ZEPHYR_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
list(APPEND EXTRA_ZEPHYR_MODULES ${SEP3_ZEPHYR_ROOT})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sep3_zephyr_test)

target_sources(app PRIVATE
    src/main.c
    ${SEP3_ZEPHYR_ROOT}/../safe-c/safe_c.c
    ${SEP3_ZEPHYR_ROOT}/../framer7b-c/framer7b.c
)

target_include_directories(app PRIVATE
    ${SEP3_ZEPHYR_ROOT}/../safe-c
    ${SEP3_ZEPHYR_ROOT}/../framer7b-c
    ${SEP3_ZEPHYR_ROOT}/../crc-c
)

target_compile_options(app PRIVATE -Werror -Wno-attributes)
