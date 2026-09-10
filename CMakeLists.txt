if(CONFIG_SEP3_ZEPHYR)
    zephyr_library()

    zephyr_library_sources(
        src/sep3_zephyr.c
        sep3-c/sep3.c
    )

    zephyr_include_directories(
        include
        sep3-c
        ${CMAKE_CURRENT_SOURCE_DIR}/../safe-c
        ${CMAKE_CURRENT_SOURCE_DIR}/../framer7b-c
        ${CMAKE_CURRENT_SOURCE_DIR}/../crc-c
    )
endif()
