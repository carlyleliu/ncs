zephyr_include_directories(inc)

aux_source_directory(src BLE_SRCS)

zephyr_library_sources(
    ${BLE_SRCS}
)
