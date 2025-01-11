zephyr_include_directories(inc)

aux_source_directory(src PERIPHERALS_SRCS)

zephyr_library_sources(
    ${PERIPHERALS_SRCS}
)
