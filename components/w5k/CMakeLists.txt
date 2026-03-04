idf_component_register(
    SRCS "w5k_udp_wrapper.c" "w5k_tcp_wrapper.c"
    INCLUDE_DIRS "."
    REQUIRES w5500_eth
)

target_include_directories(${COMPONENT_LIB} PRIVATE
    "${CMAKE_SOURCE_DIR}/ioLibrary_Driver/Ethernet"
    "${CMAKE_SOURCE_DIR}/ioLibrary_Driver/Ethernet/W5500"
)

