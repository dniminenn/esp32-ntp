set(W5500_SRCS
    "../../ioLibrary_Driver/Ethernet/W5500/w5500.c"
    "../../ioLibrary_Driver/Ethernet/wizchip_conf.c"
    "../../ioLibrary_Driver/Ethernet/socket.c"
    "../../ioLibrary_Driver/Internet/DHCP/dhcp.c"
)

idf_component_register(
    SRCS "w5500_eth.cpp" ${W5500_SRCS}
    INCLUDE_DIRS "." "../../ioLibrary_Driver/Ethernet" "../../ioLibrary_Driver/Internet/DHCP"
    REQUIRES driver esp_netif config efuse esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC _WIZCHIP_=5500)

# Disable specific warnings for external library code
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-misleading-indentation)

