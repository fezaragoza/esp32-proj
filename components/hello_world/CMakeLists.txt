idf_component_register(
    SRCS "hello_world.c"
    # SRC_DIRS "./src"
    INCLUDE_DIRS "./include"
    REQUIRES idf::spi_flash)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-unused-variable -Wall)