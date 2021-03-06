idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "./include"
    PRIV_REQUIRES hello_world pwm_lib ota_lib wifi_connect
)

set_source_files_properties( main.c
    PROPERTIES COMPILE_OPTIONS
    "-Wno-unused-variable;-Wuninitialized"
    # "-Wno-unused-variable;-Wpedantic;-Wall;-Wextra"
)