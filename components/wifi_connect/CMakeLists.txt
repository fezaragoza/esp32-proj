idf_component_register(SRCS "wifi_connect.c"
                    INCLUDE_DIRS "./include"
                    REQUIRES nvs_flash)

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-unused-variable -Wall)