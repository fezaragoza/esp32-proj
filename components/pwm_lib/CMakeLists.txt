idf_component_register(SRCS "pwm_lib.c"
                    INCLUDE_DIRS "./include")

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-unused-variable -Wall)