idf_component_register(SRCS "ota_lib.c"
                    INCLUDE_DIRS "./include"
                    REQUIRES app_update esp_http_client esp_https_ota nvs_flash
                    EMBED_TXTFILES ${project_dir}/components/ota_lib/server_certs/ca_cert.pem  # Embed the server root certificate into the final binary
                    )

target_compile_options(${COMPONENT_LIB} PUBLIC -Wno-unused-variable -Wall)