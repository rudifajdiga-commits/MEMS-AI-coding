idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi esp_http_server esp_netif esp_driver_i2c driver esp_timer
                    EMBED_FILES "webapp/index.html" "webapp/FinalBaseMesh.glb.gz")
