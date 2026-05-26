idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp_driver_rmt esp_driver_uart nvs_flash
)
