
idf_component_register(
    SRCS "src/FUKY_main.c"
    INCLUDE_DIRS "include"
    REQUIRES PAW3395
    REQUIRES PAW3805
    REQUIRES BNO080
    REQUIRES BLE
    REQUIRES driver
    REQUIRES esp_adc
    REQUIRES esp_timer
    REQUIRES freertos
)
