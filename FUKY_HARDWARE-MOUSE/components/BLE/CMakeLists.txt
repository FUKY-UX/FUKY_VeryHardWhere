idf_component_register(SRCS "ble_hidd.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                            REQUIRES bt
                            REQUIRES driver
                            PRIV_REQUIRES nvs_flash
                    INCLUDE_DIRS ".")

#target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
