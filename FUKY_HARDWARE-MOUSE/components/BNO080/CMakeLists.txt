idf_component_register(SRCS "src/BNO080.c" "src/shtp.c" "src/sh2.c" "src/sh2_util.c" "src/sh2_SensorValue.c"
                       INCLUDE_DIRS "include" 
                       REQUIRES driver
                       REQUIRES esp_timer
                      )
