idf_component_register(SRCS "main.c"
                    REQUIRES "neo_si7021" "neo_i2c" "icm42670" "unity" "blink"
                    INCLUDE_DIRS ".")