idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES lsm6dsox lis3mdl sensors esp_timer driver uart_wrapper)
