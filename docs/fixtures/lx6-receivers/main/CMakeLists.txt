idf_component_register(SRCS "main.c" REQUIRES esp_driver_dac esp_driver_gpio esp_driver_i2s esp_driver_rmt)
if(LX6_RECEIVER_STICK)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE LX6_RECEIVER_STICK=1)
endif()
