idf_component_register(SRCS "encoder_microros_main.c"
                       PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio esp_timer micro_ros_espidf_component
                       INCLUDE_DIRS ".")
