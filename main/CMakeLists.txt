idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_tinyusb esp_driver_uart)

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

if(tusb_lib)
    target_include_directories(${tusb_lib} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
endif()