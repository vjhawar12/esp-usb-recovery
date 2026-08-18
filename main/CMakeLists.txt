idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_tinyusb )

target_include_directories(${COMPONENT_LIB} PRIVATE 
    "${IDF_PATH}/components/tinyusb/tinyusb/src"
)