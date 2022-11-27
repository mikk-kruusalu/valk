idf_component_register(SRCS "src/line_sensor.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES "CPPLEDC" "CPPGPIO" "CPPANALOGIO" "nvs_flash")
