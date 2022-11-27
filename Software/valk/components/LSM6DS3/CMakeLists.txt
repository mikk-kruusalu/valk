idf_component_register(SRCS "src/LSM6DS3.cpp" "driver/lsm6ds3tr-c_reg.c"
                    INCLUDE_DIRS "include" "driver"
                    REQUIRES "CPPI2C")
