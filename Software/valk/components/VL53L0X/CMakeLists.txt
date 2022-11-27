set(COMPONENT_SRCS
    api/core/src/vl53l0x_api.c
    api/core/src/vl53l0x_api_calibration.c
    api/core/src/vl53l0x_api_core.c
    api/core/src/vl53l0x_api_ranging.c
    api/core/src/vl53l0x_api_strings.c
    api/platform/src/vl53l0x_platform.c
    api/platform/src/vl53l0x_platform_log.c
)

set(COMPONENT_ADD_INCLUDEDIRS
    inc
    api/core/inc
    api/platform/inc
)

idf_component_register(SRCS ${COMPONENT_SRCS}
                    INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS})
