idf_component_register(SRC_DIRS "." "ov7670_driver"
                    INCLUDE_DIRS "ov7670_driver/inc")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
