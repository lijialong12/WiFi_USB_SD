set(src_dirs
            LED
            SD
            USB)

set(include_dirs
            LED
            SD
            USB)

set(requires
            driver
            esp_driver_sdmmc
            esp_tinyusb)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})

component_compile_options(-ffast-math -O3 -Wno-error=format=-Wno-format)