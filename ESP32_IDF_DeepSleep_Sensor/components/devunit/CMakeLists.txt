#set(COMPONENT_SRCDIRS .)
set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_REQUIRES "driver" "esp_wifi" "nvs_flash" "esp_netif" "lwip" "esp_event" "mqtt" "esp_http_client")

list(APPEND COMPONENT_SRCDIRS mysystem)
list(APPEND COMPONENT_ADD_INCLUDEDIRS mysystem)

if(CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT)
    list(APPEND COMPONENT_SRCDIRS wifi)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS wifi)
endif()

if(CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT)
    list(APPEND COMPONENT_SRCDIRS pcf8563)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS pcf8563)
endif()

if(CONFIG_SOFTWARE_SENSOR_SHT4X)
    list(APPEND COMPONENT_SRCDIRS sht4x)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS sht4x)
endif()

if(CONFIG_SOFTWARE_SENSOR_SHT3X)
    list(APPEND COMPONENT_SRCDIRS sht3x)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS sht3x)
endif()

if(CONFIG_SOFTWARE_SENSOR_BMP280)
    list(APPEND COMPONENT_SRCDIRS bmp280)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS bmp280)
endif()

if(CONFIG_SOFTWARE_SENSOR_SCD30)
    list(APPEND COMPONENT_SRCDIRS scd30)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS scd30)
endif()

if(CONFIG_SOFTWARE_SENSOR_BH1750)
    list(APPEND COMPONENT_SRCDIRS bh1750)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS bh1750)
endif()

register_component()