#set(COMPONENT_SRCDIRS .)
set(COMPONENT_ADD_INCLUDEDIRS .)
set(COMPONENT_REQUIRES "driver" "esp_wifi" "nvs_flash" "esp_netif" "lwip" "esp_event" "mqtt")

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

if(CONFIG_SOFTWARE_SENSOR_BMP280)
    list(APPEND COMPONENT_SRCDIRS bmp280)
    list(APPEND COMPONENT_ADD_INCLUDEDIRS bmp280)
endif()

register_component()