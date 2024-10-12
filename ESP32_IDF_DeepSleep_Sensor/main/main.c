#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
#include "driver/i2c_master.h"
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT

#if CONFIG_SOFTWARE_ESP_MQTT_SUPPORT
#include "mqtt_client.h"
#endif //CONFIG_SOFTWARE_ESP_MQTT_SUPPORT

#if (  CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT \
    || CONFIG_SOFTWARE_INTERNAL_BUTTON_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT \
    || CONFIG_SOFTWARE_SENSOR_SHT3X \
    || CONFIG_SOFTWARE_SENSOR_SHT4X \
    || CONFIG_SOFTWARE_SENSOR_BMP280 \
    || CONFIG_SOFTWARE_SENSOR_QMP6988 \
    || CONFIG_SOFTWARE_SENSOR_BME680 \
    || CONFIG_SOFTWARE_SENSOR_ADT7410 \
    || CONFIG_SOFTWARE_SENSOR_SCD30 \
    || CONFIG_SOFTWARE_SENSOR_SCD40 \
    || CONFIG_SOFTWARE_SENSOR_MHZ19C \
    || CONFIG_SOFTWARE_SENSOR_BH1750 \
    || CONFIG_SOFTWARE_EXTERNAL_6DIGIT_DISPLAY_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_SK6812_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_RTC_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_BUTTON_SUPPORT \
    || CONFIG_SOFTWARE_EXTERNAL_LED_SUPPORT )
#include "devunit.h"
#endif

static const char *TAG = "MY-MAIN";

#if ( CONFIG_SOFTWARE_ESP_MQTT_SUPPORT || CONFIG_SOFTWARE_SENSOR_USE_SENSOR )
int8_t g_sensor_mode = 0;
float g_temperature = 0.0;
float g_humidity = 0.0;
float g_pressure = 0.0;
int g_co2 = 0;
uint16_t g_lux = 0;
#endif //CONFIG_SOFTWARE_SENSOR_USE_SENSOR

RTC_DATA_ATTR static uint32_t boot_count = 0;
//static void obtain_time(void);

#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
i2c_master_bus_handle_t i2c0_master_bus_handle;
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT

/*
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK( esp_netif_init() );

    while ( wifi_isConnected() != ESP_OK )
    {
        vTaskDelay( pdMS_TO_TICKS(5000) );
    }

    ESP_LOGI(TAG, "Initializing and starting SNTP");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_NTP_SERVER_NAME);
    config.sync_cb = time_sync_notification_cb;     // Note: This is only needed if we want
    esp_netif_sntp_init(&config);

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 3;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    esp_netif_sntp_deinit();
}

void sntp_main()
{
    // Set timezone to Japan Standard Time and print local time
    setenv("TZ", "JST-9", 1);
    tzset();

    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];

    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Japan is: %s", strftime_buf);
}
*/

void sensor_mode()
{
    #if CONFIG_SOFTWARE_SENSOR_TYPE_TEMPERATURE
    g_sensor_mode += 1;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_HUMIDITY
    g_sensor_mode += 2;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_PRESSURE
    g_sensor_mode += 4;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_CO2
    g_sensor_mode += 8;
    #endif
    #if CONFIG_SOFTWARE_SENSOR_TYPE_LIGHT
    g_sensor_mode += 16;
    #endif

    ESP_LOGD(TAG, "SENSOR(mode) mode %d", g_sensor_mode);
}

void sensor_viewer()
{
    switch (g_sensor_mode)
    {
    case 1:
        ESP_LOGI(TAG, "SENSOR temperature:%f", g_temperature);
        break;
    case 2:
        ESP_LOGI(TAG, "SENSOR humidity:%f", g_humidity);
        break;
    case 3:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f", g_temperature, g_humidity);
        break;
    case 4:
        ESP_LOGI(TAG, "SENSOR pressure:%f", g_pressure);
        break;
    case 5:
        ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f", g_temperature, g_pressure);
        break;
    case 6:
        ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f", g_humidity, g_pressure);
        break;
    case 7:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f", g_temperature, g_humidity, g_pressure);
        break;
    case 8:
        ESP_LOGI(TAG, "SENSOR CO2:%d", g_co2);
        break;
    case 9:
        ESP_LOGI(TAG, "SENSOR temperature:%f CO2:%d", g_temperature, g_co2);
        break;
    case 10:
        ESP_LOGI(TAG, "SENSOR humidity:%f CO2:%d", g_humidity, g_co2);
        break;
    case 11:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f CO2:%d", g_temperature, g_humidity, g_co2);
        break;
    case 12:
        ESP_LOGI(TAG, "SENSOR pressure:%f CO2:%d", g_pressure, g_co2);
        break;
    case 13:
        ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f CO2:%d", g_temperature, g_pressure, g_co2);
        break;
    case 14:
        ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f CO2:%d", g_humidity, g_pressure, g_co2);
        break;
    case 15:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f CO2:%d", g_temperature, g_humidity, g_pressure, g_co2);
        break;
    case 16:
        ESP_LOGI(TAG, "SENSOR lux:%u", g_lux);
        break;
    case 17:
        ESP_LOGI(TAG, "SENSOR temperature:%f lux:%u", g_temperature, g_lux);
        break;
    case 18:
        ESP_LOGI(TAG, "SENSOR humidity:%f lux:%u", g_humidity, g_lux);
        break;
    case 19:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f lux:%u", g_temperature, g_humidity, g_lux);
        break;
    case 20:
        ESP_LOGI(TAG, "SENSOR pressure:%f lux:%u", g_pressure, g_lux);
        break;
    case 21:
        ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f lux:%u", g_temperature, g_pressure, g_lux);
        break;
    case 22:
        ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f lux:%u", g_humidity, g_pressure, g_lux);
        break;
    case 23:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f lux:%u", g_temperature, g_humidity, g_pressure, g_lux);
        break;
    case 24:
        ESP_LOGI(TAG, "SENSOR CO2:%d lux:%u", g_co2, g_lux);
        break;
    case 25:
        ESP_LOGI(TAG, "SENSOR temperature:%f CO2:%d lux:%u", g_temperature, g_co2, g_lux);
        break;
    case 26:
        ESP_LOGI(TAG, "SENSOR humidity:%f CO2:%d lux:%u", g_humidity, g_co2, g_lux);
        break;
    case 27:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f CO2:%d lux:%u", g_temperature, g_humidity, g_co2, g_lux);
        break;
    case 28:
        ESP_LOGI(TAG, "SENSOR pressure:%f CO2:%d lux:%u", g_pressure, g_co2, g_lux);
        break;
    case 29:
        ESP_LOGI(TAG, "SENSOR temperature:%f pressure:%f CO2:%d lux:%u", g_temperature, g_pressure, g_co2, g_lux);
        break;
    case 30:
        ESP_LOGI(TAG, "SENSOR humidity:%f pressure:%f CO2:%d lux:%u", g_humidity, g_pressure, g_co2, g_lux);
        break;
    case 31:
        ESP_LOGI(TAG, "SENSOR temperature:%f humidity:%f pressure:%f CO2:%d lux:%u", g_temperature, g_humidity, g_pressure, g_co2, g_lux);
        break;
    default:
        break;
    }
}

void sensor_main()
{
    esp_err_t ret = ESP_OK;
#if CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT
    i2c_master_bus_config_t i2c_mst_config_0 = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C0_MASTER_PORT,
        .scl_io_num = I2C0_MASTER_SCL_PIN,
        .sda_io_num = I2C0_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ret = i2c_new_master_bus(&i2c_mst_config_0, &i2c0_master_bus_handle);
    vTaskDelay( pdMS_TO_TICKS(100) );
    if (ret == ESP_OK)
    {
        ESP_LOGD(TAG, "i2c_new_master_bus is OK.");
    }
    else
    {
        ESP_LOGE(TAG, "I2C i2c_new_master_bus error");
    }
#endif //CONFIG_SOFTWARE_EXTERNAL_I2C_SUPPORT

#if CONFIG_SOFTWARE_SENSOR_BMP280
    bool _isSensorBmp280 = false;
    ret = Bmp280_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Bmp280_Init() is OK!");
        _isSensorBmp280 = true;
    }
    else
    {
        ESP_LOGE(TAG, "Bmp280_Init Error");
        g_sensor_mode -= 4;
    }
#endif // CONFIG_SOFTWARE_SENSOR_BMP280

#if CONFIG_SOFTWARE_SENSOR_SHT4X
    bool _isSensorSht4x = false;
    ret = Sht4x_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Sht4x_Init() is OK!");
        _isSensorSht4x = true;
    }
    else
    {
        ESP_LOGE(TAG, "Sht4x_Init Error");
        g_sensor_mode -= 3;
    }
#endif // CONFIG_SOFTWARE_SENSOR_SHT4X

#if CONFIG_SOFTWARE_SENSOR_SHT3X
    bool _isSensorSht3x = false;
    ret = Sht3x_Init(i2c0_master_bus_handle);
    if (ret == ESP_OK) {
        ESP_LOGD(TAG, "Sht3x_Init() is OK!");
        _isSensorSht3x = true;
    }
    else
    {
        ESP_LOGE(TAG, "Sht3x_Init Error");
        g_sensor_mode -= 3;
    }
#endif // CONFIG_SOFTWARE_SENSOR_SHT3X

#if CONFIG_SOFTWARE_SENSOR_BMP280
        if (_isSensorBmp280) {
            g_pressure = Bmp280_getPressure() / 100.0;
        }
#endif // CONFIG_SOFTWARE_SENSOR_BMP280

#if CONFIG_SOFTWARE_SENSOR_SHT4X
        if (_isSensorSht4x) {
            ret = Sht4x_Read();
            if (ret == ESP_OK) {
                vTaskDelay( pdMS_TO_TICKS(100) );
                g_temperature = Sht4x_GetTemperature();
                g_humidity = Sht4x_GetHumidity();
            }
        }
#endif // CONFIG_SOFTWARE_SENSOR_SHT4X

#if CONFIG_SOFTWARE_SENSOR_SHT3X
        if (_isSensorSht3x) {
            ret = Sht3x_Read();
            if (ret == ESP_OK) {
                vTaskDelay( pdMS_TO_TICKS(100) );
                g_temperature = Sht3x_GetTemperature();
                g_humidity = Sht3x_GetHumidity();
            }
        }
#endif // CONFIG_SOFTWARE_SENSOR_SHT3X

}

esp_mqtt_client_handle_t mqttClient;
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%ld", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA, TOPIC=%s, DATA=%s", event->topic , event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGD(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
        .buffer.size = CONFIG_BROKER_BUFFER_SIZE,
        .session.protocol_ver = CONFIG_MQTT_PROTOCOL_311,
        .session.last_will.qos = CONFIG_BROKER_LWT_QOS,
        .credentials.client_id = CONFIG_BROKER_MY_DEVICE_ID,
    };

    mqttClient = esp_mqtt_client_init(&mqtt_cfg);
    if (mqttClient == NULL) {
        ESP_LOGE(TAG, "esp_mqtt_client_init() is error.");
        return;
    }
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(mqttClient, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqttClient));
}

static void mqtt_app_stop(void)
{
    ESP_ERROR_CHECK(esp_mqtt_client_disconnect(mqttClient));
    ESP_ERROR_CHECK(esp_mqtt_client_stop(mqttClient));
    ESP_ERROR_CHECK(esp_mqtt_client_destroy(mqttClient));
}

void mqtt_main()
{
    // connected wifi
    if (wifi_isConnected() != ESP_OK) {
        return;
    }
    mqtt_app_start();

    int msg_id;
    char pubMessage[128] = {0};
    ESP_LOGD(TAG, "SENSOR mode %d", g_sensor_mode);
    switch (g_sensor_mode)
    {
    case 1:
        sprintf(pubMessage, "{\"temperature\":%4.1f}", g_temperature);
        break;
    case 2:
        sprintf(pubMessage, "{\"humidity\":%4.1f}", g_humidity);
        break;
    case 3:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"humidity\":%4.1f}", g_temperature, g_humidity);
        break;
    case 4:
        sprintf(pubMessage, "{\"pressure\":%4.1f}", g_pressure);
        break;
    case 5:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"pressure\":%4.1f}", g_temperature, g_pressure);
        break;
    case 6:
        sprintf(pubMessage, "{\"humidity\":%4.1f,\"pressure\":%4.1f}", g_humidity, g_pressure);
        break;
    case 7:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"humidity\":%4.1f,\"pressure\":%4.1f}", g_temperature, g_humidity, g_pressure);
        break;
    case 8:
        sprintf(pubMessage, "{\"co2\":%4d}", g_co2);
        break;
    case 9:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"co2\":%4d}", g_temperature, g_co2);
        break;
    case 10:
        sprintf(pubMessage, "{\"humidity\":%4.1f,\"co2\":%4d}", g_humidity, g_co2);
        break;
    case 11:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"humidity\":%4.1f,\"co2\":%4d}", g_temperature, g_humidity, g_co2);
        break;
    case 12:
        sprintf(pubMessage, "{\"pressure\":%4.1f,\"co2\":%4d}", g_pressure, g_co2);
        break;
    case 13:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"pressure\":%4.1f,\"co2\":%4d}", g_temperature, g_pressure, g_co2);
        break;
    case 14:
        sprintf(pubMessage, "{\"humidity\":%4.1f,\"pressure\":%4.1f,\"co2\":%4d}", g_humidity, g_pressure, g_co2);
        break;
    case 15:
        sprintf(pubMessage, "{\"temperature\":%4.1f,\"humidity\":%4.1f,\"pressure\":%4.1f,\"co2\":%4d}", g_temperature, g_humidity, g_pressure, g_co2);
        break;
    case 16:
        sprintf(pubMessage, "{\"lux\":%u}", g_lux);
        break;
    case 17:
        sprintf(pubMessage, "{\"temperature\":%f,\"lux\":%u}", g_temperature, g_lux);
        break;
    case 18:
        sprintf(pubMessage, "{\"humidity\":%f,\"lux\":%u}", g_humidity, g_lux);
        break;
    case 19:
        sprintf(pubMessage, "{\"temperature\":%f,\"humidity\":%f,\"lux\":%u}", g_temperature, g_humidity, g_lux);
        break;
    case 20:
        sprintf(pubMessage, "{\"pressure\":%f,\"lux\":%u}", g_pressure, g_lux);
        break;
    case 21:
        sprintf(pubMessage, "{\"temperature\":%f,\"pressure\":%f,\"lux\":%u}", g_temperature, g_pressure, g_lux);
        break;
    case 22:
        sprintf(pubMessage, "{\"humidity\":%f,\"pressure\":%f,\"lux\":%u}", g_humidity, g_pressure, g_lux);
        break;
    case 23:
        sprintf(pubMessage, "{\"temperature\":%f,\"humidity\":%f,\"pressure\":%f,\"lux\":%u}", g_temperature, g_humidity, g_pressure, g_lux);
        break;
    case 24:
        sprintf(pubMessage, "{\"CO2:%d,\"lux\":%u}", g_co2, g_lux);
        break;
    case 25:
        sprintf(pubMessage, "{\"temperature\":%f,\"CO2\":%d,\"lux\":%u}", g_temperature, g_co2, g_lux);
        break;
    case 26:
        sprintf(pubMessage, "{\"humidity\":%f,\"CO2\":%d,\"lux\":%u}", g_humidity, g_co2, g_lux);
        break;
    case 27:
        sprintf(pubMessage, "{\"temperature\":%f,\"humidity\":%f,\"CO2\":%d,\"lux\":%u}", g_temperature, g_humidity, g_co2, g_lux);
        break;
    case 28:
        sprintf(pubMessage, "{\"pressure\":%f,\"CO2\":%d,\"lux\":%u}", g_pressure, g_co2, g_lux);
        break;
    case 29:
        sprintf(pubMessage, "{\"temperature\":%f,\"pressure\":%f,\"CO2\":%d,\"lux\":%u}", g_temperature, g_pressure, g_co2, g_lux);
        break;
    case 30:
        sprintf(pubMessage, "{\"humidity\":%f,\"pressure\":%f,\"CO2\":%d,\"lux\":%u}", g_humidity, g_pressure, g_co2, g_lux);
        break;
    case 31:
        sprintf(pubMessage, "{\"temperature\":%f,\"humidity\":%f,\"pressure\":%f,\"CO2\":%d,\"lux\":%u}", g_temperature, g_humidity, g_pressure, g_co2, g_lux);
        break;
    default:
        break;
    }
    msg_id = esp_mqtt_client_publish(mqttClient, CONFIG_BROKER_MY_PUB_TOPIC, pubMessage, 0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d pubMessage=%s", msg_id, pubMessage);

    mqtt_app_stop();

}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
void app_main(void)
{
    ESP_LOGI(TAG, "app_main() start.");
    esp_log_level_set("*", ESP_LOG_ERROR);
    esp_log_level_set("MY-MAIN", ESP_LOG_INFO);
    esp_log_level_set("MY-WIFI", ESP_LOG_INFO);

#if CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT
    wifi_initialise();

//    sntp_main();
#endif //CONFIG_SOFTWARE_INTERNAL_WIFI_SUPPORT

#if CONFIG_SOFTWARE_SENSOR_USE_SENSOR
    sensor_main();

    sensor_mode();

    sensor_viewer();

#if CONFIG_SOFTWARE_ESP_MQTT_SUPPORT
    mqtt_main();
#endif //CONFIG_SOFTWARE_ESP_MQTT_SUPPORT

#endif //CONFIG_SOFTWARE_SENSOR_USE_SENSOR

    if (boot_count == UINT32_MAX)
    {
        boot_count = 0;
    }
    else
    {
        ++boot_count;
    }
    ESP_LOGI(TAG, "Boot count: %"PRIu32, boot_count);

#if CONFIG_DEEP_SLEEP_SUPPORT
    const int deep_sleep_sec = CONFIG_DEEP_SLEEP_WAKEUP_TIME_SEC;
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", deep_sleep_sec);
    esp_deep_sleep(1000000LL * deep_sleep_sec);
#endif //CONFIG_DEEP_SLEEP_SUPPORT
}